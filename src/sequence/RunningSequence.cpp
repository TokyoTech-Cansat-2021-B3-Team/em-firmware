#include "RunningSequence.h"

RunningSequence::RunningSequence(Navigation *navigation, Localization *localization, LSM9DS1 *imu,
                                 MotorSpeed *leftMotorSpeed, MotorSpeed *rightMotorSpeed,
                                 WheelControl *leftWheelControl, WheelControl *rightWheelControl, Console *console,
                                 Logger *logger)
    : _navigation(navigation), _localization(localization), _imu(imu), _leftMotorSpeed(leftMotorSpeed),
      _rightMotorSpeed(rightMotorSpeed), _leftWheelControl(leftWheelControl), _rightWheelControl(rightWheelControl),
      _console(console), _logger(logger), _state(UNDEFINED), _thread() {}

void RunningSequence::start(RunningSqequneceType sequenceType) {
  if (sequenceType == FIRST) {
    init();
    _console->lprintf("running", "init running sequence\n");
    setStatus(WAITING_FIRST_TO_SECOND_POLE);
    _console->lprintf("running", "waiting first to second pole\n");
  } else if (sequenceType == SECOND) {
    setStatus(WAITING_SECOND_TO_THIRD_POLE);
    _console->lprintf("running", "waiting second to third pole\n");
  } else if (sequenceType == THIRD) {
    setStatus(WAITING_THIRD_TO_FOURTH_POLE);
    _console->lprintf("running", "waiting third to fourth pole\n");
  }
  _thread = make_unique<Thread>(RUNNINGSEQUENCE_THREAD_PRIORITY, RUNNINGSEQUENCE_THREAD_STACK_SIZE, nullptr,
                                RUNNINGSEQUENCE_THREAD_NAME);
  _thread->start(callback(this, &RunningSequence::threadLoop));
}

void RunningSequence::init() {
  _imu->start();
  _leftMotorSpeed->start();
  _rightMotorSpeed->start();
  _localization->start();
  _leftWheelControl->start();
  _rightWheelControl->start();
  _navigation->start();
  _logger->init();
  _console->init();
}

void RunningSequence::stop() {
  _thread->terminate();
  _thread.reset();
}

void RunningSequence::threadLoop() {
  /*
  位置推定->偏差->制御量決定->モータ回転数変更
  のループはNavigateによってすべて行われる
  そのためシーケンスでは
  ・目標位置設定
  ・一定時間後に強制終了
  ・制御完了後の取次およびロギング
  の部分のみ処理を行う.
  */
  if (isWaiting()) {
    shiftStatusToMovingAndSetTargetPosition();
  } else {
    // WAITING状態でない状態でstartされた場合は待機
    while (!isWaiting()) {
      ThisThread::sleep_for(RUNNINGSEQUENCE_PERIOD);
    }
  }
  //エラー以外はwhileループが回る
  // waitingは対応するmovingになっているため正常時はすべてループに入る
  while (!isError()) {
    //正常終了の確認
    if (_navigation->checkArrivingTarget()) {
      shiftStatusToArrived();
      break;
    }
    //強制終了の確認
    if (_currentStateCount > 600) {
      _leftWheelControl->setTargetSpeed(0);
      _rightWheelControl->setTargetSpeed(0);

      setStatus(TERMINATE);
      _console->lprintf("running", "terminate\n");
      break;
    }
    _currentStateCount++;
    Logger::RunningData _tmpData = {_localization->x(),
                                    _localization->y(),
                                    _localization->theta(),
                                    _navigation->leftTargetSpeed(),
                                    _navigation->rightTargetSpeed(),
                                    _leftMotorSpeed->currentSpeedRPM(),
                                    _rightMotorSpeed->currentSpeedRPM()};
    _logger->runningLog(&_tmpData);
    ThisThread::sleep_for(RUNNINGSEQUENCE_PERIOD);
  }
  //エラー時もしくは完了時はループから抜ける
}

void RunningSequence::setStatus(RunningSequenceState state) {
  _state = state;
  _currentStateCount = 0;
}

void RunningSequence::shiftStatusToArrived() {
  switch (_state) {
  case MOVING_FIRST_TO_SECOND_POLE:
    setStatus(ARRIVED_SECOND_POLE);
    _console->lprintf("running", "arrived second pole\n");
    break;
  case MOVING_SECOND_TO_THIRD_POLE:
    setStatus(ARRIVED_THIRD_POLE);
    _console->lprintf("running", "arrived third pole\n");
    break;
  case MOVING_THIRD_TO_FOURTH_POLE:
    setStatus(ARRIVED_FOURTH_POLE);
    _console->lprintf("running", "arrived fourth pole\n");
    break;
  default:
    break;
  }
}

void RunningSequence::shiftStatusToMovingAndSetTargetPosition() {
  switch (_state) {
  case WAITING_FIRST_TO_SECOND_POLE:
    setStatus(MOVING_FIRST_TO_SECOND_POLE);
    _console->lprintf("running", "moving first to second pole\n");
    _navigation->setTargetPosition(_secondPolePosition[0], _secondPolePosition[1], _secondPoleEPS);
    break;
  case WAITING_SECOND_TO_THIRD_POLE:
    setStatus(MOVING_SECOND_TO_THIRD_POLE);
    _navigation->setTargetPosition(_thirdPolePosition[0], _thirdPolePosition[1], _thirdPoleEPS);
    _console->lprintf("running", "moving second to third pole\n");
    break;
  case WAITING_THIRD_TO_FOURTH_POLE:
    setStatus(MOVING_THIRD_TO_FOURTH_POLE);
    _navigation->setTargetPosition(_fourthPolePosition[0], _fourthPolePosition[1], _fourthPoleEPS);
    _console->lprintf("running", "moving third to fourth pole\n");
    break;
  default:
    break;
  }
}

bool RunningSequence::isMoving() {
  if (_state == MOVING_FIRST_TO_SECOND_POLE) {
    return true;
  }
  if (_state == MOVING_SECOND_TO_THIRD_POLE) {
    return true;
  }
  if (_state == MOVING_THIRD_TO_FOURTH_POLE) {
    return true;
  }
  return false;
}

bool RunningSequence::isError() {
  if (_state == TERMINATE) {
    return true;
  }
  if (_state == UNDEFINED) {
    return true;
  }
  return false;
}

bool RunningSequence::isWaiting() {
  if (_state == WAITING_FIRST_TO_SECOND_POLE) {
    return true;
  }
  if (_state == WAITING_SECOND_TO_THIRD_POLE) {
    return true;
  }
  if (_state == WAITING_THIRD_TO_FOURTH_POLE) {
    return true;
  }
  return false;
}

bool RunningSequence::isArrived() {
  if (_state == ARRIVED_SECOND_POLE) {
    return true;
  }
  if (_state == ARRIVED_THIRD_POLE) {
    return true;
  }
  if (_state == ARRIVED_FOURTH_POLE) {
    return true;
  }
  return false;
}

RunningSequenceState RunningSequence::state() {
  return _state;
}

void RunningSequence::pushDataToLogger() {}
