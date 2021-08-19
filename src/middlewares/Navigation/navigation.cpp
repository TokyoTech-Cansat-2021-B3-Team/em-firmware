#include "navigation.h"
#include "localization.h"
#include "WheelControl.h"

Navigation::Navigation(Localization* localization, WheelControl* leftWheelControl, WheelControl* rightWheelControl):
_localization(localization),
_leftWheelControl(leftWheelControl),
_rightWheelControl(rightWheelControl),
_thread()
{

}

void Navigation::start() {
    _thread = make_unique<Thread>(NAVIGATION_THREAD_PRIORITY, NAVIGATION_THREAD_STACK_SIZE, nullptr, NAVIGATION_THREAD_NAME);
    _thread->start(callback(this, &Navigation::threadLoop));
}

void Navigation::stop() {
  _thread->terminate();
  _thread.reset();
}

void Navigation::threadLoop(){
    while(true){
        if(checkArrivingTarget()){
            _leftTargetSpeed = 0.0;
            _rightTargetSpeed = 0.0;
        }else{
            updateDifference();
            _deltaV = _gainKL * _y_diff + _gainKT * _theta_diff;
            _leftTargetSpeed = _cruiseSpeed + _deltaV;
            _rightTargetSpeed = _cruiseSpeed - _deltaV;
        }
        _leftWheelControl->setTargetSpeed(_leftTargetSpeed);
        _rightWheelControl->setTargetSpeed(_rightTargetSpeed);
        ThisThread::sleep_for(NAVIGATION_PERIOD);
    }
}

double Navigation::norm(double x, double y){
    return sqrt(x*x + y*y); 
}

void Navigation::updateDifference(){
    /*
    目標軌道は y = 0 の直線（x軸）であるとして考える
    これにより y を0に近づけるようなフィードバック制御をおこなう
    加えて姿勢角 theta も 0　に近づけるようなフィードバック制御も行う
    参考：http://www.mech.tohoku-gakuin.ac.jp/rde/contents/course/robotics/wheelrobot.html
    */
    _y_diff = _localization->y() - 0.0;
    _theta_diff = _localization->theta() - 0.0; 
}

bool Navigation::checkArrivingTarget(){
    if((_localization->x() - _targetX + _eps) > 0){
        return true;
    }else{
        return false;
    }
    /*
    double positionX_diff = _localization->x() - _targetX;
    double positionY_diff = _localization->y() - _targetY;
    double distance = norm(positionX_diff, positionY_diff);
    if(distance < _eps){
        return true;
    }else{
        return false;
    }
    */
}

void Navigation::setTargetPosition(double targetX, double targetY, double eps){
    _targetX = targetX;
    _targetY = targetY;
    _eps = eps;
}

double Navigation::leftTargetSpeed(){
    return _leftTargetSpeed;
}

double Navigation::rightTargetSpeed(){
    return _rightTargetSpeed;
}