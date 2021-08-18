#include "localization.h"

#include <memory>
#include "mbed.h"
#include "fusion-odometry.h"

Localization::Localization(WheelControl* wheelControl, FusionOdometry* ekf):
_wheelControl(wheelControl),
_ekf(ekf),
_thread(){
}

void Localization::start() {
    _thread = make_unique<Thread>(LOCALIZATION_THREAD_PRIORITY, LOCALIZATION_THREAD_STACK_SIZE, nullptr, LOCALIZATION_THREAD_NAME);
    _thread->start(callback(this, &Localization::threadLoop));
}

void Localization::stop() {
  _thread->terminate();
  _thread.reset();
}

void Localization::threadLoop(){
    while(true){

        ThisThread::sleep_for(LOCALIZATION_PERIOD);
    }
}