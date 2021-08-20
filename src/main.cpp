#include "mbed.h"
#include "PinAssignment.h"

#include "lsm9ds1.h"
#include <cstring>
#include <exception>

#define PRINT_BUFFER_SIZE 128

BufferedSerial serial(UART_TX, UART_RX);//
I2C i2c(I2C_SDA, I2C_SCL);//

char printBuffer[PRINT_BUFFER_SIZE];


#include "PinAssignment.h"

#include "QEI.h"

#include "fusion-odometry.h"
#include "WheelMotor.h"
#include "WheelPID.h"
#include "WheelControl.h"
#include "MotorSpeed.h"
#include "localization.h"
#include "navigation.h"

#include "RunningSequence.h"

#define PRINT_BUFFER_SIZE 128

enum ExperimentMode{
    RunningAllSequence,
    RunningPoleToPole
};

ExperimentMode flag = RunningPoleToPole;

PwmOut motor1In1(M1_IN1);
PwmOut motor1In2(M1_IN2);

PwmOut motor2In1(M2_IN1);
PwmOut motor2In2(M2_IN2);

WheelMotor leftWheelMotor(&motor1In1, &motor1In2);
WheelMotor rightWheelMotor(&motor2In1, &motor2In2);

QEI leftEncoder(ENC1_A, NC, NC, 6, QEI::CHANNEL_A_ENCODING);
QEI rightEncoder(ENC2_A, NC, NC, 6, QEI::CHANNEL_A_ENCODING);

LSM9DS1 imu(&i2c);

MotorSpeed leftMotorSpeed(&leftEncoder, 298.0);
MotorSpeed rightMotorSpeed(&rightEncoder, 298.0);

WheelPID leftPID;
WheelPID rightPID;

WheelControl leftControl(&leftWheelMotor,&leftPID,&leftMotorSpeed);
WheelControl rightControl(&rightWheelMotor,&rightPID,&rightMotorSpeed);

FusionOdometry ekf(KALMANFILTER_PERIOD);

Localization localization(&leftMotorSpeed, &rightMotorSpeed, &imu, &ekf, 180.0e-3, 52.0e-3);

Navigation navi(&localization, &leftControl, &rightControl);

RunningSequence runningSequence(&navi, &localization, &imu, &leftMotorSpeed, &rightMotorSpeed, &leftControl, &rightControl);

DigitalIn SafetyPin(FUSE_GATE);

Thread speedThread(osPriorityAboveNormal, 1024, nullptr, nullptr);
Thread printThread(osPriorityAboveNormal, 1024, nullptr, nullptr);

void printThreadLoop(){
    while(true){
        snprintf(printBuffer, PRINT_BUFFER_SIZE, "$%d %f %f %f %f %f %f %f;\r\n",runningSequence.state(), navi.leftTargetSpeed(), navi.rightTargetSpeed(),leftMotorSpeed.currentSpeedRPM(), rightMotorSpeed.currentSpeedRPM() ,localization.theta(), localization.x(), localization.y());
        serial.write(printBuffer,strlen(printBuffer));
        ThisThread::sleep_for(20ms);
    }
}

void speedThreadLoop(){
    if(flag==RunningAllSequence){
        runningSequence.start(FIRST);
        if(imu.getStatus()==LSM9DS1_STATUS_SUCCESS_TO_CONNECT){
            snprintf(printBuffer, PRINT_BUFFER_SIZE, "Succeeded connecting LSM9DS1.\r\n");
            serial.write(printBuffer,strlen(printBuffer));
        }else{
            snprintf(printBuffer, PRINT_BUFFER_SIZE, "Failed to connect LSM9DS1.\r\n");
            serial.write(printBuffer,strlen(printBuffer));
        }
        int i = 0;
        while(true){
            if(runningSequence.state() == ARRIVED_SECOND_POLE){
                runningSequence.stop();
                ThisThread::sleep_for(1s);
                runningSequence.start(SECOND);
            }
            if(runningSequence.state() == ARRIVED_THIRD_POLE){
                runningSequence.stop();
                ThisThread::sleep_for(1s);
                runningSequence.start(THIRD);
            }
            if(runningSequence.state() == ARRIVED_FOURTH_POLE){
                runningSequence.stop();
                snprintf(printBuffer, PRINT_BUFFER_SIZE, "SUCCESS\r\n");
                serial.write(printBuffer,strlen(printBuffer));
                while(true){
                    ThisThread::sleep_for(100ms);
                }
            }
            ThisThread::sleep_for(100ms);
        }
    }else if(flag==RunningPoleToPole){
        imu.start();
        leftMotorSpeed.start();
        rightMotorSpeed.start();
        localization.start();
        leftControl.start();
        rightControl.start();
        navi.start();
        navi.setTargetPosition(1.0, 0.0, 0.1);
    }
}

// main() runs in its own thread in the OS
int main() {
    for(int i = 1; i < 51; i++){
        snprintf(printBuffer, PRINT_BUFFER_SIZE, "Waiting . . .\r\n");
        serial.write(printBuffer,strlen(printBuffer));
        ThisThread::sleep_for(100ms);
    }
    navi.setCruiseSpeed(30);
    SafetyPin.mode(PullDown);
    printThread.start(printThreadLoop);
    speedThread.start(speedThreadLoop);
    while(true){
        if(SafetyPin==0){
        speedThread.terminate();
        printThread.terminate();
        snprintf(printBuffer, PRINT_BUFFER_SIZE, "STOP PROGRAM\r\n");
        serial.write(printBuffer,strlen(printBuffer));
        leftWheelMotor.stop();
        rightWheelMotor.stop();
        ThisThread::sleep_for(500ms);

        exit(1);
        while(true){
                ThisThread::sleep_for(20ms);
        }
        }
        ThisThread::sleep_for(20ms);
    }

}
