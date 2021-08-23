#include "mbed.h"
#include "PinAssignment.h"

#include <cstring>
#include <exception>

#include "QEI.h"
#include "lsm9ds1.h"
#include "MU2.h"
#include "BME280.h"
#include "PA1010D.h"

#include "Fusing.h"

#include "WheelMotor.h"
#include "WheelPID.h"
#include "WheelControl.h"
#include "MotorSpeed.h"
#include "DCMotor.h"
#include "DrillMotor.h"
#include "Stepper.h"

#define PRINT_BUFFER_SIZE 128

BufferedSerial serial(UART_TX, UART_RX);//


enum ExperimentMode{
    CHECK_ALL
};

char printBuffer[PRINT_BUFFER_SIZE];

ExperimentMode flag = CHECK_ALL;

PwmOut motor1In1(M1_IN1);
PwmOut motor1In2(M1_IN2);

PwmOut motor2In1(M2_IN1);
PwmOut motor2In2(M2_IN2);

PwmOut motor3In1(M3_IN1);
PwmOut motor3In2(M3_IN2);

PwmOut motor4In1(M4_IN1);

PwmOut fuseGate(FUSE_GATE);

DigitalOut motor5Enable(M5_ENABLE);
DigitalOut motor5Step(M5_STEP);

WheelMotor leftWheelMotor(&motor1In1, &motor1In2);
WheelMotor rightWheelMotor(&motor2In1, &motor2In2);

DCMotor verticalMotor(&motor3In1, &motor3In2);
DrillMotor drillMotor(&motor4In1);
QEI verticalEncoder(ENC3_A, NC, NC, 6, QEI::CHANNEL_A_ENCODING);

Stepper loadingMotor(&motor5Step, &motor5Enable);
QEI leftEncoder(ENC1_A, NC, NC, 6, QEI::CHANNEL_A_ENCODING);
QEI rightEncoder(ENC2_A, NC, NC, 6, QEI::CHANNEL_A_ENCODING);

I2C i2c(I2C_SDA, I2C_SCL);

LSM9DS1 imu(&i2c);
MU2 mu2(&serial);
BME280 bme280(&i2c);

PA1010D pa1010d(&i2c);

Fusing fusing(&fuseGate);
DigitalOut led(LED1);

MotorSpeed leftMotorSpeed(&leftEncoder, 298.0);
MotorSpeed rightMotorSpeed(&rightEncoder, 298.0);

WheelPID leftPID;
WheelPID rightPID;

WheelControl leftControl(&leftWheelMotor,&leftPID,&leftMotorSpeed);
WheelControl rightControl(&rightWheelMotor,&rightPID,&rightMotorSpeed);

DigitalIn SafetyPin(FUSE_GATE);

BufferedSerial bufferedSerial(UART_TX, UART_RX, MU2_SERIAL_BAUDRATE);

// main() runs in its own thread in the OS
int main() {
    if(flag==CHECK_ALL){
        //init GPIO
        snprintf(printBuffer, PRINT_BUFFER_SIZE, "Initializing GPIO and Setting I2C\r\n");
        serial.write(printBuffer,strlen(printBuffer));
        loadingMotor.idleCurrent(false);
        i2c.frequency(400000);

        //Check Wheel Motor
        snprintf(printBuffer, PRINT_BUFFER_SIZE, "Check Wheel Motor\r\n");
        serial.write(printBuffer,strlen(printBuffer));
        snprintf(printBuffer, PRINT_BUFFER_SIZE, "Target Speed is 15RPM\r\n");
        serial.write(printBuffer,strlen(printBuffer));
        leftControl.setTargetSpeed(15);
        rightControl.setTargetSpeed(15);
        //5s間実行
        for(int i = 0; i < 50; i++){
            snprintf(printBuffer, PRINT_BUFFER_SIZE, "L : %f \t R : %f [RPM]\r\n", leftMotorSpeed.currentSpeedRPM(), rightMotorSpeed.currentSpeedRPM());
            serial.write(printBuffer,strlen(printBuffer));
            ThisThread::sleep_for(100ms);
        }
        snprintf(printBuffer, PRINT_BUFFER_SIZE, "Left and Right Motor OFF\r\n");
        serial.write(printBuffer,strlen(printBuffer));
        leftControl.setTargetSpeed(0);
        rightControl.setTargetSpeed(0);
        ThisThread::sleep_for(1s);

        //Check LSM9DS1
        snprintf(printBuffer, PRINT_BUFFER_SIZE, "Check LSM9DS1\r\n");
        serial.write(printBuffer,strlen(printBuffer));
        imu.start();
        if(imu.getStatus()==LSM9DS1_STATUS_SUCCESS_TO_CONNECT){
            imu.start();
            //2s間みる
            for(int i = 0; i < 100; i++){
                snprintf(printBuffer, PRINT_BUFFER_SIZE, "$%f %f %f %f %f %f %f %f %f;\r\n",imu.accX(), imu.accY(), imu.accZ(), imu.gyrX(), imu.gyrY(), imu.gyrZ(), imu.magX(), imu.magY(), imu.magZ());
                serial.write(printBuffer,strlen(printBuffer));  
                ThisThread::sleep_for(20ms);
            }
            imu.stop();
        }
        
        //Check MU-2
        snprintf(printBuffer, PRINT_BUFFER_SIZE, "Check MU-2\r\n");
        serial.write(printBuffer,strlen(printBuffer));
        mu2.init();
        for (int i = 0; i < 10; i++) {
            const char *str = "Hello MU2";
            mu2.transmit(str, strlen(str));

            ThisThread::sleep_for(1s);
        }

        //Check bme280
        snprintf(printBuffer, PRINT_BUFFER_SIZE, "Check BME280\r\n");
        serial.write(printBuffer,strlen(printBuffer));
        bme280.start();
        for (int i = 0; i < 50; i++) {
            printf("press: %lf Pa\n", bme280.getPressure());
            printf("temp: %lf DegC\n", bme280.getTemprature());
            printf("hum: %lf %%RH\n", bme280.getHumidity());

            ThisThread::sleep_for(100ms);
        }

        //Check Fuse
        snprintf(printBuffer, PRINT_BUFFER_SIZE, "Check Fuse in 5s\r\n");
        serial.write(printBuffer,strlen(printBuffer));
        snprintf(printBuffer, PRINT_BUFFER_SIZE, "Heating will start in 10s. \r\n CAUTION!!!!!!\r\n");
        serial.write(printBuffer,strlen(printBuffer));
        led = 0;
        ThisThread::sleep_for(10s);
        led = 1;
        fusing.heat(10s);
        led = 0;
        snprintf(printBuffer, PRINT_BUFFER_SIZE, "Cooling Start\r\n");
        serial.write(printBuffer,strlen(printBuffer));
        ThisThread::sleep_for(2s);

        //Check PA1010D
        snprintf(printBuffer, PRINT_BUFFER_SIZE, "Check PA1010A\r\n");
        serial.write(printBuffer,strlen(printBuffer));
        pa1010d.start();
        //   PA1010D::RMCPacket rmc;
        PA1010D::GGAPacket gga;
        for (int i = 0; i < 5; i++) {
            // pa1010d.getRMC(&rmc);

            // printf("utc: %lf\n", rmc.utc);
            // printf("status: %c\n", rmc.status);
            // printf("latitude: %lf\n", rmc.latitude);
            // printf("nsIndicator: %c\n", rmc.nsIndicator);
            // printf("longitude: %lf\n", rmc.longitude);
            // printf("ewIndicator: %c\n", rmc.ewIndicator);
            // printf("speedOverGround: %f\n", rmc.speedOverGround);
            // printf("courseOverGround: %f\n", rmc.courseOverGround);
            // printf("date: %u\n", rmc.date);
            // printf("magneticVariation: %f\n", rmc.magneticVariation);
            // printf("variationDirection: %c\n", rmc.variationDirection);
            // printf("mode: %c\n", rmc.mode);

            pa1010d.getGGA(&gga);

            printf("utc: %lf\n", gga.utc);
            printf("latitude: %lf\n", gga.latitude);
            printf("nsIndicator: %c\n", gga.nsIndicator);
            printf("longitude: %lf\n", gga.longitude);
            printf("ewIndicator: %c\n", gga.ewIndicator);
            printf("positionFixIndicator: %u\n", gga.positionFixIndicator);
            printf("satellitesUsed: %u\n", gga.satellitesUsed);
            printf("hdop: %f\n", gga.hdop);
            printf("mslAltitude: %f\n", gga.mslAltitude);
            printf("geoidalSeparation: %f\n", gga.geoidalSeparation);
            printf("ageOfDiffCorr: %f\n", gga.ageOfDiffCorr);
            printf("stationID: %u\n", gga.stationID);

            ThisThread::sleep_for(1s);
        }
        while(true){
            snprintf(printBuffer, PRINT_BUFFER_SIZE, "All Checking is DONE\r\n");
            serial.write(printBuffer,strlen(printBuffer));
            ThisThread::sleep_for(500ms);
        }
    }
}
