#pragma once

// em-board + Nucleo-L432KC のピンアサイン

// 左車輪用DCモータ
#define M1_IN1 PA_9
#define M1_IN2 PB_0

// 右車輪用DCモータ
#define M2_IN1 PA_10
#define M2_IN2 PB_1

// 上下駆動用DCモータ
#define M3_IN1 PA_8
#define M3_IN2 PA_6

// 電極回転用DCモータ
#define M4_IN1 PA_1

// 装填用ステッピングモータ
#define M5_ENABLE PA_0
#define M5_STEP PA_5

// 左車輪用エンコーダ
#define ENC1_A PA_12

// 右車輪用エンコーダ
#define ENC2_A PB_7

// 上下駆動用エンコーダ
#define ENC3_A PA_3

// UART Bus
#define UART_TX PA_2
#define UART_RX NC

// I2C Bus
#define I2C_SCL PA_7
#define I2C_SDA PB_4

// SPI Bus
#define SPI_SCLK PB_3
#define SPI_MISO PA_11
#define SPI_MOSI PB_5
#define SPI_SSEL PA_4

// 溶断機構
#define FUSE_GATE PB_6
