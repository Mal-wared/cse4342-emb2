#ifndef CUSTOM_ICM_20948_H_
#define CUSTOM_ICM_20948_H_

// ICM20948 specific macros
#define ICM20948_ADDR 0x69

// REG_BANK_SEL
#define REG_BANK_SEL 0x7F
#define BANK0 0x00
#define BANK1 0x10
#define BANK2 0x20
#define BANK3 0x30

// BANK0
#define PWR_MGMT_1 0x06     // clock source & circuitry modes
#define PWR_MGMT_2 0x07     // accelerometer enables
#define ACCEL_XOUT_H 0x2D
#define ACCEL_XOUT_L 0x2E
#define ACCEL_YOUT_H 0x2F
#define ACCEL_YOUT_L 0x30
#define ACCEL_ZOUT_H 0x31
#define ACCEL_ZOUT_L 0x32
#define GYRO_XOUT_H 0x33
#define GYRO_XOUT_L 0x34
#define GYRO_YOUT_H 0x35
#define GYRO_YOUT_L 0x36
#define GYRO_ZOUT_H 0x37
#define GYRO_ZOUT_L 0x38

// BANK2
#define GYRO_SMPLRT_DIV 0x00
#define GYRO_CONFIG_1 0x01
#define ODR_ALIGN_EN 0x09
#define ACCEL_SMPLRT_DIV_1 0x10
#define ACCEL_SMPLRT_DIV_2 0x11
#define ACCEL_CONFIG 0x14

#endif
