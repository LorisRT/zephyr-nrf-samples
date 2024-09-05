#ifndef APP_MPU6050_H_
#define APP_MPU6050_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <hal/nrf_gpio.h>
#include <zephyr/drivers/i2c.h>

#define SUCCESS 0
#define MPU6050_WHO_AM_I_REG 0x75
#define MPU6050_PWR_MGMT_1_REG 0x6b
#define MPU6050_ACCEL_XOUT_H 0x3b

#define SIZE_OF_CONFIG_ARRAY_FOR_WRITE_READ 2
#define SIZE_OF_ACCEL_OUT_REG_FOR_BURST_READ 6

typedef enum {
    MPU6050_SUCCESS,
    MPU6050_DEVICE_NOT_READY,
    MPU6050_CHECK_SANITY_ERROR,
    MPU6050_READ_ERROR,
    MPU6050_WRITE_ERROR,
    MPU6050_UNEXPECTED_ERROR
} MPU6050_STATUS_e;

/* Function prototype */
MPU6050_STATUS_e mpu6050_sanity_check(const struct i2c_dt_spec *);
MPU6050_STATUS_e mpu6050_sleep_control(const struct i2c_dt_spec *, bool);
MPU6050_STATUS_e mpu6050_get_accel(const struct i2c_dt_spec *, uint16_t *, uint16_t *, uint16_t *);

#endif 