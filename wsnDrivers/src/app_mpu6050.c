#include "app_mpu6050.h"


/**
 * @brief function to read acceleration values from mpu6050 ACCEL_OUT registers, i.e.: from 0x3b to 0x40
 */
MPU6050_STATUS_e mpu6050_get_accel(const struct i2c_dt_spec * device_mpu6050, uint16_t * ax, uint16_t * ay, uint16_t * az){
    uint8_t temp_buffer[SIZE_OF_ACCEL_OUT_REG_FOR_BURST_READ] = {0};
    
    if (SUCCESS != i2c_burst_read_dt(device_mpu6050, MPU6050_ACCEL_XOUT_H, temp_buffer, SIZE_OF_ACCEL_OUT_REG_FOR_BURST_READ)){
        return MPU6050_READ_ERROR;
    }
    else{
        *ax = (uint16_t)((temp_buffer[0]<<8) | (temp_buffer[1]<<0));
        *ay = (uint16_t)((temp_buffer[2]<<8) | (temp_buffer[3]<<0));
        *az = (uint16_t)((temp_buffer[4]<<8) | (temp_buffer[5]<<0));
        return MPU6050_SUCCESS;
    }

    return MPU6050_UNEXPECTED_ERROR;
}

/**
 * @brief function checking the availability of the mpu6050 through i2c
 */
MPU6050_STATUS_e mpu6050_sanity_check(const struct i2c_dt_spec * device_mpu6050){
    uint8_t who_am_i_check = 0x00;
    uint8_t mpu6050_who_am_i_reg = MPU6050_WHO_AM_I_REG;

    if (false == device_is_ready(device_mpu6050->bus)){
			return MPU6050_DEVICE_NOT_READY;
	}

    if (SUCCESS != i2c_write_read_dt(device_mpu6050, &mpu6050_who_am_i_reg, 1, &who_am_i_check, 1)){
        return MPU6050_READ_ERROR;
    }

    if (0x68 == who_am_i_check){
		return MPU6050_SUCCESS;
	}
    else{
        return MPU6050_CHECK_SANITY_ERROR;
    }

    return MPU6050_UNEXPECTED_ERROR;
}


/**
 * @brief function to enable or disable sleep mode of the mpu6050
 */
MPU6050_STATUS_e mpu6050_sleep_control(const struct i2c_dt_spec * device_mpu6050, bool sleep_on_status_value){
    uint8_t pwr_mgmt_1_value = 0x00;
    uint8_t mpu6050_pwr_mgmt_1_reg =  MPU6050_PWR_MGMT_1_REG;

    uint8_t config[SIZE_OF_CONFIG_ARRAY_FOR_WRITE_READ] = {0};

    /* Get pwr_mgmt_1_reg value */
    if (SUCCESS != i2c_write_read_dt(device_mpu6050, &mpu6050_pwr_mgmt_1_reg, 1, &pwr_mgmt_1_value, 1)){
        return MPU6050_READ_ERROR;
    }

    if (true == sleep_on_status_value){
        pwr_mgmt_1_value &= (uint8_t)(~(0<<6)); // enable sleep bit
        config[0] = MPU6050_PWR_MGMT_1_REG;
        config[1] = pwr_mgmt_1_value;
        if (SUCCESS != i2c_write_dt(device_mpu6050, config, SIZE_OF_CONFIG_ARRAY_FOR_WRITE_READ)){
            return MPU6050_WRITE_ERROR;
        }
        return MPU6050_SUCCESS;
    }
    else{
        pwr_mgmt_1_value &= (uint8_t)(~(1<<6)); // disable sleep bit 
        config[0] = MPU6050_PWR_MGMT_1_REG;
        config[1] = pwr_mgmt_1_value;
        if (SUCCESS != i2c_write_dt(device_mpu6050, config, SIZE_OF_CONFIG_ARRAY_FOR_WRITE_READ)){
            return MPU6050_WRITE_ERROR;
        }
        return MPU6050_SUCCESS;
    }

    return MPU6050_UNEXPECTED_ERROR;
}

