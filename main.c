#include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#define I2C_MASTER_SCL_IO           22          /*!< GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO           21          /*!< GPIO number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE   0           /*!< I2C master Tx buffer disabled */
#define I2C_MASTER_RX_BUF_DISABLE   0           /*!< I2C master Rx buffer disabled */
#define I2C_FREQ_HZ                 100000      /*!< I2C master clock frequency */
#define I2C_CLK_FLAG                0           /*!< I2C master clock flag */
#define MPU6050_ADDR                0x68        /*!< MPU6050 device address */

#define SERVO_GPIO                  4
#define SERVO_FREQ                  50
#define SERVO_MIN_PULSEWIDTH        500         //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH        2500        //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE            180         //Maximum angle in degree upto which servo can rotate
#define SERVO_MID_ANGLE             90          //Mid angle for servo

#define GYRO_RANGE_DPS             0.0075      //Gyro range in dps

#define TAG "MY_GIMBAL"

static float acc_x_offset = 0, acc_y_offset = 0, acc_z_offset = 0;
static float gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;
static float angle_x = 0, angle_y = 0;
static float angle_x_prev = 0;

float angle = 0.0f;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} mpu6050_acc_data_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} mpu6050_gyro_data_t;

esp_err_t mpu6050_read_acc_data(mpu6050_acc_data_t* acc_data);
esp_err_t mpu6050_read_gyro_data(mpu6050_gyro_data_t* gyro_data);

esp_err_t i2c_master_init()
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_HZ;
    conf.clk_flags = I2C_CLK_FLAG;
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        return ret;
    }
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t mpu6050_init()
{
    uint8_t data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x6B, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x1B, true);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    printf("MPU6050 WHO_AM_I: 0x%02x\n", data);
    // Calibration
    mpu6050_acc_data_t acc_data;
    mpu6050_gyro_data_t gyro_data;
    for (int i = 0; i < 1000; i++) {
        mpu6050_read_acc_data(&acc_data);
        mpu6050_read_gyro_data(&gyro_data);
        acc_x_offset += (float)acc_data.x / 16384.0;
        acc_y_offset += (float)acc_data.y / 16384.0;
        acc_z_offset += (float)acc_data.z / 16384.0;
        gyro_x_offset += (float)gyro_data.x / 131.0;
        gyro_y_offset += (float)gyro_data.y / 131.0;
        gyro_z_offset += (float)gyro_data.z / 131.0;
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
    acc_x_offset /= 1000.0;
    acc_y_offset /= 1000.0;
    acc_z_offset /= 1000.0;
    gyro_x_offset /= 1000.0;
    gyro_y_offset /= 1000.0;
    gyro_z_offset /= 1000.0;
    return ESP_OK;
}

void servo_init() {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_NUM_12); // Set GPIO 12 as PWM0A
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); //Configure PWM0A with above settings
}

int constrain(int value, int min, int max) {
    if (value < min) {
        return min;
    } else if (value > max) {
        return max;
    } else {
        return value;
    }
}

double map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



esp_err_t mpu6050_read_acc_data(mpu6050_acc_data_t* acc_data)
{
    uint8_t raw_data[6];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x3B, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_READ, true);
    i2c_master_read(cmd, raw_data, 5, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &raw_data[5], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    acc_data->x = (raw_data[0] << 8) | raw_data[1];
    acc_data->y = (raw_data[2] << 8) | raw_data[3];
    acc_data->z = (raw_data[4] << 8) | raw_data[5];
    acc_data->x -= (int16_t)(acc_x_offset * 16384.0);
    acc_data->y -= (int16_t)(acc_y_offset * 16384.0);
    acc_data->z -= (int16_t)(acc_z_offset * 16384.0);
    return ESP_OK;
}

esp_err_t mpu6050_read_gyro_data(mpu6050_gyro_data_t* gyro_data)
{
    uint8_t raw_data[6];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x43, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_READ, true);
    i2c_master_read(cmd, raw_data, 5, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &raw_data[5], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    gyro_data->x = (raw_data[0] << 8) | raw_data[1];
    gyro_data->y = (raw_data[2] << 8) | raw_data[3];
    gyro_data->z = (raw_data[4] << 8) | raw_data[5];
    gyro_data->x -= (int16_t)(gyro_x_offset * 131.0);
    gyro_data->y -= (int16_t)(gyro_y_offset * 131.0);
    gyro_data->z -= (int16_t)(gyro_z_offset * 131.0);
    return ESP_OK;
}

void mpu6050_task(void* pvParameters) {
    mpu6050_acc_data_t acc_data;
    mpu6050_gyro_data_t gyro_data;

    while (1) {
        if (mpu6050_read_acc_data(&acc_data) == ESP_OK) {
            ESP_LOGI(TAG, "Acc data: x=%d, y=%d, z=%d", acc_data.x, acc_data.y, acc_data.z);
        } else {
            ESP_LOGE(TAG, "Failed to read Acc data");
        }
        if (mpu6050_read_gyro_data(&gyro_data) == ESP_OK) {
            ESP_LOGI(TAG, "Gyro data: x=%d, y=%d, z=%d", gyro_data.x, gyro_data.y, gyro_data.z);
        } else {
            ESP_LOGE(TAG, "Failed to read Gyro data");
        }

        // Calculate angle from accelerometer data
        angle_x = atan2(acc_data.y, acc_data.z) * 180.0 / M_PI;
        angle_y = atan2(-acc_data.x, sqrt(acc_data.y * acc_data.y + acc_data.z * acc_data.z)) * 180.0 / M_PI;

        gyro_data.x -= gyro_x_offset;
        gyro_data.y -= gyro_y_offset;
        gyro_data.z -= gyro_z_offset;

        float gyro_sensitivity = 1.0 / (float)(32767 / GYRO_RANGE_DPS);
        // Convert gyro readings to degrees per second
        gyro_data.x = gyro_data.x * gyro_sensitivity;
        gyro_data.y = gyro_data.y * gyro_sensitivity;
        gyro_data.z = gyro_data.z * gyro_sensitivity;

        // Integrate angular velocity to get change in angle
        float dt = 0.1f; // Sample time in seconds
        angle += gyro_data.z * dt;

        // Convert angle to degrees
        angle *= 180.0f / M_PI;

        // Print the angle of turn in degrees
        printf("Angle of turn: %.2f degrees\n", angle);
        printf("Angle x: %f, Angle y: %f\n", angle_x, angle_y);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void sg90_task(void *pvParameters) {
    while (1) {
        if (fabs(angle_x - angle_x_prev) > 1) { // Check if angle has changed more than 1 degree
            int servo_angle = SERVO_MID_ANGLE - (angle_x / 2); // Calculate servo angle
            servo_angle = constrain(servo_angle, 0, SERVO_MAX_DEGREE); // Constrain servo angle between 0 and 180 degrees
            int duty_us = SERVO_MIN_PULSEWIDTH + (servo_angle * (SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) / SERVO_MAX_DEGREE);
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_us);
            angle_x_prev = angle_x;
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}

void app_main()
{
    i2c_master_init();
    ESP_LOGI(TAG, "I2C initialized");
    mpu6050_init();
    ESP_LOGI(TAG, "MPU6050 initialized");
    servo_init();
    ESP_LOGI(TAG, "SG90 SERVO initialized");
    xTaskCreate(mpu6050_task, "mpu6050_task", 2048, NULL, 5, NULL);
    xTaskCreate(sg90_task, "sg90_task", 2048, NULL, 5, NULL);
}
