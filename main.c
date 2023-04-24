#include <math.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

/* MPU6050 settings */
#define I2C_MASTER_SCL_IO           22          /* GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO           21          /* GPIO number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0   /* I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE   0           /* I2C master Tx buffer disabled */
#define I2C_MASTER_RX_BUF_DISABLE   0           /* I2C master Rx buffer disabled */
#define I2C_FREQ_HZ                 100000      /* I2C master clock frequency */
#define I2C_CLK_FLAG                0           /* I2C master clock flag */
#define MPU9150_ADDR                0x68        /* MPU9150 device address */
#define AK8975_ADDR                 0x0C        /* AK8975 device address */
#define CAL_SAMPLES                 1000        /* Number of loops to calibrate MPU6050 */

#define MPU9150_PWR_MGMT_1          0x00
#define AK8975_CNTL 0x0A
#define AK8975_POWER_DOWN 0x00
#define AK8975_SINGLE_MEASUREMENT 0x01
#define MPU9150_INT_PIN_CFG 0x37
#define MPU9150_USER_CTRL 0x6A
#define MPU9150_I2C_MST_EN 0x20
#define MPU9150_I2C_MST_CLK 0x0D
#define MPU9150_I2C_SLV0_ADDR 0x25
#define MPU9150_I2C_SLV0_REG 0x26
#define MPU9150_I2C_SLV0_CTRL 0x27
#define MPU9150_SMPLRT_DIV 0x07
#define MPU9150_CONFIG 0x06
#define MPU9150_GYRO_CONFIG 0x18
#define MPU9150_ACCEL_CONFIG 0x10
#define MPU9150_I2C_MST_CTRL 0x24


/* Hextronik HXT900 servo settings */
#define SERVO_FREQ                  20          /* Frequency of a servo */
#define SERVO_MID_ANGLE             90          /* Mid angle for servo */
#define SERVO_MIN_PULSEWIDTH        450         /* Minimum pulse width in microsecond */
#define SERVO_MAX_PULSEWIDTH        2300        /* Maximum pulse width in microsecond */
#define SERVO_MAX_DEGREE            180         /* Maximum angle in degree upto which servo can rotate */
#define SERVO_ROLL_GPIO             18          /* Servo controlling roll */

#define MPU9150_I2C_ADDRESS     (0x68) // MPU9150 I2C address
#define MPU9150_RA_WHO_AM_I     (0x75) // MPU9150 WHO_AM_I register address
#define MPU9150_WHO_AM_I        (0x68) // MPU9150 WHO_AM_I register value
#define MPU9150_RA_PWR_MGMT_1   (0x6B) // MPU9150 PWR_MGMT_1 register address
#define MPU9150_PWR1_CLKSEL_BIT (2)    // MPU9150 PWR_MGMT_1 CLKSEL bit
#define MPU9150_PWR1_SLEEP_BIT  (6)    // MPU9150 PWR_MGMT_1 SLEEP bit
#define MPU9150_PWR1_DEVICE_RESET_BIT (7) // MPU9150 PWR_MGMT_1 DEVICE_RESET bit
#define MPU9150_RA_SMPLRT_DIV   (0x19) // MPU9150 SMPLRT_DIV register address
#define MPU9150_RA_CONFIG       (0x1A) // MPU9150 CONFIG register address
#define MPU9150_RA_GYRO_CONFIG  (0x1B) // MPU9150 GYRO_CONFIG register address
#define MPU9150_RA_ACCEL_CONFIG (0x1C) // MPU9150 ACCEL_CONFIG register address
#define MPU9150_RA_INT_PIN_CFG  (0x37) // MPU9150 INT_PIN_CFG register address
#define MPU9150_RA_ACCEL_XOUT_H (0x3B) // MPU9150 ACCEL_XOUT_H register address
#define MPU9150_RA_PWR_MGMT_2   (0x6C) // MPU9150 PWR_MGMT_2 register address
#define AK8975_I2C_ADDRESS      (0x0C) // AK8975 I2C address
#define AK8975_RA_WIA           (0x00) // AK8975 WIA register address
#define AK8975_WIA              (0x48) // AK8975 WIA register value
#define AK8975_RA_CNTL          (0x0A) // AK8975 CNTL register address
#define AK8975_CNTL_MODE_MEASURE  (0x01) // AK8975 CNTL mode: single measurement
#define AK8975_RA_HXL           (0x03) // AK8975 HXL register address

#define TAG "MY_GIMBAL"

/* Variables to store offset and angle values of MPU9150 */
static float acc_x_offset = 0, acc_y_offset = 0, acc_z_offset = 0;
static float gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;
static float angle_x = 0, angle_y = 0;
static float angle_x_prev = 0;

/* Low-pass variables */
static const float alpha = 0.2;
float previousOutput = 0;

/* PID variables */
const float kp = 0.1;
const float ki = 0.1;
const float kd = 0.01;
float previous_error = 0;
float integral = 0;
float desired_angle = 0, current_angle = 0;
int dutyCycle = 0;

/* Structs to store acc and gyro variables */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} mpu9150_acc_data_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} mpu9150_gyro_data_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} mpu9150_mag_data_t;

esp_err_t mpu9150_read_acc_data(mpu9150_acc_data_t* acc_data);
esp_err_t mpu9150_read_gyro_data(mpu9150_gyro_data_t* gyro_data);
esp_err_t mpu9150_read_mag_data(mpu9150_mag_data_t* mag_data);
int pidOutputToDutyCycle(float pidOutput);

/* I2C initialization */
static esp_err_t i2c_master_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
        .clk_flags = I2C_CLK_FLAG
    };
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        return ret;
    }
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}



/* MPU9150 initialization and calibration */
esp_err_t mpu9150_init()
{
    uint8_t data;
    // Wake up MPU9150
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU9150_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x6B, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    // Read the contents of accelerometer config register 
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU9150_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU9150_RA_ACCEL_CONFIG, true);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    printf("MPU6050 WHO_AM_I: 0x%02x\n", data);
    // Read the contents of gyroscope config register 
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU9150_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU9150_RA_GYRO_CONFIG, true);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    // Enable I2C bypass
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU9150_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU9150_RA_INT_PIN_CFG, true);
    i2c_master_write_byte(cmd, 0x02, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    // Set magnetometer to single measure mode
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AK8975_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x0A, true);
    i2c_master_write_byte(cmd, 0x01, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    // Check magnetometer WHO_AM_I register
    // cmd = i2c_cmd_link_create();
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, AK8975_ADDR << 1 | I2C_MASTER_WRITE, true);
    // i2c_master_write_byte(cmd, 0x00, true);
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, AK8975_ADDR << 1 | I2C_MASTER_READ, true);
    // i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK);
    // i2c_master_stop(cmd);
    // ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    // i2c_cmd_link_delete(cmd);
    // if (ret != ESP_OK) {
    //     return ret;
    // }
    // printf("AK8975 WHO_AM_I: 0x%02x\n", data);

    // return ESP_OK;
    /* Calibration */
    mpu9150_acc_data_t acc_data;
    mpu9150_gyro_data_t gyro_data;
    mpu9150_mag_data_t mag_data;
    for (int i = 0; i < CAL_SAMPLES; i++) {
        mpu9150_read_acc_data(&acc_data);
        mpu9150_read_gyro_data(&gyro_data);
        mpu9150_read_mag_data(&mag_data);
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

int pidOutputToDutyCycle(float pidOutput) {
    float angle = constrain(pidOutput, 0, 180); // Constrain the PID output to the servo's range (0-180 degrees)
    int dutyCycle = map(angle, 0, 180, SERVO_MIN_PULSEWIDTH, SERVO_MAX_PULSEWIDTH); // Map the angle to the servo's duty cycle range
    return dutyCycle;
}

esp_err_t mpu9150_read_acc_data(mpu9150_acc_data_t* acc_data)
{
    uint8_t raw_data[6];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU9150_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x3B, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU9150_ADDR << 1 | I2C_MASTER_READ, true);
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

esp_err_t mpu9150_read_gyro_data(mpu9150_gyro_data_t* gyro_data)
{
    uint8_t raw_data[6];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU9150_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x43, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU9150_ADDR << 1 | I2C_MASTER_READ, true);
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

esp_err_t mpu9150_read_mag_data(mpu9150_mag_data_t *mag_data)
{
    // Set AK8975 magnetometer to single measurement mode
    // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, AK8975_ADDR << 1 | I2C_MASTER_WRITE, true);
    // i2c_master_write_byte(cmd, 0x0A, true);
    // i2c_master_write_byte(cmd, 0x01, true);
    // i2c_master_stop(cmd);
    // esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    // i2c_cmd_link_delete(cmd);

    // Read magnetometer data from AK8975 data registers
    uint8_t raw_data[6];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AK8975_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x03, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AK8975_ADDR << 1 | I2C_MASTER_READ, true);
    i2c_master_read(cmd, raw_data, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    // Convert raw data to magnetometer data structure
    mag_data->x = (raw_data[1] << 8) | raw_data[0];
    mag_data->y = (raw_data[3] << 8) | raw_data[2];
    mag_data->z = (raw_data[5] << 8) | raw_data[4];
    return ESP_OK;
}

float lowPassFilter(float input) {
  float output = alpha * input + (1 - alpha) * previousOutput;
  previousOutput = output;
  return output;
}

// void mpu6050_task(void* pvParameters) {
//     mpu6050_acc_data_t acc_data;
//     mpu6050_gyro_data_t gyro_data;

//     while (1) {
//         if (mpu6050_read_acc_data(&acc_data) == ESP_OK) {
//             ESP_LOGI(TAG, "Acc data: x=%d, y=%d, z=%d", acc_data.x, acc_data.y, acc_data.z);
//         } else {
//             ESP_LOGE(TAG, "Failed to read Acc data");
//         }
//         if (mpu6050_read_gyro_data(&gyro_data) == ESP_OK) {
//             ESP_LOGI(TAG, "Gyro data: x=%d, y=%d, z=%d", gyro_data.x, gyro_data.y, gyro_data.z);
//         } else {
//             ESP_LOGE(TAG, "Failed to read Gyro data");
//         }

//         // Calculate angle from accelerometer data
//         angle_x = lowPassFilter(atan2(acc_data.y, acc_data.z) * 180.0 / M_PI);
//         angle_y = lowPassFilter(atan2(-acc_data.x, sqrt(acc_data.y * acc_data.y + acc_data.z * acc_data.z)) * 180.0 / M_PI);
//         printf("Angle x: %f, Angle y: %f\n", angle_x, angle_y);
//         vTaskDelay(100 / portTICK_PERIOD_MS);
//     }
// }

void mpu9150_task(void* pvParameters) {
    mpu9150_acc_data_t acc_data;
    mpu9150_gyro_data_t gyro_data;
    mpu9150_mag_data_t mag_data;

    while (1) {
        mpu9150_read_acc_data(&acc_data);
        mpu9150_read_gyro_data(&gyro_data);
        mpu9150_read_mag_data(&mag_data);
        // Calculate angle from accelerometer data
        angle_x = lowPassFilter(atan2(acc_data.y, acc_data.z) * 180.0 / M_PI);
        angle_y = lowPassFilter(atan2(-acc_data.x, sqrt(acc_data.y * acc_data.y + acc_data.z * acc_data.z)) * 180.0 / M_PI);
        // printf("Angle x: %f, Angle y: %f\n", angle_x, angle_y);
        printf("ACC DATA: X=%d, Y=%d, Z=%d\n", acc_data.x, acc_data.y, acc_data.z);
        printf("GYRO DATA: X=%d, Y=%d, Z=%d\n", gyro_data.x, gyro_data.y, gyro_data.z);
        printf("MAG DATA: X=%d, Y=%d, Z=%d\n", mag_data.x, mag_data.y, mag_data.z);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void sg90_task(void *pvParameters) {
    float desired_angle = 0; // Horizontal position
    while (1) {
    float current_angle = angle_x;
    float error = desired_angle - current_angle;
    // Calculate the integral term
    integral += error;
    // Calculate the derivative term
    float derivative = error - previous_error;
    // Calculate the PID output
    float output = kp * error + ki * integral + kd * derivative;
    // Update the previous error
    previous_error = error;
    int value = pidOutputToDutyCycle(output);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, value);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    i2c_master_init();
    ESP_LOGI(TAG, "I2C initialized");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_ROLL_GPIO);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50; //set frequency to 50Hz - standard for servos
    pwm_config.cmpr_a = 0; //set duty cycle of PWM A to 0
    pwm_config.cmpr_b = 0; //set duty cycle of PWM B to 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    // Set the servo to the middle position
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, (SERVO_MAX_PULSEWIDTH + SERVO_MIN_PULSEWIDTH) / 2);
    mpu9150_init();
    ESP_LOGI(TAG, "MPU6050 initialized");
    // servo_init();
    // ESP_LOGI(TAG, "SG90 SERVO initialized");
    xTaskCreate(mpu9150_task, "mpu6050_task", 2048, NULL, 5, NULL);
    xTaskCreate(sg90_task, "sg90_task", 2048, NULL, 5, NULL);
}
