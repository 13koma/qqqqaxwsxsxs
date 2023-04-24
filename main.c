#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#define MPU9150_ADDR 0x68
#define AK8975_ADDR  0x0C

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} mpu9150_mag_data_t;

void i2c_init() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GPIO_NUM_21;
    conf.scl_io_num = GPIO_NUM_22;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    conf.clk_flags = 0;
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

void initialize_mpu9150() {
    i2c_cmd_handle_t cmd;

    // Set I2C bypass enable
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9150_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x37, true);
    i2c_master_write_byte(cmd, 0x02, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void read_magnetometer(mpu9150_mag_data_t *mag_data) {
    uint8_t data[6];
    i2c_cmd_handle_t cmd;

    // Set AK8975 to single measurement mode
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AK8975_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x0A, true);
    i2c_master_write_byte(cmd, 0x01, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for data collection

    // Read magnetometer data
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AK8975_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x03, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AK8975_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    mag_data->x = (data[1] << 8) | data[0];
    mag_data->y = (data[3] << 8) | data[2];
    mag_data->z = (data[5] << 8) | data[4];
}

void mpu9150_task(void *pvParameters) {
    mpu9150_mag_data_t mag_data;
    i2c_init();
    initialize_mpu9150();

    while (1) {
        read_magnetometer(&mag_data);

        printf("Mag: X=%d, Y=%d, Z=%d\n", mag_data.x, mag_data.y, mag_data.z);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main() {
    xTaskCreate(mpu9150_task, "mpu9150_task", 2048, NULL, 5, NULL);
}
