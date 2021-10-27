// Copyright 2021-2021 The jdh99 Authors. All rights reserved.
// 温湿度传感器AHT10的驱动
// Authors: jdh99 <jdh821@163.com>

#include "aht10.h"
#include "bror.h"

#include "driver/i2c.h"

// 线程堆栈大小.单位:KByte
#define THREAD_STACK_SIZE 4

#define AHT10_CLK_SPEED 100000

// 命令字
#define CMD_INIT 0xE0
#define CMD_GET 0xAC

// 读取时等待间隔.单位:ms
#define GET_WAIT_INTERVAL 100

#define SlaveAddress 0x70

/*!< I2C port number */
#define IIC_CTRL_NUM I2C_NUM_0

/*!< I2C master write */
#define WRITE_BIT 0x00
/*!< I2C master read */
#define READ_BIT 0x01
/*!< I2C master will check ack from slave*/
#define ACK_CHECK_EN 0x1
/*!< I2C master will not check ack from slave */
#define ACK_CHECK_DIS 0x0
/*!< I2C ack value */
#define ACK_VAL 0x0
/*!< I2C nack value */
#define NACK_VAL 0x1

static Aht10Value ahtValue;

static void i2cMasterInit(int pinSda, int pinScl);
static void aht10Thread(void* param);
static esp_err_t i2cWriteBytes(uint8_t WriteAddr, uint8_t* dataWr, uint8_t length);
static esp_err_t i2cReadBytes(uint8_t* dataRd, uint8_t length);

// Aht10Load 模块载入
void Aht10Load(int pinSda, int pinScl) {
    i2cMasterInit(pinSda, pinScl);

    uint8_t buf[2] = {0x08, 0x00};
    i2cWriteBytes(CMD_INIT, buf, 2);

    BrorThreadCreate(aht10Thread, "aht10Thread", BROR_THREAD_PRIORITY_LOWEST, 
        THREAD_STACK_SIZE * 1024);
}

static void i2cMasterInit(int pinSda, int pinScl) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = pinSda,
        .scl_io_num = pinScl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = AHT10_CLK_SPEED,
    };
    i2c_param_config(I2C_NUM_0, &conf);

    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

static void aht10Thread(void* param) {
    BrorDelayMS(1);

    uint8_t buf[10] = {0};
    uint32_t temp = 0;
    uint8_t ack = 0;

    while (1) {
        buf[0] = 0x33;
        buf[1] = 0x00;
        i2cWriteBytes(CMD_GET, buf, 2);
        BrorDelayMS(GET_WAIT_INTERVAL);

        i2cReadBytes(buf, 6);
        ack = buf[0];

        if ((ack & 0x40) == 0) {
            temp = (buf[1] << 12) + (buf[2] << 4) + (buf[3] >> 4);
            ahtValue.Humidity = (temp * 100) >> 20;

            temp = ((buf[3] & 0x0f) << 16) + (buf[4] << 8) + buf[5];
            ahtValue.Temperature = (int)((temp * 2000) >> 20) - 500;
        }

        BrorDelay(1);
    }

    BrorThreadDeleteMe();
}

static esp_err_t i2cWriteBytes(uint8_t WriteAddr, uint8_t* dataWr, uint8_t length) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SlaveAddress | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, WriteAddr, ACK_CHECK_EN);
    for (int i = 0; i < length; i++){
        i2c_master_write_byte(cmd, (intptr_t)dataWr + i, ACK_CHECK_EN);
    }
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(IIC_CTRL_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2cReadBytes(uint8_t* dataRd, uint8_t length) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, SlaveAddress | READ_BIT, ACK_CHECK_EN);
    for (int i = 0;i < (length-1);i++){
        i2c_master_read_byte(cmd, dataRd + i, ACK_VAL);
    }
    i2c_master_read_byte(cmd, dataRd + length - 1, NACK_VAL);

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(IIC_CTRL_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Aht10Get 读取温湿度
Aht10Value Aht10Get(void) {
    return ahtValue;
}
