// Copyright 2021-2021 The jdh99 Authors. All rights reserved.
// 温湿度传感器AHT10的驱动
// Authors: jdh99 <jdh821@163.com>

#include "aht10.h"
#include "bror.h"
#include "lagan.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

#define TAG "aht10"

// 线程堆栈大小.单位:KByte
#define THREAD_STACK_SIZE 4

// 时钟操作间隔.单位:us
#define CLK_INTERVAL 5

// 命令字
#define CMD_INIT 0xE0
#define CMD_GET 0xAC

// 读取时等待间隔.单位:ms
#define GET_WAIT_INTERVAL 100

// 从机地址
#define SLAVE_ADDRESS 0x70

static Aht10Value ahtValue;
static int gPinSda;
static int gPinScl;

static void i2cInit(void);
static void aht10Thread(void* param);
static void i2cStart(void);
static void i2cStop(void);
static bool i2cWaitAck(void);
static void i2cSendAck(void);
static void i2cSendNoAck(void);
static void i2cSendByte(uint8_t data);
static uint8_t i2cReceiveByte(void);
static bool readBytes(uint8_t* data, int dataLen);
static bool writeBytes(uint8_t addr, uint8_t* data, int dataLen);

// Aht10Load 模块载入
void Aht10Load(int pinSda, int pinScl) {
    gPinSda = pinSda;
    gPinScl = pinScl;
    i2cInit();

    BrorThreadCreate(aht10Thread, TAG, BROR_THREAD_PRIORITY_LOWEST, 
        THREAD_STACK_SIZE * 1024);
}

static void i2cInit(void) {
    gpio_config_t ioConf;
    ioConf.intr_type = GPIO_INTR_DISABLE;
    ioConf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    ioConf.pin_bit_mask = (1<<gPinScl) | (1<<gPinSda);
    ioConf.pull_down_en = 0;
    ioConf.pull_up_en = 1;
    gpio_config(&ioConf);

    gpio_set_level(gPinScl, true);
    gpio_set_level(gPinSda, true);
}

static void aht10Thread(void* param) {
    BrorDelayMS(1);

    uint8_t buf[10] = {0};
    uint32_t temp = 0;
    uint8_t ack = 0;

    while (1) {
        buf[0] = 0x33;
        buf[1] = 0x00;
        if (writeBytes(CMD_GET, buf, 2) == false) {
            LE(TAG, "write bytes error!");
            BrorDelay(1);
            continue;
        }

        BrorDelayMS(GET_WAIT_INTERVAL);

        if (readBytes(buf, 6) == false) {
            LE(TAG, "read bytes error!");
            BrorDelay(1);
            continue;
        }

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

// Aht10Get 读取温湿度
Aht10Value Aht10Get(void) {
    return ahtValue;
}

static void i2cStart(void) {
    gpio_set_level(gPinSda, 1);
    ets_delay_us(CLK_INTERVAL);
    gpio_set_level(gPinScl, 1);
    ets_delay_us(CLK_INTERVAL);
    gpio_set_level(gPinSda, 0);
    ets_delay_us(CLK_INTERVAL);
    gpio_set_level(gPinScl, 0);
    ets_delay_us(CLK_INTERVAL);
}

static void i2cStop(void) {
    gpio_set_level(gPinScl, 0);
    ets_delay_us(CLK_INTERVAL);
    gpio_set_level(gPinSda, 0);
    ets_delay_us(CLK_INTERVAL);
    gpio_set_level(gPinScl, 1);
    ets_delay_us(CLK_INTERVAL);
    gpio_set_level(gPinSda, 1);
    ets_delay_us(CLK_INTERVAL);
}

static bool i2cWaitAck(void) {
    gpio_set_level(gPinSda, 1);
    ets_delay_us(CLK_INTERVAL);
    gpio_set_level(gPinScl, 1);
    ets_delay_us(CLK_INTERVAL);
    if (gpio_get_level(gPinSda)) {
        i2cStop();
        return false;
    }
    gpio_set_level(gPinScl, 0);
    return true; 
}

static void i2cSendAck(void) {
    gpio_set_level(gPinSda, 0);
    ets_delay_us(CLK_INTERVAL);
    gpio_set_level(gPinScl, 1);
    ets_delay_us(CLK_INTERVAL);
    gpio_set_level(gPinScl, 0);    
}

static void i2cSendNoAck(void) {
    gpio_set_level(gPinSda, 1);
    ets_delay_us(CLK_INTERVAL);
    gpio_set_level(gPinScl, 1);
    ets_delay_us(CLK_INTERVAL);
    gpio_set_level(gPinScl, 0);    
}

static void i2cSendByte(uint8_t data) {
    uint8_t i = 8;
    while (i--) {
        gpio_set_level(gPinScl, 0);
        ets_delay_us(CLK_INTERVAL);
        if (data & 0x80) {
            gpio_set_level(gPinSda, 1);
        } else {
            gpio_set_level(gPinSda, 0);
        }
        
        ets_delay_us(CLK_INTERVAL);
        data <<= 1;
        gpio_set_level(gPinScl, 1);
        ets_delay_us(CLK_INTERVAL);
        gpio_set_level(gPinScl, 0);
        ets_delay_us(CLK_INTERVAL);
    }    
}

static uint8_t i2cReceiveByte(void) {
    uint8_t i = 8;
    uint8_t data = 0;
    
    gpio_set_level(gPinSda, 1);
    while (i--) {
        data <<= 1;
        gpio_set_level(gPinScl, 0);
        ets_delay_us(CLK_INTERVAL);
        gpio_set_level(gPinScl, 1);
        ets_delay_us(CLK_INTERVAL);
        if (gpio_get_level(gPinSda)) {
            data |= 0x01;
        }
    }
    gpio_set_level(gPinScl, 0);
    
    return data;
}

static bool readBytes(uint8_t* data, int dataLen) {
    i2cStart();
    i2cSendByte(SLAVE_ADDRESS | 0x1);
    if (i2cWaitAck() == false) {
        i2cStop();
        return false;
    }
    
    for (int i = 0; i < dataLen; i++) {
        *data++ = i2cReceiveByte();
        if (i < dataLen - 1) {
            i2cSendAck();
        }
    }
    i2cSendNoAck();
    i2cStop();

    return true;
}

static bool writeBytes(uint8_t addr, uint8_t* data, int dataLen) {
    i2cStart();
    i2cSendByte(SLAVE_ADDRESS);
    if(i2cWaitAck() == false) {
        i2cStop();
        return false;
    }
    i2cSendByte(addr);
    if(i2cWaitAck() == false) {
        i2cStop();
        return false;
    }

    for (int i = 0; i < dataLen; i++) {
        i2cSendByte(*data);
        if(i2cWaitAck() == false) {
            i2cStop();
            return false;
        }
    }
    
    i2cStop();
    return true;
}
