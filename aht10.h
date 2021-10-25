// Copyright 2021-2021 The jdh99 Authors. All rights reserved.
// 温湿度传感器AHT10的驱动
// Authors: jdh99 <jdh821@163.com>

#ifndef AHT10_H
#define AHT10_H

#include <stdint.h>
#include <stdbool.h>

// AhtValue AHT传感器读取的温湿度值
typedef struct {
    // 温度.分度:0.1℃
    int16_t Temperature;
    // 湿度1%RH
    uint8_t Humidity;
} Aht10Value;

// Aht10Load 模块载入
void Aht10Load(int pinSda, int pinScl);

// Aht10Get 读取温湿度
Aht10Value Aht10Get(void);

#endif
