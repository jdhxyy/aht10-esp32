# aht10-esp32

## 1. 介绍
esp32的aht10驱动。

## 2. API
```c
// Aht10Load 模块载入
void Aht10Load(int pinSda, int pinScl);

// Aht10Get 读取温湿度
Aht10Value Aht10Get(void);
```
