# HDC1000 package for RT-Thread

## 简介

本软件包是基于RT-Thread实现的TI HDC1000 温湿度传感器提供的通用传感器驱动包。通过使用此软件包，开发者可以快速的利用 RT-Thread 将此传感器驱动起来。

本篇文档主要内容如下：

- 传感器介绍
- 支持情况
- 使用说明

## 传感器介绍

HDC1000是德州仪器（TI）生产的一款集成式湿度和温度传感器，其能够以超低功耗提供出色的测量精度。

### 测量范围与精度
| 传感器         | 测量范围 | 测量精度 |
| ------------- | ------- | ------ |
| **温度**      |   -40°C 至 125°C     |   ±0.2°C     |
| **湿度**      |  0% 至 100%     |  ±3%      |

### 功耗
 - 平均电源电流（每秒测量 1 次 ）: 1.2µA @ RH（11 位）+ 温度（11 位）

## 支持情况

| 包含设备         | 温度计 | 湿度计 |
| ---------------- | ------ | ------ |
| **通讯接口**     |        |        |
| IIC              | √      | √      |
| SPI              |        |        |
| **工作模式**     |        |        |
| 轮询             | √      | √      |
| 中断             |        |        |
| FIFO             |        |        |
| **电源模式**     |        |        |
| 掉电             |       |        |
| 低功耗           | √      | √      |
| 普通             |        |        |
| **数据输出速率** |        |        |
| **测量范围**     |        |        |
| **自检**         |        |        |
| **多实例**       |        |        |

## 使用说明

### 依赖

- RT-Thread 4.0.0+
- Sensor 组件
- IIC 驱动：HDC1000 设备使用 IIC 进行数据通讯，需要RT-Thread系统 IIC 驱动框架支持；

### 获取软件包

使用 HDC1000 软件包需要在 RT-Thread 的包管理中选中它，具体路径如下：

```
RT-Thread online packages --->
    peripheral libraries and drivers --->
        sensors drivers --->
            [*] HDC1000: HDC1000 sensor driver package --->
                    Version (latest)  --->
```
**HDC1000: HDC1000 sensor driver package**：使能湿度计&温度计

**Version**：软件包版本选择

### 使用软件包

HDC1000 软件包初始化函数如下所示：

```
int rt_hw_hdc1000_init(const char *name, struct rt_sensor_config *cfg);
```

该函数需要由用户调用，函数主要完成的功能有，

- 设备配置和初始化（根据传入的配置信息，配置接口设备和中断引脚）；
- 注册相应的传感器设备，完成 HDC1000 设备的注册；

#### 初始化示例

```
#include "sensor_ti_hdc1000.h"


```
int rt_hw_hdc1000_port(void)
{
    struct rt_sensor_config cfg;
    rt_int8_t result;
    cfg.intf.dev_name = "i2c1";
    cfg.intf.user_data = (void *)HDC1000_ADDR_DEFAULT;
    cfg.irq_pin.pin = RT_PIN_NONE;
    result = rt_hw_hdc1000_init("hdc1000", &cfg);
    return result;
}
INIT_APP_EXPORT(rt_hw_hdc1000_port);
```

## 注意事项

暂无

## 联系人信息

维护人:

- [Forest-Rain](https://github.com/Forest-Rain) 
