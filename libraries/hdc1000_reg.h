/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author          Notes
 * 2019-11-30     Forest-Rain     the first version
 */

#ifndef __HDC1000_SENSOR_H_
#define __HDC1000_SENSOR_H_

#include <rtthread.h>

#define BASE_NUM	(0x10000)

/*  temperature and humidity, conver factor */
#define HDC1000_TEMP_CONST_14BIT          0.0025177                                     //For 14bit resolution
#define HDC1000_HUMI_CONST_14BIT          0.0015259                                       //For 14bit resolution
//#define BASE_NUM	(0x10000)

/*  temperature and humidity, HDC1000 gpio */	
#define HDC1000_DYDRn_PIN								GPIO_PIN_9
#define HDC1000_DYDRn_GPIO_PORT							GPIOC
#define HDC1000_DYDRn_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOC_CLK_ENABLE()
#define HDC1000_DYDRn_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOC_CLK_DISABLE()

/* according to hdc1000 spec and specific hardware */
// ADR0 = 0,ADR1 = 0 £¬ 0x40 (7-bit address) for soft i2c driver 
// 0x40 for soft i2c driver
// (0x40 << 1) for stm32 hardware i2c 
#define HDC1000_DEVICE_IIC_BUS_ADDRESS      0x40 

/* register address */
#define HDC1000_TEMPERATURE_REG         0x00
#define HDC1000_HUMIDITY_REG            0x01
#define HDC1000_CONFIGURATION_REG       0x02
#define HDC1000_MANUFACTURER_ID_REG     0xFE	//reset value : 0x5449 (ID of Texas Instruments)
#define HDC1000_DEVICE_ID_REG           0xFF	//reset value : 0x1000 (ID of HDC1000 device)

/* 
** 0x02 configuration register
** BIT[15]:		RST
** BIT[13]:		Enable Heater		
** BIT[12]:		Select acquisition mode
** BIT[11]:		Battery status(read only)
** BIT[10]:		Temperature resolution
** BIT[9:8]:	Humidity resolution		
*/
#define HDC1000_SOFT_RESET                      (0x1 << 15)	// soft reset
#define HDC1000_ENHT	                        (0x1 << 13)	// enable heater
#define HDC1000_ACQUISITION_MODE_IN_SEQUENCE	(0x1 << 12)	// 1 = Temperature and Humidity are acquired in sequence
#define HDC1000_ACQUISITION_MODE_SEPARATELY 	(0x0 << 12)	// 0 = Temperature and Humidity are acquired separately	
#define HDC1000_TRES	                        (0x0 << 10)	// 0 - 14 bit resolution, 1 - 11 bit resolution
#define HDC1000_RHRES	                        (0x0 << 8)	// 0 - 14 bit resolution, 1 - 11 bit resolution, 2 - 7 bit resolution


/* hdc1000 config structure */
struct hdc1000_config
{
    rt_uint16_t configuration;
};

/* hdc1000 device structure */
struct hdc1000_device
{
    rt_device_t bus;
    rt_uint8_t id;
    rt_uint8_t i2c_addr;
    
    struct hdc1000_config config;
};

/* function */

rt_err_t hdc1000_get_temperature_and_humidity(struct hdc1000_device *dev,uint16_t *temp_raw,uint16_t *humi_raw);
rt_err_t hdc1000_read_configuration(struct hdc1000_device *dev,rt_uint16_t *config);
rt_err_t hdc1000_read_manufacturer_id(struct hdc1000_device *dev,rt_uint16_t *manufacturer_id);
rt_err_t hdc1000_read_device_id(struct hdc1000_device *dev,rt_uint16_t *dev_id);	
struct hdc1000_device *hdc1000_init(const char *dev_name, rt_uint8_t i2c_addr);
void hdc1000_deinit(struct hdc1000_device *dev);
#endif
