
#include <rtthread.h>
#include <rtdevice.h>

#include <string.h>
#include <stdlib.h>

#define DBG_TAG "hdc1000"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>


#include "hdc1000_reg.h"

#define HDC1000_DELAY_MS(x) rt_thread_mdelay(x)

/**
 * @brief This function config the value of the register for hdc1000
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for hdc1000
 * @param data value to write
 *
 * @return the writing status, RT_EOK reprensents  writing the value of the register successfully.
 */
static rt_err_t hdc1000_write_reg(struct hdc1000_device *dev,rt_uint8_t reg, rt_uint16_t data)
{
    rt_int8_t res = 0;

    struct rt_i2c_msg msgs;
    rt_uint8_t buf[3] = {0};
    
    buf[ 0 ] = reg;
    // MSB
    buf[ 1 ] = ( data >> 8 ) & 0xFF;
    buf[ 2 ] = data & 0xFF;

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
        msgs.addr  = dev->i2c_addr;    /* slave address */
        msgs.flags = RT_I2C_WR;        /* write flag */
        msgs.buf   = buf;              /* Send data pointer */
        msgs.len   = 3;

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, &msgs, 1) == 1)
        {
            res = RT_EOK;
        }
        else
        {
            res = -RT_ERROR;
        }
    }

    return res;
}

/**
 * @brief This function read the value of register for hdc1000
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for hdc1000
 * @param buf read data pointer
 *
 * @return the reading status, RT_EOK reprensents  reading the value of registers successfully.
 */
static rt_err_t hdc1000_read_reg(struct hdc1000_device *dev, rt_uint8_t reg, rt_uint16_t *data)
{
    rt_int8_t res = 0;

    struct rt_i2c_msg msgs[2];
    
    rt_uint8_t buffer[2] = {0};
  
    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
        msgs[0].addr  = dev->i2c_addr;    /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = &reg;             /* Slave register address */
        msgs[0].len   = 1;                /* Number of bytes sent */

        msgs[1].addr  = dev->i2c_addr;    /* Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = buffer;           /* Read data pointer */
        msgs[1].len   = 2;              /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 2) == 2)
        {
            res = RT_EOK;
            
            *data = (buffer[0] << 8) | buffer[1];
        }
        else
        {
            res = -RT_ERROR;
        }
    }

    return res;
}

/**
 * @brief This function reset the hdc1000 device.
 *
 * @param dev_name the name of transfer device
 * @param param the i2c device address for i2c communication, RT_NULL for spi
 *
 * @return the pointer of device driver structure, RT_NULL reprensents  initialization failed.
 */
static rt_err_t hdc1000_soft_reset(struct hdc1000_device *dev)
{	
	return ( hdc1000_write_reg(dev, HDC1000_CONFIGURATION_REG,HDC1000_SOFT_RESET) );
}

/**
 * @brief  Get the HTS221 temperature value
 *
 * @param  pObj the device pObj
 * @param  Value pointer where the temperature value is written
 *
 * @retval 0 in case of success, an error code otherwise
 */
static rt_err_t hdc1000_setup_mode(struct hdc1000_device *dev,uint16_t mode)
{	
	return ( hdc1000_write_reg(dev,HDC1000_CONFIGURATION_REG, mode) );
}

/**
 * @brief  Get the HDC1000 humidity value
 *
 * @param  dev the pointer of device driver structure
 * @param  raw pointer where the temperature and humidity value is written
 *
 * @retval RT_EOK in case of success, an error code otherwise
 */
rt_err_t hdc1000_get_temperature_and_humidity(struct hdc1000_device *dev,uint16_t *temp_raw,uint16_t *humi_raw)
{
    rt_int8_t res = 0;
    
    struct rt_i2c_msg msgs[2];
    
    rt_uint8_t buffer[4] = { HDC1000_TEMPERATURE_REG };// 0x00 or HDC1000_HUMIDITY_REG(01)
    
    /* Trigger a measurement (executing a pointer write transaction with the address point 0x00 or 0x01) */
    {
        msgs[0].addr  = dev->i2c_addr;             /* Slave address */
        msgs[0].flags = RT_I2C_WR;                 /* Write flag */
        msgs[0].buf   = buffer;                    /* Slave register address,buffer[0] is the address point*/
        msgs[0].len   = 1;                         /* Number of bytes sent */
        
        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 1) != 1)
        {
            res = -RT_ERROR;
        }
    }
    /* wait for the measurements to complete or wait for the assertion of DRDYn 
        - Temperture sensor conversion time ~6.35ms @ 14 bit resolution
        - Humidity sensor conversion time ~6.5ms @ 14 bit resolution
     */
    HDC1000_DELAY_MS(15);
    
    /* read the temperture and humidity raw value */
    {
        //msgs[0].addr  = dev->i2c_addr;                     /* Slave address */
        msgs[0].flags = RT_I2C_RD;                           /* Read flag */
        msgs[0].buf   = buffer;                              /* Read data pointer */
        msgs[0].len   = 4;                                   /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 1) == 1)
        {
            res = RT_EOK;
            
            *temp_raw = (buffer[0] << 8) | buffer[1];
            *humi_raw = (buffer[2] << 8) | buffer[3];
        }
        else
        {
            res = -RT_ERROR;
        }
    }
    return res;
}

/**
 * @brief  read the HDC1000 configuration register
 *
 * @param  dev the pointer of device driver structure
 * @param  raw pointer where the configuration register is written
 *
 * @retval RT_EOK in case of success, an error code otherwise
 */
rt_err_t hdc1000_read_configuration(struct hdc1000_device *dev,rt_uint16_t *config)
{
    return ( hdc1000_read_reg(dev,HDC1000_CONFIGURATION_REG, config) );
}

/**
 * @brief  Get the HDC1000 manufacturer id
 *
 * @param  dev the pointer of device driver structure
 * @param  raw pointer where the manufacturer id is written
 *
 * @retval RT_EOK in case of success, an error code otherwise
 */
rt_err_t hdc1000_read_manufacturer_id(struct hdc1000_device *dev,rt_uint16_t *manufacturer_id)
{
	return( hdc1000_read_reg(dev, HDC1000_MANUFACTURER_ID_REG, manufacturer_id) );	
}

/**
 * @brief  Get the HDC1000 device id
 *
 * @param  dev the pointer of device driver structure
 * @param  raw pointer where the device id is written
 *
 * @retval RT_EOK in case of success, an error code otherwise
 */
rt_err_t hdc1000_read_device_id(struct hdc1000_device *dev,rt_uint16_t *dev_id)
{
    return ( hdc1000_read_reg(dev, HDC1000_DEVICE_ID_REG, dev_id) );
}

/**
 * @brief This function initialize the hdc1000 device.
 *
 * @param dev_name the name of transfer device
 * @param i2c_addr the i2c device address for i2c communication
 * @param config hdc1000 configuaration
 *
 * @return the pointer of device driver structure, RT_NULL reprensents  initialization failed.
 */
struct hdc1000_device *hdc1000_init(const char *dev_name, rt_uint8_t i2c_addr)
{
    struct hdc1000_device *dev = RT_NULL;
    rt_uint16_t device_id = 0xFFFF, res = RT_EOK;

    RT_ASSERT(dev_name);

    dev = rt_calloc(1, sizeof(struct hdc1000_device));
    if (dev == RT_NULL)
    {
        LOG_E("Can't allocate memory for hdc1000 device on '%s' ", dev_name);
        goto __exit;
    }

    dev->bus = rt_device_find(dev_name);
    if (dev->bus == RT_NULL)
    {
        LOG_E("Can't find device:'%s'", dev_name);
        goto __exit;
    }

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
        if (i2c_addr != RT_NULL)
        {
            dev->i2c_addr = i2c_addr;
        }
        else                
        {
            /* find hdc1000 device at address */
            dev->i2c_addr = HDC1000_DEVICE_IIC_BUS_ADDRESS;
            if (hdc1000_read_device_id(dev, &device_id) != RT_EOK)
            {
                LOG_E("Can't find device at '%s'!", dev_name);
                LOG_E("Failed to read device id!");
                goto __exit;
            }
            LOG_D("Device i2c address is:'0x%x'!", dev->i2c_addr);
        }
    }
    
    else
    {
        LOG_E("Unsupported device:'%s'!", dev_name);
        goto __exit;
    }
 
    res += hdc1000_soft_reset(dev);	
	HDC1000_DELAY_MS(15);	//15ms
    res += hdc1000_setup_mode(dev,HDC1000_ACQUISITION_MODE_IN_SEQUENCE);
    
    res += hdc1000_read_configuration(dev, &dev->config.configuration);
    
    if (res == RT_EOK)
    {
        LOG_I("Device init succeed!");
    }
    else
    {
        LOG_W("Error in device initialization!");
        goto __exit;
    }
    return dev;

__exit:
    if (dev != RT_NULL)
    {
        rt_free(dev);
    }
    return RT_NULL;
}


/**
 * This function releases memory
 *
 * @param dev the pointer of device driver structure
 */
void hdc1000_deinit(struct hdc1000_device *dev)
{
    RT_ASSERT(dev);

    rt_free(dev);
}

//----------------------------------------------------------------------------------


