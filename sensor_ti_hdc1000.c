
#include "sensor_ti_hdc1000.h"
#include "string.h"

#define DBG_ENABLE
#define DBG_LEVEL DBG_LOG
#define DBG_SECTION_NAME  "sensor.ti.hdc1000"
#define DBG_COLOR
#include <rtdbg.h>

static int hdc1000_selftest(void);

/***********  Common  *****************/
static struct hdc1000_device *hdc1000_dev;

static rt_err_t _hdc1000_init(struct rt_sensor_intf *intf)
{
    rt_uint8_t  i2c_addr = (rt_uint32_t)(intf->user_data) & 0xff;
    
    hdc1000_dev = hdc1000_init(intf->dev_name, i2c_addr);
   
    if (hdc1000_dev == RT_NULL)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_size_t hdc1000_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    struct rt_sensor_data *data = (struct rt_sensor_data *)buf;
    rt_uint16_t temp_value, humi_value;
    
    hdc1000_get_temperature_and_humidity(hdc1000_dev, &temp_value, &humi_value);
    
    if (sensor->module)
    {
       // RT_SENSOR_CLASS_NONE
    }
        
    if (sensor->info.type == RT_SENSOR_CLASS_TEMP)
    {
        data->type = RT_SENSOR_CLASS_TEMP;
        data->data.temp = (( float )( temp_value * HDC1000_TEMP_CONST_14BIT ) - 40 ) * 10; // x10
        data->timestamp = rt_sensor_get_ts();
    }
    else if (sensor->info.type == RT_SENSOR_CLASS_HUMI)
    {
        data->type = RT_SENSOR_CLASS_HUMI;
        data->data.humi = (( float )humi_value * HDC1000_HUMI_CONST_14BIT ) * 10;  // x10
        data->timestamp = rt_sensor_get_ts();
    }

    return 1;
}

static rt_err_t hdc1000_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        hdc1000_read_device_id(hdc1000_dev, args);
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
        return -RT_ERROR;
    case RT_SENSOR_CTRL_SELF_TEST:
        hdc1000_selftest();
        break;
    default:
        return -RT_ERROR;
    }
    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    hdc1000_fetch_data,
    hdc1000_control
};

int rt_hw_hdc1000_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_temp = RT_NULL, sensor_humi = RT_NULL;

    /* Termperture sensor register */
    sensor_temp = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor_temp == RT_NULL)
        return -1;

    sensor_temp->info.type       = RT_SENSOR_CLASS_TEMP;
    sensor_temp->info.vendor     = RT_SENSOR_VENDOR_STM;
    sensor_temp->info.model      = "hdc1000_temp";
    sensor_temp->info.unit       = RT_SENSOR_UNIT_DCELSIUS;
    sensor_temp->info.intf_type  = RT_SENSOR_INTF_I2C;
    sensor_temp->info.range_max  = 125;
    sensor_temp->info.range_min  = -40;
    sensor_temp->info.period_min = 80;

    rt_memcpy(&sensor_temp->config, cfg, sizeof(struct rt_sensor_config));
    sensor_temp->ops = &sensor_ops;
    
    result = rt_hw_sensor_register(sensor_temp, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
    if (result != RT_EOK)
    {
        LOG_E("temp sensor register err code: %d", result);
        goto __exit;
    }
    
    LOG_I("temp sensor init success");
    
    /* humidity sensor register */
    sensor_humi = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor_humi == RT_NULL)
        return -1;

    sensor_humi->info.type       = RT_SENSOR_CLASS_HUMI;
    sensor_humi->info.vendor     = RT_SENSOR_VENDOR_STM;
    sensor_humi->info.model      = "hdc1000_humi";
    sensor_humi->info.unit       = RT_SENSOR_UNIT_PERMILLAGE;
    sensor_humi->info.intf_type  = RT_SENSOR_INTF_I2C;
    sensor_humi->info.range_max  = 100;
    sensor_humi->info.range_min  = 0;
    sensor_humi->info.period_min = 80;

    rt_memcpy(&sensor_humi->config, cfg, sizeof(struct rt_sensor_config));
    sensor_humi->ops = &sensor_ops;
    
    result = rt_hw_sensor_register(sensor_humi, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
    if (result != RT_EOK)
    {
        LOG_E("humi sensor register err code: %d", result);
        goto __exit;
    }

    LOG_I("humi sensor init success");

    /* hdc1000 init */
    result = _hdc1000_init(&cfg->intf);
    if (result != RT_EOK)
    {
        LOG_E("hdc1000 init err code: %d", result);
        goto __exit;
    }
    LOG_I("hdc1000 init success");
    
    return result;
    
__exit:
    if (sensor_temp)
        rt_free(sensor_temp);
    if (sensor_humi)
        rt_free(sensor_humi);
    if (hdc1000_dev)
        hdc1000_deinit(hdc1000_dev);    
    
    return -RT_ERROR;   
}


/* SelfTest function */
static int hdc1000_selftest(void)
{
    struct hdc1000_device *dev;
    rt_uint16_t device_id,manufacturer_id,temp_value,humi_value;
    int i;

    /* Initialize hdc1000, The parameter is RT_NULL, means auto probing for i2c*/
    dev = hdc1000_init("i2c1",RT_NULL);

    if (dev == RT_NULL)
    {
        rt_kprintf("hdc1000 init failed\n");
        return -1;
    }
    rt_kprintf("hdc1000 init succeed\n");

    hdc1000_read_device_id(hdc1000_dev, &device_id);
    hdc1000_read_manufacturer_id(hdc1000_dev, &manufacturer_id);

    for (i = 0; i < 5; i++)
    {
        hdc1000_get_temperature_and_humidity(hdc1000_dev, &temp_value, &humi_value);

        temp_value = (int16_t)(((float)temp_value/65536*165-40));   // ¡æ
        humi_value = (int16_t)(((float)humi_value/65536)*100);      // %RH

        rt_kprintf("device_id = %X, manufacturer_id = %X, temp = %d C, humi = %d%RH\r\n", device_id,manufacturer_id,temp_value,humi_value);
        
        rt_thread_mdelay(100);
    }

    hdc1000_deinit(dev);

    return 0;
}
MSH_CMD_EXPORT(hdc1000_selftest, hdc1000 sensor sleftest function);
