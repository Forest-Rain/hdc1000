
// Application demo
#include "sensor_ti_hdc1000.h"
#define DBG_TAG  "hdc1000.sensor"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

int rt_hw_hdc1000_port(void)
{
    struct rt_sensor_config cfg;
    rt_int8_t result;
    cfg.intf.dev_name = "i2c1";
#if defined(RTTHREAD_VERSION) && (RTTHREAD_VERSION >= RT_VERSION_CHECK(5, 0, 0))
    cfg.intf.arg = (void *)HDC1000_SLAVE_ADDR_DEFAULT;
#else
    cfg.intf.user_data = (void *)HDC1000_SLAVE_ADDR_DEFAULT;
#endif
    cfg.irq_pin.pin = RT_PIN_NONE;

    result = rt_hw_hdc1000_init("hdc1000", &cfg);
    return result;
}
INIT_APP_EXPORT(rt_hw_hdc1000_port);


void application_get_sensor_val(void)
{
    struct rt_sensor_data sensor_data;
    rt_size_t res;
    rt_device_t dev = RT_NULL;

    dev = rt_device_find("temp_hdc1000");
    if (rt_device_open(dev, RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
      LOG_E("open device failed!");
      return;
    }
    res = rt_device_read(dev, 0, &sensor_data, 1);
    if (res != 1)
    {
      LOG_E("read data failed!size is %d", res);
    }
    else
    {
      LOG_I("temp:%3d.%dC, timestamp:%5d", sensor_data.data.temp / 10, sensor_data.data.temp % 10, sensor_data.timestamp);  
      LOG_I("humi:%3d.%dC, timestamp:%5d", sensor_data.data.humi / 10, sensor_data.data.humi % 10, sensor_data.timestamp);
    }
    rt_device_close(dev);
}