
// Ó¦ÓÃÊ¾Àý
#include "sensor_ti_hdc1000.h"

int rt_hw_hdc1000_port(void)
{
    struct rt_sensor_config cfg;
    rt_int8_t result;
    cfg.intf.dev_name = "i2c1";
    cfg.intf.user_data = (void *)HDC1000_SLAVE_ADDR_DEFAULT;
    cfg.irq_pin.pin = RT_PIN_NONE;

    result = rt_hw_hdc1000_init("hdc1000", &cfg);
    return result;
}
INIT_APP_EXPORT(rt_hw_hdc1000_port);

