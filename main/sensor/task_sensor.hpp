#ifndef TASK_SENSOR_HPP_INCLUDED
#define TASK_SENSOR_HPP_INCLUDED

#include "sensor_task_interface.hpp"

#include "driver/i2c_master.h"

i2c_master_bus_handle_t initMasterI2C();
i2c_master_dev_handle_t initTempPressureI2CSlave(const i2c_master_bus_handle_t busHandle);
i2c_master_dev_handle_t initIlluminanceI2CSlave(const i2c_master_bus_handle_t busHandle);

void task_sensor(void *pvParameters);

#endif