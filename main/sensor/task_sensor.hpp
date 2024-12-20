#ifndef TASK_SENSOR_HPP_INCLUDED
#define TASK_SENSOR_HPP_INCLUDED

#include "sensor_task_interface.hpp"

void initI2C();
void task_sensor(void *pvParameters);

#endif