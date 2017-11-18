//
// Created by MightyPork on 2017/11/17.
//

#ifndef PROJ_VOC_SENSOR_H
#define PROJ_VOC_SENSOR_H

#include "../Drivers/BME680/bme680.h"

extern struct bme680_dev gas_sensor;

void voc_init(void);
void voc_measure(void);

#endif //PROJ_VOC_SENSOR_H
