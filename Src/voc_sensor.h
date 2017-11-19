//
// Created by MightyPork on 2017/11/17.
//

#ifndef PROJ_VOC_SENSOR_H
#define PROJ_VOC_SENSOR_H

#include "../Drivers/BME680/bme680.h"

extern struct bme680_dev gas_sensor;

void voc_init(void);
uint32_t voc_start_measure(void);
void voc_read(struct bme680_field_data *data);

#endif //PROJ_VOC_SENSOR_H
