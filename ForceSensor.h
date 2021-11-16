#ifndef FORCE_SENSOR_H
#define FORCE_SENSOR_H

#include "HAL/ADC_TIVA.h"

struct ForceSensor
{
    uint32_t ADCBase;
    bool enabled;

    float upperLimitNewtons;
    float lowerLimitNewtons;

    float slope;
    float offset;

    uint32_t raw;
    float newtons;
};
typedef struct ForceSensor ForceSensor;

ForceSensor forceSensorConstruct(uint32_t ADCBase);
void forceSensorEnable(ForceSensor* forceSensor);
// TODO: Add a force sensor disable function
void readLoadCell(ForceSensor* forceSensor);

#endif /* FORCE_SENSOR_H */
