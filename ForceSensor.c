#include "ForceSensor.h"

ForceSensor forceSensorConstruct(uint32_t ADCBase)
{
    ForceSensor forceSensor;

    forceSensor.ADCBase = ADCBase;
    forceSensor.enabled = true;

    forceSensor.upperLimitNewtons = 1800;
    forceSensor.lowerLimitNewtons = -1800;

    forceSensor.slope = 0.0;
    forceSensor.offset = 0.0;

    forceSensor.raw = 0;
    forceSensor.newtons = 0;

    return forceSensor;
}

void forceSensorEnable(ForceSensor* forceSensor)
{
    if(forceSensor->ADCBase == ADC0_BASE)
        ADCConfig0();
    else if(forceSensor->ADCBase == ADC1_BASE)
        ADCConfig1();
    else
        return;

    forceSensor->enabled = true;
}

void readLoadCell(ForceSensor* forceSensor)
{
    ADCProcessorTrigger(forceSensor->ADCBase, 0);                       // Causes a processor trigger for a sample sequence, trigger the sample sequence.
    while(!ADCIntStatusEx(forceSensor->ADCBase, false)){}               // Wait until the sample sequence has completed.
    ADCIntClear(forceSensor->ADCBase, 0);                               // Clear the ADC interrupt flag generated upon ADC completion.
    ADCSoftwareOversampleDataGet(forceSensor->ADCBase, 0, &forceSensor->raw, 1);   // Obtain single software averaged ADC value.
}
