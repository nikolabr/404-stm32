#include "simple_regulator.h"

void simple_regulator_update(SimpleRegulatorStruct* regulator)
{
    unsigned long sampling_time = HAL_GetTick() - regulator->prev_time;

    regulator->error += ( regulator->reference - *(regulator->input) ) * (regulator->Ki) * sampling_time;

    if (regulator->error < regulator->min)
    {
        regulator->error = regulator->min;
    }

    if (regulator->error > regulator->max)
    {
        regulator->error = regulator->max;
    }

    *(regulator->output) = regulator->error;

    regulator->prev_time = HAL_GetTick();
}