#ifndef REGULATOR_H
#define REGULATOR_H

extern unsigned long HAL_GetTick();

typedef struct {
    double Ki;
    double* input;
    double* output;
    double reference;
    double error;

    double min;
    double max;

    unsigned long prev_time;
} SimpleRegulatorStruct;

void simple_regulator_update(SimpleRegulatorStruct* regulator);

#endif