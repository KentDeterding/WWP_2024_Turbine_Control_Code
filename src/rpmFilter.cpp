#include <Arduino.h>
#include <stdlib.h>

struct RpmFilter {
    volatile uint32_t *array;
    unsigned int size;
    unsigned int index;
    unsigned char poles;
    uint32_t last_time;
    char lock;
};

struct RpmFilter* new_rpm_filter(unsigned int size, unsigned char poles) {
    struct RpmFilter *filter = (RpmFilter*)malloc(sizeof(RpmFilter));
    filter->size = size;
    filter->array = (uint32_t*)malloc(sizeof(uint32_t) * size);
    filter->index = 0;
    filter->poles = poles;
    filter->lock = 0;
    filter->last_time = 0;
    return filter;
}

void rpm_filter_insert(RpmFilter *filter) {
    unsigned int time = micros();
    uint32_t period = time - filter->last_time;
    filter->last_time = time;
    filter->array[filter->index] = period;

    filter->index++;
    if (filter->index >= filter->size)
        filter->index = 0;

    filter->lock++;
}

float rpm_filter_get(RpmFilter *filter) {
    char lock = filter->lock;

    uint32_t sum = 0;

    for (int i = 0; i < filter->size; i++) {
        sum += filter->array[i];
    }

    float period = ((float)sum) / (float)(filter->size);
    period = period / 1000000.0; // convert us -> s
    float freq = 1 / period;
    float rpm = freq * 120 / filter->poles;

    if (rpm > 10000.0 || micros() - filter->last_time > 50000)
        rpm = -1.0;

    //if (lock != filter->lock)
        //return -1;

    return rpm;
}