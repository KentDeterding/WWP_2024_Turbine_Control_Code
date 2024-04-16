#include <Arduino.h>
#include <stdlib.h>

struct RpmFilter {
    volatile uint32_t *array;
    unsigned int size;
    unsigned int last;
    unsigned char poles;
    char lock;
};

struct RpmFilter* new_rpm_filter(unsigned int size, unsigned char poles) {
    struct RpmFilter *filter;
    filter->size = size;
    filter->array = (uint32_t*)malloc(sizeof(uint32_t) * size);
    filter->last = 0;
    filter->poles = poles;
    filter->lock = 0;
    return filter;
}

void rpm_filter_insert(RpmFilter *filter) {
    unsigned int time = micros();

    filter->last++;
    if (filter->last >= filter->size)
        filter->last = 0;
    filter->array[filter->last] = time;

    filter->lock++;
}

float rpm_filter_get(RpmFilter *filter) {
    char lock = filter->lock;

    unsigned int first_idx;

    if (filter->last >= filter->size - 1) {
        first_idx = 0;
    } else {
        first_idx = filter->last + 1;
    }

    uint32_t period = (filter->array[filter->last] - filter->array[first_idx]) / (filter->size - 1);
    period = period / 100000; // convert us -> s
    float rpm = 60 / period / (filter->poles / 2);

    if (rpm > 10000)
        rpm = 0.0;

    if (lock != filter->lock)
        return -1;

    return rpm;
}