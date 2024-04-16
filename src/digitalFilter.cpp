#include <stdlib.h>

struct DigitalFilter {
    float* array;
    unsigned int size;
    unsigned int last;
};

struct DigitalFilter* new_digital_filter(unsigned int size) {
    struct DigitalFilter* filter = (struct DigitalFilter*)malloc(sizeof(struct DigitalFilter));
    filter->size = size;
    filter->array = (float*)malloc(sizeof(float) * size);
    filter->last = 0;
    return filter;
}

void digital_filter_insert(struct DigitalFilter *filter, float value) {
    filter->last++;
    if (filter->last >= filter->size) {
        filter->last = 0;
    }
    filter->array[filter->last] = value;
}

float digital_filter_get_avg(struct DigitalFilter *filter) {
    float sum = 0;
    for (int i = 0; i < filter->size; i++) {
        sum += filter->array[i];
    }
    return sum / filter->size;
}