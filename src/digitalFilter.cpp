#include <stdlib.h>

struct DigitalFilter {
    unsigned int size;
    float* array;
};

struct DigitalFilter* new_digital_filter (unsigned int size) {
    struct DigitalFilter* filter = (struct DigitalFilter*)malloc(sizeof(struct DigitalFilter));
    filter->size = size;


    return filter;
}