#ifndef digital_filter_h
#define digital_filter_h

#include <stdlib.h>

struct DigitalFilter;

struct DigitalFilter* new_digital_filter(unsigned int size);

void digital_filter_insert(struct DigitalFilter *filter, float value);

float digital_filter_get_avg(struct DigitalFilter *filter);

#endif