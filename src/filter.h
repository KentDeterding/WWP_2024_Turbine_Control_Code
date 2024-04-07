#ifndef rpm_filter_h
#define rpm_filter_h

#include <stdlib.h>

struct Filter;

struct Filter* new_filter(unsigned int size, int peaks_per_rotation);

void insert(struct Filter* filter, int value);

double get_average_val(struct Filter* filter);

double get_rpm_buffered(struct Filter* filter);

#endif