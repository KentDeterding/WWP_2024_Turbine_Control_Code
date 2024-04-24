#ifndef rpm_filter_h
#define rpm_filter_h

#include <Arduino.h>
#include <stdlib.h>

struct RpmFilter;

struct RpmFilter* new_rpm_filter(unsigned int size, unsigned char poles);

void rpm_filter_insert(struct RpmFilter *filter);

float rpm_filter_get(struct RpmFilter *filter);

#endif