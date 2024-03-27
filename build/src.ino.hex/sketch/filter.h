#line 1 "C:\\Users\\Kent4\\Projects\\Wind_Power\\WWP_2024_Turbine_Control_Code\\src\\filter.h"
#ifndef rpm_filter_h
#define rpm_filter_h

#include <stdlib.h>

struct Filter;

struct Filter* CreateFilter(unsigned int size, int peaks_per_rotation);

void Insert(struct Filter* filter, int value);

double GetAverageVal(struct Filter* filter);

double GetRpmBuffered(struct Filter* filter);

#endif