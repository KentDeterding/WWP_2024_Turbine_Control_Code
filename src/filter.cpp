#include <stdlib.h>

struct Filter {
    unsigned int size;
    volatile int oldestMoment;
    int peaks_per_rotation;
    volatile int* array;
};

struct Filter* new_filter(unsigned int size, int peaks_per_rotation) {
    struct Filter* filter = (struct Filter*)malloc(sizeof(struct Filter));
    filter->size = size;
    filter->oldestMoment = 0;
    filter->peaks_per_rotation = peaks_per_rotation;
    filter->array = (int*)malloc(sizeof(int) * size);
    return filter;
}

void insert(struct Filter* filter, int value) {
    int index = filter->oldestMoment;
    *(filter->array + index) = value;
    filter->oldestMoment++;
    if (filter->oldestMoment >= (int)filter->size) {
        filter->oldestMoment = 0;
    }
}

double get_average_val(struct Filter* filter) {
    double sum = 0.0;
    for (int i = 0; i < (int)filter->size - 2; i++) {
        int array_index = filter->oldestMoment + i;
        if (array_index > (int)filter->size - 1) {
            array_index -= (int)filter->size;
        }
        sum += (double) *(filter->array + array_index);
    }
    return sum / (double)(filter->size - 2);
}

// Throws away 2 (oldest) values as a buffer and returns the average of the rest
double get_rpm_buffered(struct Filter* filter) {
    int highIndex = filter->oldestMoment - 1;
    if (highIndex < 0) { highIndex = (int)filter->size - 1; }
    int lowIndex = filter->oldestMoment + 1;
    if (lowIndex >= (int)filter->size) { lowIndex = 0; }
    lowIndex++;
    if (lowIndex >= (int)filter->size) { lowIndex = 0; }
    
    double high = (double) *(filter->array + highIndex);
    double low = (double) *(filter->array + lowIndex);
    
    if (high < low) {
        return -1;
    }
    
    double period = (high - low) / (filter->size - 3);
    period = period / 1000000; // convert us -> s
    double rpm = 60 / period;
    rpm = rpm / (double)filter->peaks_per_rotation;
    if (rpm > 10000.0) {
        return -1;
    } else {
        return rpm;
    }
}