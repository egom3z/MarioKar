#include "imu_filters.h"
#include <string.h>

void fir_filter_init(fir_filter_t *filter, const float *coefficients) {
    memcpy(filter->coefficients, coefficients, sizeof(float) * FIR_FILTER_TAPS);
    memset(filter->history, 0, sizeof(float) * FIR_FILTER_TAPS);
    filter->index = 0;
}

float fir_filter_update(fir_filter_t *filter, float input) {
    // Add new sample to circular buffer
    filter->history[filter->index] = input;
    filter->index = (filter->index + 1) % FIR_FILTER_TAPS;
    
    // Calculate filtered output
    float output = 0.0f;
    for (int i = 0; i < FIR_FILTER_TAPS; i++) {
        int idx = (filter->index + i) % FIR_FILTER_TAPS;
        output += filter->coefficients[i] * filter->history[idx];
    }
    
    return output;
}