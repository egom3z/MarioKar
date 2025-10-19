#ifndef IMU_FILTERS_H
#define IMU_FILTERS_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// FIR FILTER
// ============================================================================

#define FIR_FILTER_TAPS 5  // Number of coefficients

typedef struct {
    float coefficients[FIR_FILTER_TAPS];
    float history[FIR_FILTER_TAPS];
    uint8_t index;
} fir_filter_t;

/**
 * @brief Initialize FIR filter with custom coefficients
 * @param coefficients Array of coefficients (should sum to 1.0 for unity gain)
 */
void fir_filter_init(fir_filter_t *filter, const float *coefficients);

/**
 * @brief Apply FIR filter to a new sample
 * @return Filtered output
 */
float fir_filter_update(fir_filter_t *filter, float input);

#endif // IMU_FILTERS_H