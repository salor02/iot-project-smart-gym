#pragma once

// moving avarage core struct
typedef struct {
    float *buf;   // data buffer
    int n;        // maximum number of single raw data
    int idx;      // current index
    int count;    // counter for the real (raw) data received, this allows to calculate the moving avarage in the first iterations
    float sum;    // current sum
} ma_filter_t;

void ma_init(ma_filter_t *f, float *buf, int n);
float ma_update(ma_filter_t *f, float x);

