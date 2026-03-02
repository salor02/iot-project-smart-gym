#include "moving_avarage.h"

// initialization of the moving avarage
void ma_init(ma_filter_t *f, float *buf, int n){
    f->buf = buf; 
    f->n = n; 
    f->idx = 0; 
    f->count = 0; 
    f->sum = 0;

    // the buffer is initialized to zero
    for (int i = 0; i < n; i++){
        f->buf[i] = 0;
    }
}

// filter update
float ma_update(ma_filter_t *f, float new_data){
    // this branch is useful in the first n iterations
    if (f->count < f->n){
        f->buf[f->idx] = new_data;
        f->sum += new_data;
        f->count++;
    }
    // after the first n iterations, the older value is detracted from the sum and replace with a new one
    else{
        f->sum -= f->buf[f->idx];
        f->buf[f->idx] = new_data;
        f->sum += new_data;
    }
    f->idx = (f->idx + 1) % f->n;

    // avarage calculation
    return f->sum / f->count;
}
