#include "kalman_1d.h"

void kalmanUpdate(kalman_1d_inst* k, float z_k)
{
    // Prediction Step ---------------------------------------
    // I. Predicting State and Error Covariance 
    float xh_k_m = k->A * k->xh_k;
    float Ph_k_m = k->A * k->Ph_k * k->A + k->Q;

    // Update Step -------------------------------------------
    // II. Compute Kalman Gain
    float K_k = Ph_k_m * k->H * 1/(k->H * Ph_k_m * k->H + k->R);
    
    // III. Compute the Estimate
    k->xh_k = xh_k_m + K_k * (z_k - k->H * xh_k_m);

    // IV. Compute the Error Covariance
    k->Ph_k = Ph_k_m - K_k * k->H * Ph_k_m;
}