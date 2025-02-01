/*
 * kalman_1d.h
 *
 *  Created on: Feb 1, 2025
 *      Author: Jason
 */

#ifndef INC_KALMAN_1D_H_
#define INC_KALMAN_1D_H_

#include "main.h"

typedef struct{
    float A;               /* State Transition Matrix (1x1) */
    float Q;               /* Covariance Matrix of State Transition Noise (1x1) */
    float H;               /* State to Measurement Matrix (1x1) */
    float R;               /* Covariance Matrix of Measurement Noise (1x1) */
    float xh_k;            /* Estimate of State Vector (1x1) */
    float Ph_k;            /* Estimate of State Covariance Matrix (1x1) */
}kalman_1d_inst;

void kalmanUpdate(kalman_1d_inst* k, float z_k);

#endif