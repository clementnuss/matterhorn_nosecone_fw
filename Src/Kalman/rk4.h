

#ifndef KALMAN_EMILIEN_EKF_RK4_H
#define KALMAN_EMILIEN_EKF_RK4_H

#include "EKF_Config.h"

double rk4(double t0, double u0, double dt, double f(double t, double u));

double *rk4vec(double t0, int n, double u0[], double dt, ekf_t *ekf, double *(*f)(double, int, double *, ekf_t *));

void timestamp(void);

#endif