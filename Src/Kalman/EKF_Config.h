/*
 * TinyEKF: Extended Kalman Filter for embedded processors.
 *
 * tinyekf_config.h: static configuration parameters
 *
 * Copyright (C) 2015 Simon D. Levy
 *
 * MIT License
 */

#ifndef KALMAN_EMILIEN_EKF_CONFIG_ERT_H
#define KALMAN_EMILIEN_EKF_CONFIG_ERT_H

/* states */
#define Nsta 2

/* observables */
#define Mobs 1

typedef struct {

    int n;          /* number of state values */
    int m;          /* number of observables */

    double x[Nsta];    /* state vector */

    double P[Nsta][Nsta];  /* prediction error covariance */
    double Q[Nsta][Nsta];  /* process noise covariance */
    double R[Mobs][Mobs];  /* measurement error covariance */

    double G[Nsta][Mobs];  /* Kalman gain; a.k.a. K */

    double F[Nsta][Nsta];  /* Jacobian of process model */
    double H[Mobs][Nsta];  /* Jacobian of measurement model */

    double Ht[Nsta][Mobs]; /* transpose of measurement Jacobian */
    double Ft[Nsta][Nsta]; /* transpose of process Jacobian */
    double Pp[Nsta][Nsta]; /* P, post-prediction, pre-update */

    double fx[Nsta];   /* output of user defined f() state-transition function */
    double hx[Mobs];   /* output of user defined h() measurement function */

    double Pdot[Nsta][Nsta];
    /* temporary storage */
    double tmp0[Nsta][Nsta];
    double tmp1[Nsta][Mobs];
    double tmp2[Mobs][Nsta];
    double tmp3[Mobs][Mobs];
    double tmp4[Mobs][Mobs];
    double tmp5[Mobs];
    double tmp6[Nsta][Nsta];

    /* ERT PARAM */
    double t;
    double F_Thrust;
    double M;
    double dMdt;
    double rho;
    double p;
    double a;
    double Temp;

    /* ROCKET PARAM */
    double g;
    double S_ref;    // Cross Area;
    double Cd;       // Drag Coefficient
} ekf_t;


#endif //KALMAN_EMILIEN_EKF_CONFIG_ERT_H
