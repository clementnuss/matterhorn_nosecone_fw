/* Extended Kalman Filter
 * Emilien Mingard
 * Based on:
 *      TinyEKF:    https://github.com/simondlevy/TinyEKF
 *      RK_4:       http://people.sc.fsu.edu/~jburkardt/c_src/rk4/rk4.html
 *
 * Modified by Emilien Mingard, EPFL Lausanne, 25 april 2018
 */

# include <Misc/Common.h>
# include <cmsis_os.h>

# include <stdlib.h>
# include <math.h>

# include "EKF_Config.h"
# include "rk4.h"
# include "tiny_ekf.h"
# include <Misc/rocket_constants.h>

# define NBdataMotor 63

volatile float32_t kalman_air_speed, kalman_altitude;

/** ---------------------------- DECLARED FUNCTIONS ---------------------------- **/
void rk4_CALL (double dt, double t0, double tmax, double x0[Nsta], ekf_t *ekf); // Call Runge-Kutta Solver
double *Rocket_f (double t, int n, double u[], ekf_t *ekf); // Rocket Behaviour Description
static void Thrust (ekf_t *ekf);

static void Time (ekf_t *ekf, double Time);

static void stdAtmos (ekf_t *ekf);

static void Mass (ekf_t *ekf);

static void Init (ekf_t *ekf);

static void Jacobian (ekf_t *ekf);

ekf_t ekf;

/* DATA MEASURES (FOR DEBUG)*/
/** ---------------------------- MAIN FUNCTION ---------------------------- **/
void TK_Kalman (const void* args)
{

  /* Initialisation Kalman et Structure ekf */
  ekf_init (&ekf, Nsta, Mobs);
  Init (&ekf);

  uint32_t lastKalmanUpdate = 0;

  while (LIFTOFF_TIME == 0)
    {
      osDelay (10);
    }

  for (;;)
    {
      /* A chaque mesures reçue:
       *  Fournir les mesures dans Mesures[] = {METTRE LES MESURES ICI}
       *  Fournir le temps écoulé depuis liftoff dans CURRENT_TIME
       *  Fournir la covariance du capteur en ligne 227
       * */
      double Mesures[Mobs] =
        { getCurrentBARO_data ()->altitude - calib_initial_altitude };
      double x0[Nsta + Nsta * Nsta] =
        { ekf.x[0], ekf.x[1], ekf.tmp6[0][0], ekf.tmp6[0][1], ekf.tmp6[1][0], ekf.tmp6[1][1] };
      float currentFlightTime = (HAL_GetTick () - LIFTOFF_TIME) / 1000.0;
      rk4_CALL (0.01, ekf.t, currentFlightTime, x0, &ekf);
      ekf_step (&ekf, &Mesures[0]);

      kalman_altitude = ekf.x[0];
      kalman_air_speed = ekf.x[1];

      int elapsed = HAL_GetTick() - lastKalmanUpdate;
      if (100 - elapsed > 0) {
          osDelay(100 - elapsed);
      }
      lastKalmanUpdate = HAL_GetTick();

    }
}

#define U_ZERO_SIZE (Nsta + Nsta*Nsta)
double u0[U_ZERO_SIZE];

/** ---------------------------- CALLED FUNCTIONS ---------------------------- **/
/* Runge-Kutta 4th Order Call */
void rk4_CALL (double dt, double t0, double tmax, double x0[Nsta], ekf_t *ekf)
{
  /* PARAMETER */
  int i;
  int n = U_ZERO_SIZE; // x and P
  double t1;
  double *u1;

  /* INITIAL CONDITION */

  for (int p = 0; p < n; p++)
    {
      u0[p] = x0[p];
    }

  for (;;)
    {

      /* Stop at tmax */
      if (tmax <= t0)
        {
          break;
        }

      /* Step Advance */
      t1 = t0 + dt;
      u1 = rk4vec (t0, n, u0, dt, ekf, Rocket_f);

      /* New Step Preparation */
      t0 = t1;
      for (i = 0; i < n; i++)
        {
          u0[i] = u1[i];
        }
    }
  /* State Time Update */
  for (int k = 0; k < Nsta; k++)
    {
      ekf->x[k] = u0[k];
      ekf->x[k] = u0[k];
    }

  /* Observation Time Update */
  ekf->hx[0] = u0[0];

  /* Error Covariance Time Update */
  for (int k = 0; k < Nsta; k++)
    {
      for (int i = 0; i < Nsta; i++)
        {
          ekf->P[k][i] = u0[Nsta + k * Nsta + i];
        }
    }
  return;
}

/* Motor Definition: Change Define NBdataMotor */
double T_t[NBdataMotor] =
  { 0, 0.02, 0.07, 0.12, 0.17, 0.22, 0.27, 0.32, 0.37, 0.42, 0.47, 0.52, 0.57, 0.62, 0.67, 0.72, 0.77, 0.82, 0.87, 0.92,
      0.97, 1.02, 1.07, 1.12, 1.17, 1.22, 1.27, 1.32, 1.37, 1.42, 1.47, 1.52, 1.57, 1.62, 1.67, 1.72, 1.77, 1.82, 1.87,
      1.92, 1.97, 2.02, 2.07, 2.12, 2.17, 2.22, 2.27, 2.32, 2.37, 2.42, 2.47, 2.52, 2.57, 2.62, 2.67, 2.72, 2.77, 2.82,
      2.87, 2.92, 2.97, 3.02, 3.07 };
double T_Burn = 3.07;
double T_N[NBdataMotor] =
  { 0, 493.44, 1231.32, 1302.36, 1297.7, 1281.4, 1266.36, 1256.53, 1248.98, 1241.75, 1239.36, 1251.15, 1260.55, 1265.11,
      1271.89, 1276.62, 1280.64, 1292.8, 1295.25, 1300.58, 1304.32, 1314.2, 1309.75, 1304.76, 1310.13, 1308.62, 1305.4,
      1302.59, 1296.83, 1285.97, 1276.41, 1273.98, 1261.96, 1257.99, 1258.05, 1246.64, 1232.14, 1220.52, 1211.34,
      1201.14, 1196.07, 1185.1, 1176.96, 1161.97, 1148.11, 1135.56, 1123.62, 1108.09, 1095.49, 1080.5, 1061.55, 1043.02,
      1026.13, 1016.03, 1002.89, 1007.83, 1014.45, 1002.12, 915.16, 590.76, 280.46, 78.64, 2.57 };

double Interpolation_Thrust (ekf_t *ekf)
{
  double y = 0;
  if (ekf->t > T_Burn)
    {
      return y;
    }
  else
    {
      for (int k = 0; k < NBdataMotor; k++)
        {
          if (ekf->t < T_t[k])
            {
              y = T_N[k - 1] + (T_N[k] - T_N[k - 1]) / (T_t[k] - T_t[k - 1]) * (ekf->t - T_t[k - 1]);
              return y;
            }
        }
    }
}

static void Thrust (ekf_t *ekf)
{
  if (ekf->t > T_Burn)
    {
      ekf->F_Thrust = 0;
    }
  else
    {
      ekf->F_Thrust = Interpolation_Thrust (ekf);
    }
}

/* Rocket Mass */
double m_i = 20.24, dm = 2.065; // MOTOR L1150
static void Mass (ekf_t *ekf)
{
  if (ekf->t > T_Burn)
    {
      ekf->M = m_i - dm;
    }
  else
    {
      ekf->dMdt = -dm / T_Burn;
      ekf->M = m_i + ekf->t * ekf->dMdt;
    }
}

/* Standard Atmosphere */
static double R_air = 287.04, gamma_air = 1.4, p0 = 101325, rho0 = 1.1225, T0 = 288.15, a0 = 340.294, g0 = 9.80665,
    dTdh = -6.5;

static void stdAtmos (ekf_t *ekf)
{
  ekf->Temp = T0 + dTdh * (ekf->x[0] / 1000.0);
  ekf->p = p0 * pow (1 + dTdh / 1000.0 * ekf->x[0] / T0, -g0 / R_air / dTdh * 1000.0);
  ekf->rho = ekf->p / R_air / ekf->Temp;
  ekf->a = sqrt (gamma_air * R_air * ekf->Temp);
}

/* Time Update */
static void Time (ekf_t *ekf, double Time)
{
  ekf->t = Time;
}

/* System Equation Jacobian and Observation Matrix */
static void Jacobian (ekf_t *ekf)
{
  /* System Equation Jacobian: Nsta*Nsta Values To Define*/
  ekf->F[0][0] = 0;
  ekf->F[0][1] = 1;
  ekf->F[1][0] = 0;
  ekf->F[1][1] = -(ekf->rho * ekf->S_ref * ekf->Cd * ekf->x[1] + ekf->dMdt);

  /* Observation Matix: Nsta*Mobs Values To Define*/
  ekf->H[0][0] = 1;
  ekf->H[0][1] = 0;
}

double* Rocket_f (double t, int n, double u[], ekf_t *ekf)
{
  /* Variables Initialization */

  double* uprime = (double *) pvPortMalloc (n * sizeof(double));
  /* Update state and error vector */
  for (int k = 0; k < Nsta; k++)
    {
      ekf->x[k] = u[k];
      ekf->x[k] = u[k];
    }
  for (int k = 0; k < Nsta; k++)
    {
      for (int i = 0; i < Nsta; i++)
        {
          ekf->P[k][i] = u[Nsta + k * Nsta + i];
        }
    }

  /* Update Structure Data */
  Time (ekf, t);
  Thrust (ekf);
  stdAtmos (ekf);
  Mass (ekf);
  Jacobian (ekf);
  Riccati (ekf);

  /* State Time Derivative */
  uprime[0] = u[1];
  uprime[1] = (ekf->F_Thrust - 0.5 * ekf->rho * ekf->S_ref * ekf->Cd * pow (u[1], 2) - ekf->dMdt * u[1]) / ekf->M
      - ekf->g;
  uprime[2] = ekf->Pdot[0][0];
  uprime[3] = ekf->Pdot[0][1];
  uprime[4] = ekf->Pdot[1][0];
  uprime[5] = ekf->Pdot[1][1];
  /* Return Values */
  return uprime;
}

static void Init (ekf_t *ekf)
{
  /* Initial State: Nsta Values To Define*/
  ekf->x[0] = 0;
  ekf->x[1] = 0;

  /* Initial Error Covariance: Nsta*Nsta Values To Define */
  ekf->P[0][0] = 10;
  ekf->P[0][1] = 0;
  ekf->P[1][0] = 0;
  ekf->P[1][1] = 10;

  /* Process Noise Covariance: Nsta*Nsta Values To Define */
  ekf->Q[0][0] = 0;
  ekf->Q[0][1] = 0;
  ekf->Q[1][0] = 0;
  ekf->Q[1][1] = 0;

  /* Measurement Noise Covariance: Mobs*Mobs Values To Define */
  ekf->R[0][0] = 2; // TODO: CHANGE THIS VALUE

  /* Environment and Rocket: Fill All*/
  ekf->g = 9.80665;
  ekf->S_ref = 0.01184;
  ekf->Cd = 0.97;

  /* Basic Initialization: Nothing to Do */
  ekf->dMdt = 0;
  ekf->M = 0;
  ekf->F_Thrust = 0;
  ekf->t = 0;

  /* Temporary Matrix Initialisation */
  for (int k = 0; k < Nsta; k++)
    {
      for (int i = 0; i < Nsta; i++)
        {
          ekf->tmp6[k][i] = ekf->P[k][i];
        }
    }
}
