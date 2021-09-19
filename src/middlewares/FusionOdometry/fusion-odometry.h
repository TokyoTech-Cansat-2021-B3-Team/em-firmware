#pragma once

#include <chrono>
#include <math.h>
#define Nsta 6 // the size of states
#define Mobs 6 // the size of measurements

#include "TinyEKF.h"
#include "mbed.h"

class FusionOdometry : public TinyEKF {
public:
  FusionOdometry() : _timer(Timer()) {
    // process noise
    this->setQ(THETA, THETA, sigma_3_w); // theta
    this->setQ(THETA, OMEGA_Z, sigma_2_w);
    this->setQ(BETA_Z, BETA_Z, sigma_beta); // beta
    this->setQ(OMEGA_Z, THETA, sigma_2_w);  // omega_zk
    this->setQ(OMEGA_Z, OMEGA_Z, sigma_1_w);
    this->setQ(X, X, sigma_3_v); // xpk
    this->setQ(X, V, cos(initial_theta) * sigma_2_v);
    this->setQ(Y, Y, sigma_3_v); // ypk
    this->setQ(Y, V, sin(initial_theta) * sigma_2_v);
    this->setQ(V, V, sigma_1_v); // vxk
    this->setQ(V, X, cos(initial_theta) * sigma_2_v);
    this->setQ(V, Y, sin(initial_theta) * sigma_2_v);

    // the noise of measurements
    // the noise of measurements
    this->setR(OMEGA_ZW, OMEGA_ZW, sigma_w_w);  // w_w Dynamic
    this->setR(OMEGA_ZG, OMEGA_ZG, sigma_w_gz); // w_g
    this->setR(V_W, V_W, sigma_v_w);            // v_w Dynamic

    _timer.start();
  }

  void updateQ() {
    this->setQ(X, V, cos(this->x[0]) * sigma_2_v);
    this->setQ(Y, V, sin(this->x[0]) * sigma_2_v);
    this->setQ(V, X, cos(this->x[0]) * sigma_2_v);
    this->setQ(V, Y, sin(this->x[0]) * sigma_2_v);
    this->setR(OMEGA_ZW, OMEGA_ZW, sigma_w_w); // w_w Dynamic
    this->setR(V_W, V_W, sigma_v_w);           // v_w Dynamic
  }
  void updateR(double *z) {
    sigma_w_w = delta_w_w * this->_dt * z[0] + delta_w_w * this->_dt * z[2];
    sigma_v_w = delta_v_w * this->_dt * z[0] + delta_v_w * this->_dt * z[2];
    this->setR(OMEGA_ZW, OMEGA_ZW, sigma_w_w); // w_w Dynamic
    this->setR(V_W, V_W, sigma_v_w);           // v_w Dynamic
  }

  bool step_with_updateQR(double *z) {
    _dt = (_timer.elapsed_time().count() - _previousTime) * 1.0e-6;
    _previousTime = _timer.elapsed_time().count();
    updateQ();
    updateR(z);
    return this->step(z);
  }

protected:
  void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]) {
    // process model
    // refer to p1295 column L
    fx[THETA] = this->x[THETA] + this->x[OMEGA_Z] * this->_dt;
    fx[BETA_Z] = this->x[BETA_Z];
    fx[OMEGA_Z] = this->x[OMEGA_Z];
    fx[X] = this->x[X] + cos(this->x[THETA]) * this->_dt * this->x[V];
    fx[Y] = this->x[Y] + sin(this->x[THETA]) * this->_dt * this->x[V];
    fx[V] = this->x[V];

    // Matrix F
    F[THETA][THETA] = 1;
    F[THETA][OMEGA_Z] = this->_dt;
    F[BETA_Z][BETA_Z] = 1;
    F[OMEGA_Z][OMEGA_Z] = 1;
    F[X][X] = 1;
    F[X][THETA] = -sin(this->x[0]) * this->_dt * this->x[5];
    F[X][V] = cos(this->x[0]) * this->_dt;
    F[Y][Y] = 1;
    F[Y][THETA] = cos(this->x[0]) * this->_dt * this->x[5];
    F[Y][V] = sin(this->x[0]) * this->_dt;
    F[V][V] = 1;

    // measurements function
    hx[OMEGA_ZW] = this->x[OMEGA_Z];
    hx[OMEGA_ZG] = this->x[OMEGA_Z] + this->x[BETA_Z];
    hx[V_W] = this->x[V];

    // Matrix H
    H[OMEGA_ZW][OMEGA_Z] = 1;
    H[OMEGA_ZG][OMEGA_Z] = 1;
    H[OMEGA_ZG][BETA_Z] = 1;
    H[V_W][V] = 1;
  }
  const uint8_t THETA = 0;
  const uint8_t BETA_Z = 1;
  const uint8_t OMEGA_Z = 2;
  const uint8_t X = 3;
  const uint8_t Y = 4;
  const uint8_t V = 5;

  const uint8_t OMEGA_ZW = 0;
  const uint8_t OMEGA_ZG = 1;
  const uint8_t V_W = 2;
  const double initial_theta = 0.0;
  const double initial_w_wheel = 0.0;
  const double initial_v_wheel = 0.0;

  const double sigma_1_w = 0.01f;  // 調整する変数
  const double sigma_1_v = 0.01f;  // 調整する変数
  const double sigma_beta = 0.01f; // 調整する変数
  const double sigma_2_w = sigma_1_w * sigma_1_w / 2;
  const double sigma_3_w = sigma_1_w * sigma_1_w * sigma_1_w / 3;
  const double sigma_2_v = sigma_1_v * sigma_1_v / 2;
  const double sigma_3_v = sigma_1_v * sigma_1_v * sigma_1_v / 3;

  double sigma_w_w;
  double sigma_v_w;
  const double delta_w_w = 0.01f;  // 調整する変数
  const double delta_v_w = 0.01f;  // 調整する変数
  const double sigma_w_gz = 0.01f; // 調整する変数
  double _dt = 0;
  double _previousTime = 0;
  Timer _timer;
};