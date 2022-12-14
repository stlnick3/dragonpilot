#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_224171671284045072) {
   out_224171671284045072[0] = delta_x[0] + nom_x[0];
   out_224171671284045072[1] = delta_x[1] + nom_x[1];
   out_224171671284045072[2] = delta_x[2] + nom_x[2];
   out_224171671284045072[3] = delta_x[3] + nom_x[3];
   out_224171671284045072[4] = delta_x[4] + nom_x[4];
   out_224171671284045072[5] = delta_x[5] + nom_x[5];
   out_224171671284045072[6] = delta_x[6] + nom_x[6];
   out_224171671284045072[7] = delta_x[7] + nom_x[7];
   out_224171671284045072[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1225745529409332408) {
   out_1225745529409332408[0] = -nom_x[0] + true_x[0];
   out_1225745529409332408[1] = -nom_x[1] + true_x[1];
   out_1225745529409332408[2] = -nom_x[2] + true_x[2];
   out_1225745529409332408[3] = -nom_x[3] + true_x[3];
   out_1225745529409332408[4] = -nom_x[4] + true_x[4];
   out_1225745529409332408[5] = -nom_x[5] + true_x[5];
   out_1225745529409332408[6] = -nom_x[6] + true_x[6];
   out_1225745529409332408[7] = -nom_x[7] + true_x[7];
   out_1225745529409332408[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4001476234351128998) {
   out_4001476234351128998[0] = 1.0;
   out_4001476234351128998[1] = 0;
   out_4001476234351128998[2] = 0;
   out_4001476234351128998[3] = 0;
   out_4001476234351128998[4] = 0;
   out_4001476234351128998[5] = 0;
   out_4001476234351128998[6] = 0;
   out_4001476234351128998[7] = 0;
   out_4001476234351128998[8] = 0;
   out_4001476234351128998[9] = 0;
   out_4001476234351128998[10] = 1.0;
   out_4001476234351128998[11] = 0;
   out_4001476234351128998[12] = 0;
   out_4001476234351128998[13] = 0;
   out_4001476234351128998[14] = 0;
   out_4001476234351128998[15] = 0;
   out_4001476234351128998[16] = 0;
   out_4001476234351128998[17] = 0;
   out_4001476234351128998[18] = 0;
   out_4001476234351128998[19] = 0;
   out_4001476234351128998[20] = 1.0;
   out_4001476234351128998[21] = 0;
   out_4001476234351128998[22] = 0;
   out_4001476234351128998[23] = 0;
   out_4001476234351128998[24] = 0;
   out_4001476234351128998[25] = 0;
   out_4001476234351128998[26] = 0;
   out_4001476234351128998[27] = 0;
   out_4001476234351128998[28] = 0;
   out_4001476234351128998[29] = 0;
   out_4001476234351128998[30] = 1.0;
   out_4001476234351128998[31] = 0;
   out_4001476234351128998[32] = 0;
   out_4001476234351128998[33] = 0;
   out_4001476234351128998[34] = 0;
   out_4001476234351128998[35] = 0;
   out_4001476234351128998[36] = 0;
   out_4001476234351128998[37] = 0;
   out_4001476234351128998[38] = 0;
   out_4001476234351128998[39] = 0;
   out_4001476234351128998[40] = 1.0;
   out_4001476234351128998[41] = 0;
   out_4001476234351128998[42] = 0;
   out_4001476234351128998[43] = 0;
   out_4001476234351128998[44] = 0;
   out_4001476234351128998[45] = 0;
   out_4001476234351128998[46] = 0;
   out_4001476234351128998[47] = 0;
   out_4001476234351128998[48] = 0;
   out_4001476234351128998[49] = 0;
   out_4001476234351128998[50] = 1.0;
   out_4001476234351128998[51] = 0;
   out_4001476234351128998[52] = 0;
   out_4001476234351128998[53] = 0;
   out_4001476234351128998[54] = 0;
   out_4001476234351128998[55] = 0;
   out_4001476234351128998[56] = 0;
   out_4001476234351128998[57] = 0;
   out_4001476234351128998[58] = 0;
   out_4001476234351128998[59] = 0;
   out_4001476234351128998[60] = 1.0;
   out_4001476234351128998[61] = 0;
   out_4001476234351128998[62] = 0;
   out_4001476234351128998[63] = 0;
   out_4001476234351128998[64] = 0;
   out_4001476234351128998[65] = 0;
   out_4001476234351128998[66] = 0;
   out_4001476234351128998[67] = 0;
   out_4001476234351128998[68] = 0;
   out_4001476234351128998[69] = 0;
   out_4001476234351128998[70] = 1.0;
   out_4001476234351128998[71] = 0;
   out_4001476234351128998[72] = 0;
   out_4001476234351128998[73] = 0;
   out_4001476234351128998[74] = 0;
   out_4001476234351128998[75] = 0;
   out_4001476234351128998[76] = 0;
   out_4001476234351128998[77] = 0;
   out_4001476234351128998[78] = 0;
   out_4001476234351128998[79] = 0;
   out_4001476234351128998[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3055040620036447315) {
   out_3055040620036447315[0] = state[0];
   out_3055040620036447315[1] = state[1];
   out_3055040620036447315[2] = state[2];
   out_3055040620036447315[3] = state[3];
   out_3055040620036447315[4] = state[4];
   out_3055040620036447315[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3055040620036447315[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3055040620036447315[7] = state[7];
   out_3055040620036447315[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1598002714274555115) {
   out_1598002714274555115[0] = 1;
   out_1598002714274555115[1] = 0;
   out_1598002714274555115[2] = 0;
   out_1598002714274555115[3] = 0;
   out_1598002714274555115[4] = 0;
   out_1598002714274555115[5] = 0;
   out_1598002714274555115[6] = 0;
   out_1598002714274555115[7] = 0;
   out_1598002714274555115[8] = 0;
   out_1598002714274555115[9] = 0;
   out_1598002714274555115[10] = 1;
   out_1598002714274555115[11] = 0;
   out_1598002714274555115[12] = 0;
   out_1598002714274555115[13] = 0;
   out_1598002714274555115[14] = 0;
   out_1598002714274555115[15] = 0;
   out_1598002714274555115[16] = 0;
   out_1598002714274555115[17] = 0;
   out_1598002714274555115[18] = 0;
   out_1598002714274555115[19] = 0;
   out_1598002714274555115[20] = 1;
   out_1598002714274555115[21] = 0;
   out_1598002714274555115[22] = 0;
   out_1598002714274555115[23] = 0;
   out_1598002714274555115[24] = 0;
   out_1598002714274555115[25] = 0;
   out_1598002714274555115[26] = 0;
   out_1598002714274555115[27] = 0;
   out_1598002714274555115[28] = 0;
   out_1598002714274555115[29] = 0;
   out_1598002714274555115[30] = 1;
   out_1598002714274555115[31] = 0;
   out_1598002714274555115[32] = 0;
   out_1598002714274555115[33] = 0;
   out_1598002714274555115[34] = 0;
   out_1598002714274555115[35] = 0;
   out_1598002714274555115[36] = 0;
   out_1598002714274555115[37] = 0;
   out_1598002714274555115[38] = 0;
   out_1598002714274555115[39] = 0;
   out_1598002714274555115[40] = 1;
   out_1598002714274555115[41] = 0;
   out_1598002714274555115[42] = 0;
   out_1598002714274555115[43] = 0;
   out_1598002714274555115[44] = 0;
   out_1598002714274555115[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1598002714274555115[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1598002714274555115[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1598002714274555115[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1598002714274555115[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1598002714274555115[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1598002714274555115[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1598002714274555115[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1598002714274555115[53] = -9.8000000000000007*dt;
   out_1598002714274555115[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1598002714274555115[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1598002714274555115[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1598002714274555115[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1598002714274555115[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1598002714274555115[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1598002714274555115[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1598002714274555115[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1598002714274555115[62] = 0;
   out_1598002714274555115[63] = 0;
   out_1598002714274555115[64] = 0;
   out_1598002714274555115[65] = 0;
   out_1598002714274555115[66] = 0;
   out_1598002714274555115[67] = 0;
   out_1598002714274555115[68] = 0;
   out_1598002714274555115[69] = 0;
   out_1598002714274555115[70] = 1;
   out_1598002714274555115[71] = 0;
   out_1598002714274555115[72] = 0;
   out_1598002714274555115[73] = 0;
   out_1598002714274555115[74] = 0;
   out_1598002714274555115[75] = 0;
   out_1598002714274555115[76] = 0;
   out_1598002714274555115[77] = 0;
   out_1598002714274555115[78] = 0;
   out_1598002714274555115[79] = 0;
   out_1598002714274555115[80] = 1;
}
void h_25(double *state, double *unused, double *out_3860808966802948963) {
   out_3860808966802948963[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5352761963100751621) {
   out_5352761963100751621[0] = 0;
   out_5352761963100751621[1] = 0;
   out_5352761963100751621[2] = 0;
   out_5352761963100751621[3] = 0;
   out_5352761963100751621[4] = 0;
   out_5352761963100751621[5] = 0;
   out_5352761963100751621[6] = 1;
   out_5352761963100751621[7] = 0;
   out_5352761963100751621[8] = 0;
}
void h_24(double *state, double *unused, double *out_3568143761156116010) {
   out_3568143761156116010[0] = state[4];
   out_3568143761156116010[1] = state[5];
}
void H_24(double *state, double *unused, double *out_5823219445144090345) {
   out_5823219445144090345[0] = 0;
   out_5823219445144090345[1] = 0;
   out_5823219445144090345[2] = 0;
   out_5823219445144090345[3] = 0;
   out_5823219445144090345[4] = 1;
   out_5823219445144090345[5] = 0;
   out_5823219445144090345[6] = 0;
   out_5823219445144090345[7] = 0;
   out_5823219445144090345[8] = 0;
   out_5823219445144090345[9] = 0;
   out_5823219445144090345[10] = 0;
   out_5823219445144090345[11] = 0;
   out_5823219445144090345[12] = 0;
   out_5823219445144090345[13] = 0;
   out_5823219445144090345[14] = 1;
   out_5823219445144090345[15] = 0;
   out_5823219445144090345[16] = 0;
   out_5823219445144090345[17] = 0;
}
void h_30(double *state, double *unused, double *out_1488331123436966155) {
   out_1488331123436966155[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8566285780481191797) {
   out_8566285780481191797[0] = 0;
   out_8566285780481191797[1] = 0;
   out_8566285780481191797[2] = 0;
   out_8566285780481191797[3] = 0;
   out_8566285780481191797[4] = 1;
   out_8566285780481191797[5] = 0;
   out_8566285780481191797[6] = 0;
   out_8566285780481191797[7] = 0;
   out_8566285780481191797[8] = 0;
}
void h_26(double *state, double *unused, double *out_3513342144223879490) {
   out_3513342144223879490[0] = state[7];
}
void H_26(double *state, double *unused, double *out_9094265281974807845) {
   out_9094265281974807845[0] = 0;
   out_9094265281974807845[1] = 0;
   out_9094265281974807845[2] = 0;
   out_9094265281974807845[3] = 0;
   out_9094265281974807845[4] = 0;
   out_9094265281974807845[5] = 0;
   out_9094265281974807845[6] = 0;
   out_9094265281974807845[7] = 1;
   out_9094265281974807845[8] = 0;
}
void h_27(double *state, double *unused, double *out_6568718515051636131) {
   out_6568718515051636131[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7656864222044416602) {
   out_7656864222044416602[0] = 0;
   out_7656864222044416602[1] = 0;
   out_7656864222044416602[2] = 0;
   out_7656864222044416602[3] = 1;
   out_7656864222044416602[4] = 0;
   out_7656864222044416602[5] = 0;
   out_7656864222044416602[6] = 0;
   out_7656864222044416602[7] = 0;
   out_7656864222044416602[8] = 0;
}
void h_29(double *state, double *unused, double *out_4269653790556771801) {
   out_4269653790556771801[0] = state[1];
}
void H_29(double *state, double *unused, double *out_9076517124795583981) {
   out_9076517124795583981[0] = 0;
   out_9076517124795583981[1] = 1;
   out_9076517124795583981[2] = 0;
   out_9076517124795583981[3] = 0;
   out_9076517124795583981[4] = 0;
   out_9076517124795583981[5] = 0;
   out_9076517124795583981[6] = 0;
   out_9076517124795583981[7] = 0;
   out_9076517124795583981[8] = 0;
}
void h_28(double *state, double *unused, double *out_8416812189527441471) {
   out_8416812189527441471[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3994118107726053407) {
   out_3994118107726053407[0] = 1;
   out_3994118107726053407[1] = 0;
   out_3994118107726053407[2] = 0;
   out_3994118107726053407[3] = 0;
   out_3994118107726053407[4] = 0;
   out_3994118107726053407[5] = 0;
   out_3994118107726053407[6] = 0;
   out_3994118107726053407[7] = 0;
   out_3994118107726053407[8] = 0;
}
void h_31(double *state, double *unused, double *out_5311440359431832130) {
   out_5311440359431832130[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5322116001223791193) {
   out_5322116001223791193[0] = 0;
   out_5322116001223791193[1] = 0;
   out_5322116001223791193[2] = 0;
   out_5322116001223791193[3] = 0;
   out_5322116001223791193[4] = 0;
   out_5322116001223791193[5] = 0;
   out_5322116001223791193[6] = 0;
   out_5322116001223791193[7] = 0;
   out_5322116001223791193[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_224171671284045072) {
  err_fun(nom_x, delta_x, out_224171671284045072);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1225745529409332408) {
  inv_err_fun(nom_x, true_x, out_1225745529409332408);
}
void car_H_mod_fun(double *state, double *out_4001476234351128998) {
  H_mod_fun(state, out_4001476234351128998);
}
void car_f_fun(double *state, double dt, double *out_3055040620036447315) {
  f_fun(state,  dt, out_3055040620036447315);
}
void car_F_fun(double *state, double dt, double *out_1598002714274555115) {
  F_fun(state,  dt, out_1598002714274555115);
}
void car_h_25(double *state, double *unused, double *out_3860808966802948963) {
  h_25(state, unused, out_3860808966802948963);
}
void car_H_25(double *state, double *unused, double *out_5352761963100751621) {
  H_25(state, unused, out_5352761963100751621);
}
void car_h_24(double *state, double *unused, double *out_3568143761156116010) {
  h_24(state, unused, out_3568143761156116010);
}
void car_H_24(double *state, double *unused, double *out_5823219445144090345) {
  H_24(state, unused, out_5823219445144090345);
}
void car_h_30(double *state, double *unused, double *out_1488331123436966155) {
  h_30(state, unused, out_1488331123436966155);
}
void car_H_30(double *state, double *unused, double *out_8566285780481191797) {
  H_30(state, unused, out_8566285780481191797);
}
void car_h_26(double *state, double *unused, double *out_3513342144223879490) {
  h_26(state, unused, out_3513342144223879490);
}
void car_H_26(double *state, double *unused, double *out_9094265281974807845) {
  H_26(state, unused, out_9094265281974807845);
}
void car_h_27(double *state, double *unused, double *out_6568718515051636131) {
  h_27(state, unused, out_6568718515051636131);
}
void car_H_27(double *state, double *unused, double *out_7656864222044416602) {
  H_27(state, unused, out_7656864222044416602);
}
void car_h_29(double *state, double *unused, double *out_4269653790556771801) {
  h_29(state, unused, out_4269653790556771801);
}
void car_H_29(double *state, double *unused, double *out_9076517124795583981) {
  H_29(state, unused, out_9076517124795583981);
}
void car_h_28(double *state, double *unused, double *out_8416812189527441471) {
  h_28(state, unused, out_8416812189527441471);
}
void car_H_28(double *state, double *unused, double *out_3994118107726053407) {
  H_28(state, unused, out_3994118107726053407);
}
void car_h_31(double *state, double *unused, double *out_5311440359431832130) {
  h_31(state, unused, out_5311440359431832130);
}
void car_H_31(double *state, double *unused, double *out_5322116001223791193) {
  H_31(state, unused, out_5322116001223791193);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
