#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_224171671284045072);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1225745529409332408);
void car_H_mod_fun(double *state, double *out_4001476234351128998);
void car_f_fun(double *state, double dt, double *out_3055040620036447315);
void car_F_fun(double *state, double dt, double *out_1598002714274555115);
void car_h_25(double *state, double *unused, double *out_3860808966802948963);
void car_H_25(double *state, double *unused, double *out_5352761963100751621);
void car_h_24(double *state, double *unused, double *out_3568143761156116010);
void car_H_24(double *state, double *unused, double *out_5823219445144090345);
void car_h_30(double *state, double *unused, double *out_1488331123436966155);
void car_H_30(double *state, double *unused, double *out_8566285780481191797);
void car_h_26(double *state, double *unused, double *out_3513342144223879490);
void car_H_26(double *state, double *unused, double *out_9094265281974807845);
void car_h_27(double *state, double *unused, double *out_6568718515051636131);
void car_H_27(double *state, double *unused, double *out_7656864222044416602);
void car_h_29(double *state, double *unused, double *out_4269653790556771801);
void car_H_29(double *state, double *unused, double *out_9076517124795583981);
void car_h_28(double *state, double *unused, double *out_8416812189527441471);
void car_H_28(double *state, double *unused, double *out_3994118107726053407);
void car_h_31(double *state, double *unused, double *out_5311440359431832130);
void car_H_31(double *state, double *unused, double *out_5322116001223791193);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}