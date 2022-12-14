#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_8762667531083925801);
void live_err_fun(double *nom_x, double *delta_x, double *out_2181421917658467336);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_4232698826433939920);
void live_H_mod_fun(double *state, double *out_3067005223293357501);
void live_f_fun(double *state, double dt, double *out_8579961002706476838);
void live_F_fun(double *state, double dt, double *out_1644457020271563935);
void live_h_4(double *state, double *unused, double *out_8874573301421709056);
void live_H_4(double *state, double *unused, double *out_1035369595663371810);
void live_h_9(double *state, double *unused, double *out_6859767989661938166);
void live_H_9(double *state, double *unused, double *out_8322588530927819280);
void live_h_10(double *state, double *unused, double *out_3522379116783669364);
void live_H_10(double *state, double *unused, double *out_8921858597104494624);
void live_h_12(double *state, double *unused, double *out_2753692107753195021);
void live_H_12(double *state, double *unused, double *out_6054826003695333605);
void live_h_35(double *state, double *unused, double *out_1753658750763024923);
void live_H_35(double *state, double *unused, double *out_8800389036020347314);
void live_h_32(double *state, double *unused, double *out_5727823348043794344);
void live_H_32(double *state, double *unused, double *out_4533196437976719655);
void live_h_13(double *state, double *unused, double *out_3495741932579616235);
void live_H_13(double *state, double *unused, double *out_7446734811616818018);
void live_h_14(double *state, double *unused, double *out_6859767989661938166);
void live_H_14(double *state, double *unused, double *out_8322588530927819280);
void live_h_33(double *state, double *unused, double *out_4144169622681746727);
void live_H_33(double *state, double *unused, double *out_6495798033050346698);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}