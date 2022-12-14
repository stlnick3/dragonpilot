#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_8437425954008409374);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_8396079497970260920);
void gnss_H_mod_fun(double *state, double *out_1730548818970258390);
void gnss_f_fun(double *state, double dt, double *out_4636080576513414149);
void gnss_F_fun(double *state, double dt, double *out_5550456960560640777);
void gnss_h_6(double *state, double *sat_pos, double *out_5552468654693606142);
void gnss_H_6(double *state, double *sat_pos, double *out_2735579608549628968);
void gnss_h_20(double *state, double *sat_pos, double *out_6233928774851547786);
void gnss_H_20(double *state, double *sat_pos, double *out_7971916037378030474);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_1191559184406871030);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_4579410319838777172);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_1191559184406871030);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_4579410319838777172);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}