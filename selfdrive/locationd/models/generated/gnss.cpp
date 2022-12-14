#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_8437425954008409374) {
   out_8437425954008409374[0] = delta_x[0] + nom_x[0];
   out_8437425954008409374[1] = delta_x[1] + nom_x[1];
   out_8437425954008409374[2] = delta_x[2] + nom_x[2];
   out_8437425954008409374[3] = delta_x[3] + nom_x[3];
   out_8437425954008409374[4] = delta_x[4] + nom_x[4];
   out_8437425954008409374[5] = delta_x[5] + nom_x[5];
   out_8437425954008409374[6] = delta_x[6] + nom_x[6];
   out_8437425954008409374[7] = delta_x[7] + nom_x[7];
   out_8437425954008409374[8] = delta_x[8] + nom_x[8];
   out_8437425954008409374[9] = delta_x[9] + nom_x[9];
   out_8437425954008409374[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8396079497970260920) {
   out_8396079497970260920[0] = -nom_x[0] + true_x[0];
   out_8396079497970260920[1] = -nom_x[1] + true_x[1];
   out_8396079497970260920[2] = -nom_x[2] + true_x[2];
   out_8396079497970260920[3] = -nom_x[3] + true_x[3];
   out_8396079497970260920[4] = -nom_x[4] + true_x[4];
   out_8396079497970260920[5] = -nom_x[5] + true_x[5];
   out_8396079497970260920[6] = -nom_x[6] + true_x[6];
   out_8396079497970260920[7] = -nom_x[7] + true_x[7];
   out_8396079497970260920[8] = -nom_x[8] + true_x[8];
   out_8396079497970260920[9] = -nom_x[9] + true_x[9];
   out_8396079497970260920[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_1730548818970258390) {
   out_1730548818970258390[0] = 1.0;
   out_1730548818970258390[1] = 0;
   out_1730548818970258390[2] = 0;
   out_1730548818970258390[3] = 0;
   out_1730548818970258390[4] = 0;
   out_1730548818970258390[5] = 0;
   out_1730548818970258390[6] = 0;
   out_1730548818970258390[7] = 0;
   out_1730548818970258390[8] = 0;
   out_1730548818970258390[9] = 0;
   out_1730548818970258390[10] = 0;
   out_1730548818970258390[11] = 0;
   out_1730548818970258390[12] = 1.0;
   out_1730548818970258390[13] = 0;
   out_1730548818970258390[14] = 0;
   out_1730548818970258390[15] = 0;
   out_1730548818970258390[16] = 0;
   out_1730548818970258390[17] = 0;
   out_1730548818970258390[18] = 0;
   out_1730548818970258390[19] = 0;
   out_1730548818970258390[20] = 0;
   out_1730548818970258390[21] = 0;
   out_1730548818970258390[22] = 0;
   out_1730548818970258390[23] = 0;
   out_1730548818970258390[24] = 1.0;
   out_1730548818970258390[25] = 0;
   out_1730548818970258390[26] = 0;
   out_1730548818970258390[27] = 0;
   out_1730548818970258390[28] = 0;
   out_1730548818970258390[29] = 0;
   out_1730548818970258390[30] = 0;
   out_1730548818970258390[31] = 0;
   out_1730548818970258390[32] = 0;
   out_1730548818970258390[33] = 0;
   out_1730548818970258390[34] = 0;
   out_1730548818970258390[35] = 0;
   out_1730548818970258390[36] = 1.0;
   out_1730548818970258390[37] = 0;
   out_1730548818970258390[38] = 0;
   out_1730548818970258390[39] = 0;
   out_1730548818970258390[40] = 0;
   out_1730548818970258390[41] = 0;
   out_1730548818970258390[42] = 0;
   out_1730548818970258390[43] = 0;
   out_1730548818970258390[44] = 0;
   out_1730548818970258390[45] = 0;
   out_1730548818970258390[46] = 0;
   out_1730548818970258390[47] = 0;
   out_1730548818970258390[48] = 1.0;
   out_1730548818970258390[49] = 0;
   out_1730548818970258390[50] = 0;
   out_1730548818970258390[51] = 0;
   out_1730548818970258390[52] = 0;
   out_1730548818970258390[53] = 0;
   out_1730548818970258390[54] = 0;
   out_1730548818970258390[55] = 0;
   out_1730548818970258390[56] = 0;
   out_1730548818970258390[57] = 0;
   out_1730548818970258390[58] = 0;
   out_1730548818970258390[59] = 0;
   out_1730548818970258390[60] = 1.0;
   out_1730548818970258390[61] = 0;
   out_1730548818970258390[62] = 0;
   out_1730548818970258390[63] = 0;
   out_1730548818970258390[64] = 0;
   out_1730548818970258390[65] = 0;
   out_1730548818970258390[66] = 0;
   out_1730548818970258390[67] = 0;
   out_1730548818970258390[68] = 0;
   out_1730548818970258390[69] = 0;
   out_1730548818970258390[70] = 0;
   out_1730548818970258390[71] = 0;
   out_1730548818970258390[72] = 1.0;
   out_1730548818970258390[73] = 0;
   out_1730548818970258390[74] = 0;
   out_1730548818970258390[75] = 0;
   out_1730548818970258390[76] = 0;
   out_1730548818970258390[77] = 0;
   out_1730548818970258390[78] = 0;
   out_1730548818970258390[79] = 0;
   out_1730548818970258390[80] = 0;
   out_1730548818970258390[81] = 0;
   out_1730548818970258390[82] = 0;
   out_1730548818970258390[83] = 0;
   out_1730548818970258390[84] = 1.0;
   out_1730548818970258390[85] = 0;
   out_1730548818970258390[86] = 0;
   out_1730548818970258390[87] = 0;
   out_1730548818970258390[88] = 0;
   out_1730548818970258390[89] = 0;
   out_1730548818970258390[90] = 0;
   out_1730548818970258390[91] = 0;
   out_1730548818970258390[92] = 0;
   out_1730548818970258390[93] = 0;
   out_1730548818970258390[94] = 0;
   out_1730548818970258390[95] = 0;
   out_1730548818970258390[96] = 1.0;
   out_1730548818970258390[97] = 0;
   out_1730548818970258390[98] = 0;
   out_1730548818970258390[99] = 0;
   out_1730548818970258390[100] = 0;
   out_1730548818970258390[101] = 0;
   out_1730548818970258390[102] = 0;
   out_1730548818970258390[103] = 0;
   out_1730548818970258390[104] = 0;
   out_1730548818970258390[105] = 0;
   out_1730548818970258390[106] = 0;
   out_1730548818970258390[107] = 0;
   out_1730548818970258390[108] = 1.0;
   out_1730548818970258390[109] = 0;
   out_1730548818970258390[110] = 0;
   out_1730548818970258390[111] = 0;
   out_1730548818970258390[112] = 0;
   out_1730548818970258390[113] = 0;
   out_1730548818970258390[114] = 0;
   out_1730548818970258390[115] = 0;
   out_1730548818970258390[116] = 0;
   out_1730548818970258390[117] = 0;
   out_1730548818970258390[118] = 0;
   out_1730548818970258390[119] = 0;
   out_1730548818970258390[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_4636080576513414149) {
   out_4636080576513414149[0] = dt*state[3] + state[0];
   out_4636080576513414149[1] = dt*state[4] + state[1];
   out_4636080576513414149[2] = dt*state[5] + state[2];
   out_4636080576513414149[3] = state[3];
   out_4636080576513414149[4] = state[4];
   out_4636080576513414149[5] = state[5];
   out_4636080576513414149[6] = dt*state[7] + state[6];
   out_4636080576513414149[7] = dt*state[8] + state[7];
   out_4636080576513414149[8] = state[8];
   out_4636080576513414149[9] = state[9];
   out_4636080576513414149[10] = state[10];
}
void F_fun(double *state, double dt, double *out_5550456960560640777) {
   out_5550456960560640777[0] = 1;
   out_5550456960560640777[1] = 0;
   out_5550456960560640777[2] = 0;
   out_5550456960560640777[3] = dt;
   out_5550456960560640777[4] = 0;
   out_5550456960560640777[5] = 0;
   out_5550456960560640777[6] = 0;
   out_5550456960560640777[7] = 0;
   out_5550456960560640777[8] = 0;
   out_5550456960560640777[9] = 0;
   out_5550456960560640777[10] = 0;
   out_5550456960560640777[11] = 0;
   out_5550456960560640777[12] = 1;
   out_5550456960560640777[13] = 0;
   out_5550456960560640777[14] = 0;
   out_5550456960560640777[15] = dt;
   out_5550456960560640777[16] = 0;
   out_5550456960560640777[17] = 0;
   out_5550456960560640777[18] = 0;
   out_5550456960560640777[19] = 0;
   out_5550456960560640777[20] = 0;
   out_5550456960560640777[21] = 0;
   out_5550456960560640777[22] = 0;
   out_5550456960560640777[23] = 0;
   out_5550456960560640777[24] = 1;
   out_5550456960560640777[25] = 0;
   out_5550456960560640777[26] = 0;
   out_5550456960560640777[27] = dt;
   out_5550456960560640777[28] = 0;
   out_5550456960560640777[29] = 0;
   out_5550456960560640777[30] = 0;
   out_5550456960560640777[31] = 0;
   out_5550456960560640777[32] = 0;
   out_5550456960560640777[33] = 0;
   out_5550456960560640777[34] = 0;
   out_5550456960560640777[35] = 0;
   out_5550456960560640777[36] = 1;
   out_5550456960560640777[37] = 0;
   out_5550456960560640777[38] = 0;
   out_5550456960560640777[39] = 0;
   out_5550456960560640777[40] = 0;
   out_5550456960560640777[41] = 0;
   out_5550456960560640777[42] = 0;
   out_5550456960560640777[43] = 0;
   out_5550456960560640777[44] = 0;
   out_5550456960560640777[45] = 0;
   out_5550456960560640777[46] = 0;
   out_5550456960560640777[47] = 0;
   out_5550456960560640777[48] = 1;
   out_5550456960560640777[49] = 0;
   out_5550456960560640777[50] = 0;
   out_5550456960560640777[51] = 0;
   out_5550456960560640777[52] = 0;
   out_5550456960560640777[53] = 0;
   out_5550456960560640777[54] = 0;
   out_5550456960560640777[55] = 0;
   out_5550456960560640777[56] = 0;
   out_5550456960560640777[57] = 0;
   out_5550456960560640777[58] = 0;
   out_5550456960560640777[59] = 0;
   out_5550456960560640777[60] = 1;
   out_5550456960560640777[61] = 0;
   out_5550456960560640777[62] = 0;
   out_5550456960560640777[63] = 0;
   out_5550456960560640777[64] = 0;
   out_5550456960560640777[65] = 0;
   out_5550456960560640777[66] = 0;
   out_5550456960560640777[67] = 0;
   out_5550456960560640777[68] = 0;
   out_5550456960560640777[69] = 0;
   out_5550456960560640777[70] = 0;
   out_5550456960560640777[71] = 0;
   out_5550456960560640777[72] = 1;
   out_5550456960560640777[73] = dt;
   out_5550456960560640777[74] = 0;
   out_5550456960560640777[75] = 0;
   out_5550456960560640777[76] = 0;
   out_5550456960560640777[77] = 0;
   out_5550456960560640777[78] = 0;
   out_5550456960560640777[79] = 0;
   out_5550456960560640777[80] = 0;
   out_5550456960560640777[81] = 0;
   out_5550456960560640777[82] = 0;
   out_5550456960560640777[83] = 0;
   out_5550456960560640777[84] = 1;
   out_5550456960560640777[85] = dt;
   out_5550456960560640777[86] = 0;
   out_5550456960560640777[87] = 0;
   out_5550456960560640777[88] = 0;
   out_5550456960560640777[89] = 0;
   out_5550456960560640777[90] = 0;
   out_5550456960560640777[91] = 0;
   out_5550456960560640777[92] = 0;
   out_5550456960560640777[93] = 0;
   out_5550456960560640777[94] = 0;
   out_5550456960560640777[95] = 0;
   out_5550456960560640777[96] = 1;
   out_5550456960560640777[97] = 0;
   out_5550456960560640777[98] = 0;
   out_5550456960560640777[99] = 0;
   out_5550456960560640777[100] = 0;
   out_5550456960560640777[101] = 0;
   out_5550456960560640777[102] = 0;
   out_5550456960560640777[103] = 0;
   out_5550456960560640777[104] = 0;
   out_5550456960560640777[105] = 0;
   out_5550456960560640777[106] = 0;
   out_5550456960560640777[107] = 0;
   out_5550456960560640777[108] = 1;
   out_5550456960560640777[109] = 0;
   out_5550456960560640777[110] = 0;
   out_5550456960560640777[111] = 0;
   out_5550456960560640777[112] = 0;
   out_5550456960560640777[113] = 0;
   out_5550456960560640777[114] = 0;
   out_5550456960560640777[115] = 0;
   out_5550456960560640777[116] = 0;
   out_5550456960560640777[117] = 0;
   out_5550456960560640777[118] = 0;
   out_5550456960560640777[119] = 0;
   out_5550456960560640777[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_5552468654693606142) {
   out_5552468654693606142[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_2735579608549628968) {
   out_2735579608549628968[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2735579608549628968[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2735579608549628968[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2735579608549628968[3] = 0;
   out_2735579608549628968[4] = 0;
   out_2735579608549628968[5] = 0;
   out_2735579608549628968[6] = 1;
   out_2735579608549628968[7] = 0;
   out_2735579608549628968[8] = 0;
   out_2735579608549628968[9] = 0;
   out_2735579608549628968[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_6233928774851547786) {
   out_6233928774851547786[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_7971916037378030474) {
   out_7971916037378030474[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7971916037378030474[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7971916037378030474[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7971916037378030474[3] = 0;
   out_7971916037378030474[4] = 0;
   out_7971916037378030474[5] = 0;
   out_7971916037378030474[6] = 1;
   out_7971916037378030474[7] = 0;
   out_7971916037378030474[8] = 0;
   out_7971916037378030474[9] = 1;
   out_7971916037378030474[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_1191559184406871030) {
   out_1191559184406871030[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_4579410319838777172) {
   out_4579410319838777172[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4579410319838777172[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4579410319838777172[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4579410319838777172[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4579410319838777172[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4579410319838777172[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4579410319838777172[6] = 0;
   out_4579410319838777172[7] = 1;
   out_4579410319838777172[8] = 0;
   out_4579410319838777172[9] = 0;
   out_4579410319838777172[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_1191559184406871030) {
   out_1191559184406871030[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_4579410319838777172) {
   out_4579410319838777172[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4579410319838777172[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4579410319838777172[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4579410319838777172[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4579410319838777172[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4579410319838777172[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4579410319838777172[6] = 0;
   out_4579410319838777172[7] = 1;
   out_4579410319838777172[8] = 0;
   out_4579410319838777172[9] = 0;
   out_4579410319838777172[10] = 0;
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

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_8437425954008409374) {
  err_fun(nom_x, delta_x, out_8437425954008409374);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_8396079497970260920) {
  inv_err_fun(nom_x, true_x, out_8396079497970260920);
}
void gnss_H_mod_fun(double *state, double *out_1730548818970258390) {
  H_mod_fun(state, out_1730548818970258390);
}
void gnss_f_fun(double *state, double dt, double *out_4636080576513414149) {
  f_fun(state,  dt, out_4636080576513414149);
}
void gnss_F_fun(double *state, double dt, double *out_5550456960560640777) {
  F_fun(state,  dt, out_5550456960560640777);
}
void gnss_h_6(double *state, double *sat_pos, double *out_5552468654693606142) {
  h_6(state, sat_pos, out_5552468654693606142);
}
void gnss_H_6(double *state, double *sat_pos, double *out_2735579608549628968) {
  H_6(state, sat_pos, out_2735579608549628968);
}
void gnss_h_20(double *state, double *sat_pos, double *out_6233928774851547786) {
  h_20(state, sat_pos, out_6233928774851547786);
}
void gnss_H_20(double *state, double *sat_pos, double *out_7971916037378030474) {
  H_20(state, sat_pos, out_7971916037378030474);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_1191559184406871030) {
  h_7(state, sat_pos_vel, out_1191559184406871030);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_4579410319838777172) {
  H_7(state, sat_pos_vel, out_4579410319838777172);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_1191559184406871030) {
  h_21(state, sat_pos_vel, out_1191559184406871030);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_4579410319838777172) {
  H_21(state, sat_pos_vel, out_4579410319838777172);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
