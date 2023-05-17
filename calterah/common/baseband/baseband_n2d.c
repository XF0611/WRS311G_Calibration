#include <string.h>
#include "math.h"
#include "calterah_math.h"

#include "baseband.h"
#include "sensor_config.h"
#include "baseband_n2d.h"

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#ifndef M_PI
#define M_PI 3.1415926535f
#endif
#define RAD2DEG 57.295779513082323

n2d_info_t n2d_info;

void cal_coe(complex_t *sv0, complex_t *sv1, complex_t *X, int elem_num, complex_t *coe_array)
{
        complex_t sv0X = dot_product(sv0, X, elem_num);
        complex_t sv1X = dot_product(sv1, X, elem_num);

        complex_t sHX[2] = {sv0X, sv1X};

        complex_t sv0sv1 = dot_product(sv0, sv1, elem_num);
        complex_t sv0sv1_conj = {.r = sv0sv1.r, .i = -1 * sv0sv1.i};

        complex_t c_elem_num = {.r = 1.0 * elem_num, .i = 0.0};

        complex_t minus_sv0sv1;
        crmult(&sv0sv1, -1.0, &minus_sv0sv1);
        complex_t minus_sv0sv1_conj;
        crmult(&sv0sv1_conj, -1.0, &minus_sv0sv1_conj);

        complex_t c0[2] = {c_elem_num, cconj(&minus_sv0sv1)};
        complex_t c1[2] = {cconj(&minus_sv0sv1_conj), c_elem_num};

        coe_array[0] = dot_product(c0, sHX, 2);
        coe_array[1] = dot_product(c1, sHX, 2);
}

bool is_sw_combined_mode(sensor_config_t* cfg,
                         doa_info_t *doa_info) {
        if (cfg->doa_mode != 1 || cfg->doa_max_obj_per_bin[0] == 1) {
                /* if is not configured as combined mode + multiple objects, return false to follow old procedure */
                return false;
        }

        if (!(doa_info->ang_vld_0 && doa_info->ang_vld_1)) {
                /* if DoA detects only one theta angle, use old procedure to handle */
                return false;
        }

        return true;
}


void sw_combined_mode(radar_sys_params_t* sys_params, sensor_config_t* cfg,
                      doa_info_t *doa_info,
                      volatile track_cdi_t *track_cdi,
                      int *j, int32_t rng_idx, int32_t vel_idx,
                      float vel_acc) {

        uint8_t azi_line_num = n2d_info.azi_line_num;

        uint32_t ang0_idx = doa_info->ang_idx_0 + ((cfg->doa_method == 2 && doa_info->ang_acc_0 != 0) ? 512 : 0);
        uint32_t ang1_idx = doa_info->ang_idx_1 + ((cfg->doa_method == 2 && doa_info->ang_acc_1 != 0) ? 512 : 0);

        complex_t x_total[32];
        baseband_hw_t * bb_hw = &((baseband_t *)cfg->bb)->bb_hw;

        uint8_t bpm_idx_min = 0;
        uint8_t bpm_idx_max = cfg->nvarray - 1;
        if (cfg->anti_velamb_en) {
                bpm_idx_min = 1;
                bpm_idx_max = cfg->nvarray;
        }

        uint32_t old1 = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_BUF);
        for (int bpm_index = bpm_idx_min; bpm_index <= bpm_idx_max; bpm_index++) {
                complex_t tw = {.r = 1, .i = 0};
                if (n2d_info.ph_comp_flg && bpm_index != bpm_idx_min) {
                        tw = expj(n2d_info.Tr_2_PI_Over_TDN * (bpm_index - bpm_idx_min) * vel_acc);
                }

                for (uint8_t ch_index = 0; ch_index < MAX_NUM_RX; ch_index++) {
                        uint32_t fft_mem = baseband_hw_get_fft_mem(bb_hw,
                                                                   ch_index,
                                                                   rng_idx, vel_idx,
                                                                   bpm_index);
                        complex_t complex_fft = cfl_to_complex(fft_mem, 14, 1, true, 4, false);

                        if (n2d_info.ph_comp_flg && (bpm_index != bpm_idx_min)) {
                                cmult(&complex_fft, &tw, &complex_fft);
                        }

                        x_total[ch_index + (bpm_index - bpm_idx_min) * MAX_NUM_RX] = complex_fft;
                }
        }
        baseband_switch_mem_access(bb_hw, old1);

        complex_t Y[2][4];

        float phi[2] = {0.0, 0.0};
        int y_elem_idx = 0;
        float factor_sum = 0;

        int last_line_idx = 0;

        for (int line_idx = 0; line_idx < azi_line_num; line_idx++) {
                int elem_num = n2d_info.azi_vector_size[line_idx];
                if (elem_num >= 3) {
                        last_line_idx = line_idx;
                        break;
                }
        }

        for (int line_idx = 0; line_idx < azi_line_num; line_idx++) {
                int elem_num = n2d_info.azi_vector_size[line_idx];
                if (elem_num < 3)
                        continue;

                complex_t x[32];
                complex_t sv0[32];
                complex_t sv1[32];

                for (int idx = 0; idx < elem_num; idx++) {
                        x[idx] = x_total[n2d_info.azi_logic_channel_idx[line_idx][idx]];
                }

                read_steering_vec_from_mem(bb_hw,
                                           elem_num,
                                           n2d_info.azi_sv_add[line_idx],
                                           ang0_idx, sv0);

                read_steering_vec_from_mem(bb_hw,
                                           elem_num,
                                           n2d_info.azi_sv_add[line_idx],
                                           ang1_idx, sv1);

                complex_t coe_array[2];
                cal_coe(sv0, sv1, x, elem_num, coe_array);

                Y[0][y_elem_idx] = coe_array[0];
                Y[1][y_elem_idx] = coe_array[1];

                if (y_elem_idx > 0) {
                        complex_t y1y0;
                        float diff   = n2d_info.vertical_dis[line_idx] - n2d_info.vertical_dis[last_line_idx];
                        float factor = n2d_info.azi_vector_size[line_idx] * n2d_info.azi_vector_size[last_line_idx];

                        cmult_conj(&(Y[0][y_elem_idx]), &(Y[0][y_elem_idx - 1]), &y1y0);
                        phi[0] += factor * asinf(atan2f(y1y0.i, y1y0.r) / (2 * M_PI * diff));

                        cmult_conj(&(Y[1][y_elem_idx]), &(Y[1][y_elem_idx - 1]), &y1y0);
                        phi[1] += factor * asinf(atan2f(y1y0.i, y1y0.r) / (2 * M_PI * diff));
                        factor_sum += factor;
                }

                last_line_idx = line_idx;
                y_elem_idx++;
        }

        phi[0] /= factor_sum;
        phi[1] /= factor_sum;

        float phi0_rad = phi[0];
        float phi1_rad = phi[1];

        phi[0] *= RAD2DEG;
        phi[1] *= RAD2DEG;

        float sin_theta_0;
        float sin_theta_1;
        float sin_alpha_0;
        float sin_alpha_1;

        if (cfg->doa_method == 2) {
                sin_alpha_0 = sys_params->dml_sin_az_left + ang0_idx * sys_params->dml_sin_az_step;
                sin_alpha_1 = sys_params->dml_sin_az_left + ang1_idx * sys_params->dml_sin_az_step;
        } else {
                float dbf_alpha_0 = sys_params->bfm_az_left + ang0_idx * (sys_params->bfm_az_right - sys_params->bfm_az_left) / sys_params->doa_npoint[0];
                float dbf_alpha_1 = sys_params->bfm_az_left + ang1_idx * (sys_params->bfm_az_right - sys_params->bfm_az_left) / sys_params->doa_npoint[0];
                sin_alpha_0 = sinf(dbf_alpha_0);
                sin_alpha_1 = sinf(dbf_alpha_1);
        }

        sin_theta_0 = sin_alpha_0 / cosf(phi0_rad);
        sin_theta_1 = sin_alpha_1 / cosf(phi1_rad);

        float theta0, theta1;

        if (sin_theta_0 <= 1 && sin_theta_0 >= -1) {
                theta0 = asinf(sin_theta_0) * RAD2DEG;
        } else {
                theta0 = asinf(sin_alpha_0) * RAD2DEG;
        }

        if (sin_theta_1 <= 1 && sin_theta_1 >= -1) {
                theta1 = asinf(sin_theta_1) * RAD2DEG;
        } else {
                theta1 = asinf(sin_alpha_1) * RAD2DEG;
        }

        bool theta0_exist = false;
        if (   theta0 > sys_params->trk_fov_az_left && theta0 < sys_params->trk_fov_az_right
            && phi[0] > sys_params->trk_fov_ev_down && phi[0] < sys_params->trk_fov_ev_up) {
                track_cdi[*j].raw_z.ang = theta0;
#if TRK_CONF_3D
                track_cdi[*j].raw_z.ang_elv = phi[0];
#endif
                /* theta0 and phi0's power is already set */

                *j += 1;
                theta0_exist = true;
        }

        if (theta0_exist) {
                track_cdi[*j].raw_z = track_cdi[*j - 1].raw_z;
        }

        if (   theta1 > sys_params->trk_fov_az_left && theta1 < sys_params->trk_fov_az_right
            && phi[1] > sys_params->trk_fov_ev_down && phi[1] < sys_params->trk_fov_ev_up) {
                track_cdi[*j].raw_z.ang = theta1;
#if TRK_CONF_3D
                track_cdi[*j].raw_z.ang_elv = phi[1];
#endif
                if (cfg->track_obj_snr_sel == 0) {
                        uint32_t hw_sig = doa_info->sig_1;
                        track_cdi[*j].raw_z.sig = fl_to_float(hw_sig, 15, 1, false, 5, false);
                }
                *j += 1;
        }
}




