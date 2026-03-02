/*
 * Microsemi Switchtec(tm) PCIe Management Library
 * Copyright (c) 2017, Microsemi Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 */

/**
 * @file
 * @brief Gen5-specific operations
 */

#include "../switchtec_priv.h"

extern int switchtec_diag_ltssm_log_gen5(struct switchtec_dev *dev,
				 int port, int *log_count, void *log_data);

extern int switchtec_diag_port_eq_tx_coeff_gen5(struct switchtec_dev *dev,
						int port_id, int prev_speed,
						int end, int link, void *res);

extern int switchtec_diag_port_eq_tx_table_gen5(struct switchtec_dev *dev,
						int port_id, int prev_speed,
						int link, void *res);

extern int switchtec_diag_port_eq_tx_fslf_gen5(struct switchtec_dev *dev,
					       int port_id, int prev_speed,
					       int lane_id, int end, int link,
					       void *res);

extern int switchtec_diag_loopback_set_gen5(struct switchtec_dev *dev,
					    int port_id, int enable,
					    int enable_parallel,
					    int enable_external,
					    int enable_ltssm, int enable_pipe,
					    int ltssm_speed);

extern int switchtec_diag_loopback_get_gen5(struct switchtec_dev *dev,
					    int port_id, int *enabled,
					    int *ltssm_speed);

extern int switchtec_diag_eye_read_gen5(struct switchtec_dev *dev, int lane_id,
		      	    int bin, int* num_phases, double* ber_data);

extern int switchtec_diag_eye_start_gen5(struct switchtec_dev *dev, int lane_mask[4],
					void *x_range, void *y_range,
					int step_interval, int capture_depth, int sar_sel,
					int intleav_sel, int hstep, int data_mode,
					int eye_mode, uint64_t refclk, int vstep);

/* Gen5 shares crosshair implementation with Gen4 */
extern int switchtec_diag_cross_hair_enable_gen4(struct switchtec_dev *dev,
						 int lane_id);

extern int switchtec_diag_cross_hair_disable_gen4(struct switchtec_dev *dev);

extern int switchtec_diag_cross_hair_get_gen4(struct switchtec_dev *dev,
					      int start_lane_id, int num_lanes,
					      void *res);

extern int switchtec_diag_pattern_gen_set_gen5(struct switchtec_dev *dev,
					       int port_id, int type,
					       int link_speed);

/* Gen5 reuses gen4 pattern functions except for gen_set */
extern int switchtec_diag_pattern_gen_get_gen4(struct switchtec_dev *dev,
					       int port_id, int *type);

extern int switchtec_diag_pattern_mon_set_gen4(struct switchtec_dev *dev,
					       int port_id, int type);

extern int switchtec_diag_pattern_mon_get_gen4(struct switchtec_dev *dev,
					       int port_id, int lane_id,
					       int *type,
					       unsigned long long *err_cnt);

extern int switchtec_diag_pattern_inject_gen4(struct switchtec_dev *dev,
					      int port_id, int err_cnt);

/* Gen5 reuses gen4 rcvr_obj */
extern int switchtec_diag_rcvr_obj_gen4(struct switchtec_dev *dev, int port_id,
					int lane_id, int link, void *res);

extern int switchtec_inject_err_tlp_lcrc_gen5(struct switchtec_dev *dev,
					      int phys_port_id, int enable,
					      uint8_t rate);

/* Gen5 reuses gen4 inject_err functions */
extern int switchtec_inject_err_tlp_seqnum_gen4(struct switchtec_dev *dev,
						int phys_port_id);

extern int switchtec_inject_err_ack_nack_gen4(struct switchtec_dev *dev,
					      int phys_port_id, uint16_t seq_num,
					      uint8_t count);

extern int switchtec_inject_err_cto_gen5(struct switchtec_dev *dev,
					 int phys_port_id);

extern int switchtec_inject_err_dllp_gen4(struct switchtec_dev *dev,
					  int phys_port_id, int data);

extern int switchtec_inject_err_dllp_crc_gen4(struct switchtec_dev *dev,
					      int phys_port_id, int enable,
					      uint16_t rate);
/**
 * @brief Gen5-specific operations vtable
 */
const struct switchtec_gen_ops switchtec_gen5_ops = {
	/* Diagnostics - NULL means use common implementation or not supported */
	.diag_cross_hair_enable = switchtec_diag_cross_hair_enable_gen4,
	.diag_cross_hair_disable = switchtec_diag_cross_hair_disable_gen4,
	.diag_cross_hair_get = switchtec_diag_cross_hair_get_gen4,
	.diag_eye_set_mode = NULL,
	.diag_eye_start = switchtec_diag_eye_start_gen5,
	.diag_eye_fetch = NULL,
	.diag_eye_cancel = NULL,
	.diag_eye_read = switchtec_diag_eye_read_gen5,
	.diag_loopback_set = switchtec_diag_loopback_set_gen5,
	.diag_loopback_get = switchtec_diag_loopback_get_gen5,
	.diag_pattern_gen_set = switchtec_diag_pattern_gen_set_gen5,
	.diag_pattern_gen_get = switchtec_diag_pattern_gen_get_gen4,
	.diag_pattern_mon_set = switchtec_diag_pattern_mon_set_gen4,
	.diag_pattern_mon_get = switchtec_diag_pattern_mon_get_gen4,
	.diag_pattern_inject = switchtec_diag_pattern_inject_gen4,
	.diag_ltssm_log = switchtec_diag_ltssm_log_gen5,
	.diag_ltssm_log_set = NULL,
	.diag_port_eq_tx_coeff = switchtec_diag_port_eq_tx_coeff_gen5,
	.diag_port_eq_tx_table = switchtec_diag_port_eq_tx_table_gen5,
	.diag_port_eq_tx_fslf = switchtec_diag_port_eq_tx_fslf_gen5,
	.diag_rcvr_obj = switchtec_diag_rcvr_obj_gen4,
	.diag_rcvr_ext = NULL,
	.diag_refclk_ctl = NULL,
	.inject_err_tlp_lcrc = switchtec_inject_err_tlp_lcrc_gen5,
	.inject_err_cto = switchtec_inject_err_cto_gen5,
	.inject_err_ack_nack = switchtec_inject_err_ack_nack_gen4,
	.inject_err_tlp_seqnum = switchtec_inject_err_tlp_seqnum_gen4,
	.inject_err_tlp_ecrc = NULL,
	.inject_err_dllp_crc = switchtec_inject_err_dllp_crc_gen4,
	.inject_err_dup_tlp = NULL,

	/* Manufacturing */
	.security_config_get = NULL,
	.security_config_set = NULL,
	.mailbox_to_file = NULL,
	.active_image_index_get = NULL,
	.active_image_index_set = NULL,
	.fw_exec = NULL,
	.boot_resume = NULL,
	.sn_ver_get = NULL,
	.secure_state_set = NULL,
	.kmsk_set = NULL,
	.debug_unlock = NULL,
	.debug_lock_update = NULL,

	/* Firmware */
	.fw_part_id_to_type = NULL,
	.fw_type_to_part_id = NULL,
	.fw_part_id_to_str = NULL,
};
