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
 * @brief Gen4-specific operations
 */

#include "../switchtec_priv.h"

extern int switchtec_diag_ltssm_log_gen4(struct switchtec_dev *dev,
				 int port, int *log_count, void *log_data);

extern int switchtec_diag_port_eq_tx_coeff_gen4(struct switchtec_dev *dev,
						int port_id, int prev_speed,
						int end, int link, void *res);

extern int switchtec_diag_port_eq_tx_table_gen4(struct switchtec_dev *dev,
						int port_id, int prev_speed,
						int link, void *res);

extern int switchtec_diag_port_eq_tx_fslf_gen4(struct switchtec_dev *dev,
					       int port_id, int prev_speed,
					       int lane_id, int end, int link,
					       void *res);
/**
 * @brief Gen4-specific operations vtable
 */
const struct switchtec_gen_ops switchtec_gen4_ops = {
	/* Diagnostics - NULL means use common implementation or not supported */
	.diag_cross_hair_enable = NULL,
	.diag_cross_hair_disable = NULL,
	.diag_cross_hair_get = NULL,
	.diag_eye_set_mode = NULL,
	.diag_eye_start = NULL,
	.diag_eye_fetch = NULL,
	.diag_eye_cancel = NULL,
	.diag_loopback_set = NULL,
	.diag_loopback_get = NULL,
	.diag_pattern_gen_set = NULL,
	.diag_pattern_gen_get = NULL,
	.diag_pattern_mon_set = NULL,
	.diag_pattern_mon_get = NULL,
	.diag_pattern_inject = NULL,
	.diag_ltssm_log = switchtec_diag_ltssm_log_gen4,
	.diag_ltssm_log_set = NULL,
	.diag_port_eq_tx_coeff = switchtec_diag_port_eq_tx_coeff_gen4,
	.diag_port_eq_tx_table = switchtec_diag_port_eq_tx_table_gen4,
	.diag_port_eq_tx_fslf = switchtec_diag_port_eq_tx_fslf_gen4,
	.diag_rcvr_obj = NULL,
	.diag_rcvr_ext = NULL,
	.diag_refclk_ctl = NULL,
	.inject_err_tlp_lcrc = NULL,
	.inject_err_tlp_seqnum = NULL,
	.inject_err_tlp_ecrc = NULL,
	.inject_err_dup_dllp = NULL,
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
