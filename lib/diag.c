/*
 * Microsemi Switchtec(tm) PCIe Management Library
 * Copyright (c) 2021, Microsemi Corporation
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
 * @brief Switchtec diagnostic functions
 */

#define SWITCHTEC_LIB_CORE
#define SWITCHTEC_LTSSM_MAX_LOGS 61

#include "switchtec_priv.h"
#include "switchtec/diag.h"
#include "switchtec/endian.h"
#include "switchtec/switchtec.h"
#include "switchtec/utils.h"

#include <errno.h>
#include <math.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>

/**
 * @brief Enable cross hair on specified lane
 * @param[in]  dev	Switchtec device handle
 * @param[in]  lane_id	Lane to enable, or SWITCHTEC_DIAG_CROSS_HAIR_ALL_LANES
 *			for all lanes.
 *
 * @return 0 on success, error code on failure
 */
int switchtec_diag_cross_hair_enable(struct switchtec_dev *dev, int lane_id)
{
	struct switchtec_diag_cross_hair_in in = {
		.sub_cmd = MRPC_CROSS_HAIR_ENABLE,
		.lane_id = lane_id,
		.all_lanes = lane_id == SWITCHTEC_DIAG_CROSS_HAIR_ALL_LANES,
	};

	return switchtec_cmd(dev, MRPC_CROSS_HAIR, &in, sizeof(in), NULL, 0);
}

/**
 * @brief Disable active cross hair
 * @param[in]  dev	Switchtec device handle
 *
 * @return 0 on success, error code on failure
 */
int switchtec_diag_cross_hair_disable(struct switchtec_dev *dev)
{
	struct switchtec_diag_cross_hair_in in = {
		.sub_cmd = MRPC_CROSS_HAIR_DISABLE,
	};

	return switchtec_cmd(dev, MRPC_CROSS_HAIR, &in, sizeof(in), NULL, 0);
}

/**
 * @brief Disable active cross hair
 * @param[in]  dev		Switchtec device handle
 * @param[in]  start_lane_id	Start lane ID to get
 * @param[in]  num_lanes	Number of lanes to get
 * @param[out] res		Resulting cross hair data
 *
 * @return 0 on success, error code on failure
 */
int switchtec_diag_cross_hair_get(struct switchtec_dev *dev, int start_lane_id,
		int num_lanes, struct switchtec_diag_cross_hair *res)
{
	struct switchtec_diag_cross_hair_in in = {
		.sub_cmd = MRPC_CROSS_HAIR_GET,
		.lane_id = start_lane_id,
		.num_lanes = num_lanes,
	};
	struct switchtec_diag_cross_hair_get out[num_lanes];
	int i, ret;

	ret = switchtec_cmd(dev, MRPC_CROSS_HAIR, &in, sizeof(in), &out,
			    sizeof(out));
	if (ret)
		return ret;

	for (i = 0; i < num_lanes; i++) {
		memset(&res[i], 0, sizeof(res[i]));
		res[i].state = out[i].state;
		res[i].lane_id = out[i].lane_id;

		if (out[i].state <= SWITCHTEC_DIAG_CROSS_HAIR_WAITING) {
			continue;
		} else if (out[i].state < SWITCHTEC_DIAG_CROSS_HAIR_DONE) {
			res[i].x_pos = out[i].x_pos;
			res[i].y_pos = out[i].y_pos;
		} else if (out[i].state == SWITCHTEC_DIAG_CROSS_HAIR_DONE) {
			res[i].eye_left_lim = out[i].eye_left_lim;
			res[i].eye_right_lim = out[i].eye_right_lim;
			res[i].eye_bot_left_lim = out[i].eye_bot_left_lim;
			res[i].eye_bot_right_lim = out[i].eye_bot_right_lim;
			res[i].eye_top_left_lim = out[i].eye_top_left_lim;
			res[i].eye_top_right_lim = out[i].eye_top_right_lim;
		} else if (out[i].state == SWITCHTEC_DIAG_CROSS_HAIR_ERROR) {
			res[i].x_pos = out[i].x_pos;
			res[i].y_pos = out[i].y_pos;
			res[i].prev_state = out[i].prev_state;
		}
	}

	return 0;
}

/**
 * @brief Set the data mode for the next Eye Capture
 * @param[in]  dev	       Switchtec device handle
 * @param[in]  mode	       Mode to use (raw or ratio)
 *
 * @return 0 on success, error code on failure
 */
int switchtec_diag_eye_set_mode(struct switchtec_dev *dev,
				enum switchtec_diag_eye_data_mode mode)
{
	if (GEN_OPS(dev) && GEN_OPS(dev)->diag_eye_set_mode)
		return GEN_OPS(dev)->diag_eye_set_mode(dev, mode);
	errno = ENOTSUP;
	return -1;	
}

/**
 * @brief Start a PCIe Eye Read Gen5
 * @param[in]  dev	       Switchtec device handle
 * @param[in]  lane_id         lane_id
 * @param[in]  bin             bin
 * @param[in]  num_phases      pointer to the number of phases
 * @param[in]  ber_data        pointer to the Ber data
 *
 * @return 0 on success, error code on failure
 */
int switchtec_diag_eye_read(struct switchtec_dev *dev, int lane_id,
		      	    int bin, int* num_phases, double* ber_data)
{
	if (GEN_OPS(dev) && GEN_OPS(dev)->diag_eye_read)
		return GEN_OPS(dev)->diag_eye_read(dev, lane_id, bin, num_phases, ber_data);
	errno = ENOTSUP;
	return -1;
}

/**
 * @brief Start a PCIe Eye Capture
 * @param[in]  dev	       Switchtec device handle
 * @param[in]  lane_mask       Bitmap of the lanes to capture
 * @param[in]  x_range         Time range: start should be between 0 and 63,
 *			       end between start and 63.
 * @param[in]  y_range         Voltage range: start should be between -255 and 255,
 *			       end between start and 255.
 * @param[in]  step_interval   Sampling time in milliseconds for each step
 *
 * @return 0 on success, error code on failure
 */
int switchtec_diag_eye_start(struct switchtec_dev *dev, int lane_mask[4],
			     struct range *x_range, struct range *y_range,
			     int step_interval, int capture_depth, int sar_sel,
			     int intleav_sel, int hstep, int data_mode, 
			     int eye_mode, uint64_t refclk, int vstep)
{
	if (GEN_OPS(dev) && GEN_OPS(dev)->diag_eye_start)
		return GEN_OPS(dev)->diag_eye_start(dev, lane_mask, x_range, 
						y_range, step_interval, 
						capture_depth, sar_sel, 
						intleav_sel, hstep, data_mode, 
						eye_mode, refclk, vstep);
	errno = ENOTSUP;
	return -1;
}

/**
 * @brief Start a PCIe Eye Capture
 * @param[in]  dev	       Switchtec device handle
 * @param[out] pixels          Resulting pixel data
 * @param[in]  pixel_cnt       Space in pixel array
 * @param[out] lane_id         The lane for the resulting pixels
 *
 * @return number of pixels fetched on success, error code on failure
 *
 * pixel_cnt needs to be greater than 62 in raw mode or 496 in ratio
 * mode, otherwise data will be lost and the number of pixels fetched
 * will be greater than the space in the pixel buffer.
 */
int switchtec_diag_eye_fetch(struct switchtec_dev *dev, double *pixels,
			     size_t pixel_cnt, int *lane_id)
{
	if (GEN_OPS(dev) && GEN_OPS(dev)->diag_eye_fetch)
		return GEN_OPS(dev)->diag_eye_fetch(dev, pixels, pixel_cnt, lane_id);
	errno = ENOTSUP;
	return -1;
}

/**
 * @brief Cancel in-progress eye capture
 * @param[in]  dev	       Switchtec device handle
 *
 * @return 0 on success, error code on failure
 */
int switchtec_diag_eye_cancel(struct switchtec_dev *dev)
{
	if (GEN_OPS(dev) && GEN_OPS(dev)->diag_eye_cancel)
		return GEN_OPS(dev)->diag_eye_cancel(dev);
	errno = ENOTSUP;
	return -1;
}

/**
 * @brief Setup Loopback Mode
 * @param[in]  dev	    Switchtec device handle
 * @param[in]  port_id	    Physical port ID
 * @param[in]  enable		Enable bitmap - Gen 4
 * @param[in]  enable_parallel	Enable the parallel SERDES loopback - Gen 5
 * @param[in]  enable_external	Enable the external physical loopback - Gen 5
 * @param[in]  enable_ltssm	Enable the ltssm loopback
 * @param[in]  ltssm_speed  LTSSM loopback max speed
 *
 * @return 0 on success, error code on failure
 */
int switchtec_diag_loopback_set(struct switchtec_dev *dev, int port_id,
				int enable, int enable_parallel,
				int enable_external, int enable_ltssm,
				int enable_pipe,
				enum switchtec_diag_ltssm_speed ltssm_speed)
{
	if (GEN_OPS(dev) && GEN_OPS(dev)->diag_loopback_set)
		return GEN_OPS(dev)->diag_loopback_set(dev, port_id, enable,
						       enable_parallel,
						       enable_external,
						       enable_ltssm,
						       enable_pipe, ltssm_speed);
	errno = ENOTSUP;
	return -1;
}

/**
 * @brief Get Loopback Mode
 * @param[in]  dev	     Switchtec device handle
 * @param[in]  port_id	     Physical port ID
 * @param[out] enabled       Set of enum switchtec_diag_loopback_enable
 *			     indicating which loopback modes are enabled
 * @param[out] ltssm_speed   LTSSM loopback max speed
 *
 * @return 0 on succes, error code on failure
 */
int switchtec_diag_loopback_get(struct switchtec_dev *dev,
				int port_id, int *enabled,
				enum switchtec_diag_ltssm_speed *ltssm_speed)
{
	if (GEN_OPS(dev) && GEN_OPS(dev)->diag_loopback_get)
		return GEN_OPS(dev)->diag_loopback_get(dev, port_id, enabled,
						       (int *)ltssm_speed);
	errno = ENOTSUP;
	return -1;
}

/**
 * @brief Setup Pattern Generator
 * @param[in]  dev	 Switchtec device handle
 * @param[in]  port_id	 Physical port ID
 * @param[in]  type      Pattern type to enable
 *
 * @return 0 on success, error code on failure
 */
int switchtec_diag_pattern_gen_set(struct switchtec_dev *dev, int port_id,
				   enum switchtec_diag_pattern type,
				   enum switchtec_diag_pattern_link_rate link_speed)
{
	struct switchtec_diag_pat_gen_in in = {
		.sub_cmd = MRPC_PAT_GEN_SET_GEN,
		.port_id = port_id,
		.pattern_type = type,
		.lane_id = link_speed
	};
	if (switchtec_is_gen5(dev))
		in.sub_cmd = MRPC_PAT_GEN_SET_GEN_GEN5;
	if (switchtec_is_gen6(dev))
		in.sub_cmd = MRPC_PAT_GEN_SET_GEN_GEN6;

	return switchtec_cmd(dev, MRPC_PAT_GEN, &in, sizeof(in), NULL, 0);
}

/**
 * @brief Get Pattern Generator set on port
 * @param[in]  dev	 Switchtec device handle
 * @param[in]  port_id	 Physical port ID
 * @param[out] type      Pattern type to enable
 *
 * @return 0 on success, error code on failure
 */
int switchtec_diag_pattern_gen_get(struct switchtec_dev *dev, int port_id,
				   enum switchtec_diag_pattern *type)
{
	struct switchtec_diag_pat_gen_in in = {
		.sub_cmd = MRPC_PAT_GEN_GET_GEN,
		.port_id = port_id,
	};
	if (switchtec_is_gen6(dev))
		in.sub_cmd = MRPC_PAT_GEN_GET_GEN_GEN6;
	struct switchtec_diag_pat_gen_out out;
	int ret;

	ret = switchtec_cmd(dev, MRPC_PAT_GEN, &in, sizeof(in), &out,
			    sizeof(out));
	if (ret)
		return ret;

	if (type)
		*type = out.pattern_type;

	return 0;
}

/**
 * @brief Setup Pattern Monitor
 * @param[in]  dev	 Switchtec device handle
 * @param[in]  port_id	 Physical port ID
 * @param[in]  type      Pattern type to enable
 *
 * @return 0 on success, error code on failure
 */
int switchtec_diag_pattern_mon_set(struct switchtec_dev *dev, int port_id,
				   enum switchtec_diag_pattern type)
{
	struct switchtec_diag_pat_gen_in in = {
		.sub_cmd = MRPC_PAT_GEN_SET_MON,
		.port_id = port_id,
		.pattern_type = type,
	};
	if (switchtec_is_gen6(dev))
		in.sub_cmd = MRPC_PAT_GEN_SET_MON_GEN6;

	return switchtec_cmd(dev, MRPC_PAT_GEN, &in, sizeof(in), NULL, 0);
}

/**
 * @brief Get Pattern Monitor
 * @param[in]  dev	 Switchtec device handle
 * @param[in]  port_id	 Physical port ID
 * @param[out] type      Pattern type to enable
 * @param[out] err_cnt   Number of errors seen
 *
 * @return 0 on success, error code on failure
 */
int switchtec_diag_pattern_mon_get(struct switchtec_dev *dev, int port_id,
				   int lane_id, enum switchtec_diag_pattern *type,
				   unsigned long long *err_cnt)
{
	struct switchtec_diag_pat_gen_in in = {
		.sub_cmd = MRPC_PAT_GEN_GET_MON,
		.port_id = port_id,
		.lane_id = lane_id,
	};
	if (switchtec_is_gen6(dev))
		in.sub_cmd = MRPC_PAT_GEN_GET_MON_GEN6;
	struct switchtec_diag_pat_gen_out out;
	int ret;

	ret = switchtec_cmd(dev, MRPC_PAT_GEN, &in, sizeof(in), &out,
			    sizeof(out));
	if (ret)
		return ret;

	if (type)
		*type = out.pattern_type;

	if (err_cnt)
		*err_cnt = (htole32(out.err_cnt_lo) |
			    ((uint64_t)htole32(out.err_cnt_hi) << 32));

	return 0;
}

/**
 * @brief Inject error into pattern generator
 * @param[in]  dev	 Switchtec device handle
 * @param[in]  port_id	 Physical port ID
 * @param[in] err_cnt   Number of errors seen
 *
 * Injects up to err_cnt errors into each lane of the TX port. It's
 * recommended that the err_cnt be less than 1000, otherwise the
 * firmware runs the risk of consuming too many resources and crashing.
 *
 * @return 0 on success, error code on failure
 */
int switchtec_diag_pattern_inject(struct switchtec_dev *dev, int port_id,
				  unsigned int err_cnt)
{
	struct switchtec_diag_pat_gen_inject in = {
		.sub_cmd = MRPC_PAT_GEN_INJ_ERR,
		.port_id = port_id,
		.err_cnt = err_cnt,
	};
	int ret;

	ret = switchtec_cmd(dev, MRPC_PAT_GEN, &in, sizeof(in), NULL, 0);
	if (ret)
		return ret;

	return 0;
}

/**
 * @brief Get the receiver object
 * @param[in]  dev	Switchtec device handle
 * @param[in]  port_id	Physical port ID
 * @param[in]  lane_id  Lane ID
 * @param[in]  link     Current or previous link-up
 * @param[out] res      Resulting receiver object
 *
 * @return 0 on success, error code on failure
 */
int switchtec_diag_rcvr_obj(struct switchtec_dev *dev, int port_id,
		int lane_id, enum switchtec_diag_link link,
		struct switchtec_rcvr_obj *res)
{
	struct switchtec_diag_rcvr_obj_dump_out out = {};
	struct switchtec_diag_rcvr_obj_dump_in in = {
		.port_id = port_id,
		.lane_id = lane_id,
	};
	struct switchtec_diag_ext_recv_obj_dump_in ext_in = {
		.sub_cmd = MRPC_EXT_RCVR_OBJ_DUMP_PREV,
		.port_id = port_id,
		.lane_id = lane_id,
	};
	int i, ret;

	if (!res) {
		errno = -EINVAL;
		return -1;
	}

	if (link == SWITCHTEC_DIAG_LINK_CURRENT) {
		ret = switchtec_cmd(dev, MRPC_RCVR_OBJ_DUMP, &in, sizeof(in),
				    &out, sizeof(out));
	} else if (link == SWITCHTEC_DIAG_LINK_PREVIOUS) {
		ret = switchtec_cmd(dev, MRPC_EXT_RCVR_OBJ_DUMP, &ext_in,
				    sizeof(ext_in), &out, sizeof(out));
	} else {
		errno = -EINVAL;
		return -1;
	}

	if (ret)
		return -1;

	res->port_id = out.port_id;
	res->lane_id = out.lane_id;
	res->ctle = out.ctle;
	res->target_amplitude = out.target_amplitude;
	res->speculative_dfe = out.speculative_dfe;
	for (i = 0; i < ARRAY_SIZE(res->dynamic_dfe); i++)
		res->dynamic_dfe[i] = out.dynamic_dfe[i];

	return 0;
}

/**
 * @brief Get the Gen5 port equalization TX coefficients
 * @param[in]  dev	Switchtec device handle
 * @param[in]  port_id	Physical port ID
 * @param[in]  end      Get coefficents for the Local or the Far End
 * @param[out] res      Resulting port equalization coefficients
 *
 * @return 0 on success, error code on failure
 */
/**
 * @brief Get the port equalization TX coefficients
 * @param[in]  dev	Switchtec device handle
 * @param[in]  port_id	Physical port ID
 * @param[in]  end      Get coefficents for the Local or the Far End
 * @param[out] res      Resulting port equalization coefficients
 *
 * @return 0 on success, error code on failure
 */
int switchtec_diag_port_eq_tx_coeff(struct switchtec_dev *dev, int port_id,
				    int prev_speed, enum switchtec_diag_end end,
				    enum switchtec_diag_link link,
				    struct switchtec_port_eq_coeff *res)
{
	if (GEN_OPS(dev) && GEN_OPS(dev)->diag_port_eq_tx_coeff)
		return GEN_OPS(dev)->diag_port_eq_tx_coeff(dev, port_id, 
							   prev_speed, end, 
							   link, res);
	errno = ENOTSUP;
	return -1;
}

/**
 * @brief Get the far end TX equalization table
 * @param[in]  dev	Switchtec device handle
 * @param[in]  port_id	Physical port ID
 * @param[out] res      Resulting port equalization table
 *
 * @return 0 on success, error code on failure
 */
int switchtec_diag_port_eq_tx_table(struct switchtec_dev *dev, int port_id,
				    int prev_speed, enum switchtec_diag_link link,
				    struct switchtec_port_eq_table *res)
{
	if (GEN_OPS(dev) && GEN_OPS(dev)->diag_port_eq_tx_table)
		return GEN_OPS(dev)->diag_port_eq_tx_table(dev, port_id, 
							   prev_speed, link, 
							   res);
	errno = ENOTSUP;
	return -1;
}

/**
 * @brief Get the equalization FS/LF
 * @param[in]  dev	Switchtec device handle
 * @param[in]  port_id	Physical port ID
 * @param[in]  lane_id	Physical port ID
 * @param[in]  end      Get coefficents for the Local or the Far End
 * @param[out] res      Resulting FS/LF values
 *
 * @return 0 on success, error code on failure
 */
int switchtec_diag_port_eq_tx_fslf(struct switchtec_dev *dev, int port_id,
				   int prev_speed, int lane_id,
				   enum switchtec_diag_end end,
				   enum switchtec_diag_link link,
				   struct switchtec_port_eq_tx_fslf *res)
{
	if (GEN_OPS(dev) && GEN_OPS(dev)->diag_port_eq_tx_fslf)
		return GEN_OPS(dev)->diag_port_eq_tx_fslf(dev, port_id, 
							  prev_speed, lane_id, 
							  end, link, res);
	errno = ENOTSUP;
	return -1;
}

/**
 * @brief Get the Extended Receiver Object
 * @param[in]  dev 	Switchtec device handle
 * @param[in]  port_id	Physical port ID
 * @param[in]  lane_id  Lane ID
 * @param[in]  link     Current or previous link-up
 * @param[out] res      Resulting receiver object
 *
 * @return 0 on success, error code on failure
 */
int switchtec_diag_rcvr_ext(struct switchtec_dev *dev, int port_id,
			    int lane_id, enum switchtec_diag_link link,
			    struct switchtec_rcvr_ext *res)
{
	struct switchtec_diag_rcvr_ext_out out = {};
	struct switchtec_diag_ext_recv_obj_dump_in in = {
		.port_id = port_id,
		.lane_id = lane_id,
	};
	int ret;

	if (!res) {
		errno = -EINVAL;
		return -1;
	}

	if (link == SWITCHTEC_DIAG_LINK_CURRENT) {
		in.sub_cmd = MRPC_EXT_RCVR_OBJ_DUMP_RCVR_EXT;
	} else if (link == SWITCHTEC_DIAG_LINK_PREVIOUS) {
		in.sub_cmd = MRPC_EXT_RCVR_OBJ_DUMP_RCVR_EXT_PREV;
	} else {
		errno = -EINVAL;
		return -1;
	}

	ret = switchtec_cmd(dev, MRPC_EXT_RCVR_OBJ_DUMP, &in, sizeof(in),
			    &out, sizeof(out));
	if (ret)
		return -1;

	res->ctle2_rx_mode = out.ctle2_rx_mode;
	res->dtclk_9 = out.dtclk_9;
	res->dtclk_8_6 = out.dtclk_8_6;
	res->dtclk_5 = out.dtclk_5;

	return 0;
}

/**
 * @brief Get the permission table
 * @param[in]  dev	Switchtec device handle
 * @param[out] table    Resulting MRPC permission table
 *
 * @return 0 on success, error code on failure
 */
int switchtec_diag_perm_table(struct switchtec_dev *dev,
			      struct switchtec_mrpc table[MRPC_MAX_ID])
{
	uint32_t perms[(MRPC_MAX_ID + 31) / 32];
	int i, ret;

	ret = switchtec_cmd(dev, MRPC_MRPC_PERM_TABLE_GET, NULL, 0,
			    perms, sizeof(perms));
	if (ret)
		return -1;

	for (i = 0; i < MRPC_MAX_ID; i++) {
		if (perms[i >> 5] & (1 << (i & 0x1f))) {
			if (switchtec_mrpc_table[i].tag) {
				table[i] = switchtec_mrpc_table[i];
			} else {
				table[i].tag = "UNKNOWN";
				table[i].desc = "Unknown MRPC Command";
				table[i].reserved = true;
			}
		} else {
			table[i].tag = NULL;
			table[i].desc = NULL;
		}
	}

	return 0;
}

/**
 * @brief Control the refclk output for a stack
 * @param[in]  dev	Switchtec device handle
 * @param[in]  stack_id	Stack ID to control the refclk of
 * @param[in]  en	Set to true to enable, false to disable
 *
 * @return 0 on success, error code on failure
 */
int switchtec_diag_refclk_ctl(struct switchtec_dev *dev, int stack_id, bool en)
{
	struct switchtec_diag_refclk_ctl_in cmd = {
		.sub_cmd = en ? MRPC_REFCLK_S_ENABLE : MRPC_REFCLK_S_DISABLE,
		.stack_id = stack_id,
	};

	return switchtec_cmd(dev, MRPC_REFCLK_S, &cmd, sizeof(cmd), NULL, 0);
}
/**
 * @brief Get the status of all stacks of the refclk 
 * @param[in]  dev		Switchtec device handle
 * @param[in]  stack_info	Pointer to the stack information
 *
 * @return 0 on success, error code on failure
 */
int switchtec_diag_refclk_status(struct switchtec_dev *dev, uint8_t *stack_info)
{
	struct switchtec_diag_refclk_ctl_in cmd = {
		.sub_cmd = MRPC_REFCLK_S_STATUS,
	};

	return switchtec_cmd(dev, MRPC_REFCLK_S, &cmd, sizeof(cmd), stack_info, 
			     sizeof(uint8_t) * SWITCHTEC_MAX_STACKS);
}

/**
 * @brief Determine the generation and call the related LTSSM log func
 * @param[in]	dev    Switchtec device handle
 * @param[in]	port   Switchtec Port
 * @param[inout] log_count number of log entries
 * @param[out] log    A pointer to an array containing the log
 */
int switchtec_diag_ltssm_log(struct switchtec_dev *dev,
			    int port, int *log_count,
			    struct switchtec_diag_ltssm_log *log_data)
{
	if (GEN_OPS(dev) && GEN_OPS(dev)->diag_ltssm_log)
		return GEN_OPS(dev)->diag_ltssm_log(dev, port, log_count, log_data);
	errno = ENOTSUP;
	return -1;
}

/**
 * @brief Call the LTSSM clear MRPC command
 * @param[in]	dev    Switchtec device handle
 * @param[in]	port   Switchtec Port
 */
int switchtec_diag_ltssm_clear(struct switchtec_dev *dev, int port)
{
	int ret;
	struct {
		uint8_t subcmd;
		uint8_t port_id;
		uint16_t reserved;
	} ltssm_clear;

	ltssm_clear.subcmd = MRPC_LTMON_CLEAR_LOG;
	ltssm_clear.port_id = port;

	ret = switchtec_cmd(dev, MRPC_DIAG_PORT_LTSSM_LOG, &ltssm_clear,
			    sizeof(ltssm_clear), NULL, 0);
	return ret;
}

int switchtec_tlp_inject(struct switchtec_dev *dev, int port_id, int tlp_type,
			 int tlp_length, int ecrc, uint32_t *raw_tlp_data)
{
	uint32_t tlp_out;
	int ret = 1;
	struct switchtec_tlp_inject_in tlp_in = {
		.dest_port = port_id,
		.tlp_type = tlp_type,
		.tlp_length = tlp_length,
		.ecrc = ecrc
	};
	for (int i = 0; i < tlp_in.tlp_length; i++) {
		tlp_in.raw_tlp_data[i] = htole32(*(raw_tlp_data + i));
	}
	free(raw_tlp_data);

	ret = switchtec_cmd(dev, MRPC_DIAG_TLP_INJECT, &tlp_in, sizeof(tlp_in),
			    &tlp_out, sizeof(tlp_out));
	return ret;
}

/**
 * @brief Call the aer event gen function to generate AER events
 * @param[in]   dev    Switchtec device handle
 * @param[in]   port   Switchtec Port
 * @param[in]   aer_error_id aer error bit
 * @param[out]  trigger_event One of the trigger events
 */
int switchtec_aer_event_gen(struct switchtec_dev *dev, int port_id,
			    int aer_error_id, int trigger_event)
{
	uint32_t output;
	int ret_val;

	struct switchtec_aer_event_gen_in sub_cmd_id = {
		.sub_cmd = trigger_event,
		.phys_port_id = port_id,
		.err_mask = aer_error_id,
		.hdr_log[0] = 0,
		.hdr_log[1] = 0,
		.hdr_log[2] = 0,
		.hdr_log[3] = 0
	};

	ret_val = switchtec_cmd(dev, MRPC_AER_GEN, &sub_cmd_id,
				sizeof(sub_cmd_id), &output, sizeof(output));
	return ret_val;
}

/**
 * @brief Inject a DLLP into a physical port
 * @param[in] dev	Switchtec device handle
 * @param[in] phys_port_id Physical port id
 * @param[in] data	DLLP data
 * @return 0 on success, or a negative value on failure
 */
int switchtec_inject_err_dllp(struct switchtec_dev *dev, int phys_port_id,
			      int data)
{
	uint32_t output;

	struct switchtec_lnkerr_dllp_in cmd = {
		.subcmd = MRPC_ERR_INJ_DLLP,
		.phys_port_id = phys_port_id,
		.data = data,
	};

	return switchtec_cmd(dev, MRPC_MRPC_ERR_INJ, &cmd,
			     sizeof(cmd), &output, sizeof(output));
}

/**
 * @brief Inject a DLLP CRC error into a physical port
 * @param[in] dev	Switchtec device handle
 * @param[in] phys_port_id Physical port id
 * @param[in] enable	Enable DLLP CRC error injection
 * @param[in] rate 	Rate of the error injection
 * @return 0 on success, or a negative value on failure
 */
int switchtec_inject_err_dllp_crc(struct switchtec_dev *dev,
				  int phys_port_id, int enable,
				  uint16_t rate)
{
	uint32_t output;

	struct switchtec_lnkerr_dllp_crc_in cmd = {
		.subcmd = MRPC_ERR_INJ_DLLP_CRC,
		.phys_port_id = phys_port_id,
		.enable = enable,
		.rate = rate,
	};

	return switchtec_cmd(dev, MRPC_MRPC_ERR_INJ, &cmd,
			     sizeof(cmd), &output, sizeof(output));
}

static int switchtec_inject_err_tlp_lcrc_gen4(struct switchtec_dev *dev,
					      int phys_port_id, int enable,
					      uint8_t rate)
{
	uint32_t output;

	struct switchtec_lnkerr_tlp_lcrc_gen4_in cmd = {
		.subcmd = MRPC_ERR_INJ_TLP_LCRC,
		.phys_port_id = phys_port_id,
		.enable = enable,
		.rate = rate,
	};
	printf("enable: %d\n", enable);

	return switchtec_cmd(dev, MRPC_MRPC_ERR_INJ, &cmd,
			     sizeof(cmd), &output, sizeof(output));
}

static int switchtec_inject_err_tlp_lcrc_gen5(struct switchtec_dev *dev,
					      int phys_port_id, int enable,
					      uint8_t rate)
{
	uint32_t output;

	struct switchtec_lnkerr_tlp_lcrc_gen5_in cmd = {
		.subcmd = MRPC_ERR_INJ_TLP_LCRC,
		.phys_port_id = phys_port_id,
		.enable = enable,
		.rate = rate,
	};

	return switchtec_cmd(dev, MRPC_MRPC_ERR_INJ, &cmd,
			     sizeof(cmd), &output, sizeof(output));
}

/**
 * @brief Inject a TLP LCRC error into a physical port
 * @param[in] dev	Switchtec device handle
 * @param[in] phy_port Physical port id
 * @param[in] rate	Rate of the error injection
 * @return 0 on success, or a negative value on failure
 */
int switchtec_inject_err_tlp_lcrc(struct switchtec_dev *dev, int phy_port,
				  int enable, uint8_t rate)
{
	int ret;
	if (switchtec_is_gen4(dev)) {
		ret = switchtec_inject_err_tlp_lcrc_gen4(dev, phy_port, enable, rate);
		return ret;
	} else if (switchtec_is_gen5(dev)) {
		ret = switchtec_inject_err_tlp_lcrc_gen5(dev, phy_port, enable, rate);
		return ret;
	}
	fprintf(stderr, "The TLP LCRC is not supported for Gen3 switches.\n");
	return -1;
}

/**
 * @brief Inject a TLP Sequence Number error into a physical port
 * @param[in] dev	Switchtec device handle
 * @param[in] phys_port_id Physical port id
 * @return 0 on success, or a negative value on failure
 */
int switchtec_inject_err_tlp_seq_num(struct switchtec_dev *dev, int phys_port_id)
{
	uint32_t output;

	struct switchtec_lnkerr_tlp_seqn_in cmd = {
		.subcmd = MRPC_ERR_INJ_TLP_SEQ,
		.phys_port_id = phys_port_id,
	};

	return switchtec_cmd(dev, MRPC_MRPC_ERR_INJ, &cmd,
			     sizeof(cmd), &output, sizeof(output));
}

/**
 * @brief Inject an ACK to NACK error into a physical port
 * @param[in] dev	Switchtec device handle
 * @param[in] phys_port_id Physical port id
 * @param[in] seq_num	Sequence Number of ACK to be changed to a NACK (0-4095)
 * @param[in] count		Number of times to replace ACK with NACK (0-255)
 * @return 0 on success, or a negative value on failure
 */
int switchtec_inject_err_ack_nack(struct switchtec_dev *dev, int phys_port_id,
				  uint16_t seq_num, uint8_t count)
{
	uint32_t output;

	struct switchtec_lnkerr_ack_nack_in cmd = {
		.subcmd = MRPC_ERR_INJ_ACK_NACK,
		.phys_port_id = phys_port_id,
		.seq_num = seq_num,
		.count = count,
	};

	return switchtec_cmd(dev, MRPC_MRPC_ERR_INJ, &cmd,
			     sizeof(cmd), &output, sizeof(output));
}

/**
 * @brief Inject Credit Timeout error into a physical port
 * @param[in] dev	Switchtec device handle
 * @param[in] phys_port_id Physical port id
 * @return 0 on success, or a negative value on failure
 */
int switchtec_inject_err_cto(struct switchtec_dev *dev, int phys_port_id)
{
	uint32_t output;

	struct switchtec_lnkerr_cto_in cmd = {
		.subcmd = MRPC_ERR_INJ_CTO,
		.phys_port_id = phys_port_id,
	};

	return switchtec_cmd(dev, MRPC_MRPC_ERR_INJ, &cmd,
			     sizeof(cmd), &output, sizeof(output));
}

int switchtec_osa_capture_data(struct switchtec_dev *dev, int stack_id,
			       int lane, int direction)
{
	int ret = 0;
	struct {
		uint8_t sub_cmd;
		uint8_t stack_id;
		uint8_t lane;
		uint8_t direction;
		uint16_t start_entry;
		uint8_t num_entries;
		uint8_t reserved;
	} osa_data_read_in;

	struct {
		uint8_t entries_read;
		uint8_t stack_id;
		uint8_t lane;
		uint8_t direction;
		uint16_t next_entry;
		uint16_t entries_remaining;
		uint16_t wrap;
		uint16_t reserved;
	} osa_data_entries_out;

	osa_data_read_in.sub_cmd = MRPC_OSA_DATA_READ;
	osa_data_read_in.stack_id = stack_id;
	osa_data_read_in.lane = lane;
	osa_data_read_in.direction = direction;

	osa_data_read_in.start_entry = 0;
	osa_data_read_in.num_entries = 0;

	struct {
		uint8_t sub_cmd;
		uint8_t stack_id;
		uint16_t reserved;
	} osa_status_query_in;

	struct {
		uint8_t state;
		uint8_t trigger_lane;
		uint8_t trigger_dir;
		uint8_t reserved;
		uint16_t trigger_reason;
		uint16_t reserved2;
	} osa_status_query_out;

	osa_status_query_in.sub_cmd = MRPC_OSA_STATUS_QUERY;
	if (switchtec_is_gen6(dev))
		osa_status_query_in.sub_cmd = MRPC_OSA_STATUS_QUERY_GEN6;
	osa_status_query_in.stack_id = stack_id;

	ret = switchtec_cmd(dev, MRPC_ORDERED_SET_ANALYZER, &osa_status_query_in,
			    sizeof(osa_status_query_in), &osa_status_query_out,
			    sizeof(osa_status_query_out));

	ret = switchtec_cmd(dev, MRPC_ORDERED_SET_ANALYZER, &osa_data_read_in,
			    sizeof(osa_data_read_in), &osa_data_entries_out,
			    sizeof(osa_data_entries_out));
	if (ret) {
		switchtec_perror("OSA data dump");
		return ret;
	}
	printf("OSA: Captured Data:\n");

	struct {
		uint8_t entries_read;
		uint8_t stack_id;
		uint8_t lane;
		uint8_t direction;
		uint16_t next_entry;
		uint16_t entries_remaining;
		uint16_t wrap;
		uint16_t reserved;
		uint32_t entry_dwords[osa_data_entries_out.entries_remaining * 6];
	} osa_data_read_out;

	osa_data_read_out.entries_remaining = osa_data_entries_out.entries_remaining;
	osa_data_read_out.next_entry = osa_data_entries_out.next_entry;

	int total_dword = 0;
	int total_entries = 0;
	int curr_entry_dword = 0;
	char osa_data[100] = "";
	char buffer[50] = "";
	uint64_t timestamp;
	uint32_t timestamp_lower, timestamp_upper, counter;
	uint8_t link_rate, trigger, os_droppped;

	char *link_rate_str[5] = {"Gen1", "Gen2", "Gen3", "Gen4", "Gen5"};

	printf("Entry\tTimestamp\tLink Rate\tCounter\t\tTrigger Indication\tOS Dropped?\tOSA Data\n");
	while (osa_data_read_out.entries_remaining != 0) {
		osa_data_read_in.num_entries = osa_data_read_out.entries_remaining;
		osa_data_read_in.start_entry = osa_data_read_out.next_entry;

		ret = switchtec_cmd(dev, MRPC_ORDERED_SET_ANALYZER,
				    &osa_data_read_in, sizeof(osa_data_read_in),
				    &osa_data_read_out, sizeof(osa_data_read_out));
		if (ret) {
			return -1;
		}
		for (int i = total_dword; i < total_dword + (osa_data_read_out.entries_read * 6); i++) {
			if (curr_entry_dword < 4) {
				snprintf(buffer, sizeof(buffer), "0x%08x ", osa_data_read_out.entry_dwords[i]);
				strncat(osa_data, buffer, sizeof(osa_data) - strlen(osa_data) - 1);
			} else if (curr_entry_dword == 4) {
				timestamp_lower = (osa_data_read_out.entry_dwords[i] >> 22) & 0x3FF;
				timestamp_upper = (osa_data_read_out.entry_dwords[i+1] & 0x1A);
				timestamp = timestamp_upper | timestamp_lower;
				
				link_rate = osa_data_read_out.entry_dwords[i] & 0x3;
				counter = (osa_data_read_out.entry_dwords[i] >> 3) & 0x12;
				trigger	= (osa_data_read_out.entry_dwords[i+1] >> 28) & 0x1;
				os_droppped = (osa_data_read_out.entry_dwords[i+1] >> 29) & 0x1;
				
				printf("%d\t", total_entries);
				printf("%ld\t\t", timestamp);
				printf("%s\t\t", link_rate_str[link_rate]);
				printf("%d\t\t", counter);
				printf("%s\t\t", trigger ? "Pre-Trigger" : "Post-Trigger");
				printf("%s\t\t", os_droppped ? "Yes" : "No");
				printf("%s\n", osa_data);
				osa_data[0] = '\0';
				buffer[0] = '\0';
				total_entries++;
			}
			curr_entry_dword++;
			if (i != 0 && i % 5 == 0) {
				curr_entry_dword = 0;
			}
		}
		total_dword += osa_data_read_out.entries_read;
	}

	return ret;
}

int switchtec_osa_capture_control(struct switchtec_dev *dev, int stack_id,
				  int lane_mask, int direction,
				  int drop_single_os, int stop_mode,
				  int snapshot_mode, int post_trigger,
				  int os_types)
{
	int ret = 0;

	struct osa_capture_ctrl_in osa_capture_ctrl_in = {0};

	osa_capture_ctrl_in.sub_cmd = MRPC_OSA_CAPTURE_CTRL;
	if (switchtec_is_gen6(dev))
		osa_capture_ctrl_in.sub_cmd = MRPC_OSA_CAPTURE_CTRL_GEN6;
	osa_capture_ctrl_in.stack_id = stack_id;
	osa_capture_ctrl_in.lane_mask = lane_mask;
	osa_capture_ctrl_in.direction = direction;
	osa_capture_ctrl_in.drop_single_os = drop_single_os;
	osa_capture_ctrl_in.stop_mode = stop_mode;
	osa_capture_ctrl_in.snapshot_mode = snapshot_mode;
	osa_capture_ctrl_in.post_trig_entries = post_trigger;
	osa_capture_ctrl_in.os_types = os_types;

	ret = switchtec_cmd(dev, MRPC_ORDERED_SET_ANALYZER, &osa_capture_ctrl_in,
			    sizeof(osa_capture_ctrl_in), NULL, 0);
	if (ret) {
		switchtec_perror("OSA capture control");
		return ret;
	}
	printf("OSA: Configuring capture control on stack %d\n", stack_id);
	return ret;
}

int switchtec_osa_config_misc(struct switchtec_dev *dev, int stack_id,
			      int trigger_en)
{
	int ret = 0;
	struct {
		uint8_t sub_cmd;
		uint8_t stack_id;
		uint16_t reserved;
		uint8_t trigger_en;
		uint8_t reserved2;
		uint16_t reserved3;
	} osa_misc_config_in;

	osa_misc_config_in.sub_cmd = MRPC_OSA_MISC_TRIG_CONFIG;
	osa_misc_config_in.stack_id = stack_id;
	osa_misc_config_in.trigger_en = trigger_en;

	ret = switchtec_cmd(dev, MRPC_ORDERED_SET_ANALYZER, &osa_misc_config_in,
			    sizeof(osa_misc_config_in), NULL, 0);
	if (ret) {
		switchtec_perror("OSA misc config");
		return ret;
	}
	printf("OSA: Enabled misc triggering config on stack %d\n", stack_id);
	return ret;
}

int switchtec_osa_config_pattern(struct switchtec_dev *dev, int stack_id,
				 int direction, int lane_mask, int link_rate,
				 uint32_t *value_data, uint32_t *mask_data)
{
	int ret = 1;

	struct osa_pattern_config_in osa_pattern_config_in = {0};
	osa_pattern_config_in.sub_cmd = MRPC_OSA_PAT_TRIG_CONFIG;
	osa_pattern_config_in.stack_id = stack_id;
	osa_pattern_config_in.direction = direction;
	osa_pattern_config_in.lane_mask = lane_mask;
	osa_pattern_config_in.link_rate = link_rate;
	osa_pattern_config_in.pat_val_dword0 = value_data[0];
	osa_pattern_config_in.pat_val_dword1 = value_data[1];
	osa_pattern_config_in.pat_val_dword2 = value_data[2];
	osa_pattern_config_in.pat_val_dword3 = value_data[3];
	osa_pattern_config_in.pat_mask_dword0 = mask_data[0];
	osa_pattern_config_in.pat_mask_dword1 = mask_data[1];
	osa_pattern_config_in.pat_mask_dword2 = mask_data[2];
	osa_pattern_config_in.pat_mask_dword3 = mask_data[3];

	ret = switchtec_cmd(dev, MRPC_ORDERED_SET_ANALYZER,
			    &osa_pattern_config_in,
			    sizeof(osa_pattern_config_in), NULL, 0);
	if (ret) {
		switchtec_perror("OSA pattern config");
		return ret;
	}
	printf("OSA: Enabled pattern triggering config on stack %d\n", stack_id);
	return ret;
}

int switchtec_osa_config_type(struct switchtec_dev *dev, int stack_id,
		int direction, int lane_mask, int link_rate, int os_types)
{
	int ret = 1;

	struct osa_type_config_in osa_type_config_in = {0};

	osa_type_config_in.sub_cmd = MRPC_OSA_TYPE_TRIG_CONFIG;
	if (switchtec_is_gen6(dev))
		osa_type_config_in.sub_cmd = MRPC_OSA_TYPE_TRIG_CONFIG_GEN6;
	osa_type_config_in.stack_id = stack_id;
	osa_type_config_in.lane_mask = lane_mask;
	osa_type_config_in.direction = direction;
	osa_type_config_in.link_rate = link_rate;
	osa_type_config_in.os_types = os_types;

	ret = switchtec_cmd(dev, MRPC_ORDERED_SET_ANALYZER, &osa_type_config_in,
			    sizeof(osa_type_config_in), NULL, 0);
	if (ret) {
		switchtec_perror("OSA type config");
		return ret;
	}
	printf("OSA: Enabled type triggering config on stack %d\n", stack_id);
	return ret;
}

int switchtec_osa_dump_conf(struct switchtec_dev *dev, int stack_id)
{
	int ret = 0;

	struct {
		uint8_t sub_cmd;
		uint8_t stack_id;
		uint16_t reserved;
	} osa_dmp_in;

	struct {
		int16_t os_type_trig_lane_mask;
		uint8_t os_type_trig_dir;
		uint8_t os_type_trig_link_rate;
		uint8_t os_type_trig_os_types;
		uint8_t reserved;
		uint16_t reserved2;
		uint16_t os_pat_trig_lane_mask;
		uint8_t os_pat_trig_dir;
		uint8_t os_pat_trig_link_rate;
		uint32_t os_pat_trig_val_dw0;
		uint32_t os_pat_trig_val_dw1;
		uint32_t os_pat_trig_val_dw2;
		uint32_t os_pat_trig_val_dw3;
		uint32_t os_pat_trig_mask_dw0;
		uint32_t os_pat_trig_mask_dw1;
		uint32_t os_pat_trig_mask_dw2;
		uint32_t os_pat_trig_mask_dw3;
		uint8_t misc_trig_en;
		uint8_t reserved3;
		uint16_t reserved4;
		uint16_t capture_lane_mask;
		uint8_t capture_dir;
		uint8_t capture_drop_os;
		uint8_t capture_stop_mode;
		uint8_t capture_snap_mode;
		uint16_t capture_post_trig_entries;
		uint8_t capture_os_types;
		uint8_t reserved5;
		uint16_t reserved6;
	} osa_dmp_out;

	osa_dmp_in.stack_id = stack_id;
	osa_dmp_in.sub_cmd = MRPC_OSA_CONFIG_DMP;
	if (switchtec_is_gen6(dev))
		osa_dmp_in.sub_cmd = MRPC_OSA_CONFIG_DMP_GEN6;

	ret = switchtec_cmd(dev, MRPC_ORDERED_SET_ANALYZER, &osa_dmp_in,
			    sizeof(osa_dmp_in), &osa_dmp_out,
			    sizeof(osa_dmp_out));
	if (ret) {
		switchtec_perror("OSA config dump");
		return ret;
	}
	printf("Config dump \n");
	printf("------- OS Type ------------------------\n");
	printf("lanes: \t\t\t%s", osa_dmp_out.os_type_trig_lane_mask & 1 ? "0," : "");
	char lane[5];
	for (int i = 1; i < 16; i++) {
		if (i == 15)
			sprintf(lane, "%d", i);
		else
			sprintf(lane, "%d,", i);
		printf("%s", (osa_dmp_out.os_type_trig_lane_mask >> i) & 1 ? lane : "");
	}
	printf("\n");

	printf("direciton: \t\t%s", osa_dmp_out.os_type_trig_dir & 1 ? "RX," : "");
	printf("%s\n", (osa_dmp_out.os_type_trig_dir >> 1) & 1 ? "TX" : "");

	printf("link rate: \t\t%s", osa_dmp_out.os_type_trig_link_rate & 1 ? "GEN1," : "");
	printf("%s", (osa_dmp_out.os_type_trig_link_rate >> 1) & 1 ? "GEN2," : "");
	printf("%s", (osa_dmp_out.os_type_trig_link_rate >> 2) & 1 ? "GEN3," : "");
	printf("%s", (osa_dmp_out.os_type_trig_link_rate >> 3) & 1 ? "GEN4," : "");
	printf("%s\n", (osa_dmp_out.os_type_trig_link_rate >> 4) & 1 ? "GEN5" : "");
	if (switchtec_is_gen6(dev))
		printf("%s\n", (osa_dmp_out.os_type_trig_link_rate >> 5) & 1 ? "GEN6" : "");

	printf("os types: \t\t");
	if (switchtec_is_gen6(dev)) {
		printf("%s", osa_dmp_out.os_type_trig_os_types & 1 ? "TS0," : "");
		printf("%s", (osa_dmp_out.os_type_trig_os_types >> 1) & 1 ? "TS1," : "");
		printf("%s", (osa_dmp_out.os_type_trig_os_types >> 2) & 1 ? "TS2," : "");
		printf("%s\n", (osa_dmp_out.os_type_trig_os_types >> 3) & 1 ? "FTS" : "");
		printf("%s\n", (osa_dmp_out.os_type_trig_os_types >> 4) & 1 ? "CTL_SKP" : "");
	} else {
		printf("%s", osa_dmp_out.os_type_trig_os_types & 1 ? "TS1," : "");
		printf("%s", (osa_dmp_out.os_type_trig_os_types >> 1) & 1 ? "TS2," : "");
		printf("%s", (osa_dmp_out.os_type_trig_os_types >> 2) & 1 ? "FTS," : "");
		printf("%s\n", (osa_dmp_out.os_type_trig_os_types >> 3) & 1 ? "CTL_SKP" : "");
	}
	
	printf("------- OS Pattern ---------------------\n");
	printf("lanes: \t\t\t%s", osa_dmp_out.os_pat_trig_lane_mask & 1 ? "0," : "");
	for (int i = 1; i < 16; i++) {
		if (i == 15)
			sprintf(lane, "%d", i);
		else
			sprintf(lane, "%d,", i);
		printf("%s", (osa_dmp_out.os_pat_trig_lane_mask >> i) & 1 ? lane : "");
	}
	printf("\n");

	printf("direciton: \t\t%s", osa_dmp_out.os_pat_trig_dir & 1 ? "RX," : "");
	printf("%s\n", (osa_dmp_out.os_pat_trig_dir >> 1) & 1 ? "TX" : "");

	printf("link rate: \t\t%s", osa_dmp_out.os_pat_trig_link_rate && 1 ? "GEN1," : "");
	printf("%s", (osa_dmp_out.os_pat_trig_link_rate >> 1) & 1 ? "GEN2," : "");
	printf("%s", (osa_dmp_out.os_pat_trig_link_rate >> 2) & 1 ? "GEN3," : "");
	printf("%s", (osa_dmp_out.os_pat_trig_link_rate >> 3) & 1 ? "GEN4," : "");
	printf("%s\n", (osa_dmp_out.os_pat_trig_link_rate >> 4) & 1 ? "GEN5" : "");
	if (switchtec_is_gen6(dev))
		printf("%s\n", (osa_dmp_out.os_type_trig_link_rate >> 5) & 1 ? "GEN6" : "");
	
	printf("patttern: \t\t0x%08x %08x %08x %08x\n", osa_dmp_out.os_pat_trig_val_dw0,
	       osa_dmp_out.os_pat_trig_val_dw1, osa_dmp_out.os_pat_trig_val_dw2,
	       osa_dmp_out.os_pat_trig_val_dw3);
	printf("mask: \t\t\t0x%08x %08x %08x %08x\n", osa_dmp_out.os_pat_trig_mask_dw0,
	       osa_dmp_out.os_pat_trig_mask_dw1, osa_dmp_out.os_pat_trig_mask_dw2,
	       osa_dmp_out.os_pat_trig_mask_dw3);
	
	printf("------- Misc ---------------------------\n");
	if (osa_dmp_out.misc_trig_en == 2 || osa_dmp_out.misc_trig_en == 0)
		printf("Misc trigger disabled");
	else
		printf("Misc trigger enabled: \t");
	printf("%s", (osa_dmp_out.misc_trig_en - 2) & 1 ? "LTMON/other HW blk, " : "");
	printf("%s\n", ((osa_dmp_out.misc_trig_en - 2) >> 2 ) & 1 ? "General purpose input" : "");

	printf("------- Capture ------------------------\n");
	printf("lanes: \t\t\t%s", osa_dmp_out.capture_lane_mask & 1 ? "0," : "");
	for (int i = 1; i < 16; i++) {
		if (i == 15)
			sprintf(lane, "%d", i);
		else
			sprintf(lane, "%d,", i);
		printf("%s", (osa_dmp_out.capture_lane_mask >> i) & 1 ? lane : "");
	}
	printf("\n");

	printf("direciton: \t\t%s", osa_dmp_out.capture_dir & 1 ? "RX," : "");
	printf("%s\n", (osa_dmp_out.capture_dir >> 1) & 1 ? "TX" : "");

	if (switchtec_is_gen6(dev))
		printf("drop single os: \t%d: Single TS0, TS1, TS2, FTS and CTL_SKP OS's %s in the capture\n", 
			osa_dmp_out.capture_drop_os, 
			osa_dmp_out.capture_drop_os ? " excluded" : "included");
	else
		printf("drop single os: \t%d: Single TS1, TS2, FTS and CTL_SKP OS's %s in the capture\n", 
			osa_dmp_out.capture_drop_os, 
			osa_dmp_out.capture_drop_os ? " excluded" : "included");

	printf("stop mode: \t\t%d: OSA will stop capturing after %s lane has stopped writing into %s allocated RAMs\n", 
		osa_dmp_out.capture_stop_mode,
		osa_dmp_out.capture_stop_mode ? "all" : "any",
		osa_dmp_out.capture_stop_mode ? "their" : "its");
	printf("snaphot mode: \t\t%d: OSes are captured %s\n", 
		osa_dmp_out.capture_snap_mode, 
		osa_dmp_out.capture_snap_mode ? "until the RAM is full" : "according to the Post-Trigger Entries value");
	printf("post-trigger entries: \t%d\n", osa_dmp_out.capture_post_trig_entries);
	
	printf("os types: \t\t");
	if (switchtec_is_gen6(dev)) {
		printf("%s", (osa_dmp_out.capture_os_types & 1) ? "TS0," : "");
		printf("%s", (osa_dmp_out.capture_os_types >> 1) & 1 ? "TS2," : "");
		printf("%s", (osa_dmp_out.capture_os_types >> 2) & 1 ? "TS2," : "");
		printf("%s", (osa_dmp_out.capture_os_types >> 3) & 1 ? "FTS," : "");
		printf("%s", (osa_dmp_out.capture_os_types >> 4) & 1 ? "CTL_SKP," : "");
		printf("%s", (osa_dmp_out.capture_os_types >> 5) & 1 ? "SKP," : "");
		printf("%s", (osa_dmp_out.capture_os_types >> 6) & 1 ? "EIEOS," : "");
		printf("%s", (osa_dmp_out.capture_os_types >> 7) & 1 ? "EIOS," : "");
		printf("%s", (osa_dmp_out.capture_os_types >> 8) & 1 ? "ERR_OS," : "");
	} else {
		printf("%s", (osa_dmp_out.capture_os_types & 1) ? "TS1," : "");
		printf("%s", (osa_dmp_out.capture_os_types >> 1) & 1 ? "TS2," : "");
		printf("%s", (osa_dmp_out.capture_os_types >> 2) & 1 ? "FTS," : "");
		printf("%s", (osa_dmp_out.capture_os_types >> 3) & 1 ? "CTL_SKP," : "");
		printf("%s", (osa_dmp_out.capture_os_types >> 4) & 1 ? "SKP," : "");
		printf("%s", (osa_dmp_out.capture_os_types >> 5) & 1 ? "EIEOS," : "");
		printf("%s", (osa_dmp_out.capture_os_types >> 6) & 1 ? "EIOS," : "");
		printf("%s", (osa_dmp_out.capture_os_types >> 7) & 1 ? "ERR_OS," : "");
	}
	printf("\n");
	return ret;
}

int switchtec_osa(struct switchtec_dev *dev, int stack_id, int operation)
{
	int ret = 0;
	struct {
		uint8_t sub_cmd;
		uint8_t stack_id;
		uint16_t reserved;
	} osa_rel_access_perm_in;

	struct {
		uint8_t sub_cmd;
		uint8_t stack_id;
		uint16_t reserved;
	} osa_status_query_in;

	struct {
		uint8_t state;
		uint8_t trigger_lane;
		uint8_t trigger_dir;
		uint8_t reserved;
		uint16_t trigger_reason;
		uint16_t reserved2;
	} osa_status_query_out;

	struct {
		uint8_t sub_cmd;
		uint8_t stack_id;
		uint8_t operation;
		uint8_t reserved;
	} osa_op_in;

	char *valid_ops[6] = {"stop", "start", "trigger", "reset", "release",
			       "status"};
	char *states[5] = {"Deactivated (not armed)", "Started (armed), not triggered",
			    "Started (armted), triggered", "Stopped, not triggered",
			    "Stopped, triggered"};
	char *directions[2] = {"TX", "RX"};
	printf("Attempting %s operation...\n", valid_ops[operation]);
	if (operation == 4) {
		osa_rel_access_perm_in.sub_cmd = MRPC_OSA_REL_ACCESS_PERM;
		osa_rel_access_perm_in.stack_id = stack_id;

		ret = switchtec_cmd(dev, MRPC_ORDERED_SET_ANALYZER,
				    &osa_rel_access_perm_in,
				    sizeof(osa_rel_access_perm_in), NULL, 0);
	}
	else if (operation == 5) {
		osa_status_query_in.sub_cmd = MRPC_OSA_STATUS_QUERY;
		if (switchtec_is_gen6(dev))
			osa_status_query_in.sub_cmd = MRPC_OSA_STATUS_QUERY_GEN6;
		osa_status_query_in.stack_id = stack_id;

		ret = switchtec_cmd(dev, MRPC_ORDERED_SET_ANALYZER,
			&osa_status_query_in, sizeof(osa_status_query_in),
			&osa_status_query_out, sizeof(osa_status_query_out));
		if (ret) {
			switchtec_perror("OSA operation");
			return ret;
		}
		printf("Status of stack %d\n", stack_id);
		printf("STATE: %s\n", states[osa_status_query_out.state]);
		printf("TRIGGER_LANE: %d\n", osa_status_query_out.trigger_lane);
		printf("TRIGGER_DIR: %s\n", directions[osa_status_query_out.trigger_dir]);
		printf("REASON_BITMASK: %d\n", osa_status_query_out.trigger_reason);
	}
	else {
		osa_op_in.sub_cmd = MRPC_OSA_ANALYZER_OP;
		osa_op_in.stack_id = stack_id;
		osa_op_in.operation = operation;

		ret = switchtec_cmd(dev, MRPC_ORDERED_SET_ANALYZER, &osa_op_in,
				    sizeof(osa_op_in), NULL, 0);
	}
	if (ret) {
		switchtec_perror("OSA operation");
		return ret;
	}
	printf("Successful %s operation!\n", valid_ops[operation]);

	return ret;
}

/**@}*/
