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

#include "../switchtec_priv.h"
#include "switchtec/diag.h"
#include "switchtec/switchtec.h"
#include "switchtec/endian.h"
#include "switchtec/utils.h"

#include <stdint.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>

static void switchtec_diag_ltssm_set_log_data_gen5(struct switchtec_diag_ltssm_log
					*log_data,
					struct switchtec_diag_ltssm_log_dmp_out
					*log_dump_out_ptr,
					int curr_idx, uint16_t num_of_logs)
{
	uint32_t dw0;
	uint32_t timestamp;

	int major;
	int minor;
	int rate;

	for (int j = 0; j < num_of_logs; j++) {
		dw0 = log_dump_out_ptr[j].dw0;
		timestamp = log_dump_out_ptr[j].ram_timestamp;

		rate = (dw0 >> 13) & 0x7;
		major = (dw0 >> 7) & 0x3f;
		minor = (dw0 >> 3) & 0xf;

		log_data[curr_idx + j].timestamp = timestamp;
		log_data[curr_idx + j].link_rate = switchtec_gen_transfers[rate+1];
		log_data[curr_idx + j].link_state = major | (minor << 8);
	}
}

/**
 * @brief Get the LTSSM log of a port on a gen5 switchtec device
 * @param[in]	dev    Switchtec device handle
 * @param[in]	port   Switchtec Port
 * @param[inout] log_count number of log entries
 * @param[out] log    A pointer to an array containing the log
 */
int switchtec_diag_ltssm_log_gen5(struct switchtec_dev *dev,
				 int port, int *log_count,
				 struct switchtec_diag_ltssm_log *log_data)
{
	struct {
		uint8_t sub_cmd;
		uint8_t port;
		uint8_t freeze;
		uint8_t unused;
	} ltssm_freeze;

	struct {
		uint8_t sub_cmd;
		uint8_t port;
	} status;

	struct {
		uint16_t log_count;
		uint16_t w0_trigger_count;
		uint16_t w1_trigger_count;
	} status_output;

	struct {
		uint8_t sub_cmd;
		uint8_t port;
		uint16_t log_index;
		uint16_t no_of_logs;
	} log_dump;

	uint8_t log_buffer[1024];

	struct switchtec_diag_ltssm_log_dmp_out *log_dump_out_ptr = NULL;

	int ret;
	int log_dmp_size = sizeof(struct switchtec_diag_ltssm_log_dmp_out);

	/* freeze logs */
	ltssm_freeze.sub_cmd = MRPC_LTMON_FREEZE;
	ltssm_freeze.port = port;
	ltssm_freeze.freeze = 1;

	ret = switchtec_cmd(dev, MRPC_DIAG_PORT_LTSSM_LOG, &ltssm_freeze,
			    sizeof(ltssm_freeze), NULL, 0);
	if (ret)
		return ret;

	/* get number of entries */
	status.sub_cmd = MRPC_LTMON_GET_STATUS_GEN5;
	status.port = port;
	ret = switchtec_cmd(dev, MRPC_DIAG_PORT_LTSSM_LOG, &status,
			    sizeof(status), &status_output,
			    sizeof(status_output));
	if (ret)
		return ret;

	*log_count = status_output.log_count;

	/* get log data */
	log_dump.sub_cmd = MRPC_LTMON_LOG_DUMP_GEN5;
	log_dump.port = port;
	log_dump.log_index = 0;
	log_dump.no_of_logs = *log_count;

	if(log_dump.no_of_logs <= SWITCHTEC_LTSSM_MAX_LOGS) {
		/* Single buffer log case */
		ret = switchtec_cmd(dev, MRPC_DIAG_PORT_LTSSM_LOG, &log_dump,
				    sizeof(log_dump), &log_buffer[0],
				    log_dump.no_of_logs * log_dmp_size + 4);
		if (ret)
			return ret;
		log_dump_out_ptr =
			(struct switchtec_diag_ltssm_log_dmp_out *)
			&(log_buffer[4]);

		switchtec_diag_ltssm_set_log_data_gen5(log_data,
						  log_dump_out_ptr,
						  0, log_dump.no_of_logs);
	} else {
		/* Multiple buffer log case */
		int buff_count = log_dump.no_of_logs / SWITCHTEC_LTSSM_MAX_LOGS;
		int curr_idx = 0;
		int buffer_size = SWITCHTEC_LTSSM_MAX_LOGS * log_dmp_size + 4;

		for (int i = 0; i < buff_count; i++) {
			log_dump.no_of_logs = SWITCHTEC_LTSSM_MAX_LOGS;
			ret = switchtec_cmd(dev, MRPC_DIAG_PORT_LTSSM_LOG,
					    &log_dump, sizeof(log_dump),
					    &log_buffer[0], buffer_size);
			if (ret)
				return ret;
			log_dump_out_ptr =
				(struct switchtec_diag_ltssm_log_dmp_out *)
				&(log_buffer[4]);

			switchtec_diag_ltssm_set_log_data_gen5(log_data,
							  log_dump_out_ptr,
							  curr_idx,
							  log_dump.no_of_logs);
			curr_idx += SWITCHTEC_LTSSM_MAX_LOGS;
			log_dump.log_index = curr_idx;
		}
		if (*log_count % SWITCHTEC_LTSSM_MAX_LOGS) {
			log_dump.no_of_logs = *log_count - curr_idx;
			buffer_size = log_dump.no_of_logs * log_dmp_size + 4;
			ret = switchtec_cmd(dev, MRPC_DIAG_PORT_LTSSM_LOG,
					    &log_dump, sizeof(log_dump),
					    &log_buffer[0], buffer_size);
			if (ret)
				return ret;
			log_dump_out_ptr =
				(struct switchtec_diag_ltssm_log_dmp_out *)
				&(log_buffer[4]);

			switchtec_diag_ltssm_set_log_data_gen5(log_data,
							  log_dump_out_ptr,
							  curr_idx,
							  log_dump.no_of_logs);
		}
	}

	/* unfreeze logs */
	ltssm_freeze.sub_cmd = MRPC_LTMON_FREEZE;
	ltssm_freeze.port = port;
	ltssm_freeze.freeze = 0;

	ret = switchtec_cmd(dev, MRPC_DIAG_PORT_LTSSM_LOG, &ltssm_freeze,
			    sizeof(ltssm_freeze), NULL, 0);

	return ret;
}

int switchtec_diag_port_eq_tx_coeff_gen5(struct switchtec_dev *dev,
					 int port_id, int prev_speed,
					 int end, int link, void *res)
{
	struct switchtec_port_eq_coeff *coeff = res;
	struct switchtec_port_eq_coeff *loc_out;
	struct switchtec_rem_port_eq_coeff *rem_out;
	struct switchtec_port_eq_coeff_in *in;
	uint8_t *buf;
	uint32_t buf_size;
	uint32_t in_size = sizeof(struct switchtec_port_eq_coeff_in);
	uint32_t out_size = 0;
	int ret = 0;
	int i;

	if (!coeff) {
		fprintf(stderr, "Error inval output buffer\n");
		errno = -EINVAL;
		return -1;
	}

	buf_size = in_size;
	if (end == SWITCHTEC_DIAG_LOCAL) {
		buf_size += sizeof(struct switchtec_port_eq_coeff);
		out_size = sizeof(struct switchtec_port_eq_coeff);
	} else if (end == SWITCHTEC_DIAG_FAR_END) {
		buf_size += sizeof(struct switchtec_rem_port_eq_coeff);
		out_size = sizeof(struct switchtec_rem_port_eq_coeff);
	} else {
		fprintf(stderr, "Error inval end option\n");
		errno = -EINVAL;
	}
	buf = (uint8_t *)malloc(buf_size);
	if (!buf) {
		fprintf(stderr, "Error in buffer alloc\n");
		errno = -ENOMEM;
		return -1;
	}

	in = (struct switchtec_port_eq_coeff_in *)buf;
	in->op_type = DIAG_PORT_EQ_STATUS_OP_PER_PORT;
	in->phys_port_id = port_id;
	in->lane_id = 0;
	in->dump_type = LANE_EQ_DUMP_TYPE_CURR;

	if (link == SWITCHTEC_DIAG_LINK_PREVIOUS) {
		in->dump_type = LANE_EQ_DUMP_TYPE_PREV;
		in->prev_rate = prev_speed;
	}

	if (end == SWITCHTEC_DIAG_LOCAL) {
		in->cmd = MRPC_GEN5_PORT_EQ_LOCAL_TX_COEFF_DUMP;
		loc_out = (struct switchtec_port_eq_coeff *)&buf[in_size];
		ret = switchtec_cmd(dev, MRPC_PORT_EQ_STATUS, in, in_size,
				    loc_out, out_size);
		if (ret) {
			fprintf(stderr, "Error in switchtec cmd:%d\n", ret);
			goto end;
		}
	} else if (end == SWITCHTEC_DIAG_FAR_END) {
		in->cmd = MRPC_GEN5_PORT_EQ_FAR_END_TX_COEFF_DUMP;
		rem_out = (struct switchtec_rem_port_eq_coeff *)&buf[in_size];
		ret = switchtec_cmd(dev, MRPC_PORT_EQ_STATUS, in, in_size,
				    rem_out, out_size);
		if (ret) {
			fprintf(stderr, "Error in switchtec cmd:%d\n", ret);
			goto end;
		}
	} else {
		fprintf(stderr, "Error inval end request\n");
		errno = -EINVAL;
		goto end;
	}

	if (end == SWITCHTEC_DIAG_LOCAL) {
		coeff->lane_cnt = loc_out->lane_cnt + 1;
		for (i = 0; i < coeff->lane_cnt; i++) {
			coeff->cursors[i].pre = loc_out->cursors[i].pre;
			coeff->cursors[i].post = loc_out->cursors[i].post;
		}
	} else {
		coeff->lane_cnt = rem_out->lane_cnt + 1;
		for (i = 0; i < coeff->lane_cnt; i++) {
			coeff->cursors[i].pre = rem_out->cursors[i].pre;
			coeff->cursors[i].post = rem_out->cursors[i].post;
		}
	}

end:
	if (buf)
		free(buf);

	return ret;
}

int switchtec_diag_port_eq_tx_table_gen5(struct switchtec_dev *dev,
					 int port_id, int prev_speed,
					 int link, void *res)
{
	struct switchtec_port_eq_table *table = res;
	struct switchtec_gen5_port_eq_table out = {};
	struct switchtec_port_eq_table_in in = {
		.sub_cmd = MRPC_GEN5_PORT_EQ_FAR_END_TX_EQ_TABLE_DUMP,
		.port_id = port_id,
	};
	int ret, i;

	if (!table) {
		errno = -EINVAL;
		return -1;
	}

	in.dump_type = LANE_EQ_DUMP_TYPE_CURR;
	in.prev_rate = 0;

	if (link == SWITCHTEC_DIAG_LINK_PREVIOUS) {
		in.dump_type = LANE_EQ_DUMP_TYPE_PREV;
		in.prev_rate = prev_speed;
	}

	ret = switchtec_cmd(dev, MRPC_PORT_EQ_STATUS, &in,
			    sizeof(struct switchtec_port_eq_table_in),
			    &out, sizeof(struct switchtec_gen5_port_eq_table));
	if (ret)
		return -1;

	table->lane_id = out.lane_id;
	table->step_cnt = out.step_cnt;

	for (i = 0; i < table->step_cnt; i++) {
		table->steps[i].pre_cursor = out.steps[i].pre_cursor;
		table->steps[i].post_cursor = out.steps[i].post_cursor;
		table->steps[i].fom = 0;
		table->steps[i].pre_cursor_up = 0;
		table->steps[i].post_cursor_up = 0;
		table->steps[i].error_status = out.steps[i].error_status;
		table->steps[i].active_status = out.steps[i].active_status;
		table->steps[i].speed = out.steps[i].speed;
	}

	return 0;
}

int switchtec_diag_port_eq_tx_fslf_gen5(struct switchtec_dev *dev,
					int port_id, int prev_speed,
					int lane_id, int end, int link,
					void *res)
{
	struct switchtec_port_eq_tx_fslf *fslf = res;
	struct switchtec_port_eq_tx_fslf_in in = {};
	struct switchtec_port_eq_tx_fslf_out out = {};
	int ret;

	if (!fslf) {
		errno = -EINVAL;
		return -1;
	}

	in.port_id = port_id;
	in.lane_id = lane_id;

	if (end == SWITCHTEC_DIAG_LOCAL) {
		in.sub_cmd = MRPC_GEN5_PORT_EQ_LOCAL_TX_FSLF_DUMP;
	} else if (end == SWITCHTEC_DIAG_FAR_END) {
		in.sub_cmd = MRPC_GEN5_PORT_EQ_FAR_END_TX_FSLF_DUMP;
	} else {
		errno = -EINVAL;
		return -1;
	}

	if (link == SWITCHTEC_DIAG_LINK_CURRENT) {
		in.dump_type = LANE_EQ_DUMP_TYPE_CURR;
	} else {
		in.dump_type = LANE_EQ_DUMP_TYPE_PREV;
		in.prev_rate = prev_speed;
	}

	ret = switchtec_cmd(dev, MRPC_PORT_EQ_STATUS, &in,
			    sizeof(struct switchtec_port_eq_tx_fslf_in), &out,
			    sizeof(struct switchtec_port_eq_tx_fslf_out));
	if (ret)
		return -1;

	fslf->fs = out.fs;
	fslf->lf = out.lf;

	return 0;
}

int switchtec_diag_loopback_set_gen5(struct switchtec_dev *dev,
				     int port_id, int enable,
				     int enable_parallel, int enable_external,
				     int enable_ltssm, int enable_pipe,
				     int ltssm_speed)
{
	struct switchtec_diag_loopback_in int_in = {
		.sub_cmd = MRPC_LOOPBACK_SET_INT_LOOPBACK,
		.port_id = port_id,
		.enable = 1,
	};
	struct switchtec_diag_loopback_ltssm_in ltssm_in = {
		.sub_cmd = MRPC_LOOPBACK_SET_LTSSM_LOOPBACK,
		.port_id = port_id,
		.enable = enable_ltssm,
		.speed = ltssm_speed,
	};
	int ret;

	if (enable_ltssm && !(enable_external || enable_parallel)) {
		ret = switchtec_cmd(dev, MRPC_INT_LOOPBACK, &ltssm_in,
				    sizeof(ltssm_in), NULL, 0);
		if (ret)
			return ret;
	} else {
		int_in.type = DIAG_LOOPBACK_PARALEL_DATAPATH;
		int_in.enable = enable_parallel;
		ret = switchtec_cmd(dev, MRPC_INT_LOOPBACK, &int_in,
				    sizeof(int_in), NULL, 0);
		if (ret)
			return ret;
		if (!enable_parallel) {
			int_in.type = DIAG_LOOPBACK_EXTERNAL_DATAPATH;
			int_in.enable = enable_external;
			ret = switchtec_cmd(dev, MRPC_INT_LOOPBACK, &int_in,
					    sizeof(int_in), NULL, 0);
			if (ret)
				return ret;
		}
		int_in.type = DIAG_LOOPBACK_PIPE_DATAPATH;
		int_in.enable = enable_pipe;
		ret = switchtec_cmd(dev, MRPC_INT_LOOPBACK, &int_in,
				    sizeof(int_in), NULL, 0);
		if (ret)
			return ret;

		ltssm_in.enable = enable_ltssm;
		ret = switchtec_cmd(dev, MRPC_INT_LOOPBACK, &ltssm_in,
				    sizeof(ltssm_in), NULL, 0);
		if (ret)
			return ret;
	}
	return 0;
}

int switchtec_diag_loopback_get_gen5(struct switchtec_dev *dev,
				     int port_id, int *enabled,
				     int *ltssm_speed)
{
	struct switchtec_diag_loopback_in int_in = {
		.sub_cmd = MRPC_LOOPBACK_GET_INT_LOOPBACK,
		.port_id = port_id,
	};
	struct switchtec_diag_loopback_ltssm_in lt_in = {
		.sub_cmd = MRPC_LOOPBACK_GET_LTSSM_LOOPBACK,
		.port_id = port_id,
	};
	struct switchtec_diag_loopback_out int_out;
	struct switchtec_diag_loopback_ltssm_out lt_out;
	int ret, en = 0;

	int_in.type = DIAG_LOOPBACK_PARALEL_DATAPATH;

	ret = switchtec_cmd(dev, MRPC_INT_LOOPBACK, &int_in, sizeof(int_in),
			    &int_out, sizeof(int_out));
	if (ret)
		return ret;

	if (int_out.enabled)
		en |= SWITCHTEC_DIAG_LOOPBACK_RX_TO_TX;

	int_in.type = DIAG_LOOPBACK_EXTERNAL_DATAPATH;

	ret = switchtec_cmd(dev, MRPC_INT_LOOPBACK, &int_in, sizeof(int_in),
			    &int_out, sizeof(int_out));
	if (ret)
		return ret;

	if (int_out.enabled)
		en |= SWITCHTEC_DIAG_LOOPBACK_TX_TO_RX;

	ret = switchtec_cmd(dev, MRPC_INT_LOOPBACK, &lt_in, sizeof(lt_in),
			    &lt_out, sizeof(lt_out));
	if (ret)
		return ret;

	if (lt_out.enabled)
		en |= SWITCHTEC_DIAG_LOOPBACK_LTSSM;

	if (enabled)
		*enabled = en;

	if (ltssm_speed)
		*ltssm_speed = lt_out.speed;

	return 0;
}

static int switchtec_diag_eye_status_gen5(struct switchtec_dev *dev)
{
	int ret;
	int eye_status;

	struct switchtec_gen5_diag_eye_status_in in = {
		.sub_cmd = MRPC_EYE_CAP_STATUS_GEN5,
	};
	struct switchtec_gen5_diag_eye_status_out out;

	do {
		ret = switchtec_cmd(dev, MRPC_GEN5_EYE_CAPTURE, &in, sizeof(in),
				    &out, sizeof(out));
		if (ret) {
			switchtec_perror("eye_status");
			return -1;
		}
		eye_status = out.status;
		usleep(200000);
	} while (eye_status == SWITCHTEC_GEN5_DIAG_EYE_STATUS_IN_PROGRESS ||
		 eye_status == SWITCHTEC_GEN5_DIAG_EYE_STATUS_PENDING);

	switch (eye_status) {
		case SWITCHTEC_GEN5_DIAG_EYE_STATUS_IDLE:
			switchtec_perror("Eye capture idle");
		case SWITCHTEC_GEN5_DIAG_EYE_STATUS_DONE:
			return 0;
		case SWITCHTEC_GEN5_DIAG_EYE_STATUS_TIMEOUT:
			switchtec_perror("Eye capture timeout");
		case SWITCHTEC_GEN5_DIAG_EYE_STATUS_ERROR:
			switchtec_perror("Eye capture error");
		return -1;
	}
	switchtec_perror("Unknown eye capture state");
	return -1;
}

int switchtec_diag_eye_cmd_gen5(struct switchtec_dev *dev, void *in,
				       size_t size)
{
	int ret;

	ret = switchtec_cmd(dev, MRPC_GEN5_EYE_CAPTURE, in, size,
			    NULL, 0);
	if (ret)
		return ret;

	usleep(200000);

	return switchtec_diag_eye_status_gen5(dev);
}

int switchtec_diag_eye_start_gen5(struct switchtec_dev *dev, int lane_mask[4],
				struct range *x_range, struct range *y_range,
				int step_interval, int capture_depth, int sar_sel,
				int intleav_sel, int hstep, int data_mode, 
				int eye_mode, uint64_t refclk, int vstep)
{
	int err, ret;
	struct switchtec_gen5_diag_eye_run_in in = {
		.sub_cmd = MRPC_EYE_CAP_RUN_GEN5,
		.capture_depth = capture_depth,
		.timeout_disable = 1,
		.lane_mask[0] = lane_mask[0],
		.lane_mask[1] = lane_mask[1],
		.lane_mask[2] = lane_mask[2],
		.lane_mask[3] = lane_mask[3],
	};

	ret = switchtec_diag_eye_cmd_gen5(dev, &in, sizeof(in));
	err = errno;
	errno = err;
	return ret;
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
int switchtec_diag_eye_read_gen5(struct switchtec_dev *dev, int lane_id,
		      	    int bin, int* num_phases, double* ber_data)
{
	struct switchtec_gen5_diag_eye_read_in in = {
		.sub_cmd = MRPC_EYE_CAP_READ_GEN5,
		.lane_id = lane_id,
		.bin = bin,
	};
	struct switchtec_gen5_diag_eye_read_out out;
	int i, ret;

	ret = switchtec_cmd(dev, MRPC_GEN5_EYE_CAPTURE, &in, sizeof(in),
			    &out, sizeof(out));
	if (ret)
		return ret;

	*num_phases = out.num_phases;

	for(i = 0; i < out.num_phases; i++)
		ber_data[i] = le64toh(out.ber_data[i]) / 281474976710656.;

	return ret;
}

int switchtec_diag_pattern_gen_set_gen5(struct switchtec_dev *dev, int port_id,
					int type, int link_speed)
{
	struct switchtec_diag_pat_gen_in in = {
		.sub_cmd = MRPC_PAT_GEN_SET_GEN_GEN5,
		.port_id = port_id,
		.pattern_type = type,
		.lane_id = link_speed
	};

	return switchtec_cmd(dev, MRPC_PAT_GEN, &in, sizeof(in), NULL, 0);
}