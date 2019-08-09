/*
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "h264dec.h"
#include "hwaccel.h"
#include "v4l2_request.h"
#include "h264-ctrls.h"

typedef struct V4L2RequestControlsH264 {
    struct v4l2_ctrl_h264_sps sps;
    struct v4l2_ctrl_h264_pps pps;
    struct v4l2_ctrl_h264_scaling_matrix scaling_matrix;
    struct v4l2_ctrl_h264_decode_params decode_params;
    struct v4l2_ctrl_h264_slice_params slice_params[16];
    unsigned int num_decoded_slices;
    unsigned int num_queued_slices;
} V4L2RequestControlsH264;

static char nalu_slice_start_code[] = { 0x00, 0x00, 0x01 };
//static uint8_t padding[16] = { };

static void fill_weight_factors(struct v4l2_h264_weight_factors *factors, int list, const H264SliceContext *sl)
{
    for (int i = 0; i < sl->ref_count[list]; i++) {
        if (sl->pwt.luma_weight_flag[list]) {
            factors->luma_weight[i] = sl->pwt.luma_weight[i][list][0];
            factors->luma_offset[i] = sl->pwt.luma_weight[i][list][1];
        } else {
            factors->luma_weight[i] = 1 << sl->pwt.luma_log2_weight_denom;
            factors->luma_offset[i] = 0;
        }
        for (int j = 0; j < 2; j++) {
            if (sl->pwt.chroma_weight_flag[list]) {
                factors->chroma_weight[i][j] = sl->pwt.chroma_weight[i][list][j][0];
                factors->chroma_offset[i][j] = sl->pwt.chroma_weight[i][list][j][1];
            } else {
                factors->chroma_weight[i][j] = 1 << sl->pwt.chroma_log2_weight_denom;
                factors->chroma_offset[i][j] = 0;
            }
        }
    }
}

static void fill_dpb_entry(struct v4l2_h264_dpb_entry *entry, const H264Picture *pic)
{
    entry->reference_ts = ff_v4l2_request_get_capture_timestamp(pic->f);
    entry->frame_num = pic->frame_num;
    entry->pic_num = pic->pic_id;
    entry->flags = V4L2_H264_DPB_ENTRY_FLAG_VALID;
    if (pic->reference)
        entry->flags |= V4L2_H264_DPB_ENTRY_FLAG_ACTIVE;
    if (pic->long_ref)
        entry->flags |= V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM;
    entry->top_field_order_cnt = pic->field_poc[0];
    entry->bottom_field_order_cnt = pic->field_poc[1];
}

static void fill_dpb(struct v4l2_ctrl_h264_decode_params *decode, const H264Context *h)
{
    int entries = 0;

    for (int i = 0; i < h->short_ref_count; i++) {
        const H264Picture *pic = h->short_ref[i];
        if (pic)
            fill_dpb_entry(&decode->dpb[entries++], pic);
    }

    if (!h->long_ref_count)
        return;

    for (int i = 0; i < FF_ARRAY_ELEMS(h->long_ref); i++) {
        const H264Picture *pic = h->long_ref[i];
        if (pic)
            fill_dpb_entry(&decode->dpb[entries++], pic);
    }
}

static uint8_t get_dpb_index(struct v4l2_ctrl_h264_decode_params *decode, const H264Ref *ref)
{
    uint64_t timestamp;

    if (!ref->parent)
        return 0xff;

    timestamp = ff_v4l2_request_get_capture_timestamp(ref->parent->f);

    for (uint8_t i = 0; i < FF_ARRAY_ELEMS(decode->dpb); i++) {
        struct v4l2_h264_dpb_entry *entry = &decode->dpb[i];
        if ((entry->flags & V4L2_H264_DPB_ENTRY_FLAG_VALID) &&
            entry->reference_ts == timestamp)
            return i;
    }

    return 0xff;
}

static void fill_sps(struct v4l2_ctrl_h264_sps *ctrl, const H264Context *h)
{
    const SPS *sps = h->ps.sps;

    *ctrl = (struct v4l2_ctrl_h264_sps) {
        .profile_idc = sps->profile_idc,
        .constraint_set_flags = sps->constraint_set_flags,
        .level_idc = sps->level_idc,
        .seq_parameter_set_id = sps->sps_id,
        .chroma_format_idc = sps->chroma_format_idc,
        .bit_depth_luma_minus8 = sps->bit_depth_luma - 8,
        .bit_depth_chroma_minus8 = sps->bit_depth_chroma - 8,
        .log2_max_frame_num_minus4 = sps->log2_max_frame_num - 4,
        .pic_order_cnt_type = sps->poc_type,
        .log2_max_pic_order_cnt_lsb_minus4 = sps->log2_max_poc_lsb - 4,
        .max_num_ref_frames = sps->ref_frame_count,
        .num_ref_frames_in_pic_order_cnt_cycle = sps->poc_cycle_length,
        //.offset_for_ref_frame[255] - not required? not set by libva-v4l2-request - copy sps->offset_for_ref_frame
        .offset_for_non_ref_pic = sps->offset_for_non_ref_pic,
        .offset_for_top_to_bottom_field = sps->offset_for_top_to_bottom_field,
        .pic_width_in_mbs_minus1 = h->mb_width - 1,
        .pic_height_in_map_units_minus1 = sps->frame_mbs_only_flag ? h->mb_height - 1 : h->mb_height / 2 - 1,
    };

    if (sps->residual_color_transform_flag)
        ctrl->flags |= V4L2_H264_SPS_FLAG_SEPARATE_COLOUR_PLANE;
    if (sps->transform_bypass)
        ctrl->flags |= V4L2_H264_SPS_FLAG_QPPRIME_Y_ZERO_TRANSFORM_BYPASS;
    if (sps->delta_pic_order_always_zero_flag)
        ctrl->flags |= V4L2_H264_SPS_FLAG_DELTA_PIC_ORDER_ALWAYS_ZERO;
    if (sps->gaps_in_frame_num_allowed_flag)
        ctrl->flags |= V4L2_H264_SPS_FLAG_GAPS_IN_FRAME_NUM_VALUE_ALLOWED;
    if (sps->frame_mbs_only_flag)
        ctrl->flags |= V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY;
    if (sps->mb_aff)
        ctrl->flags |= V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD;
    if (sps->direct_8x8_inference_flag)
        ctrl->flags |= V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE;
}

static void fill_pps(struct v4l2_ctrl_h264_pps *ctrl, const H264Context *h)
{
    const PPS *pps = h->ps.pps;
    const H264SliceContext *sl = &h->slice_ctx[0];

    *ctrl = (struct v4l2_ctrl_h264_pps) {
        .pic_parameter_set_id = sl->pps_id,
        .seq_parameter_set_id = pps->sps_id,
        .num_slice_groups_minus1 = pps->slice_group_count - 1,
        .num_ref_idx_l0_default_active_minus1 = pps->ref_count[0] - 1,
        .num_ref_idx_l1_default_active_minus1 = pps->ref_count[1] - 1,
        .weighted_bipred_idc = pps->weighted_bipred_idc,
        .pic_init_qp_minus26 = pps->init_qp - 26,
        .pic_init_qs_minus26 = pps->init_qs - 26,
        .chroma_qp_index_offset = pps->chroma_qp_index_offset[0],
        .second_chroma_qp_index_offset = pps->chroma_qp_index_offset[1],
    };

    if (pps->cabac)
        ctrl->flags |= V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE;
    if (pps->pic_order_present)
        ctrl->flags |= V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT;
    if (pps->weighted_pred)
        ctrl->flags |= V4L2_H264_PPS_FLAG_WEIGHTED_PRED;
    if (pps->deblocking_filter_parameters_present)
        ctrl->flags |= V4L2_H264_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT;
    if (pps->constrained_intra_pred)
        ctrl->flags |= V4L2_H264_PPS_FLAG_CONSTRAINED_INTRA_PRED;
    if (pps->redundant_pic_cnt_present)
        ctrl->flags |= V4L2_H264_PPS_FLAG_REDUNDANT_PIC_CNT_PRESENT;
    if (pps->transform_8x8_mode)
        ctrl->flags |= V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE;
}

int v4l2_query_h264_start_code(AVCodecContext *avctx)
{
    V4L2RequestContext *ctx = avctx->internal->hwaccel_priv_data;
    struct v4l2_queryctrl ctrl = {0};
    int ret, i;

    ctrl.id = V4L2_CID_MPEG_VIDEO_H264_STARTCODE;
    ret = ioctl(ctx->video_fd, VIDIOC_QUERYCTRL, &ctrl);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: query control failed, %s (%d)\n", __func__, strerror(errno), errno);
        return AVERROR(EINVAL);
    }

    if (ctrl.type != V4L2_CTRL_TYPE_MENU) {
        av_log(avctx, AV_LOG_ERROR, "%s: start code control is not a menu\n", __func__);
        return AVERROR(EINVAL);
    }

    for (i = ctrl.minimum; i < ctrl.maximum + 1; i++) {
	struct v4l2_querymenu menu = {0};

	menu.id = ctrl.id;
        menu.index = i;
        ret = ioctl(ctx->video_fd, VIDIOC_QUERYMENU, &menu);
        if (ret < 0)
            continue;
	ctx->start_code_support |= 1 << i;
    }
    return 0;
}

static int v4l2_request_h264_start_frame(AVCodecContext *avctx,
                                         av_unused const uint8_t *buffer,
                                         av_unused uint32_t size)
{
    const H264Context *h = avctx->priv_data;
    const PPS *pps = h->ps.pps;
    V4L2RequestControlsH264 *controls = h->cur_pic_ptr->hwaccel_picture_private;

    fill_sps(&controls->sps, h);
    fill_pps(&controls->pps, h);

    memcpy(controls->scaling_matrix.scaling_list_4x4, pps->scaling_matrix4, sizeof(controls->scaling_matrix.scaling_list_4x4));
    memcpy(controls->scaling_matrix.scaling_list_8x8, pps->scaling_matrix8, sizeof(controls->scaling_matrix.scaling_list_8x8));

    controls->decode_params = (struct v4l2_ctrl_h264_decode_params) {
        .num_slices = 0,
        .flags = h->picture_idr ? V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC : 0,
        .nal_ref_idc = h->nal_ref_idc,
        .top_field_order_cnt = h->cur_pic_ptr->field_poc[0],
        .bottom_field_order_cnt = h->cur_pic_ptr->field_poc[1],
    };

    fill_dpb(&controls->decode_params, h);
    controls->num_decoded_slices = 0;
    controls->num_queued_slices = 0;

    return ff_v4l2_request_reset_frame(avctx, h->cur_pic_ptr->f);
}

static int v4l2_request_h264_decode_pending_slice(AVCodecContext *avctx, bool is_last)
{
    const H264Context *h = avctx->priv_data;
    V4L2RequestControlsH264 *controls = h->cur_pic_ptr->hwaccel_picture_private;
    V4L2RequestDescriptor *req = (V4L2RequestDescriptor*)h->cur_pic_ptr->f->data[0];
    int ret;

    struct v4l2_ext_control control[] = {
        {
            .id = V4L2_CID_MPEG_VIDEO_H264_SPS,
            .ptr = &controls->sps,
            .size = sizeof(controls->sps),
        },
        {
            .id = V4L2_CID_MPEG_VIDEO_H264_PPS,
            .ptr = &controls->pps,
            .size = sizeof(controls->pps),
        },
        {
            .id = V4L2_CID_MPEG_VIDEO_H264_SCALING_MATRIX,
            .ptr = &controls->scaling_matrix,
            .size = sizeof(controls->scaling_matrix),
        },
        {
            .id = V4L2_CID_MPEG_VIDEO_H264_SLICE_PARAMS,
            .ptr = &controls->slice_params,
            .size = sizeof(controls->slice_params) * controls->decode_params.num_slices,
        },
        {
            .id = V4L2_CID_MPEG_VIDEO_H264_DECODE_PARAMS,
            .ptr = &controls->decode_params,
            .size = sizeof(controls->decode_params),
        },
    };

    /*
     * FIXME: per-slice decoding does not work, so we're currently packing all
     * slices in single buffer and triggering the decode operation on
     * ->end_frame().
     */
    if (!is_last)
        return 0;

    if (controls->num_decoded_slices == controls->num_queued_slices)
        return 0;

    ret = ff_v4l2_request_decode_slice(avctx, h->cur_pic_ptr->f, control,
                                       FF_ARRAY_ELEMS(control),
                                       !controls->num_decoded_slices, is_last);
    if (ret)
        return ret;

    req->output.used = 0;
    controls->num_decoded_slices++;
    return 0;
}

static int v4l2_request_h264_end_frame(AVCodecContext *avctx)
{
    return v4l2_request_h264_decode_pending_slice(avctx, true);
}

static int v4l2_request_h264_decode_slice(AVCodecContext *avctx, const uint8_t *buffer, uint32_t size)
{
    const H264Context *h = avctx->priv_data;
    V4L2RequestContext *v4l2_ctx = avctx->internal->hwaccel_priv_data;
    const PPS *pps = h->ps.pps;
    const H264SliceContext *sl = &h->slice_ctx[0];
    V4L2RequestControlsH264 *controls = h->cur_pic_ptr->hwaccel_picture_private;
    V4L2RequestDescriptor *req = (V4L2RequestDescriptor*)h->cur_pic_ptr->f->data[0];
    int i, ret, count, slice = controls->decode_params.num_slices;

    assert(controls->num_queued_slices >= controls->num_decoded_slices);
//    ret = v4l2_request_h264_decode_pending_slice(avctx, false);
//    if (ret)
//        return ret;
    assert(controls->num_queued_slices == controls->num_decoded_slices);

    controls->slice_params[slice] = (struct v4l2_ctrl_h264_slice_params) {
        /* Size in bytes, including header */
        .size = 0,
	.start_byte_offset = req->output.used,
        /* Offset in bits to slice_data() from the beginning of this slice. */
        .header_bit_size = get_bits_count(&sl->gb),

        .first_mb_in_slice = sl->first_mb_addr,
        .slice_type = ff_h264_get_slice_type(sl),
        .pic_parameter_set_id = sl->pps_id,
        .colour_plane_id = 0, /* what is this? */
        .frame_num = h->poc.frame_num,
        .idr_pic_id = sl->idr_pic_id,
        .pic_order_cnt_lsb = sl->poc_lsb,
        .delta_pic_order_cnt_bottom = sl->delta_poc_bottom,
        .delta_pic_order_cnt0 = sl->delta_poc[0],
        .delta_pic_order_cnt1 = sl->delta_poc[1],
        .redundant_pic_cnt = sl->redundant_pic_count,

        /* Size in bits of dec_ref_pic_marking() syntax element. */
        .dec_ref_pic_marking_bit_size = sl->ref_pic_marking_size_in_bits,
        /* Size in bits of pic order count syntax. */
        .pic_order_cnt_bit_size = sl->pic_order_cnt_bit_size,

        .cabac_init_idc = sl->cabac_init_idc,
        .slice_qp_delta = sl->qscale - pps->init_qp,
        .slice_qs_delta = 0, /* XXX not implemented by FFmpeg */
        .disable_deblocking_filter_idc = sl->deblocking_filter < 2 ? !sl->deblocking_filter : sl->deblocking_filter,
        .slice_alpha_c0_offset_div2 = sl->slice_alpha_c0_offset / 2,
        .slice_beta_offset_div2 = sl->slice_beta_offset / 2,
        .slice_group_change_cycle = 0, /* what is this? */

        .num_ref_idx_l0_active_minus1 = sl->list_count > 0 ? sl->ref_count[0] - 1 : 0,
        .num_ref_idx_l1_active_minus1 = sl->list_count > 1 ? sl->ref_count[1] - 1 : 0,
    };

    if (FIELD_PICTURE(h))
        controls->slice_params[slice].flags |= V4L2_H264_SLICE_FLAG_FIELD_PIC;
    if (h->picture_structure == PICT_BOTTOM_FIELD)
        controls->slice_params[slice].flags |= V4L2_H264_SLICE_FLAG_BOTTOM_FIELD;
    if (sl->slice_type == AV_PICTURE_TYPE_B && sl->direct_spatial_mv_pred)
        controls->slice_params[slice].flags |= V4L2_H264_SLICE_FLAG_DIRECT_SPATIAL_MV_PRED;

    controls->slice_params[slice].pred_weight_table.chroma_log2_weight_denom = sl->pwt.chroma_log2_weight_denom;
    controls->slice_params[slice].pred_weight_table.luma_log2_weight_denom = sl->pwt.luma_log2_weight_denom;

    count = sl->list_count > 0 ? sl->ref_count[0] : 0;
    for (i = 0; i < count; i++)
        controls->slice_params[slice].ref_pic_list0[i] = get_dpb_index(&controls->decode_params, &sl->ref_list[0][i]);
    if (count)
        fill_weight_factors(&controls->slice_params[slice].pred_weight_table.weight_factors[0], 0, sl);

    count = sl->list_count > 1 ? sl->ref_count[1] : 0;
    for (i = 0; i < count; i++)
        controls->slice_params[slice].ref_pic_list1[i] = get_dpb_index(&controls->decode_params, &sl->ref_list[1][i]);
    if (count)
        fill_weight_factors(&controls->slice_params[slice].pred_weight_table.weight_factors[1], 1, sl);

    if (v4l2_ctx->start_code_support & (1 << V4L2_MPEG_VIDEO_H264_ANNEX_B_STARTCODE)) {
        ret = ff_v4l2_request_append_output_buffer(avctx, h->cur_pic_ptr->f, nalu_slice_start_code, 3);
        if (ret)
            return ret;
    }

    ret = ff_v4l2_request_append_output_buffer(avctx, h->cur_pic_ptr->f, buffer, size);
    if (ret)
	    return ret;

    /* FIXME: should be done driver side! */
    /*
    if ((size + 3) % 16) {
        ret = ff_v4l2_request_append_output_buffer(avctx, h->cur_pic_ptr->f, padding, 16 - ((size + 3) % 16));
        if (ret)
            return ret;
    }
    */

    controls->slice_params[slice].size = req->output.used;
    controls->num_queued_slices++;
    controls->decode_params.num_slices++;
    return 0;
}

static int v4l2_request_h264_init(AVCodecContext *avctx)
{
    const H264Context *h = avctx->priv_data;
    struct v4l2_ctrl_h264_sps sps;
    struct v4l2_ctrl_h264_pps pps;

    struct v4l2_ext_control control[] = {
        {
            .id = V4L2_CID_MPEG_VIDEO_H264_SPS,
            .ptr = &sps,
            .size = sizeof(sps),
        },
        {
            .id = V4L2_CID_MPEG_VIDEO_H264_PPS,
            .ptr = &pps,
            .size = sizeof(pps),
        },
//	{
//            .id = V4L2_CID_MPEG_VIDEO_H264_DECODING_MODE,
//            .value = V4L2_MPEG_VIDEO_H264_FRAME_BASED_DECODING,
//	},
//	{
//            .id = V4L2_CID_MPEG_VIDEO_H264_START_CODE,
//	    .value = V4L2_MPEG_VIDEO_H264_ANNEX_B_START_CODE,
//	},
    };

    fill_sps(&sps, h);
    fill_pps(&pps, h);

    return ff_v4l2_request_init(avctx, V4L2_PIX_FMT_H264_SLICE, 1024 * 1024, control, FF_ARRAY_ELEMS(control));
}

const AVHWAccel ff_h264_v4l2request_hwaccel = {
    .name           = "h264_v4l2request",
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_H264,
    .pix_fmt        = AV_PIX_FMT_DRM_PRIME,
    .start_frame    = v4l2_request_h264_start_frame,
    .decode_slice   = v4l2_request_h264_decode_slice,
    .end_frame      = v4l2_request_h264_end_frame,
    .frame_priv_data_size = sizeof(V4L2RequestControlsH264),
    .init           = v4l2_request_h264_init,
    .uninit         = ff_v4l2_request_uninit,
    .priv_data_size = sizeof(V4L2RequestContext),
    .frame_params   = ff_v4l2_request_frame_params,
    .caps_internal  = HWACCEL_CAP_ASYNC_SAFE,
};
