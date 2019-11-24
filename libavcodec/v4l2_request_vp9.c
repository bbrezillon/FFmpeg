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

#include "internal.h"
#include "hwaccel.h"
#include "v4l2_request.h"
#include "vp9-ctrls.h"
#include "vp9.h"
#include "vp9dec.h"
#include "vp9data.h"

typedef struct V4L2RequestControlsVP9 {
	struct v4l2_ctrl_vp9_frame_decode_params ctrl;
} V4L2RequestControlsVP9;

static const uint8_t ff_to_v4l2_intramode[] = {
    [VERT_PRED] = V4L2_VP9_INTRA_PRED_MODE_V,
    [HOR_PRED] = V4L2_VP9_INTRA_PRED_MODE_H,
    [DC_PRED] = V4L2_VP9_INTRA_PRED_MODE_DC,
    [DIAG_DOWN_LEFT_PRED] = V4L2_VP9_INTRA_PRED_MODE_D45,
    [DIAG_DOWN_RIGHT_PRED] = V4L2_VP9_INTRA_PRED_MODE_D135,
    [VERT_RIGHT_PRED] = V4L2_VP9_INTRA_PRED_MODE_D117,
    [HOR_DOWN_PRED] = V4L2_VP9_INTRA_PRED_MODE_D153,
    [VERT_LEFT_PRED] = V4L2_VP9_INTRA_PRED_MODE_D63,
    [HOR_UP_PRED] = V4L2_VP9_INTRA_PRED_MODE_D207,
    [TM_VP8_PRED] = V4L2_VP9_INTRA_PRED_MODE_TM,
};

static int v4l2_request_vp9_set_frame_ctx(AVCodecContext *avctx, unsigned int id)
{
    VP9Context *s = avctx->priv_data;
    V4L2RequestContext *ctx = avctx->internal->hwaccel_priv_data;
    struct v4l2_ctrl_vp9_frame_ctx fctx = {};
    struct v4l2_ext_control control = {
        .id = V4L2_CID_MPEG_VIDEO_VP9_FRAME_CONTEXT(id),
	.ptr = &fctx,
	.size = sizeof(fctx),
    };
    struct v4l2_ext_controls controls = {
        .controls = &control,
        .count = 1,
        .request_fd = -1,
        .which = 0,
    };
    int ret;

    printf("%s:%i id %d\n", __func__, __LINE__, id);

    memcpy(fctx.probs.tx_probs_8x8, s->prob_ctx[id].p.tx8p, sizeof(s->prob_ctx[id].p.tx8p));
    memcpy(fctx.probs.tx_probs_16x16, s->prob_ctx[id].p.tx16p, sizeof(s->prob_ctx[id].p.tx16p));
    memcpy(fctx.probs.tx_probs_32x32, s->prob_ctx[id].p.tx32p, sizeof(s->prob_ctx[id].p.tx32p));
    printf("%s:%i tx32_probs[0][0] %02x\n", __func__, __LINE__, fctx.probs.tx_probs_32x32[0][0]);
    memcpy(fctx.probs.coef_probs, s->prob_ctx[id].coef, sizeof(s->prob_ctx[id].coef));
    /*
    memset(fctx.probs.coef_probs, 0, sizeof(fctx.probs.coef_probs));
    for (unsigned i = 0; i < 4; i++)
        for (unsigned j = 0; j < 2; j++)
            for (unsigned k = 0; k < 2; k++)
                for (unsigned l = 0; l < 6; l++)
                    for (unsigned m = 0; m < 6; m++)
                        memcpy(fctx.probs.coef_probs[i][j][k][l][m],
                               s->prob.coef[i][j][k][l][m],
			       sizeof(fctx.probs.coef_probs[0][0][0][0][0]));
    */
    memcpy(fctx.probs.skip_prob, s->prob_ctx[id].p.skip, sizeof(s->prob_ctx[id].p.skip));
    memcpy(fctx.probs.inter_mode_probs, s->prob_ctx[id].p.mv_mode, sizeof(s->prob_ctx[id].p.mv_mode));
    memcpy(fctx.probs.interp_filter_probs, s->prob_ctx[id].p.filter, sizeof(s->prob_ctx[id].p.filter));
    memcpy(fctx.probs.is_inter_prob, s->prob_ctx[id].p.intra, sizeof(s->prob_ctx[id].p.intra));
    memcpy(fctx.probs.comp_mode_prob, s->prob_ctx[id].p.comp, sizeof(s->prob_ctx[id].p.comp));
    memcpy(fctx.probs.single_ref_prob, s->prob_ctx[id].p.single_ref, sizeof(s->prob_ctx[id].p.single_ref));
    memcpy(fctx.probs.comp_ref_prob, s->prob_ctx[id].p.comp_ref, sizeof(s->prob_ctx[id].p.comp_ref));
    memcpy(fctx.probs.y_mode_probs, s->prob_ctx[id].p.y_mode, sizeof(s->prob_ctx[id].p.y_mode));
    printf("%s:%i y_mode_probs[0][0] %02x\n", __func__, __LINE__, fctx.probs.y_mode_probs[0][0]);
    for (unsigned i = 0; i < 10; i++)
        memcpy(fctx.probs.uv_mode_probs[ff_to_v4l2_intramode[i]],
               s->prob_ctx[id].p.uv_mode[i], sizeof(s->prob_ctx[id].p.uv_mode[0]));
        
    memcpy(fctx.probs.uv_mode_probs, s->prob_ctx[id].p.uv_mode, sizeof(s->prob_ctx[id].p.uv_mode));
    for (unsigned i = 0; i < 4; i++)
        memcpy(fctx.probs.partition_probs[i], s->prob.p.partition[3-i],
	       sizeof(s->prob.p.partition[0]));
    memcpy(fctx.probs.mv_joint_probs, s->prob_ctx[id].p.mv_joint, sizeof(s->prob_ctx[id].p.mv_joint));
    for (unsigned i = 0; i < 2; i++) {
         fctx.probs.mv_sign_prob[i] = s->prob_ctx[id].p.mv_comp[i].sign;
	 memcpy(fctx.probs.mv_class_probs[i], s->prob_ctx[id].p.mv_comp[i].classes,
                sizeof(s->prob_ctx[id].p.mv_comp[0].classes));
         fctx.probs.mv_class0_bit_prob[i] = s->prob_ctx[id].p.mv_comp[i].class0;
	 memcpy(fctx.probs.mv_bits_prob[i], s->prob_ctx[id].p.mv_comp[i].bits,
                sizeof(s->prob_ctx[id].p.mv_comp[0].bits));
	 memcpy(fctx.probs.mv_class0_fr_probs[i], s->prob_ctx[id].p.mv_comp[i].class0_fp,
                sizeof(s->prob_ctx[id].p.mv_comp[0].class0_fp));
	 memcpy(fctx.probs.mv_fr_probs[i], s->prob_ctx[id].p.mv_comp[i].fp,
                sizeof(s->prob_ctx[id].p.mv_comp[0].fp));
	 fctx.probs.mv_class0_hp_prob[i] = s->prob_ctx[id].p.mv_comp[i].class0_hp;
	 fctx.probs.mv_hp_prob[i] = s->prob_ctx[id].p.mv_comp[i].hp;
    }

    return ioctl(ctx->video_fd, VIDIOC_S_EXT_CTRLS, &controls);
}

static int v4l2_request_vp9_get_frame_ctx(AVCodecContext *avctx, unsigned int id)
{
    VP9Context *s = avctx->priv_data;
    V4L2RequestContext *ctx = avctx->internal->hwaccel_priv_data;
    struct v4l2_ctrl_vp9_frame_ctx fctx = {};
    struct v4l2_ext_control control = {
        .id = V4L2_CID_MPEG_VIDEO_VP9_FRAME_CONTEXT(id),
	.ptr = &fctx,
	.size = sizeof(fctx),
    };
    struct v4l2_ext_controls controls = {
        .controls = &control,
        .count = 1,
        .request_fd = -1,
        .which = 0,
    };
    int ret;

    ret = ioctl(ctx->video_fd, VIDIOC_G_EXT_CTRLS, &controls);
    if (ret)
        return ret;

    memcpy(s->prob_ctx[id].p.tx8p, fctx.probs.tx_probs_8x8, sizeof(s->prob_ctx[id].p.tx8p));
    memcpy(s->prob_ctx[id].p.tx16p, fctx.probs.tx_probs_16x16, sizeof(s->prob_ctx[id].p.tx16p));
    memcpy(s->prob_ctx[id].p.tx32p, fctx.probs.tx_probs_32x32, sizeof(s->prob_ctx[id].p.tx32p));
    memcpy(s->prob_ctx[id].coef, fctx.probs.coef_probs, sizeof(s->prob_ctx[id].coef));
    memcpy(s->prob_ctx[id].p.skip, fctx.probs.skip_prob, sizeof(s->prob_ctx[id].p.skip));
    memcpy(s->prob_ctx[id].p.mv_mode, fctx.probs.inter_mode_probs, sizeof(s->prob_ctx[id].p.mv_mode));
    memcpy(s->prob_ctx[id].p.filter, fctx.probs.interp_filter_probs, sizeof(s->prob_ctx[id].p.filter));
    memcpy(s->prob_ctx[id].p.intra, fctx.probs.is_inter_prob, sizeof(s->prob_ctx[id].p.intra));
    memcpy(s->prob_ctx[id].p.comp, fctx.probs.comp_mode_prob, sizeof(s->prob_ctx[id].p.comp));
    memcpy(s->prob_ctx[id].p.single_ref, fctx.probs.single_ref_prob, sizeof(s->prob_ctx[id].p.single_ref));
    memcpy(s->prob_ctx[id].p.comp_ref, fctx.probs.comp_ref_prob, sizeof(s->prob_ctx[id].p.comp_ref));
    /* FIXME: prediction_mode conversion? */
    memcpy(s->prob_ctx[id].p.y_mode, fctx.probs.y_mode_probs, sizeof(s->prob_ctx[id].p.y_mode));
    for (unsigned i = 0; i < 10; i++)
        memcpy(s->prob_ctx[id].p.uv_mode[i],
               fctx.probs.uv_mode_probs[ff_to_v4l2_intramode[i]],
               sizeof(s->prob_ctx[id].p.uv_mode[0]));
    for (unsigned i = 0; i < 4; i++)
        memcpy(s->prob.p.partition[3-i], fctx.probs.partition_probs[i],
	       sizeof(s->prob.p.partition[0]));
    memcpy(s->prob_ctx[id].p.mv_joint, fctx.probs.mv_joint_probs, sizeof(s->prob_ctx[id].p.mv_joint));
    for (unsigned i = 0; i < 2; i++) {
         s->prob_ctx[id].p.mv_comp[i].sign = fctx.probs.mv_sign_prob[i];
	 memcpy(s->prob_ctx[id].p.mv_comp[i].classes, fctx.probs.mv_class_probs[i],
                sizeof(s->prob_ctx[id].p.mv_comp[0].classes));
         s->prob_ctx[id].p.mv_comp[i].class0 = fctx.probs.mv_class0_bit_prob[i];
	 memcpy(s->prob_ctx[id].p.mv_comp[i].bits, fctx.probs.mv_bits_prob[i],
                sizeof(s->prob_ctx[id].p.mv_comp[0].bits));
	 memcpy(s->prob_ctx[id].p.mv_comp[i].class0_fp, fctx.probs.mv_class0_fr_probs[i],
                sizeof(s->prob_ctx[id].p.mv_comp[0].class0_fp));
	 memcpy(s->prob_ctx[id].p.mv_comp[i].fp, fctx.probs.mv_fr_probs[i],
                sizeof(s->prob_ctx[id].p.mv_comp[0].fp));
	 s->prob_ctx[id].p.mv_comp[i].class0_hp = fctx.probs.mv_class0_hp_prob[i];
	 s->prob_ctx[id].p.mv_comp[i].hp = fctx.probs.mv_hp_prob[i];
    }

    printf("%s:%i id %d\n", __func__, __LINE__, id);
    return 0;
}

static int v4l2_request_vp9_start_frame(AVCodecContext          *avctx,
                                 av_unused const uint8_t *buffer,
                                 av_unused uint32_t       size)
{
    const VP9Context *s = avctx->priv_data;
    V4L2RequestControlsVP9 *controls = s->s.frames[CUR_FRAME].hwaccel_picture_private;
    int ret;

    memset(&controls->ctrl, 0, sizeof(controls->ctrl));

    if (s->s.h.keyframe || s->s.h.errorres || (s->s.h.intraonly && s->s.h.resetctx == 3)) {
        for (unsigned i = 0; i < 4; i++) {
            if (memcmp(&s->prob_ctx[i].p, &ff_vp9_default_probs, sizeof(ff_vp9_default_probs)) ||
                memcmp(s->prob_ctx[i].coef, ff_vp9_default_coef_probs, sizeof(ff_vp9_default_coef_probs)))
		    exit(1);
            ret = v4l2_request_vp9_set_frame_ctx(avctx, i);
	    if (ret)
                return ret;
        }
    } else if (s->s.h.intraonly && s->s.h.resetctx == 2) {
            if (memcmp(&s->prob_ctx[s->s.h.framectxid].p, &ff_vp9_default_probs, sizeof(ff_vp9_default_probs)) ||
                memcmp(s->prob_ctx[s->s.h.framectxid].coef, ff_vp9_default_coef_probs, sizeof(ff_vp9_default_coef_probs)))
		    exit(1);
        ret = v4l2_request_vp9_set_frame_ctx(avctx, s->s.h.framectxid);
	if (ret)
            return ret;
    }

    return ff_v4l2_request_reset_frame(avctx, s->s.frames[CUR_FRAME].tf.f);
}

static int v4l2_request_vp9_end_frame(AVCodecContext *avctx)
{
    const VP9Context *s = avctx->priv_data;
    V4L2RequestControlsVP9 *controls = s->s.frames[CUR_FRAME].hwaccel_picture_private;
    struct v4l2_ext_control control[] = {
        {
            .id = V4L2_CID_MPEG_VIDEO_VP9_FRAME_DECODE_PARAMS,
	    .ptr = &controls->ctrl,
	    .size = sizeof(controls->ctrl),
        },
    };
    int ret;

    ret = ff_v4l2_request_decode_frame(avctx, s->s.frames[CUR_FRAME].tf.f,
				       control, FF_ARRAY_ELEMS(control));
    if (ret)
        return ret;

    sleep(1);
    if (!s->s.h.refreshctx)
        return 0;

    return v4l2_request_vp9_get_frame_ctx(avctx, s->s.h.framectxid);
}

static int v4l2_request_vp9_decode_slice(AVCodecContext *avctx,
					 const uint8_t *buffer,
					 uint32_t size)
{
    const VP9Context *s = avctx->priv_data;
    const VP9Frame *f = &s->s.frames[CUR_FRAME];
    V4L2RequestControlsVP9 *controls = f->hwaccel_picture_private;
    struct v4l2_ctrl_vp9_frame_decode_params *dec_params = &controls->ctrl;

    if (s->s.h.keyframe)
        dec_params->flags |= V4L2_VP9_FRAME_FLAG_KEY_FRAME;
    if (!s->s.h.invisible)
        dec_params->flags |= V4L2_VP9_FRAME_FLAG_SHOW_FRAME;
    if (s->s.h.errorres)
        dec_params->flags |= V4L2_VP9_FRAME_FLAG_ERROR_RESILIENT;
    if (s->s.h.intraonly)
        dec_params->flags |= V4L2_VP9_FRAME_FLAG_INTRA_ONLY;
    if (s->s.h.highprecisionmvs)
        dec_params->flags |= V4L2_VP9_FRAME_FLAG_ALLOW_HIGH_PREC_MV;
    if (s->s.h.refreshctx)
        dec_params->flags |= V4L2_VP9_FRAME_FLAG_REFRESH_FRAME_CTX;
    if (s->s.h.parallelmode)
        dec_params->flags |= V4L2_VP9_FRAME_FLAG_PARALLEL_DEC_MODE;
    if (s->ss_h)
        dec_params->flags |= V4L2_VP9_FRAME_FLAG_X_SUBSAMPLING;
    if (s->ss_v)
        dec_params->flags |= V4L2_VP9_FRAME_FLAG_Y_SUBSAMPLING;
    if (avctx->color_range == AVCOL_RANGE_JPEG)
        dec_params->flags |= V4L2_VP9_FRAME_FLAG_COLOR_RANGE_FULL_SWING;

    dec_params->header_size_in_bytes = s->s.h.compressed_header_size;
    dec_params->profile = s->s.h.profile;
    printf("%s:%i profile %d\n", __func__, __LINE__, dec_params->profile);
    dec_params->reset_frame_context = s->s.h.resetctx;
    dec_params->frame_context_idx = s->s.h.framectxid;
    dec_params->bit_depth = s->s.h.bpp;
    printf("%s:%i bit_depth %d\n", __func__, __LINE__, dec_params->bit_depth);

    switch (avctx->colorspace) {
    case AVCOL_SPC_UNSPECIFIED:
        dec_params->color_space = 0;
	break;
    case AVCOL_SPC_BT470BG:
        dec_params->color_space = 1;
	break;
    case AVCOL_SPC_BT709:
        dec_params->color_space = 2;
	break;
    case AVCOL_SPC_SMPTE170M:
        dec_params->color_space = 3;
	break;
    case AVCOL_SPC_SMPTE240M:
        dec_params->color_space = 4;
	break;
    case AVCOL_SPC_BT2020_NCL: 
        dec_params->color_space = 5;
	break;
    case AVCOL_SPC_RESERVED:
        dec_params->color_space = 6;
	break;
    case AVCOL_SPC_RGB:
        dec_params->color_space = 7;
	break;
    }

    dec_params->interpolation_filter = s->s.h.filtermode;
    dec_params->tile_cols_log2 = s->s.h.tiling.log2_tile_cols;
    dec_params->tile_rows_log2 = s->s.h.tiling.log2_tile_rows;
    dec_params->tx_mode = s->s.h.txfmmode;
    dec_params->reference_mode = s->s.h.comppredmode;
    dec_params->frame_width_minus_1 = s->w - 1;
    dec_params->frame_height_minus_1 = s->h - 1;
    printf("%s:%i W/H %d/%d\n", __func__, __LINE__, s->w, s->h);
    /* render width/height are ignored for now. */

    for (unsigned i = 0; i < 3; i++) {
        const ThreadFrame *ref = &s->s.refs[s->s.h.refidx[i]];
	const struct V4L2RequestDescriptor *req = ref->f ? ref->f->data[0] : NULL;

	dec_params->refs[i] = req ? req->capture.index + 1 : 0;
        printf("%s:%i refs[%d] %lld\n", __func__, __LINE__, i, dec_params->refs[i]);
    }

    if (s->s.h.lf_delta.enabled)
        dec_params->lf.flags |= V4L2_VP9_LOOP_FILTER_FLAG_DELTA_ENABLED;
    if (s->s.h.lf_delta.updated)
        dec_params->lf.flags |= V4L2_VP9_LOOP_FILTER_FLAG_DELTA_UPDATE;
    printf("%s:%i LF flags %x\n", __func__, __LINE__, dec_params->lf.flags);

    dec_params->lf.level = s->s.h.filter.level;
    dec_params->lf.sharpness = s->s.h.filter.sharpness;
    memcpy(dec_params->lf.ref_deltas, s->s.h.lf_delta.ref,
           sizeof(dec_params->lf.ref_deltas));
    memcpy(dec_params->lf.mode_deltas, s->s.h.lf_delta.mode,
           sizeof(dec_params->lf.mode_deltas));
    for (unsigned i = 0; i < 8; i++) {
        for (unsigned j = 0; j < 4; j++)
            memcpy(dec_params->lf.lvl_lookup[i][j],
                   s->s.h.segmentation.feat[i].lflvl[j],
		   sizeof(dec_params->lf.lvl_lookup[0][0]));
    }

    dec_params->quant.base_q_idx = s->s.h.yac_qi;
    dec_params->quant.delta_q_y_dc = s->s.h.ydc_qdelta;
    dec_params->quant.delta_q_uv_dc = s->s.h.uvdc_qdelta;
    dec_params->quant.delta_q_uv_ac = s->s.h.uvac_qdelta;
    if (s->s.h.lossless)
        dec_params->quant.flags |= V4L2_VP9_QUANTIZATION_FLAG_LOSSLESS;

    if (s->s.h.segmentation.enabled)
        dec_params->seg.flags |= V4L2_VP9_SEGMENTATION_FLAG_ENABLED;
    if (s->s.h.segmentation.update_map)
        dec_params->seg.flags |= V4L2_VP9_SEGMENTATION_FLAG_UPDATE_MAP;
    if (s->s.h.segmentation.temporal)
        dec_params->seg.flags |= V4L2_VP9_SEGMENTATION_FLAG_TEMPORAL_UPDATE;
    if (s->s.h.segmentation.update_data)
        dec_params->seg.flags |= V4L2_VP9_SEGMENTATION_FLAG_UPDATE_DATA;
    if (s->s.h.segmentation.absolute_vals)
        dec_params->seg.flags |= V4L2_VP9_SEGMENTATION_FLAG_ABS_OR_DELTA_UPDATE;

    printf("%s:%i seg flags %x\n", __func__, __LINE__, dec_params->seg.flags);
    memcpy(dec_params->seg.tree_probs, s->s.h.segmentation.prob,
           sizeof(dec_params->seg.tree_probs));
    memcpy(dec_params->seg.pred_probs, s->s.h.segmentation.pred_prob,
           sizeof(dec_params->seg.pred_probs));

    for (unsigned i = 0; i < 8; i++) {
        if (s->s.h.segmentation.feat[i].q_enabled) {
            dec_params->seg.feature_enabled[i] |= 1 << V4L2_VP9_SEGMENTATION_FEATURE_QP_DELTA;
            dec_params->seg.feature_data[i][V4L2_VP9_SEGMENTATION_FEATURE_QP_DELTA] = s->s.h.segmentation.feat[i].q_val;
	}

        if (s->s.h.segmentation.feat[i].lf_enabled) {
            dec_params->seg.feature_enabled[i] |= 1 << V4L2_VP9_SEGMENTATION_FEATURE_LF_VAL;
            dec_params->seg.feature_data[i][V4L2_VP9_SEGMENTATION_FEATURE_LF_VAL] = s->s.h.segmentation.feat[i].lf_val;
	}

        if (s->s.h.segmentation.feat[i].ref_enabled) {
            dec_params->seg.feature_enabled[i] |= 1 << V4L2_VP9_SEGMENTATION_FEATURE_REFERINFO;
            dec_params->seg.feature_data[i][V4L2_VP9_SEGMENTATION_FEATURE_REFERINFO] = s->s.h.segmentation.feat[i].ref_val;
	}

        if (s->s.h.segmentation.feat[i].skip_enabled)
            dec_params->seg.feature_enabled[i] |= 1 << V4L2_VP9_SEGMENTATION_FEATURE_FRAME_SKIP;
    }

    memcpy(dec_params->probs.tx_probs_8x8, s->prob.p.tx8p, sizeof(s->prob.p.tx8p));
    memcpy(dec_params->probs.tx_probs_16x16, s->prob.p.tx16p, sizeof(s->prob.p.tx16p));
    memcpy(dec_params->probs.tx_probs_32x32, s->prob.p.tx32p, sizeof(s->prob.p.tx32p));
    for (unsigned i = 0; i < 4; i++)
        for (unsigned j = 0; j < 2; j++)
            for (unsigned k = 0; k < 2; k++)
                for (unsigned l = 0; l < 6; l++)
                    for (unsigned m = 0; m < 6; m++)
                        memcpy(dec_params->probs.coef_probs[i][j][k][l][m],
                               s->prob.coef[i][j][k][l][m],
			       sizeof(dec_params->probs.coef_probs[0][0][0][0][0]));
//    memcpy(dec_params->probs.coef_probs, s->prob.coef, sizeof(dec_params->probs.coef_probs));
    memset(dec_params->probs.coef_probs, 0, sizeof(dec_params->probs.coef_probs));
    memcpy(dec_params->probs.skip_prob, s->prob.p.skip, sizeof(s->prob.p.skip));
    memcpy(dec_params->probs.inter_mode_probs, s->prob.p.mv_mode, sizeof(s->prob.p.mv_mode));
    memcpy(dec_params->probs.interp_filter_probs, s->prob.p.filter, sizeof(s->prob.p.filter));
    memcpy(dec_params->probs.is_inter_prob, s->prob.p.intra, sizeof(s->prob.p.intra));
    memcpy(dec_params->probs.comp_mode_prob, s->prob.p.comp, sizeof(s->prob.p.comp));
    memcpy(dec_params->probs.single_ref_prob, s->prob.p.single_ref, sizeof(s->prob.p.single_ref));
    memcpy(dec_params->probs.comp_ref_prob, s->prob.p.comp_ref, sizeof(s->prob.p.comp_ref));
    /* FIXME: prediction_mode conversion? */
    memcpy(dec_params->probs.y_mode_probs, s->prob.p.y_mode, sizeof(s->prob.p.y_mode));
    for (unsigned i = 0; i < 10; i++)
        memcpy(dec_params->probs.uv_mode_probs[ff_to_v4l2_intramode[i]],
               s->prob.p.uv_mode[i], sizeof(s->prob.p.uv_mode[0]));
    for (unsigned i = 0; i < 4; i++)
        memcpy(dec_params->probs.partition_probs[i], s->prob.p.partition[3-i],
	       sizeof(s->prob.p.partition[0]));
    memcpy(dec_params->probs.partition_probs, s->prob.p.partition, sizeof(s->prob.p.partition));
    memcpy(dec_params->probs.mv_joint_probs, s->prob.p.mv_joint, sizeof(s->prob.p.mv_joint));
    for (unsigned i = 0; i < 2; i++) {
         dec_params->probs.mv_sign_prob[i] = s->prob.p.mv_comp[i].sign;
	 memcpy(dec_params->probs.mv_class_probs[i], s->prob.p.mv_comp[i].classes,
                sizeof(s->prob.p.mv_comp[0].classes));
         dec_params->probs.mv_class0_bit_prob[i] = s->prob.p.mv_comp[i].class0;
	 memcpy(dec_params->probs.mv_bits_prob[i], s->prob.p.mv_comp[i].bits,
                sizeof(s->prob.p.mv_comp[0].bits));
	 memcpy(dec_params->probs.mv_class0_fr_probs[i], s->prob.p.mv_comp[i].class0_fp,
                sizeof(s->prob.p.mv_comp[0].class0_fp));
	 memcpy(dec_params->probs.mv_fr_probs[i], s->prob.p.mv_comp[i].fp,
                sizeof(s->prob.p.mv_comp[0].fp));
	 dec_params->probs.mv_class0_hp_prob[i] = s->prob.p.mv_comp[i].class0_hp;
	 dec_params->probs.mv_hp_prob[i] = s->prob.p.mv_comp[i].hp;
    }

    printf("%s:%i flags %x size %d\n", __func__, __LINE__, dec_params->flags, size);
    ff_v4l2_request_append_output_buffer(avctx, s->s.frames[CUR_FRAME].tf.f, buffer, size);
    unsigned padding = 16 - (size & 15) + 0x80;
    printf("%s:%i padding %x\n", __func__, __LINE__, padding);
    uint8_t pad[0x90] = {};
    return ff_v4l2_request_append_output_buffer(avctx, s->s.frames[CUR_FRAME].tf.f, pad, padding);
     
/*
    return ff_v4l2_request_append_output_buffer(avctx, s->s.frames[CUR_FRAME].tf.f,
						buffer + s->s.h.uncompressed_header_size +
						s->s.h.compressed_header_size,
						size - s->s.h.uncompressed_header_size -
						s->s.h.compressed_header_size);
						*/
}

static int v4l2_request_vp9_init(AVCodecContext *avctx)
{
    return ff_v4l2_request_init(avctx, V4L2_PIX_FMT_VP9_FRAME,
				1024 * 1024, NULL, 0);
}

const AVHWAccel ff_vp9_v4l2request_hwaccel = {
    .name                 = "vp9_v4l2request",
    .type                 = AVMEDIA_TYPE_VIDEO,
    .id                   = AV_CODEC_ID_VP9,
    .pix_fmt              = AV_PIX_FMT_DRM_PRIME,
    .start_frame          = v4l2_request_vp9_start_frame,
    .decode_slice         = v4l2_request_vp9_decode_slice,
    .end_frame            = v4l2_request_vp9_end_frame,
    .frame_priv_data_size = sizeof(V4L2RequestControlsVP9),
    .init                 = v4l2_request_vp9_init,
    .uninit               = ff_v4l2_request_uninit,
    .priv_data_size       = sizeof(V4L2RequestContext),
    .frame_params         = ff_v4l2_request_frame_params,
    .caps_internal        = HWACCEL_CAP_ASYNC_SAFE,
};
