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

#include <drm_fourcc.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "decode.h"
#include "internal.h"
#include "v4l2_request.h"
#include "media.h"
#include "videodev2.h"

#define V4L2_REQUEST_VIDEO_PATH "/dev/video0"
#define V4L2_REQUEST_MEDIA_PATH "/dev/media0"

uint64_t ff_v4l2_request_get_capture_timestamp(AVFrame *frame)
{
    V4L2RequestDescriptor *req = (V4L2RequestDescriptor*)frame->data[0];
    return req ? v4l2_timeval_to_ns(&req->capture.buffer.timestamp) : 0;
}

int ff_v4l2_request_reset_frame(AVCodecContext *avctx, AVFrame *frame)
{
    V4L2RequestDescriptor *req = (V4L2RequestDescriptor*)frame->data[0];
    memset(&req->drm, 0, sizeof(AVDRMFrameDescriptor));
    req->output.used = 0;
    return 0;
}

int ff_v4l2_request_append_output_buffer(AVCodecContext *avctx, AVFrame *frame, const uint8_t *data, uint32_t size)
{
    V4L2RequestDescriptor *req = (V4L2RequestDescriptor*)frame->data[0];
    memcpy(req->output.addr + req->output.used, data, size);
    req->output.used += size;
    return 0;
}

static int v4l2_request_set_controls(V4L2RequestContext *ctx, int request_fd, struct v4l2_ext_control *control, int count)
{
    struct v4l2_ext_controls controls = {
        .controls = control,
        .count = count,
        .request_fd = request_fd,
        .which = (request_fd >= 0) ? V4L2_CTRL_WHICH_REQUEST_VAL : 0,
    };

    if (!control || !count)
        return 0;

    return ioctl(ctx->video_fd, VIDIOC_S_EXT_CTRLS, &controls);
}

static int v4l2_request_queue_buffer(V4L2RequestContext *ctx, int request_fd, V4L2RequestBuffer *buf)
{
    struct v4l2_plane planes[1] = {};
    struct v4l2_buffer buffer = {
        .type = buf->buffer.type,
        .memory = buf->buffer.memory,
        .index = buf->index,
        .timestamp.tv_usec = buf->index,
        .bytesused = buf->used,
        .request_fd = request_fd,
        .flags = (request_fd >= 0) ? V4L2_BUF_FLAG_REQUEST_FD : 0,
    };

    if (V4L2_TYPE_IS_MULTIPLANAR(buf->buffer.type)) {
        planes[0].bytesused = buf->used;
        buffer.bytesused = 0;
        buffer.length = 1;
        buffer.m.planes = planes;
    }

    return ioctl(ctx->video_fd, VIDIOC_QBUF, &buffer);
}

static int v4l2_request_dequeue_buffer(V4L2RequestContext *ctx, V4L2RequestBuffer *buf)
{
    int ret;
    struct v4l2_plane planes[1] = {};
    struct v4l2_buffer buffer = {
        .type = buf->buffer.type,
        .memory = buf->buffer.memory,
        .index = buf->index,
    };

    if (V4L2_TYPE_IS_MULTIPLANAR(buf->buffer.type)) {
        buffer.length = 1;
        buffer.m.planes = planes;
    }

    ret = ioctl(ctx->video_fd, VIDIOC_DQBUF, &buffer);
    if (ret < 0)
        return ret;

    buf->buffer.timestamp = buffer.timestamp;
    return 0;
}

const uint32_t v4l2_request_capture_pixelformats[] = {
    V4L2_PIX_FMT_NV12,
#ifdef DRM_FORMAT_MOD_ALLWINNER_TILED
    V4L2_PIX_FMT_SUNXI_TILED_NV12,
#endif
};

static int v4l2_request_set_drm_descriptor(V4L2RequestDescriptor *req, struct v4l2_format *format)
{
    AVDRMFrameDescriptor *desc = &req->drm;
    AVDRMLayerDescriptor *layer = &desc->layers[0];
    uint32_t pixelformat = V4L2_TYPE_IS_MULTIPLANAR(format->type) ? format->fmt.pix_mp.pixelformat : format->fmt.pix.pixelformat;

    switch (pixelformat) {
    case V4L2_PIX_FMT_NV12:
        layer->format = DRM_FORMAT_NV12;
        desc->objects[0].format_modifier = DRM_FORMAT_MOD_LINEAR;
        break;
#ifdef DRM_FORMAT_MOD_ALLWINNER_TILED
    case V4L2_PIX_FMT_SUNXI_TILED_NV12:
        layer->format = DRM_FORMAT_NV12;
        desc->objects[0].format_modifier = DRM_FORMAT_MOD_ALLWINNER_TILED;
        break;
#endif
    default:
        return -1;
    }

    desc->nb_objects = 1;
    desc->objects[0].fd = req->capture.fd;
    desc->objects[0].size = req->capture.size;

    desc->nb_layers = 1;
    layer->nb_planes = 2;

    layer->planes[0].object_index = 0;
    layer->planes[0].offset = 0;
    layer->planes[0].pitch = V4L2_TYPE_IS_MULTIPLANAR(format->type) ? format->fmt.pix_mp.plane_fmt[0].bytesperline : format->fmt.pix.bytesperline;

    layer->planes[1].object_index = 0;
    layer->planes[1].offset = layer->planes[0].pitch * (V4L2_TYPE_IS_MULTIPLANAR(format->type) ? format->fmt.pix_mp.height : format->fmt.pix.height);
    layer->planes[1].pitch = layer->planes[0].pitch;

    return 0;
}

int ff_v4l2_request_decode_frame(AVCodecContext *avctx, AVFrame *frame, struct v4l2_ext_control *control, int count)
{
    V4L2RequestContext *ctx = avctx->internal->hwaccel_priv_data;
    V4L2RequestDescriptor *req = (V4L2RequestDescriptor*)frame->data[0];
    struct timeval tv = { 0, 500000 };
    fd_set except_fds;
    int ret;

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p used=%u controls=%d index=%d fd=%d request_fd=%d\n", __func__, avctx, req->output.used, count, req->capture.index, req->capture.fd, req->request_fd);

    ret = v4l2_request_set_controls(ctx, req->request_fd, control, count);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: set controls failed for request %d, %s (%d)\n", __func__, req->request_fd, strerror(errno), errno);
        return -1;
    }

    ret = v4l2_request_queue_buffer(ctx, req->request_fd, &req->output);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: queue output buffer %d failed for request %d, %s (%d)\n", __func__, req->output.index, req->request_fd, strerror(errno), errno);
        return -1;
    }

    ret = v4l2_request_queue_buffer(ctx, -1, &req->capture);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: queue capture buffer %d failed for request %d, %s (%d)\n", __func__, req->capture.index, req->request_fd, strerror(errno), errno);
        return -1;
    }

    // NOTE: do we need to dequeue when request fails/timeout?

    // 4. queue request and wait
    ret = ioctl(req->request_fd, MEDIA_REQUEST_IOC_QUEUE, NULL);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: queue request %d failed, %s (%d)\n", __func__, req->request_fd, strerror(errno), errno);
        goto fail;
    }

    FD_ZERO(&except_fds);
    FD_SET(req->request_fd, &except_fds);

    ret = select(req->request_fd + 1, NULL, NULL, &except_fds, &tv);
    if (ret == 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: request %d timeout\n", __func__, req->request_fd);
        goto fail;
    } else if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: select request %d failed, %s (%d)\n", __func__, req->request_fd, strerror(errno), errno);
        goto fail;
    }

    ret = v4l2_request_dequeue_buffer(ctx, &req->output);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: dequeue output buffer %d failed for request %d, %s (%d)\n", __func__, req->output.index, req->request_fd, strerror(errno), errno);
        return -1;
    }

    ret = v4l2_request_dequeue_buffer(ctx, &req->capture);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: dequeue capture buffer %d failed for request %d, %s (%d)\n", __func__, req->capture.index, req->request_fd, strerror(errno), errno);
        return -1;
    }

    // TODO: check errors
    // buffer.flags & V4L2_BUF_FLAG_ERROR

    ret = ioctl(req->request_fd, MEDIA_REQUEST_IOC_REINIT, NULL);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: reinit request %d failed, %s (%d)\n", __func__, req->request_fd, strerror(errno), errno);
        return -1;
    }

    return v4l2_request_set_drm_descriptor(req, &ctx->format);

fail:
    ret = v4l2_request_dequeue_buffer(ctx, &req->output);
    if (ret < 0)
        av_log(avctx, AV_LOG_ERROR, "%s: dequeue output buffer %d failed for request %d, %s (%d)\n", __func__, req->output.index, req->request_fd, strerror(errno), errno);

    ret = v4l2_request_dequeue_buffer(ctx, &req->capture);
    if (ret < 0)
        av_log(avctx, AV_LOG_ERROR, "%s: dequeue capture buffer %d failed for request %d, %s (%d)\n", __func__, req->capture.index, req->request_fd, strerror(errno), errno);

    ret = ioctl(req->request_fd, MEDIA_REQUEST_IOC_REINIT, NULL);
    if (ret < 0)
        av_log(avctx, AV_LOG_ERROR, "%s: reinit request %d failed, %s (%d)\n", __func__, req->request_fd, strerror(errno), errno);

    return -1;
}

static int v4l2_request_set_format(AVCodecContext *avctx, enum v4l2_buf_type type, uint32_t pixelformat, uint32_t buffersize)
{
    V4L2RequestContext *ctx = avctx->internal->hwaccel_priv_data;
    struct v4l2_format format = {0};

    format.type = type;
    if (V4L2_TYPE_IS_MULTIPLANAR(type)) {
        format.fmt.pix_mp.width = avctx->coded_width;
        format.fmt.pix_mp.height = avctx->coded_height;
        format.fmt.pix_mp.pixelformat = pixelformat;
        format.fmt.pix_mp.plane_fmt[0].sizeimage = buffersize;
        format.fmt.pix_mp.num_planes = 1;
    } else {
        format.fmt.pix.width = avctx->coded_width;
        format.fmt.pix.height = avctx->coded_height;
        format.fmt.pix.pixelformat = pixelformat;
        format.fmt.pix.sizeimage = buffersize;
    }

    return ioctl(ctx->video_fd, VIDIOC_S_FMT, &format);
}

static int v4l2_request_select_capture_format(AVCodecContext *avctx)
{
    V4L2RequestContext *ctx = avctx->internal->hwaccel_priv_data;

    // TODO: use a supported format (v4l2_request_capture_pixelformats) returned by VIDIOC_G_FMT or VIDIOC_ENUM_FMT

    return v4l2_request_set_format(avctx, ctx->format.type, V4L2_PIX_FMT_NV12, 0);
}

int ff_v4l2_request_init(AVCodecContext *avctx, uint32_t pixelformat, uint32_t buffersize, struct v4l2_ext_control *control, int count)
{
    V4L2RequestContext *ctx = avctx->internal->hwaccel_priv_data;
    int ret;
    struct media_device_info device_info = {0};
    struct v4l2_capability capability = {0};
    unsigned int capabilities = 0;
    const char *video_path = getenv("FFMPEG_V4L2_REQUEST_VIDEO_PATH");
    const char *media_path = getenv("FFMPEG_V4L2_REQUEST_MEDIA_PATH");

    if (!video_path)
	    video_path = V4L2_REQUEST_VIDEO_PATH;
    if (!media_path)
	    media_path = V4L2_REQUEST_MEDIA_PATH;

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p ctx=%p hw_device_ctx=%p hw_frames_ctx=%p\n", __func__, avctx, ctx, avctx->hw_device_ctx, avctx->hw_frames_ctx);

    ctx->video_fd = open(video_path, O_RDWR | O_NONBLOCK, 0);
    if (ctx->video_fd < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: opening %s failed, %s (%d)\n", __func__, V4L2_REQUEST_VIDEO_PATH, strerror(errno), errno);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ctx->media_fd = open(media_path, O_RDWR | O_NONBLOCK, 0);
    if (ctx->media_fd < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: opening %s failed, %s (%d)\n", __func__, V4L2_REQUEST_MEDIA_PATH, strerror(errno), errno);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ret = ioctl(ctx->media_fd, MEDIA_IOC_DEVICE_INFO, &device_info);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: get media device info failed, %s (%d)\n", __func__, strerror(errno), errno);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p ctx=%p driver=%s\n", __func__, avctx, ctx, device_info.driver);

    ret = ioctl(ctx->video_fd, VIDIOC_QUERYCAP, &capability);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: get video capability failed, %s (%d)\n", __func__, strerror(errno), errno);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    if (capability.capabilities & V4L2_CAP_DEVICE_CAPS)
        capabilities = capability.device_caps;
    else
        capabilities = capability.capabilities;

    if ((capabilities & V4L2_CAP_STREAMING) != V4L2_CAP_STREAMING) {
        av_log(avctx, AV_LOG_ERROR, "%s: missing required streaming capability\n", __func__);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    if ((capabilities & V4L2_CAP_VIDEO_M2M_MPLANE) == V4L2_CAP_VIDEO_M2M_MPLANE) {
        ctx->output_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        ctx->format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    } else if ((capabilities & V4L2_CAP_VIDEO_M2M) == V4L2_CAP_VIDEO_M2M) {
        ctx->output_type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        ctx->format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    } else {
        av_log(avctx, AV_LOG_ERROR, "%s: missing required mem2mem capability\n", __func__);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ret = v4l2_request_set_format(avctx, ctx->output_type, pixelformat, buffersize);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: set output format failed, %s (%d)\n", __func__, strerror(errno), errno);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ret = v4l2_request_set_controls(ctx, -1, control, count);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: set controls failed, %s (%d)\n", __func__, strerror(errno), errno);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ret = v4l2_request_select_capture_format(avctx);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: select capture format failed\n", __func__);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ret = ioctl(ctx->video_fd, VIDIOC_G_FMT, &ctx->format);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: get capture format failed, %s (%d)\n", __func__, strerror(errno), errno);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    if (V4L2_TYPE_IS_MULTIPLANAR(ctx->format.type)) {
        av_log(avctx, AV_LOG_DEBUG, "%s: pixelformat=%d width=%u height=%u bytesperline=%u sizeimage=%u\n", __func__, ctx->format.fmt.pix_mp.pixelformat, ctx->format.fmt.pix_mp.width, ctx->format.fmt.pix_mp.height, ctx->format.fmt.pix_mp.plane_fmt[0].bytesperline, ctx->format.fmt.pix_mp.plane_fmt[0].sizeimage);
    } else {
        av_log(avctx, AV_LOG_DEBUG, "%s: pixelformat=%d width=%u height=%u bytesperline=%u sizeimage=%u\n", __func__, ctx->format.fmt.pix.pixelformat, ctx->format.fmt.pix.width, ctx->format.fmt.pix.height, ctx->format.fmt.pix.bytesperline, ctx->format.fmt.pix.sizeimage);
    }

    ret = ff_decode_get_hw_frames_ctx(avctx, AV_HWDEVICE_TYPE_DRM);
    if (ret < 0)
        goto fail;

    ret = ioctl(ctx->video_fd, VIDIOC_STREAMON, &ctx->output_type);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: output stream on failed, %s (%d)\n", __func__, strerror(errno), errno);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ret = ioctl(ctx->video_fd, VIDIOC_STREAMON, &ctx->format.type);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: capture stream on failed, %s (%d)\n", __func__, strerror(errno), errno);
        ret = AVERROR(EINVAL);
        goto fail;
    }

    return 0;

fail:
    ff_v4l2_request_uninit(avctx);
    return ret;
}

int ff_v4l2_request_uninit(AVCodecContext *avctx)
{
    V4L2RequestContext *ctx = avctx->internal->hwaccel_priv_data;

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p ctx=%p\n", __func__, avctx, ctx);

    if (ctx->video_fd >= 0) {
        int ret;

        ret = ioctl(ctx->video_fd, VIDIOC_STREAMOFF, &ctx->output_type);
        if (ret < 0)
            av_log(avctx, AV_LOG_ERROR, "%s: output stream off failed, %s (%d)\n", __func__, strerror(errno), errno);

        ret = ioctl(ctx->video_fd, VIDIOC_STREAMOFF, &ctx->format.type);
        if (ret < 0)
            av_log(avctx, AV_LOG_ERROR, "%s: capture stream off failed, %s (%d)\n", __func__, strerror(errno), errno);
    }

    if (avctx->hw_frames_ctx) {
        AVHWFramesContext *hwfc = (AVHWFramesContext*)avctx->hw_frames_ctx->data;
        av_buffer_pool_reclaim(hwfc->pool);
    }

    if (ctx->media_fd >= 0)
        close(ctx->media_fd);

    if (ctx->video_fd >= 0)
        close(ctx->video_fd);

    return 0;
}

static int v4l2_request_buffer_alloc(AVCodecContext *avctx, V4L2RequestBuffer *buf, enum v4l2_buf_type type)
{
    V4L2RequestContext *ctx = avctx->internal->hwaccel_priv_data;
    struct v4l2_create_buffers buffers = {0};
    struct v4l2_plane planes[1] = {};
    int ret;

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p buf=%p type=%u\n", __func__, avctx, buf, type);

    buffers.format.type = type;
    buffers.memory = V4L2_MEMORY_MMAP;
    buffers.count = 1;

    ret = ioctl(ctx->video_fd, VIDIOC_G_FMT, &buffers.format);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: get format failed for type %u, %s (%d)\n", __func__, type, strerror(errno), errno);
        return ret;
    }

    ret = ioctl(ctx->video_fd, VIDIOC_CREATE_BUFS, &buffers);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: create buffers failed for type %u, %s (%d)\n", __func__, type, strerror(errno), errno);
        return ret;
    }

    buf->index = buffers.index;
    if (V4L2_TYPE_IS_MULTIPLANAR(type)) {
        buf->width = buffers.format.fmt.pix_mp.width;
        buf->height = buffers.format.fmt.pix_mp.height;
        buf->size = buffers.format.fmt.pix_mp.plane_fmt[0].sizeimage;
        buf->buffer.length = 1;
        buf->buffer.m.planes = planes;
    } else {
        buf->width = buffers.format.fmt.pix.width;
        buf->height = buffers.format.fmt.pix.height;
        buf->size = buffers.format.fmt.pix.sizeimage;
    }
    buf->used = 0;

    buf->buffer.type = type;
    buf->buffer.memory = V4L2_MEMORY_MMAP;
    buf->buffer.index = buf->index;
    buf->buffer.timestamp.tv_usec = buf->index;

    ret = ioctl(ctx->video_fd, VIDIOC_QUERYBUF, &buf->buffer);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: query buffer %d failed, %s (%d)\n", __func__, buf->index, strerror(errno), errno);
        return ret;
    }

    if (V4L2_TYPE_IS_OUTPUT(type)) {
        void *addr = mmap(NULL, buf->size, PROT_READ | PROT_WRITE, MAP_SHARED, ctx->video_fd, V4L2_TYPE_IS_MULTIPLANAR(type) ? buf->buffer.m.planes[0].m.mem_offset : buf->buffer.m.offset);
        if (addr == MAP_FAILED) {
            av_log(avctx, AV_LOG_ERROR, "%s: mmap failed, %s (%d)\n", __func__, strerror(errno), errno);
            return -1;
        }

        buf->addr = (uint8_t*)addr;
    } else {
        struct v4l2_exportbuffer exportbuffer = {0};
        exportbuffer.type = type;
        exportbuffer.index = buf->index;
        exportbuffer.flags = O_RDONLY;

        ret = ioctl(ctx->video_fd, VIDIOC_EXPBUF, &exportbuffer);
        if (ret < 0) {
            av_log(avctx, AV_LOG_ERROR, "%s: export buffer %d failed, %s (%d)\n", __func__, buf->index, strerror(errno), errno);
            return ret;
        }

        buf->fd = exportbuffer.fd;
    }

    av_log(avctx, AV_LOG_DEBUG, "%s: buf=%p index=%d fd=%d addr=%p width=%u height=%u size=%u\n", __func__, buf, buf->index, buf->fd, buf->addr, buf->width, buf->height, buf->size);
    return 0;
}

static void v4l2_request_buffer_free(V4L2RequestBuffer *buf)
{
    av_log(NULL, AV_LOG_DEBUG, "%s: buf=%p index=%d fd=%d addr=%p width=%u height=%u size=%u\n", __func__, buf, buf->index, buf->fd, buf->addr, buf->width, buf->height, buf->size);

    if (buf->addr)
        munmap(buf->addr, buf->size);

    if (buf->fd >= 0)
        close(buf->fd);
}

static void v4l2_request_frame_free(void *opaque, uint8_t *data)
{
    AVCodecContext *avctx = opaque;
    V4L2RequestDescriptor *req = (V4L2RequestDescriptor*)data;

    av_log(NULL, AV_LOG_DEBUG, "%s: avctx=%p data=%p request_fd=%d\n", __func__, avctx, data, req->request_fd);

    if (req->request_fd >= 0)
        close(req->request_fd);

    v4l2_request_buffer_free(&req->capture);
    v4l2_request_buffer_free(&req->output);

    av_free(data);
}

static AVBufferRef *v4l2_request_frame_alloc(void *opaque, int size)
{
    AVCodecContext *avctx = opaque;
    V4L2RequestContext *ctx = avctx->internal->hwaccel_priv_data;
    V4L2RequestDescriptor *req;
    AVBufferRef *ref;
    uint8_t *data;
    int ret;

    data = av_mallocz(size);
    if (!data)
        return NULL;

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p size=%d data=%p\n", __func__, avctx, size, data);

    ref = av_buffer_create(data, size, v4l2_request_frame_free, avctx, 0);
    if (!ref) {
        av_freep(&data);
        return NULL;
    }

    req = (V4L2RequestDescriptor*)data;
    req->request_fd = -1;
    req->output.fd = -1;
    req->capture.fd = -1;

    ret = v4l2_request_buffer_alloc(avctx, &req->output, ctx->output_type);
    if (ret < 0) {
        av_buffer_unref(&ref);
        return NULL;
    }

    ret = v4l2_request_buffer_alloc(avctx, &req->capture, ctx->format.type);
    if (ret < 0) {
        av_buffer_unref(&ref);
        return NULL;
    }

    ret = ioctl(ctx->media_fd, MEDIA_IOC_REQUEST_ALLOC, &req->request_fd);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "%s: request alloc failed, %s (%d)\n", __func__, strerror(errno), errno);
        av_buffer_unref(&ref);
        return NULL;
    }

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p size=%d data=%p request_fd=%d\n", __func__, avctx, size, data, req->request_fd);
    return ref;
}

static void v4l2_request_pool_free(void *opaque)
{
    av_log(NULL, AV_LOG_DEBUG, "%s: opaque=%p\n", __func__, opaque);
}

static void v4l2_request_hwframe_ctx_free(AVHWFramesContext *hwfc)
{
    av_log(NULL, AV_LOG_DEBUG, "%s: hwfc=%p pool=%p\n", __func__, hwfc, hwfc->pool);

    av_buffer_pool_reclaim(hwfc->pool);
    av_buffer_pool_uninit(&hwfc->pool);
}

int ff_v4l2_request_frame_params(AVCodecContext *avctx, AVBufferRef *hw_frames_ctx)
{
    V4L2RequestContext *ctx = avctx->internal->hwaccel_priv_data;
    AVHWFramesContext *hwfc = (AVHWFramesContext*)hw_frames_ctx->data;

    hwfc->format = AV_PIX_FMT_DRM_PRIME;
    hwfc->sw_format = AV_PIX_FMT_NV12;
    if (V4L2_TYPE_IS_MULTIPLANAR(ctx->format.type)) {
        hwfc->width = ctx->format.fmt.pix_mp.width;
        hwfc->height = ctx->format.fmt.pix_mp.height;
    } else {
        hwfc->width = ctx->format.fmt.pix.width;
        hwfc->height = ctx->format.fmt.pix.height;
    }

    hwfc->pool = av_buffer_pool_init2(sizeof(V4L2RequestDescriptor), avctx, v4l2_request_frame_alloc, v4l2_request_pool_free);
    if (!hwfc->pool)
        return AVERROR(ENOMEM);

    hwfc->free = v4l2_request_hwframe_ctx_free;

    hwfc->initial_pool_size = 1;

    switch (avctx->codec_id) {
    case AV_CODEC_ID_VP9:
        hwfc->initial_pool_size += 8;
        break;
    case AV_CODEC_ID_VP8:
        hwfc->initial_pool_size += 3;
        break;
    default:
        hwfc->initial_pool_size += 2;
    }

    av_log(avctx, AV_LOG_DEBUG, "%s: avctx=%p ctx=%p hw_frames_ctx=%p hwfc=%p pool=%p width=%d height=%d initial_pool_size=%d\n", __func__, avctx, ctx, hw_frames_ctx, hwfc, hwfc->pool, hwfc->width, hwfc->height, hwfc->initial_pool_size);

    return 0;
}
