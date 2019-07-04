/*
 * Copyright (c) 2003 Daniel Moreno <comac AT comac DOT darktech DOT org>
 * Copyright (c) 2010 Baptiste Coudurier
 * Copyright (c) 2012 Loren Merritt
 *
 * This file is part of FFmpeg, ported from MPlayer.
 *
 * FFmpeg is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with FFmpeg; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/**
 * @file
 * high quality 3d video denoiser, ported from MPlayer
 * libmpcodecs/vf_hqdn3d.c.
 */

#include <float.h>

#include "config.h"
#include "libavutil/attributes.h"
#include "libavutil/common.h"
#include "libavutil/pixdesc.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/opt.h"

#include "avfilter.h"
#include "formats.h"
#include "internal.h"
#include "video.h"
#include "vf_hqdn3d.h"

#define LUT_BITS (depth==16 ? 8 : 4)
#define LOAD(x) (((depth == 8 ? src[x] : AV_RN16A(src + (x) * 2)) << (16 - depth))\
                 + (((1 << (16 - depth)) - 1) >> 1))
#define STORE(x,val) (depth == 8 ? dst[x] = (val) >> (16 - depth) : \
                                   AV_WN16A(dst + (x) * 2, (val) >> (16 - depth)))

av_always_inline
static uint32_t lowpass(int prev, int cur, int16_t *coef, int depth)
{
    // printf("lowpass depth val %d\n", depth);
    // printf("prev %d curr %d\n",prev, cur);
    //索引值为 前一个像素减去后一个像素的差值 
    //差值最多 256 d最大值为 256 / 2^4 (depth = 8)
    //差值最小 -256 d最小值为 -256 / 2^4
    //右移 8-LUT_BITS 寻找到索引
    int d = (prev - cur) >> (8 - LUT_BITS);
    //printf("cur %d coef[%d] %d \n",cur, d, coef[d]);
    return cur + coef[d]; //返回滤波值, 对cur值进行一定的补偿
}

av_always_inline
static void denoise_temporal(uint8_t *src, uint8_t *dst,
                             uint16_t *frame_ant,
                             int w, int h, int sstride, int dstride,
                             int16_t *temporal, int depth)
{
    long x, y;
    uint32_t tmp;

    temporal += 256 << LUT_BITS;
    //和前一帧做时域滤波
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            frame_ant[x] = tmp = lowpass(frame_ant[x], LOAD(x), temporal, depth);
            STORE(x, tmp);
        }
        src += sstride;
        dst += dstride;
        frame_ant += w;
    }
}

av_always_inline
static void denoise_spatial(HQDN3DContext *s,
                            uint8_t *src, uint8_t *dst,
                            uint16_t *line_ant, uint16_t *frame_ant,
                            int w, int h, int sstride, int dstride,
                            int16_t *spatial, int16_t *temporal, int depth)
{
    //printf("denoise_spatial depth val %d\n", depth);
    long x, y;
    uint32_t pixel_ant;
    uint32_t tmp;

    spatial  += 256 << LUT_BITS;
    temporal += 256 << LUT_BITS;

    /* First line has no top neighbor. Only left one for each tmp and
     * last frame */
    //读取第一个像素数据
    //第一行没有上一行的数据，所以只做左右像素的空域滤波
    pixel_ant = LOAD(0);
    //printf("raw pixel_ant %d src %d ret %d\n", pixel_ant, src[0], pixel_ant /src[0]);
    for (x = 0; x < w; x++) {
        //行缓存前一个像素和当前像素的低通空域滤波
        //printf("read pixel %d depth %d\n", pixel_ant, depth);
        line_ant[x] = tmp = pixel_ant = lowpass(pixel_ant, LOAD(x), spatial, depth);
        //上一帧的第一行和当前行缓存做低通时域滤波
        frame_ant[x] = tmp = lowpass(frame_ant[x], tmp, temporal, depth);
        //printf("write pixel tmp %d depth %d\n", tmp, depth);
        STORE(x, tmp);
        //printf("write pixel dst %d\n", dst[x]);
    }


    //处理第一行以外的行
    for (y = 1; y < h; y++) {
        //更新src和dst指针位置
        src += sstride;
        dst += dstride;
        //更新前一帧指针位置
        frame_ant += w;
        //并行计算
        if (s->denoise_row[depth]) {
            s->denoise_row[depth](src, dst, line_ant, frame_ant, w, spatial, temporal);
            continue;
        }
        //读取行中第一个像素点
        pixel_ant = LOAD(0);
        //处理一行
        for (x = 0; x < w-1; x++) {
            //上一行和当前行的空域滤波
            line_ant[x] = tmp = lowpass(line_ant[x], pixel_ant, spatial, depth);
            //当前像素和下一个像素的空域滤波
            pixel_ant = lowpass(pixel_ant, LOAD(x+1), spatial, depth);
            //当前帧和前一帧做时域滤波
            frame_ant[x] = tmp = lowpass(frame_ant[x], tmp, temporal, depth);
            //printf("write pixel tmp %d depth %d\n", tmp, depth);
            //保存到结果位置
            STORE(x, tmp);
        }
        //最后一个像素点不计算pixel_ant
        line_ant[x] = tmp = lowpass(line_ant[x], pixel_ant, spatial, depth);
        frame_ant[x] = tmp = lowpass(frame_ant[x], tmp, temporal, depth);
        STORE(x, tmp);
    }
}

av_always_inline
static int denoise_depth(HQDN3DContext *s,
                         uint8_t *src, uint8_t *dst,
                         uint16_t *line_ant, uint16_t **frame_ant_ptr,
                         int w, int h, int sstride, int dstride,
                         int16_t *spatial, int16_t *temporal, int depth)
{
    // FIXME: For 16-bit depth, frame_ant could be a pointer to the previous
    // filtered frame rather than a separate buffer.
    //printf("denoise_depth depth val %d\n", depth);
    long x, y;
    uint16_t *frame_ant = *frame_ant_ptr;
    //前帧空间为空，则创建空间
    if (!frame_ant) {
        uint8_t *frame_src = src;
        //创建帧缓存数据 为int16 大小w*h
        *frame_ant_ptr = frame_ant = av_malloc_array(w, h*sizeof(uint16_t));
        if (!frame_ant)
            return AVERROR(ENOMEM);
        //保存当前帧的数据
        for (y = 0; y < h; y++, src += sstride, frame_ant += w)
            for (x = 0; x < w; x++)
                frame_ant[x] = LOAD(x);
        //指针指回首地址
        src = frame_src;
        frame_ant = *frame_ant_ptr;
    }

    if (spatial[0]){
        denoise_spatial(s, src, dst, line_ant, frame_ant,
                           w, h, sstride, dstride, spatial, temporal, depth);
    }
    else 
    {
        //printf("denoise_temporal only\n");
        denoise_temporal(src, dst, frame_ant,
                         w, h, sstride, dstride, temporal, depth);
    }
    emms_c();
    return 0;
}

#define denoise(...)                                                          \
    do {                                                                      \
        int ret = AVERROR_BUG;                                                \
        switch (s->depth) {                                                   \
            case  8: ret = denoise_depth(__VA_ARGS__,  8); break;             \
            case  9: ret = denoise_depth(__VA_ARGS__,  9); break;             \
            case 10: ret = denoise_depth(__VA_ARGS__, 10); break;             \
            case 16: ret = denoise_depth(__VA_ARGS__, 16); break;             \
        }                                                                     \
        if (ret < 0) {                                                        \
            av_frame_free(&out);                                              \
            if (!direct)                                                      \
                av_frame_free(&in);                                           \
            return ret;                                                       \
        }                                                                     \
    } while (0)

static int16_t *precalc_coefs(double dist25, int depth)
{
    printf("precalc_coefs depth val %d dist25 %0.8f\n", depth, dist25);
    int i;
    double gamma, simil, C;
    int16_t *ct = av_malloc((512<<LUT_BITS)*sizeof(int16_t));
    if (!ct)
        return NULL;
    
    //换底公式：log(1 - 强度%) (0.25) // (1 - 强度%)^gamma = 0.25 理解为误差减小25% ??
    gamma = log(0.25) / log(1.0 - FFMIN(dist25,252.0)/255.0 - 0.00001);

    //两个像素的差值范围 -256 ~ 256
    //左移4bits 编程int12
    for (i = -256<<LUT_BITS; i < 256<<LUT_BITS; i++) {
        //左移5bits变成int17 + 1111(2) / 512（相当于右移9bits, 又变成了8bits，但是这里是double保留了精度）
        //相当于变换到8bits 浮点范围
        //bin宽系数
        double f = ((i<<(9-LUT_BITS)) + (1<<(8-LUT_BITS)) - 1) / 512.0; // midpoint of the bin
        //计算差值强度百分比
        simil = FFMAX(0, 1.0 - fabs(f) / 255.0);
        //根据之前系数gamma 补偿到0.25
        C = pow(simil, gamma) * 256.0 * f;
        ct[(256<<LUT_BITS)+i] = lrint(C);
    }

    ct[0] = !!dist25;
    return ct;
}

#define PARAM1_DEFAULT 4.0
#define PARAM2_DEFAULT 3.0
#define PARAM3_DEFAULT 6.0

static av_cold int init(AVFilterContext *ctx)
{
    HQDN3DContext *s = ctx->priv;               
    //初始化亮度和色度对应强度
    if (!s->strength[LUMA_SPATIAL])
        s->strength[LUMA_SPATIAL] = PARAM1_DEFAULT;
    if (!s->strength[CHROMA_SPATIAL])
        s->strength[CHROMA_SPATIAL] = PARAM2_DEFAULT * s->strength[LUMA_SPATIAL] / PARAM1_DEFAULT;
    if (!s->strength[LUMA_TMP])
        s->strength[LUMA_TMP]   = PARAM3_DEFAULT * s->strength[LUMA_SPATIAL] / PARAM1_DEFAULT;
    if (!s->strength[CHROMA_TMP])
        s->strength[CHROMA_TMP] = s->strength[LUMA_TMP] * s->strength[CHROMA_SPATIAL] / s->strength[LUMA_SPATIAL];

    av_log(ctx, AV_LOG_VERBOSE, "ls:%f cs:%f lt:%f ct:%f\n",
           s->strength[LUMA_SPATIAL], s->strength[CHROMA_SPATIAL],
           s->strength[LUMA_TMP], s->strength[CHROMA_TMP]);

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    HQDN3DContext *s = ctx->priv;

    av_freep(&s->coefs[0]);
    av_freep(&s->coefs[1]);
    av_freep(&s->coefs[2]);
    av_freep(&s->coefs[3]);
    av_freep(&s->line);
    av_freep(&s->frame_prev[0]);
    av_freep(&s->frame_prev[1]);
    av_freep(&s->frame_prev[2]);
}

static int query_formats(AVFilterContext *ctx)
{
    static const enum AVPixelFormat pix_fmts[] = {
        AV_PIX_FMT_YUV420P,
        AV_PIX_FMT_YUV422P,
        AV_PIX_FMT_YUV444P,
        AV_PIX_FMT_YUV410P,
        AV_PIX_FMT_YUV411P,
        AV_PIX_FMT_YUV440P,
        AV_PIX_FMT_YUVJ420P,
        AV_PIX_FMT_YUVJ422P,
        AV_PIX_FMT_YUVJ444P,
        AV_PIX_FMT_YUVJ440P,
        AV_PIX_FMT_YUV420P9,
        AV_PIX_FMT_YUV422P9,
        AV_PIX_FMT_YUV444P9,
        AV_PIX_FMT_YUV420P10,
        AV_PIX_FMT_YUV422P10,
        AV_PIX_FMT_YUV444P10,
        AV_PIX_FMT_YUV420P16,
        AV_PIX_FMT_YUV422P16,
        AV_PIX_FMT_YUV444P16,
        AV_PIX_FMT_NONE
    };
    AVFilterFormats *fmts_list = ff_make_format_list(pix_fmts);
    if (!fmts_list)
        return AVERROR(ENOMEM);
    return ff_set_common_formats(ctx, fmts_list);
}

static int config_input(AVFilterLink *inlink)
{
    HQDN3DContext *s = inlink->dst->priv;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    int i;

    uninit(inlink->dst);

    s->hsub  = desc->log2_chroma_w;
    s->vsub  = desc->log2_chroma_h;
    s->depth = desc->comp[0].depth;
    printf("s->depth %d\n",  s->depth);
    //申请行缓存空间
    s->line = av_malloc_array(inlink->w, sizeof(*s->line));
    if (!s->line)
        return AVERROR(ENOMEM);

    for (i = 0; i < 4; i++) {
        s->coefs[i] = precalc_coefs(s->strength[i], s->depth);
        if (!s->coefs[i])
            return AVERROR(ENOMEM);
    }

    if (ARCH_X86)
        ff_hqdn3d_init_x86(s);

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx  = inlink->dst;
    HQDN3DContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];

    AVFrame *out;
    int c, direct = av_frame_is_writable(in) && !ctx->is_disabled;

    if (direct) {
        out = in;
    } else {
        out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }

        av_frame_copy_props(out, in);
    }

    for (c = 0; c < 3; c++) {
        denoise(s, in->data[c], out->data[c],
                s->line, &s->frame_prev[c],
                AV_CEIL_RSHIFT(in->width,  (!!c * s->hsub)), //亮度通道直接传width
                AV_CEIL_RSHIFT(in->height, (!!c * s->vsub)), //色度通道传入width >> 1
                in->linesize[c], out->linesize[c],
                s->coefs[c ? CHROMA_SPATIAL : LUMA_SPATIAL],
                s->coefs[c ? CHROMA_TMP     : LUMA_TMP]);
    }

    if (ctx->is_disabled) {
        av_frame_free(&out);
        return ff_filter_frame(outlink, in);
    }

    if (!direct)
        av_frame_free(&in);

    return ff_filter_frame(outlink, out);
}

#define OFFSET(x) offsetof(HQDN3DContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_FILTERING_PARAM
static const AVOption hqdn3d_options[] = {
    { "luma_spatial",   "spatial luma strength",    OFFSET(strength[LUMA_SPATIAL]),   AV_OPT_TYPE_DOUBLE, { .dbl = 0.0 }, 0, DBL_MAX, FLAGS },
    { "chroma_spatial", "spatial chroma strength",  OFFSET(strength[CHROMA_SPATIAL]), AV_OPT_TYPE_DOUBLE, { .dbl = 0.0 }, 0, DBL_MAX, FLAGS },
    { "luma_tmp",       "temporal luma strength",   OFFSET(strength[LUMA_TMP]),       AV_OPT_TYPE_DOUBLE, { .dbl = 0.0 }, 0, DBL_MAX, FLAGS },
    { "chroma_tmp",     "temporal chroma strength", OFFSET(strength[CHROMA_TMP]),     AV_OPT_TYPE_DOUBLE, { .dbl = 0.0 }, 0, DBL_MAX, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(hqdn3d);

static const AVFilterPad avfilter_vf_hqdn3d_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_input,
        .filter_frame = filter_frame,
    },
    { NULL }
};


static const AVFilterPad avfilter_vf_hqdn3d_outputs[] = {
    {
        .name = "default",
        .type = AVMEDIA_TYPE_VIDEO
    },
    { NULL }
};

AVFilter ff_vf_hqdn3d = {
    .name          = "hqdn3d",
    .description   = NULL_IF_CONFIG_SMALL("Apply a High Quality 3D Denoiser."),
    .priv_size     = sizeof(HQDN3DContext),
    .priv_class    = &hqdn3d_class,
    .init          = init,
    .uninit        = uninit,
    .query_formats = query_formats,
    .inputs        = avfilter_vf_hqdn3d_inputs,
    .outputs       = avfilter_vf_hqdn3d_outputs,
    .flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL,
};
