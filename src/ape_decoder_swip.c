/*
* Monkey's Audio lossless audio decoder
* Copyright (c) 2007 Benjamin Zores <ben@geexbox.org>
*  based upon libdemac from Dave Chapman.
*
* Bytestream functions
* Copyright (c) 2006 Baptiste Coudurier <baptiste.coudurier@free.fr>
* Copyright (c) 2012 Aneesh Dogra (lionaneesh) <lionaneesh@gmail.com>
*
* Copyright (c) 2020 Mediatek Inc.
* This file is modified from FFmpeg, please refer to README.
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

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <utils/Log.h>

#include "../inc/ape_decoder_exp.h"
#include "../inc/ape_decoder_swip.h"

/**
 * @file
 * Monkey's Audio lossless audio decoder
 */

#define MAX_CHANNELS        2
#define MAX_BYTESPERSAMPLE  3
#define BLOCKS_PER_LOOP     864 /* originally config to 4608 */

#define INPUT_CHUNKSIZE     (BLOCKS_PER_LOOP*MAX_CHANNELS*MAX_BYTESPERSAMPLE)
#define OUTPUT_CHUNKSIZE    (BLOCKS_PER_LOOP*MAX_CHANNELS*MAX_BYTESPERSAMPLE)

#define APE_FRAMECODE_MONO_SILENCE    1
#define APE_FRAMECODE_STEREO_SILENCE  3
#define APE_FRAMECODE_PSEUDO_STEREO   4

#define HISTORY_SIZE 512
#define PREDICTOR_ORDER 8
/** Total size of all predictor histories */
#define PREDICTOR_SIZE 50

#define YDELAYA (18 + PREDICTOR_ORDER*4)
#define YDELAYB (18 + PREDICTOR_ORDER*3)
#define XDELAYA (18 + PREDICTOR_ORDER*2)
#define XDELAYB (18 + PREDICTOR_ORDER)

#define YADAPTCOEFFSA 18
#define XADAPTCOEFFSA 14
#define YADAPTCOEFFSB 10
#define XADAPTCOEFFSB 5

#ifdef VERBOSE
static int ii=0;
#endif
static int DecCnt = 0;
/**
 * Possible compression levels
 * @{
 */
enum APECompressionLevel {
    COMPRESSION_LEVEL_FAST       = 1000,
    COMPRESSION_LEVEL_NORMAL     = 2000,
    COMPRESSION_LEVEL_HIGH       = 3000,
    COMPRESSION_LEVEL_EXTRA_HIGH = 4000,
    COMPRESSION_LEVEL_INSANE     = 5000
};
/** @} */

#define APE_FILTER_LEVELS 3

/** Filter orders depending on compression level */
static const uint16_t ape_filter_orders[5][APE_FILTER_LEVELS] = {
    {  0,   0,    0 },
    { 16,   0,    0 },
    { 64,   0,    0 },
    { 32, 256,    0 },
    { 16, 256, 1280 }
};

/** Filter fraction bits depending on compression level */
static const uint8_t ape_filter_fracbits[5][APE_FILTER_LEVELS] = {
    {  0,  0,  0 },
    { 11,  0,  0 },
    { 11,  0,  0 },
    { 10, 13,  0 },
    { 11, 13, 15 }
};


/** Filters applied to the decoded data */
typedef struct APEFilter {
    int16_t *coeffs;        ///< actual coefficients used in filtering
    int16_t *adaptcoeffs;   ///< adaptive filter coefficients used for correcting of actual filter coefficients
    int16_t *historybuffer; ///< filter memory
    int16_t *delay;         ///< filtered values

    int avg;
} APEFilter;

typedef struct APERice {
    uint32_t k;
    uint32_t ksum;
} APERice;

typedef struct APERangecoder {
    uint32_t low;           ///< low end of interval
    uint32_t range;         ///< length of interval
    uint32_t help;          ///< bytes_to_follow resp. intermediate value
    unsigned int buffer;    ///< buffer for input/output
} APERangecoder;

/** Filter histories */
typedef struct APEPredictor {
    int32_t *buf;

    int32_t lastA[2];

    int32_t filterA[2];
    int32_t filterB[2];

    int32_t coeffsA[2][4];  ///< adaption coefficients
    int32_t coeffsB[2][5];  ///< adaption coefficients
    int32_t historybuffer[HISTORY_SIZE + PREDICTOR_SIZE];

    unsigned int sample_pos;
} APEPredictor;

/** Decoder context */
typedef struct APEContext {
    int channels;
    int samples;                             ///< samples left to decode in current frame
    int bps;

    int fileversion;                         ///< codec version, very important in decoding process
    int compression_level;                   ///< compression levels
    int fset;                                ///< which filter set to use (calculated from compression level)
    int flags;                               ///< global decoder flags

    uint32_t CRC;                            ///< frame CRC
    int frameflags;                          ///< frame flags
    APEPredictor predictor;                  ///< predictor used for final reconstruction

    int32_t decoded0[BLOCKS_PER_LOOP];    ///< decoded data for channel0
    int32_t decoded1[BLOCKS_PER_LOOP];    ///< decoded data for channel1
    int blocks_per_loop;                     ///< maximum number of samples to decode for each call

    int16_t filterbuf64[(64*3 + FILTER_HISTORY_SIZE) * 4];   ///< filter memory
    int16_t filterbuf256[(256*3 + FILTER_HISTORY_SIZE) * 4];   ///< filter memory
    int16_t filterbuf1280[(1280*3 + FILTER_HISTORY_SIZE) * 4];   ///< filter memory

    APERangecoder rc;                        ///< rangecoder used to decode actual values
    APERice riceX;                           ///< rice code parameters for the second channel
    APERice riceY;                           ///< rice code parameters for the first channel
    APEFilter filters[APE_FILTER_LEVELS][2]; ///< filters used for reconstruction
    GetBitContext gb;

    uint8_t data[INPUT_CHUNKSIZE];      ///< current frame data
    uint8_t *data_end;                         ///< frame data end
    int data_size;                                ///< frame data allocated size
    const uint8_t *ptr;                         ///< current position in frame data

    int error;

    void (*entropy_decode_mono)(struct APEContext *ctx, int blockstodecode);
    void (*entropy_decode_stereo)(struct APEContext *ctx, int blockstodecode);
    void (*predictor_decode_mono)(struct APEContext *ctx, int count);
    void (*predictor_decode_stereo)(struct APEContext *ctx, int count);

    int decode_state;
    int byte_offset;
    uint32_t curr_frame;

    /* Info from Header Block */
    uint16_t formatflags;
    uint32_t finalframeblocks;
    uint32_t totalframes;
} APEContext;

static void ape_apply_filters(APEContext *ctx, int32_t *decoded0,
                              int32_t *decoded1, int count);

static void entropy_decode_mono_0000(APEContext *ctx, int blockstodecode);
static void entropy_decode_stereo_0000(APEContext *ctx, int blockstodecode);
static void entropy_decode_mono_3860(APEContext *ctx, int blockstodecode);
static void entropy_decode_stereo_3860(APEContext *ctx, int blockstodecode);
static void entropy_decode_mono_3900(APEContext *ctx, int blockstodecode);
static void entropy_decode_stereo_3900(APEContext *ctx, int blockstodecode);
static void entropy_decode_stereo_3930(APEContext *ctx, int blockstodecode);
static void entropy_decode_mono_3990(APEContext *ctx, int blockstodecode);
static void entropy_decode_stereo_3990(APEContext *ctx, int blockstodecode);

static void predictor_decode_mono_3800(APEContext *ctx, int count);
static void predictor_decode_stereo_3800(APEContext *ctx, int count);
static void predictor_decode_mono_3930(APEContext *ctx, int count);
static void predictor_decode_stereo_3930(APEContext *ctx, int count);
static void predictor_decode_mono_3950(APEContext *ctx, int count);
static void predictor_decode_stereo_3950(APEContext *ctx, int count);

static void bswap_buf(uint32_t *dst, const uint32_t *src, int w)
{
    int i;

    for (i = 0; i + 8 <= w; i += 8) {
        dst[i + 0] = BSWAP32C(src[i + 0]);
        dst[i + 1] = BSWAP32C(src[i + 1]);
        dst[i + 2] = BSWAP32C(src[i + 2]);
        dst[i + 3] = BSWAP32C(src[i + 3]);
        dst[i + 4] = BSWAP32C(src[i + 4]);
        dst[i + 5] = BSWAP32C(src[i + 5]);
        dst[i + 6] = BSWAP32C(src[i + 6]);
        dst[i + 7] = BSWAP32C(src[i + 7]);
    }
    for (; i < w; i++)
        dst[i + 0] = BSWAP32C(src[i + 0]);
}

static unsigned int bytestream_get_be32(const uint8_t **b)
{
    (*b) += 4;
    return (((uint32_t)((const uint8_t*)(*b - 4))[0] << 24) |    \
               (((const uint8_t*)(*b - 4))[1] << 16) |    \
               (((const uint8_t*)(*b - 4))[2] <<  8) |    \
                ((const uint8_t*)(*b - 4))[3]);
}

static unsigned int bytestream_get_byte(const uint8_t **b)
{
    (*b) += 1;
    return (((const uint8_t*)(*b - 1))[0]);
}

/**
 * @name APE range decoding functions
 * @{
 */

#define CODE_BITS    32
#define TOP_VALUE    ((unsigned int)1 << (CODE_BITS-1))
#define SHIFT_BITS   (CODE_BITS - 9)
#define EXTRA_BITS   ((CODE_BITS-2) % 8 + 1)
#define BOTTOM_VALUE (TOP_VALUE >> 8)

static int32_t scalarproduct_and_madd_int16(int16_t *v1, const int16_t *v2,
                                              const int16_t *v3,
                                              int order, int mul)
{
    int res = 0;

    do {
        res   += *v1 * *v2++;
        *v1++ += mul * *v3++;
        res   += *v1 * *v2++;
        *v1++ += mul * *v3++;
    } while (order-=2);
    return res;
}

/** Start the decoder */
static inline void range_start_decoding(APEContext *ctx)
{
    ctx->rc.buffer = bytestream_get_byte(&ctx->ptr);
    ALOGD("range_start_decoding crc byte:0x%x ptr:0x%x at oft:%d rc.buf:0x%x\n",
        ctx->CRC,ctx->ptr,ctx->byte_offset,ctx->rc.buffer);
    ctx->rc.low    = ctx->rc.buffer >> (8 - EXTRA_BITS);
    ctx->rc.range  = (uint32_t) 1 << EXTRA_BITS;
}

/** Perform normalization */
static inline void range_dec_normalize(APEContext *ctx)
{
    while (ctx->rc.range <= BOTTOM_VALUE) {
        ctx->rc.buffer <<= 8;
        if(ctx->ptr < ctx->data_end) {
            ctx->rc.buffer += *ctx->ptr;
            ctx->ptr++;
        } else {
            ctx->error = 1;
            ALOGE("range_dec_normalize error\n");
        }
        ctx->rc.low    = (ctx->rc.low << 8)    | ((ctx->rc.buffer >> 1) & 0xFF);
        ctx->rc.range  <<= 8;
    }
#ifdef VERBOSE
    if(ctx->samples <= 288) {
        ALOGV("range_dec_normalize range:0x%x,low:0x%x,help:0x%x,in_ptr:0x%x,oft:0x%x\n",
            ctx->rc.range,ctx->rc.low,ctx->rc.help,ctx->ptr,ctx->ptr-ctx->data);
        for (ii=0; ii<20; ii++) {
            ALOGV("0x%x ",ctx->ptr[ii]);
        }
        ALOGV("\n");
    }
#endif
}

/**
 * Calculate cumulative frequency for next symbol. Does NO update!
 * @param ctx decoder context
 * @param tot_f is the total frequency or (code_value)1<<shift
 * @return the cumulative frequency
 */
static inline int range_decode_culfreq(APEContext *ctx, int tot_f)
{
    range_dec_normalize(ctx);
    ctx->rc.help = ctx->rc.range / tot_f;
    return ctx->rc.low / ctx->rc.help;
}

/**
 * Decode value with given size in bits
 * @param ctx decoder context
 * @param shift number of bits to decode
 */
static inline int range_decode_culshift(APEContext *ctx, int shift)
{
    range_dec_normalize(ctx);
    ctx->rc.help = ctx->rc.range >> shift;
    return ctx->rc.low / ctx->rc.help;
}

/**
 * Update decoding state
 * @param ctx decoder context
 * @param sy_f the interval length (frequency of the symbol)
 * @param lt_f the lower end (frequency sum of < symbols)
 */
static inline void range_decode_update(APEContext *ctx, int sy_f, int lt_f)
{
    ctx->rc.low  -= ctx->rc.help * lt_f;
    ctx->rc.range = ctx->rc.help * sy_f;
#ifdef VERBOSE
    if (ctx->samples <= 288)
        ALOGV("range_decode_update range:0x%x, low:0x%x\n",ctx->rc.range,ctx->rc.low);
#endif
}

/** Decode n bits (n <= 16) without modelling */
static inline int range_decode_bits(APEContext *ctx, int n)
{
    int sym = range_decode_culshift(ctx, n);
    range_decode_update(ctx, 1, sym);
    return sym;
}

#define MODEL_ELEMENTS 64

/**
 * Fixed probabilities for symbols in Monkey Audio version 3.97
 */
static const uint16_t counts_3970[22] = {
        0, 14824, 28224, 39348, 47855, 53994, 58171, 60926,
    62682, 63786, 64463, 64878, 65126, 65276, 65365, 65419,
    65450, 65469, 65480, 65487, 65491, 65493,
};

/**
 * Probability ranges for symbols in Monkey Audio version 3.97
 */
static const uint16_t counts_diff_3970[21] = {
    14824, 13400, 11124, 8507, 6139, 4177, 2755, 1756,
    1104, 677, 415, 248, 150, 89, 54, 31,
    19, 11, 7, 4, 2,
};

/**
 * Fixed probabilities for symbols in Monkey Audio version 3.98
 */
static const uint16_t counts_3980[22] = {
        0, 19578, 36160, 48417, 56323, 60899, 63265, 64435,
    64971, 65232, 65351, 65416, 65447, 65466, 65476, 65482,
    65485, 65488, 65490, 65491, 65492, 65493,
};

/**
 * Probability ranges for symbols in Monkey Audio version 3.98
 */
static const uint16_t counts_diff_3980[21] = {
    19578, 16582, 12257, 7906, 4576, 2366, 1170, 536,
    261, 119, 65, 31, 19, 10, 6, 3,
    3, 2, 1, 1, 1,
};

/**
 * Decode symbol
 * @param ctx decoder context
 * @param counts probability range start position
 * @param counts_diff probability range widths
 */
static inline int range_get_symbol(APEContext *ctx,
                                   const uint16_t counts[],
                                   const uint16_t counts_diff[])
{
    int symbol, cf;

    cf = range_decode_culshift(ctx, 16);

    if(cf > 65492){
        symbol= cf - 65535 + 63;
        range_decode_update(ctx, 1, cf);
        if(cf > 65535) {
            ctx->error=1;
            ALOGE("range_get_symbol error\n");
        }
        return symbol;
    }
    /* figure out the symbol inefficiently; a binary search would be much better */
    for (symbol = 0; counts[symbol + 1] <= cf; symbol++);

    range_decode_update(ctx, counts_diff[symbol], counts[symbol]);

    return symbol;
}
/** @} */ // group rangecoder

static inline void update_rice(APERice *rice, unsigned int x)
{
    int lim = rice->k ? (1 << (rice->k + 4)) : 0;
    rice->ksum += ((x + 1) / 2) - ((rice->ksum + 16) >> 5);

    if (rice->ksum < lim)
        rice->k--;
    else if (rice->ksum >= (1 << (rice->k + 5)) && rice->k < 24)
        rice->k++;
}

static inline int get_rice_ook(GetBitContext *gb, int k)
{
    unsigned int x;

    x = get_unary(gb, 1, get_bits_left(gb));

    if (k)
        x = (x << k) | get_bits(gb, k);

    return x;
}

static inline int ape_decode_value_3860(APEContext *ctx, GetBitContext *gb,
                                        APERice *rice)
{
    unsigned int x, overflow;

    overflow = get_unary(gb, 1, get_bits_left(gb));

    if (ctx->fileversion > 3880) {
        while (overflow >= 16) {
            overflow -= 16;
            rice->k  += 4;
        }
    }

    if (!rice->k)
        x = overflow;
    else if(rice->k <= MIN_CACHE_BITS) {
        x = (overflow << rice->k) + get_bits(gb, rice->k);
    } else {
        ALOGE( "Too many bits: %d\n", rice->k);
        return APE_ERR_EOS;
    }
    rice->ksum += x - ((rice->ksum + 8) >> 4);
    if (rice->ksum < (rice->k ? 1 << (rice->k + 4) : 0))
        rice->k--;
    else if (rice->ksum >= (1 << (rice->k + 5)) && rice->k < 24)
        rice->k++;

    if (x == 0) {
        ALOGD( "meet x==0 which will calculate overflow\n");
        return 0;
    }
    /* Convert to signed */
    return (int)(((x >> 1) ^ ((x & 1) - 1)) + 1);
}

static inline int ape_decode_value_3900(APEContext *ctx, APERice *rice)
{
    unsigned int x, overflow;
    int tmpk;

    overflow = range_get_symbol(ctx, counts_3970, counts_diff_3970);

    if (overflow == (MODEL_ELEMENTS - 1)) {
        tmpk = range_decode_bits(ctx, 5);
        overflow = 0;
    } else
        tmpk = (rice->k < 1) ? 0 : rice->k - 1;

    if (tmpk <= 16 || ctx->fileversion < 3910) {
        if (tmpk > 23) {
            ALOGE( "Too many bits: %d\n", tmpk);
            return APE_ERR_EOS;
        }
        x = range_decode_bits(ctx, tmpk);
    } else if (tmpk <= 31) {
        x = range_decode_bits(ctx, 16);
        x |= (range_decode_bits(ctx, tmpk - 16) << 16);
    } else {
        ALOGE( "Too many bits: %d\n", tmpk);
        return APE_ERR_EOS;
    }
    x += overflow << tmpk;

    update_rice(rice, x);

    if (x == 0) {
        ALOGD( "meet x==0 which will calculate overflow\n");
        return 0;
    }
    /* Convert to signed */
    return (int)(((x >> 1) ^ ((x & 1) - 1)) + 1);
}

static inline int ape_decode_value_3990(APEContext *ctx, APERice *rice)
{
    unsigned int x, overflow;
    int base, pivot;

    pivot = rice->ksum >> 5;
    if (pivot == 0)
        pivot = 1;
    overflow = range_get_symbol(ctx, counts_3980, counts_diff_3980);
    if (overflow == (MODEL_ELEMENTS - 1)) {
        overflow  = (unsigned)range_decode_bits(ctx, 16) << 16;
        overflow |= range_decode_bits(ctx, 16);
    }
    if (pivot < 0x10000) {
        base = range_decode_culfreq(ctx, pivot);
        range_decode_update(ctx, 1, base);
    } else {
        int base_hi = pivot, base_lo;
        int bbits = 0;
        while (base_hi & ~0xFFFF) {
            base_hi >>= 1;
            bbits++;
        }
        base_hi = range_decode_culfreq(ctx, base_hi + 1);
        range_decode_update(ctx, 1, base_hi);
        base_lo = range_decode_culfreq(ctx, 1 << bbits);
        range_decode_update(ctx, 1, base_lo);
        base = (base_hi << bbits) + base_lo;
    }
    x = base + overflow * pivot;
    update_rice(rice, x);

    if (x == 0) {
        ALOGD( "meet x==0 which will calculate overflow\n");
        return 0;
    }
    /* Convert to signed */
    return (int)(((x >> 1) ^ ((x & 1) - 1)) + 1);
}

static void decode_array_0000(APEContext *ctx, GetBitContext *gb,
                              int32_t *out, APERice *rice, int blockstodecode)
{
    int i;
    unsigned ksummax, ksummin;

    (void)(gb);
    rice->ksum = 0;
    for (i = 0; i < MIN(blockstodecode, 5); i++) {
        out[i] = get_rice_ook(&ctx->gb, 10);
        rice->ksum += out[i];
    }
    rice->k = LOG2(rice->ksum / 10) + 1;
    if (rice->k >= 24)
        return;
    for (; i < MIN(blockstodecode, 64); i++) {
        out[i] = get_rice_ook(&ctx->gb, rice->k);
        rice->ksum += out[i];
        rice->k = LOG2(rice->ksum / ((i + 1) * 2)) + 1;
        if (rice->k >= 24)
            return;
    }
    ksummax = 1 << (rice->k + 7);
    ksummin = rice->k ? (1 << (rice->k + 6)) : 0;
    for (; i < blockstodecode; i++) {
        out[i] = get_rice_ook(&ctx->gb, rice->k);
        rice->ksum += out[i] - (unsigned)out[i - 64];
        while (rice->ksum < ksummin) {
            rice->k--;
            ksummin = rice->k ? ksummin >> 1 : 0;
            ksummax >>= 1;
        }
        while (rice->ksum >= ksummax) {
            rice->k++;
            if (rice->k > 24)
                return;
            ksummax <<= 1;
            ksummin = ksummin ? ksummin << 1 : 128;
        }
    }

    for (i = 0; i < blockstodecode; i++)
        out[i] = ((out[i] >> 1) ^ ((out[i] & 1) - 1)) + 1;
}

static void entropy_decode_mono_0000(APEContext *ctx, int blockstodecode)
{
    decode_array_0000(ctx, &ctx->gb, ctx->decoded0, &ctx->riceY,
                      blockstodecode);
}

static void entropy_decode_stereo_0000(APEContext *ctx, int blockstodecode)
{
    decode_array_0000(ctx, &ctx->gb, ctx->decoded0, &ctx->riceY,
                      blockstodecode);
    decode_array_0000(ctx, &ctx->gb, ctx->decoded1, &ctx->riceX,
                      blockstodecode);
}

static void entropy_decode_mono_3860(APEContext *ctx, int blockstodecode)
{
    int32_t *decoded0 = ctx->decoded0;

    while (blockstodecode--)
        *decoded0++ = ape_decode_value_3860(ctx, &ctx->gb, &ctx->riceY);
}

static void entropy_decode_stereo_3860(APEContext *ctx, int blockstodecode)
{
    int32_t *decoded0 = ctx->decoded0;
    int32_t *decoded1 = ctx->decoded1;
    int blocks = blockstodecode;

    while (blockstodecode--)
        *decoded0++ = ape_decode_value_3860(ctx, &ctx->gb, &ctx->riceY);
    while (blocks--)
        *decoded1++ = ape_decode_value_3860(ctx, &ctx->gb, &ctx->riceX);
}

static void entropy_decode_mono_3900(APEContext *ctx, int blockstodecode)
{
    int32_t *decoded0 = ctx->decoded0;

    while (blockstodecode--)
        *decoded0++ = ape_decode_value_3900(ctx, &ctx->riceY);
}

static void entropy_decode_stereo_3900(APEContext *ctx, int blockstodecode)
{
    int32_t *decoded0 = ctx->decoded0;
    int32_t *decoded1 = ctx->decoded1;
    int blocks = blockstodecode;

    while (blockstodecode--)
        *decoded0++ = ape_decode_value_3900(ctx, &ctx->riceY);
    range_dec_normalize(ctx);
    // because of some implementation peculiarities we need to backpedal here
    ctx->ptr -= 1;
    range_start_decoding(ctx);
    while (blocks--)
        *decoded1++ = ape_decode_value_3900(ctx, &ctx->riceX);
}

static void entropy_decode_stereo_3930(APEContext *ctx, int blockstodecode)
{
    int32_t *decoded0 = ctx->decoded0;
    int32_t *decoded1 = ctx->decoded1;

    while (blockstodecode--) {
        *decoded0++ = ape_decode_value_3900(ctx, &ctx->riceY);
        *decoded1++ = ape_decode_value_3900(ctx, &ctx->riceX);
    }
}

static void entropy_decode_mono_3990(APEContext *ctx, int blockstodecode)
{
    int32_t *decoded0 = ctx->decoded0;

    while (blockstodecode--)
        *decoded0++ = ape_decode_value_3990(ctx, &ctx->riceY);
}

static void entropy_decode_stereo_3990(APEContext *ctx, int blockstodecode)
{
    int32_t *decoded0 = ctx->decoded0;
    int32_t *decoded1 = ctx->decoded1;

    ALOGD("entropy_decode_stereo_3990 +, blockstodecode %d\n", blockstodecode);
    while (blockstodecode--) {
        *decoded0++ = ape_decode_value_3990(ctx, &ctx->riceY);
        *decoded1++ = ape_decode_value_3990(ctx, &ctx->riceX);
    }
    ALOGD("entropy_decode_stereo_3990 -, ctx->ptr 0x%x\n", ctx->ptr);
}

static int init_entropy_decoder(APEContext *ctx)
{
    /* Read the CRC */
    if (ctx->fileversion >= 3900) {
        if (ctx->data_end - ctx->ptr < 6)
            return APE_ERR_EOS;
#ifdef VERBOSE
        ALOGV("init_entropy_decoder: ");
        for (ii=0; ii<10; ii++) {
            ALOGV("%x ",ctx->ptr[ii]);
        }
        ALOGV("\n");
#endif
        ctx->CRC = bytestream_get_be32(&ctx->ptr);
    } else {
        ctx->CRC = bytestream_get_be32(&ctx->ptr);
    }

    /* Read the frame flags if they exist */
    ctx->frameflags = 0;
    if ((ctx->fileversion > 3820) && (ctx->CRC & 0x80000000)) {
        ctx->CRC &= ~0x80000000;

        if (ctx->data_end - ctx->ptr < 6)
            return APE_ERR_EOS;
        ctx->frameflags = bytestream_get_be32(&ctx->ptr);
    }

    /* Initialize the rice structs */
    ctx->riceX.k = 10;
    ctx->riceX.ksum = (1 << ctx->riceX.k) * 16;
    ctx->riceY.k = 10;
    ctx->riceY.ksum = (1 << ctx->riceY.k) * 16;

    if (ctx->fileversion >= 3900) {
        /* The first 8 bits of input are ignored. */
        ctx->ptr++;
        range_start_decoding(ctx);
    }

    ALOGD("init_entropy_decoder crc:0x%x, frameflags:0x%x ptr:0x%x\n",
        ctx->CRC,ctx->frameflags,ctx->ptr);

    return 0;
}

static const int32_t initial_coeffs_fast_3320[1] = {
    375,
};

static const int32_t initial_coeffs_a_3800[3] = {
    64, 115, 64,
};

static const int32_t initial_coeffs_b_3800[2] = {
    740, 0
};

static const int32_t initial_coeffs_3930[4] = {
    360, 317, -109, 98
};

static void init_predictor_decoder(APEContext *ctx)
{
    APEPredictor *p = &ctx->predictor;

    /* Zero the history buffers */
    memset(p->historybuffer, 0, PREDICTOR_SIZE * sizeof(*p->historybuffer));
    p->buf = p->historybuffer;

    /* Initialize and zero the coefficients */
    if (ctx->fileversion < 3930) {
        if (ctx->compression_level == COMPRESSION_LEVEL_FAST) {
            memcpy(p->coeffsA[0], initial_coeffs_fast_3320,
                   sizeof(initial_coeffs_fast_3320));
            memcpy(p->coeffsA[1], initial_coeffs_fast_3320,
                   sizeof(initial_coeffs_fast_3320));
        } else {
            memcpy(p->coeffsA[0], initial_coeffs_a_3800,
                   sizeof(initial_coeffs_a_3800));
            memcpy(p->coeffsA[1], initial_coeffs_a_3800,
                   sizeof(initial_coeffs_a_3800));
        }
    } else {
        memcpy(p->coeffsA[0], initial_coeffs_3930, sizeof(initial_coeffs_3930));
        memcpy(p->coeffsA[1], initial_coeffs_3930, sizeof(initial_coeffs_3930));
    }
    memset(p->coeffsB, 0, sizeof(p->coeffsB));
    if (ctx->fileversion < 3930) {
        memcpy(p->coeffsB[0], initial_coeffs_b_3800,
               sizeof(initial_coeffs_b_3800));
        memcpy(p->coeffsB[1], initial_coeffs_b_3800,
               sizeof(initial_coeffs_b_3800));
    }

    p->filterA[0] = p->filterA[1] = 0;
    p->filterB[0] = p->filterB[1] = 0;
    p->lastA[0]   = p->lastA[1]   = 0;

    p->sample_pos = 0;
}

/** Get inverse sign of integer (-1 for positive, 1 for negative and 0 for zero) */
static inline int APESIGN(int32_t x) {
    return (x < 0) - (x > 0);
}

static inline int filter_fast_3320(APEPredictor *p,
                                             const int decoded, const int filterInt,
                                             const int delayA)
{
    int32_t predictionA;
    unsigned int filter = (unsigned int)filterInt;

    p->buf[delayA] = p->lastA[filter];
    if (p->sample_pos < 3) {
        p->lastA[filter]   = decoded;
        p->filterA[filter] = decoded;
        return decoded;
    }

    predictionA = p->buf[delayA] * 2 - p->buf[delayA - 1];
    p->lastA[filter] = decoded + (predictionA  * p->coeffsA[filter][0] >> 9);

    if ((decoded ^ predictionA) > 0)
        p->coeffsA[filter][0]++;
    else
        p->coeffsA[filter][0]--;

    p->filterA[filter] += (unsigned)p->lastA[filter];

    return p->filterA[filter];
}

static inline int filter_3800(APEPredictor *p,
                                        const int decoded, const int filterInt,
                                        const int delayA,  const int delayB,
                                        const int start,   const int shift)
{
    int32_t predictionA, predictionB, sign;
    int32_t d0, d1, d2, d3, d4;
    unsigned int filter = (unsigned int)filterInt;

    p->buf[delayA] = p->lastA[filter];
    p->buf[delayB] = p->filterB[filter];
    if (p->sample_pos < start) {
        predictionA = decoded + p->filterA[filter];
        p->lastA[filter]   = decoded;
        p->filterB[filter] = decoded;
        p->filterA[filter] = predictionA;
        return predictionA;
    }
    d2 =  p->buf[delayA];
    d1 = (p->buf[delayA] - p->buf[delayA - 1]) * 2U;
    d0 =  p->buf[delayA] + ((p->buf[delayA - 2] - p->buf[delayA - 1]) * 8U);
    d3 =  p->buf[delayB] * 2U - p->buf[delayB - 1];
    d4 =  p->buf[delayB];

    predictionA = d0 * p->coeffsA[filter][0] +
                  d1 * p->coeffsA[filter][1] +
                  d2 * p->coeffsA[filter][2];

    sign = APESIGN(decoded);
    p->coeffsA[filter][0] += (((d0 >> 30) & 2) - 1) * sign;
    p->coeffsA[filter][1] += (((d1 >> 28) & 8) - 4) * sign;
    p->coeffsA[filter][2] += (((d2 >> 28) & 8) - 4) * sign;

    predictionB = d3 * p->coeffsB[filter][0] -
                  d4 * p->coeffsB[filter][1];
    p->lastA[filter] = decoded + (predictionA >> 11);
    sign = APESIGN(p->lastA[filter]);
    p->coeffsB[filter][0] += (((d3 >> 29) & 4) - 2) * sign;
    p->coeffsB[filter][1] -= (((d4 >> 30) & 2) - 1) * sign;

    p->filterB[filter] = p->lastA[filter] + (predictionB >> shift);
    p->filterA[filter] = p->filterB[filter] + (unsigned)((int)(p->filterA[filter] * 31U) >> 5);

    return p->filterA[filter];
}

static void long_filter_high_3800(int32_t *buffer, int order, int shift, int length)
{
    int i, j;
    int32_t dotprod, sign;
    int32_t coeffs[256], delay[256];

    if (order >= length)
        return;

    memset(coeffs, 0, order * sizeof(*coeffs));
    for (i = 0; i < order; i++)
        delay[i] = buffer[i];
    for (i = order; i < length; i++) {
        dotprod = 0;
        sign = APESIGN(buffer[i]);
        for (j = 0; j < order; j++) {
            dotprod += delay[j] * (unsigned)coeffs[j];
            coeffs[j] += ((delay[j] >> 31) | 1) * sign;
        }
        buffer[i] -= dotprod >> shift;
        for (j = 0; j < order - 1; j++)
            delay[j] = delay[j + 1];
        delay[order - 1] = buffer[i];
    }
}

static void long_filter_ehigh_3830(int32_t *buffer, int length)
{
    int i, j;
    int32_t dotprod, sign;
    int32_t delay[8] = { 0 };
    uint32_t coeffs[8] = { 0 };

    for (i = 0; i < length; i++) {
        dotprod = 0;
        sign = APESIGN(buffer[i]);
        for (j = 7; j >= 0; j--) {
            dotprod += delay[j] * coeffs[j];
            coeffs[j] += ((delay[j] >> 31) | 1) * sign;
        }
        for (j = 7; j > 0; j--)
            delay[j] = delay[j - 1];
        delay[0] = buffer[i];
        buffer[i] -= dotprod >> 9;
    }
}

static void predictor_decode_stereo_3800(APEContext *ctx, int count)
{
    APEPredictor *p = &ctx->predictor;
    int32_t *decoded0 = ctx->decoded0;
    int32_t *decoded1 = ctx->decoded1;
    int start = 4, shift = 10;

    if (ctx->compression_level == COMPRESSION_LEVEL_HIGH) {
        start = 16;
        long_filter_high_3800(decoded0, 16, 9, count);
        long_filter_high_3800(decoded1, 16, 9, count);
    } else if (ctx->compression_level == COMPRESSION_LEVEL_EXTRA_HIGH) {
        int order = 128, shift2 = 11;

        if (ctx->fileversion >= 3830) {
            order <<= 1;
            shift++;
            shift2++;
            long_filter_ehigh_3830(decoded0 + order, count - order);
            long_filter_ehigh_3830(decoded1 + order, count - order);
        }
        start = order;
        long_filter_high_3800(decoded0, order, shift2, count);
        long_filter_high_3800(decoded1, order, shift2, count);
    }

    while (count--) {
        int X = *decoded0, Y = *decoded1;
        if (ctx->compression_level == COMPRESSION_LEVEL_FAST) {
            *decoded0 = filter_fast_3320(p, Y, 0, YDELAYA);
            decoded0++;
            *decoded1 = filter_fast_3320(p, X, 1, XDELAYA);
            decoded1++;
        } else {
            *decoded0 = filter_3800(p, Y, 0, YDELAYA, YDELAYB,
                                    start, shift);
            decoded0++;
            *decoded1 = filter_3800(p, X, 1, XDELAYA, XDELAYB,
                                    start, shift);
            decoded1++;
        }

        /* Combined */
        p->buf++;
        p->sample_pos++;

        /* Have we filled the history buffer? */
        if (p->buf == p->historybuffer + HISTORY_SIZE) {
            memmove(p->historybuffer, p->buf,
                    PREDICTOR_SIZE * sizeof(*p->historybuffer));
            p->buf = p->historybuffer;
        }
    }
}

static void predictor_decode_mono_3800(APEContext *ctx, int count)
{
    APEPredictor *p = &ctx->predictor;
    int32_t *decoded0 = ctx->decoded0;
    int start = 4, shift = 10;

    if (ctx->compression_level == COMPRESSION_LEVEL_HIGH) {
        start = 16;
        long_filter_high_3800(decoded0, 16, 9, count);
    } else if (ctx->compression_level == COMPRESSION_LEVEL_EXTRA_HIGH) {
        int order = 128, shift2 = 11;

        if (ctx->fileversion >= 3830) {
            order <<= 1;
            shift++;
            shift2++;
            long_filter_ehigh_3830(decoded0 + order, count - order);
        }
        start = order;
        long_filter_high_3800(decoded0, order, shift2, count);
    }

    while (count--) {
        if (ctx->compression_level == COMPRESSION_LEVEL_FAST) {
            *decoded0 = filter_fast_3320(p, *decoded0, 0, YDELAYA);
            decoded0++;
        } else {
            *decoded0 = filter_3800(p, *decoded0, 0, YDELAYA, YDELAYB,
                                    start, shift);
            decoded0++;
        }

        /* Combined */
        p->buf++;
        p->sample_pos++;

        /* Have we filled the history buffer? */
        if (p->buf == p->historybuffer + HISTORY_SIZE) {
            memmove(p->historybuffer, p->buf,
                    PREDICTOR_SIZE * sizeof(*p->historybuffer));
            p->buf = p->historybuffer;
        }
    }
}

static inline int predictor_update_3930(APEPredictor *p,
                                                  const int decoded, const int filterInt,
                                                  const int delayA)
{
    int32_t predictionA, sign;
    int32_t d0, d1, d2, d3;
    unsigned int filter = (unsigned int)filterInt;

    p->buf[delayA]     = p->lastA[filter];
    d0 = p->buf[delayA    ];
    d1 = p->buf[delayA    ] - p->buf[delayA - 1];
    d2 = p->buf[delayA - 1] - p->buf[delayA - 2];
    d3 = p->buf[delayA - 2] - p->buf[delayA - 3];

    predictionA = d0 * p->coeffsA[filter][0] +
                  d1 * p->coeffsA[filter][1] +
                  d2 * p->coeffsA[filter][2] +
                  d3 * p->coeffsA[filter][3];

    p->lastA[filter] = decoded + (predictionA >> 9);
    p->filterA[filter] = p->lastA[filter] + ((int)(p->filterA[filter] * 31U) >> 5);

    sign = APESIGN(decoded);
    p->coeffsA[filter][0] += ((d0 < 0) * 2 - 1) * sign;
    p->coeffsA[filter][1] += ((d1 < 0) * 2 - 1) * sign;
    p->coeffsA[filter][2] += ((d2 < 0) * 2 - 1) * sign;
    p->coeffsA[filter][3] += ((d3 < 0) * 2 - 1) * sign;

    return p->filterA[filter];
}

static void predictor_decode_stereo_3930(APEContext *ctx, int count)
{
    APEPredictor *p = &ctx->predictor;
    int32_t *decoded0 = ctx->decoded0;
    int32_t *decoded1 = ctx->decoded1;

    ape_apply_filters(ctx, ctx->decoded0, ctx->decoded1, count);

    while (count--) {
        /* Predictor Y */
        int Y = *decoded1, X = *decoded0;
        *decoded0 = predictor_update_3930(p, Y, 0, YDELAYA);
        decoded0++;
        *decoded1 = predictor_update_3930(p, X, 1, XDELAYA);
        decoded1++;

        /* Combined */
        p->buf++;

        /* Have we filled the history buffer? */
        if (p->buf == p->historybuffer + HISTORY_SIZE) {
            memmove(p->historybuffer, p->buf,
                    PREDICTOR_SIZE * sizeof(*p->historybuffer));
            p->buf = p->historybuffer;
        }
    }
}

static void predictor_decode_mono_3930(APEContext *ctx, int count)
{
    APEPredictor *p = &ctx->predictor;
    int32_t *decoded0 = ctx->decoded0;

    ape_apply_filters(ctx, ctx->decoded0, NULL, count);

    while (count--) {
        *decoded0 = predictor_update_3930(p, *decoded0, 0, YDELAYA);
        decoded0++;

        p->buf++;

        /* Have we filled the history buffer? */
        if (p->buf == p->historybuffer + HISTORY_SIZE) {
            memmove(p->historybuffer, p->buf,
                    PREDICTOR_SIZE * sizeof(*p->historybuffer));
            p->buf = p->historybuffer;
        }
    }
}

static inline int predictor_update_filter(APEPredictor *p,
                                                    const int decoded, const int filterInt,
                                                    const int delayA,  const int delayB,
                                                    const int adaptA,  const int adaptB)
{
    int32_t predictionA, predictionB, sign;
    unsigned int filter = (unsigned int)filterInt;

    p->buf[delayA]     = p->lastA[filter];
    p->buf[adaptA]     = APESIGN(p->buf[delayA]);
    p->buf[delayA - 1] = p->buf[delayA] - (unsigned)p->buf[delayA - 1];
    p->buf[adaptA - 1] = APESIGN(p->buf[delayA - 1]);

    predictionA = p->buf[delayA    ] * p->coeffsA[filter][0] +
                  p->buf[delayA - 1] * p->coeffsA[filter][1] +
                  p->buf[delayA - 2] * p->coeffsA[filter][2] +
                  p->buf[delayA - 3] * p->coeffsA[filter][3];

    /*  Apply a scaled first-order filter compression */
    p->buf[delayB]     = p->filterA[filter ^ 1] - ((int)(p->filterB[filter] * 31U) >> 5);
    p->buf[adaptB]     = APESIGN(p->buf[delayB]);
    p->buf[delayB - 1] = p->buf[delayB] - (unsigned)p->buf[delayB - 1];
    p->buf[adaptB - 1] = APESIGN(p->buf[delayB - 1]);
    p->filterB[filter] = p->filterA[filter ^ 1];

    predictionB = p->buf[delayB    ] * p->coeffsB[filter][0] +
                  p->buf[delayB - 1] * p->coeffsB[filter][1] +
                  p->buf[delayB - 2] * p->coeffsB[filter][2] +
                  p->buf[delayB - 3] * p->coeffsB[filter][3] +
                  p->buf[delayB - 4] * p->coeffsB[filter][4];

    p->lastA[filter] = decoded + ((int)((unsigned)predictionA + (predictionB >> 1)) >> 10);
    p->filterA[filter] = p->lastA[filter] + ((int)(p->filterA[filter] * 31U) >> 5);

    sign = APESIGN(decoded);
    p->coeffsA[filter][0] += p->buf[adaptA    ] * sign;
    p->coeffsA[filter][1] += p->buf[adaptA - 1] * sign;
    p->coeffsA[filter][2] += p->buf[adaptA - 2] * sign;
    p->coeffsA[filter][3] += p->buf[adaptA - 3] * sign;
    p->coeffsB[filter][0] += p->buf[adaptB    ] * sign;
    p->coeffsB[filter][1] += p->buf[adaptB - 1] * sign;
    p->coeffsB[filter][2] += p->buf[adaptB - 2] * sign;
    p->coeffsB[filter][3] += p->buf[adaptB - 3] * sign;
    p->coeffsB[filter][4] += p->buf[adaptB - 4] * sign;

    return p->filterA[filter];
}

static void predictor_decode_stereo_3950(APEContext *ctx, int count)
{
    APEPredictor *p = &ctx->predictor;
    int32_t *decoded0 = ctx->decoded0;
    int32_t *decoded1 = ctx->decoded1;

    ape_apply_filters(ctx, ctx->decoded0, ctx->decoded1, count);

    while (count--) {
        /* Predictor Y */
        *decoded0 = predictor_update_filter(p, *decoded0, 0, YDELAYA, YDELAYB,
                                            YADAPTCOEFFSA, YADAPTCOEFFSB);
        decoded0++;
        *decoded1 = predictor_update_filter(p, *decoded1, 1, XDELAYA, XDELAYB,
                                            XADAPTCOEFFSA, XADAPTCOEFFSB);
        decoded1++;

        /* Combined */
        p->buf++;

        /* Have we filled the history buffer? */
        if (p->buf == p->historybuffer + HISTORY_SIZE) {
            memmove(p->historybuffer, p->buf,
                    PREDICTOR_SIZE * sizeof(*p->historybuffer));
            p->buf = p->historybuffer;
        }
    }
}

static void predictor_decode_mono_3950(APEContext *ctx, int count)
{
    APEPredictor *p = &ctx->predictor;
    int32_t *decoded0 = ctx->decoded0;
    int32_t predictionA, currentA, A, sign;

    ape_apply_filters(ctx, ctx->decoded0, NULL, count);

    currentA = p->lastA[0];

    while (count--) {
        A = *decoded0;

        p->buf[YDELAYA] = currentA;
        p->buf[YDELAYA - 1] = p->buf[YDELAYA] - (unsigned)p->buf[YDELAYA - 1];

        predictionA = p->buf[YDELAYA    ] * p->coeffsA[0][0] +
                      p->buf[YDELAYA - 1] * p->coeffsA[0][1] +
                      p->buf[YDELAYA - 2] * p->coeffsA[0][2] +
                      p->buf[YDELAYA - 3] * p->coeffsA[0][3];

        currentA = A + (unsigned)(predictionA >> 10);

        p->buf[YADAPTCOEFFSA]     = APESIGN(p->buf[YDELAYA    ]);
        p->buf[YADAPTCOEFFSA - 1] = APESIGN(p->buf[YDELAYA - 1]);

        sign = APESIGN(A);
        p->coeffsA[0][0] += p->buf[YADAPTCOEFFSA    ] * sign;
        p->coeffsA[0][1] += p->buf[YADAPTCOEFFSA - 1] * sign;
        p->coeffsA[0][2] += p->buf[YADAPTCOEFFSA - 2] * sign;
        p->coeffsA[0][3] += p->buf[YADAPTCOEFFSA - 3] * sign;

        p->buf++;

        /* Have we filled the history buffer? */
        if (p->buf == p->historybuffer + HISTORY_SIZE) {
            memmove(p->historybuffer, p->buf,
                    PREDICTOR_SIZE * sizeof(*p->historybuffer));
            p->buf = p->historybuffer;
        }

        p->filterA[0] = currentA + (unsigned)((int)(p->filterA[0] * 31U) >> 5);
        *(decoded0++) = p->filterA[0];
    }

    p->lastA[0] = currentA;
}

static void do_init_filter(APEFilter *f, int16_t *buf, int order)
{
    f->coeffs = buf;
    f->historybuffer = buf + order;
    f->delay       = f->historybuffer + order * 2;
    f->adaptcoeffs = f->historybuffer + order;

    memset(f->historybuffer, 0, (order * 2) * sizeof(*f->historybuffer));
    memset(f->coeffs, 0, order * sizeof(*f->coeffs));
    f->avg = 0;
}

static void init_filter(APEContext *ctx, APEFilter *f, int16_t *buf, int order)
{
    (void)(ctx);
    do_init_filter(&f[0], buf, order);
    do_init_filter(&f[1], buf + order * 3 + HISTORY_SIZE, order);
}

static void do_apply_filter(APEContext *ctx, int version, APEFilter *f,
                            int32_t *data, int count, int order, int fracbits)
{
    int res;
    int absres;

    (void)(ctx);

    while (count--) {
        /* round fixedpoint scalar product */
        res = scalarproduct_and_madd_int16(f->coeffs,
                                                     f->delay - order,
                                                     f->adaptcoeffs - order,
                                                     order, APESIGN(*data));
        res = (int)(res + (1U << (fracbits - 1))) >> fracbits;
        res += (unsigned)*data;
        *data++ = res;

        /* Update the output history */
        *f->delay++ = clip_int16(res);

        if (version < 3980) {
            /* Version ??? to < 3.98 files (untested) */
            f->adaptcoeffs[0]  = (res == 0) ? 0 : ((res >> 28) & 8) - 4;
            f->adaptcoeffs[-4] >>= 1;
            f->adaptcoeffs[-8] >>= 1;
        } else {
            /* Version 3.98 and later files */

            /* Update the adaption coefficients */
            absres = res < 0 ? -(unsigned)res : res;
            if (absres)
                *f->adaptcoeffs = APESIGN(res) *
                                  (8 << ((absres > f->avg * 3) + (absres > f->avg * 4 / 3)));
                /* equivalent to the following code
                    if (absres <= f->avg * 4 / 3)
                        *f->adaptcoeffs = APESIGN(res) * 8;
                    else if (absres <= f->avg * 3)
                        *f->adaptcoeffs = APESIGN(res) * 16;
                    else
                        *f->adaptcoeffs = APESIGN(res) * 32;
                */
            else
                *f->adaptcoeffs = 0;

            f->avg += (int)(absres - (unsigned)f->avg) / 16;

            f->adaptcoeffs[-1] >>= 1;
            f->adaptcoeffs[-2] >>= 1;
            f->adaptcoeffs[-8] >>= 1;
        }

        f->adaptcoeffs++;

        /* Have we filled the history buffer? */
        if (f->delay == f->historybuffer + HISTORY_SIZE + (order * 2)) {
            memmove(f->historybuffer, f->delay - (order * 2),
                    (order * 2) * sizeof(*f->historybuffer));
            f->delay = f->historybuffer + order * 2;
            f->adaptcoeffs = f->historybuffer + order;
        }
    }
}

static void apply_filter(APEContext *ctx, APEFilter *f,
                         int32_t *data0, int32_t *data1,
                         int count, int order, int fracbits)
{
    do_apply_filter(ctx, ctx->fileversion, &f[0], data0, count, order, fracbits);
    if (data1)
        do_apply_filter(ctx, ctx->fileversion, &f[1], data1, count, order, fracbits);
}

static void ape_apply_filters(APEContext *ctx, int32_t *decoded0,
                              int32_t *decoded1, int count)
{
    int i;

    for (i = 0; i < APE_FILTER_LEVELS; i++) {
        if (!ape_filter_orders[(unsigned int)ctx->fset][i])
            break;
        apply_filter(ctx, ctx->filters[i], decoded0, decoded1, count,
                     ape_filter_orders[(unsigned int)ctx->fset][i],
                     ape_filter_fracbits[(unsigned int)ctx->fset][i]);
    }
}

static int init_frame_decoder(APEContext *ctx)
{
    int i, ret;
    if ((ret = init_entropy_decoder(ctx)) < 0)
        return ret;
    init_predictor_decoder(ctx);

    for (i = 0; i < APE_FILTER_LEVELS; i++) {
        if (!ape_filter_orders[(unsigned int)ctx->fset][i])
            break;
        switch (i) {
        case 0:
            init_filter(ctx, ctx->filters[i], ctx->filterbuf64,
                    ape_filter_orders[(unsigned int)ctx->fset][i]);
            break;
        case 1:
            init_filter(ctx, ctx->filters[i], ctx->filterbuf256,
                    ape_filter_orders[(unsigned int)ctx->fset][i]);
            break;
        case 2:
            init_filter(ctx, ctx->filters[i], ctx->filterbuf1280,
                    ape_filter_orders[(unsigned int)ctx->fset][i]);
            break;
        }
    }
    return 0;
}

static void ape_unpack_mono(APEContext *ctx, int count)
{
    if (ctx->frameflags & APE_FRAMECODE_STEREO_SILENCE) {
        /* We are pure silence, so we're done. */
        ALOGE("pure silence mono\n");
        return;
    }
    ctx->entropy_decode_mono(ctx, count);
    /* Now apply the predictor decoding */
    ctx->predictor_decode_mono(ctx, count);
    /* Pseudo-stereo - just copy left channel to right channel */
    if (ctx->channels == 2) {
        memcpy(ctx->decoded1, ctx->decoded0, count * sizeof(*ctx->decoded1));
    }
}

static void ape_unpack_stereo(APEContext *ctx, int count)
{
    unsigned left, right;
    int32_t *decoded0 = ctx->decoded0;
    int32_t *decoded1 = ctx->decoded1;

    if ((ctx->frameflags & APE_FRAMECODE_STEREO_SILENCE) == APE_FRAMECODE_STEREO_SILENCE) {
        /* We are pure silence, so we're done. */
        ALOGE( "pure silence stereo\n");
        return;
    }
    ctx->entropy_decode_stereo(ctx, count);
    /* Now apply the predictor decoding */
    ctx->predictor_decode_stereo(ctx, count);
    /* Decorrelate and scale to output depth */
    while (count--) {
        left = *decoded1 - (unsigned)(*decoded0 / 2);
        right = left + *decoded0;

        *(decoded0++) = left;
        *(decoded1++) = right;
    }
}

#define APE_STATE_FRAME_INIT    0
#define APE_STATE_CHUNK_DECODE  1

// Codec Version Definition

// project (bit31-28)
#define CODEC_PROJECT_SP_TABLET      (0x8)
#define CODEC_PROJECT_FP             (0x9)
#define CODEC_PROJECT_BOX_TV         (0xA)

// compiler (bit27-24)
#define CODEC_COMPILER_VC            (0x0)
#define CODEC_COMPILER_ARM_RVCT      (0x1)
#define CODEC_COMPILER_ARM_GCC       (0x2)
#define CODEC_COMPILER_APOLLO        (0x3)

// Version x.xx
#define CODEC_APEDEC_MINOR_VER      (0x01)
#define CODEC_APEDEC_RELEASE_VER     (0x00)
int ape_decoder_get_version(void)
{
   unsigned char Project = 0;
   unsigned char Comp = 0;
   unsigned char Major = 0;
   unsigned char Minor = 0;
   unsigned char Release = 0;

#if defined(__ANDROID_SP_TABLET__)
   Project = CODEC_PROJECT_SP_TABLET;
#elif defined(__ANDROID_BOX_TV__)
   Project = CODEC_PROJECT_BOX_TV;
#else
   Project = CODEC_PROJECT_FP;
#endif

#if (__CC_ARM)
   Comp = CODEC_COMPILER_ARM_RVCT;
#elif (__GNUC__)
   Comp = CODEC_COMPILER_ARM_GCC;
#else
   Comp = CODEC_COMPILER_VC;
#endif

   Minor   = CODEC_APEDEC_MINOR_VER;
   Release = CODEC_APEDEC_RELEASE_VER;

   return (((unsigned int)Project << 28) +
            ((unsigned int)Comp << 24)   +
            ((unsigned int)Major << 16)  +
            ((unsigned int)Minor << 8)   +
            (unsigned int)Release);
}

void ape_decoder_get_mem_size(unsigned int *bs_buffer,
                         unsigned int *working_buffer,
                         unsigned int *pcm_buffer)
{
    *bs_buffer = INPUT_CHUNKSIZE;
    *working_buffer = sizeof(struct APEContext);
    *pcm_buffer = OUTPUT_CHUNKSIZE;
}

ape_decoder_handle ape_decoder_init(void*  working_buffer,
                 struct ape_decoder_init_param* ape_param)
{
    struct APEContext * s = working_buffer;
    memset(s, 0, sizeof(struct APEContext));

    DecCnt = 0;

    s->fileversion = ape_param->fileversion;
    s->compression_level = ape_param->compressiontype; //to check
    s->blocks_per_loop = ape_param->blocksperframe; //to check
    s->bps = ape_param->bps;
    s->channels = ape_param->channels;

    s->finalframeblocks = ape_param->finalframeblocks;
    s->totalframes = ape_param->totalframes;

    s->decode_state = APE_STATE_FRAME_INIT;
    s->curr_frame = 0;
    s->byte_offset = 0;

    ALOGD("ape dec init: v%d com%d b%d b%d ch%d fin%d tot%d \n",
        s->fileversion,s->compression_level,s->blocks_per_loop,
        s->bps,s->channels,s->finalframeblocks,s->totalframes);

    if (s->channels > 2) {
        ALOGE("Only mono and stereo is supported\n");
        return 0;
    }

    if (s->compression_level % 1000 || s->compression_level > COMPRESSION_LEVEL_INSANE ||
        !s->compression_level ||
        (s->fileversion < 3930 && s->compression_level == COMPRESSION_LEVEL_INSANE)) {
        ALOGE("Incorrect compression level %d\n", s->compression_level);
        return 0;
    }
    s->fset = s->compression_level / 1000 - 1;

    if (s->fileversion < 3860) {
        s->entropy_decode_mono   = entropy_decode_mono_0000;
        s->entropy_decode_stereo = entropy_decode_stereo_0000;
    } else if (s->fileversion < 3900) {
        s->entropy_decode_mono   = entropy_decode_mono_3860;
        s->entropy_decode_stereo = entropy_decode_stereo_3860;
    } else if (s->fileversion < 3930) {
        s->entropy_decode_mono   = entropy_decode_mono_3900;
        s->entropy_decode_stereo = entropy_decode_stereo_3900;
    } else if (s->fileversion < 3990) {
        s->entropy_decode_mono   = entropy_decode_mono_3900;
        s->entropy_decode_stereo = entropy_decode_stereo_3930;
    } else {
        s->entropy_decode_mono   = entropy_decode_mono_3990;
        s->entropy_decode_stereo = entropy_decode_stereo_3990;
    }

    if (s->fileversion < 3930) {
        s->predictor_decode_mono   = predictor_decode_mono_3800;
        s->predictor_decode_stereo = predictor_decode_stereo_3800;
    } else if (s->fileversion < 3950) {
        s->predictor_decode_mono   = predictor_decode_mono_3930;
        s->predictor_decode_stereo = predictor_decode_stereo_3930;
    } else {
        s->predictor_decode_mono   = predictor_decode_mono_3950;
        s->predictor_decode_stereo = predictor_decode_stereo_3950;
    }

    return s;
}

int ape_decoder_reset(ape_decoder_handle handle,
                  int firstbyte,
                  int newframe)
{
    struct APEContext * s = handle;
    s->decode_state = APE_STATE_FRAME_INIT;
    s->curr_frame = newframe;
    s->byte_offset = 3 - firstbyte; //current ape dec use big endian which is not same as before

    return 0;
}

int ape_decoder_decode(ape_decoder_handle handle,
                   unsigned char* inbuffer,
                   int* bytes_consumed,
                   unsigned char* outbuffer,
                   int* bytes_produced)
{
    const uint8_t *buf = inbuffer;
    APEContext *s = handle;
    int i, ret;
    int blockstodecode;
    unsigned char* p = outbuffer;
    int16_t sample16;
    int32_t sample32;

    DecCnt++;

    /* this should never be negative, but bad things will happen if it is, so
       check it just to make sure. */
    //assert(s->samples >= 0);
    ALOGD("ape dec start +\n");
    *bytes_consumed = 0;
    *bytes_produced = 0;

    //swap flow need do first
    int buf_size=INPUT_CHUNKSIZE;
    uint32_t offset=0;
#ifdef VERBOSE
    for (ii = 0; ii < 800; ) {
        ALOGD("swap before input %d:oft%d %x %x %x %x %x %x %x %x %x %x\n",
            DecCnt, ii, inbuffer[ii], inbuffer[ii+1], inbuffer[ii+2],
            inbuffer[ii+3], inbuffer[ii+4], inbuffer[ii+5],inbuffer[ii+6],
            inbuffer[ii+7], inbuffer[ii+8], inbuffer[ii+9]);
        ii = ii+100;
    }
#endif
    bswap_buf((uint32_t *) s->data, (const uint32_t *) buf,
                  buf_size >> 2);
    memset(s->data + (buf_size & ~3), 0, buf_size & 3);
#ifdef VERBOSE
    ALOGD("swap after input %d:oft%d %x %x %x %x %x %x %x %x %x %x\n",
        DecCnt, ii, s->data[ii], s->data[ii+1], s->data[ii+2], s->data[ii+3],
        s->data[ii+4], s->data[ii+5], s->data[ii+6], s->data[ii+7],
        s->data[ii+8], s->data[ii+9]);
#endif

    s->ptr = s->data;
    s->data_end = s->data + INPUT_CHUNKSIZE;
    s->ptr += s->byte_offset;

    if(s->decode_state == APE_STATE_FRAME_INIT) {
        uint32_t nblocks=0;

        //calculate current frame and blocks
        if (s->curr_frame == s->totalframes - 1)
            nblocks = s->finalframeblocks;
        else if (s->curr_frame < s->totalframes - 1)
            nblocks = s->blocks_per_loop;
        else
            return APE_ERR_EOS;

        if (s->fileversion >= 3900) {
            if (offset > 3) {
                ALOGE("Incorrect offset passed\n");
                s->data_size = 0;
                return -1;
            }
            if (s->data_end - s->ptr < offset) {
                ALOGE("Packet is too small\n");
                return -1;
            }
            s->ptr += offset;
        } else {
            if ((ret = init_get_bits8(&s->gb, s->ptr, s->data_end - s->ptr)) < 0)
                return ret;
            if (s->fileversion > 3800)
                skip_bits_long(&s->gb, offset * 8);
            else
                skip_bits_long(&s->gb, offset);
        }

        if (!nblocks) {
            ALOGE("Invalid sample count is zero\n");
            return -1;
        }

        /* Initialize the frame decoder */
        if (init_frame_decoder(s) < 0) {
            ALOGE("Error reading frame header\n");
            return -1;
        }
        s->samples = nblocks;
        s->decode_state = APE_STATE_CHUNK_DECODE;

        *bytes_consumed = (s->ptr - s->data)&0xfffffffc;
        *bytes_produced = 0;
        s->byte_offset = (s->ptr - s->data)%4;
        ALOGD("ape dec dec init case *bytes_consumed:%d, offset:%d\n",
            *bytes_consumed,s->byte_offset);
        return 0;
    }

    blockstodecode = MIN(BLOCKS_PER_LOOP, s->samples);
    // for old files coefficients were not interleaved,
    // so we need to decode all of them at once
    if (s->fileversion < 3930)
        blockstodecode = s->samples;

    /* frame->data is output buffer */

    s->error=0;
    ALOGD("dec in data:0x%x,ptr:0x%x\n", s->data, s->ptr);

    if ((s->channels == 1) || (s->frameflags & APE_FRAMECODE_PSEUDO_STEREO))
        ape_unpack_mono(s, blockstodecode);
    else
        ape_unpack_stereo(s, blockstodecode);
    emms_c();
    ALOGD("dec out data:0x%x,ptr:0x%x,todec:0x%x\n", s->data, s->ptr, blockstodecode);
#ifdef VERBOSE
    ALOGD("dec out data %d:oft%d %x %x %x %x %x %x %x %x %x\n",
        DecCnt, ii, s->decoded0[ii], s->decoded0[ii+1], s->decoded0[ii+2],
        s->decoded0[ii+3], s->decoded0[ii+4], s->decoded0[ii+5],
        s->decoded0[ii+6], s->decoded0[ii+7], s->decoded0[ii+8]);
#endif

    if (s->error) {
        s->samples=0;
        ALOGE("Error decoding frame\n");
        return APE_ERR_EOS;
    }
    p = (uint8_t *)outbuffer;
    switch (s->bps) {
    case 8:
        for (i = 0; i < blockstodecode; i++) {
            *p++ = 0;
            *p++ = (s->decoded0[i]) & 0xff;
            if (s->channels == 2) {
                *p++ = 0;
                *p++ = (s->decoded1[i]) & 0xff;
            }
        }
        break;
    case 16:
        for (i = 0; i < blockstodecode; i++) {
            sample16 = s->decoded0[i];
            *(p++) = sample16 & 0xff;
            *(p++) = (sample16 >> 8) & 0xff;
            if (s->channels == 2) {
                sample16 = s->decoded1[i];
                *(p++) = sample16 & 0xff;
                *(p++) = (sample16 >> 8) & 0xff;
            }
        }
        break;
    case 24:
        for (i = 0; i < blockstodecode; i++) {
            sample32 = s->decoded0[i];
            *(p++) = sample32 & 0xff;
            *(p++) = (sample32 >> 8) & 0xff;
            *(p++) = (sample32 >> 16) & 0xff;
            if (s->channels == 2) {
                sample32 = s->decoded1[i];
                *(p++) = sample32 & 0xff;
                *(p++) = (sample32 >> 8) & 0xff;
                *(p++) = (sample32 >> 16) & 0xff;
            }
        }
        break;
    }

    *bytes_produced = p - outbuffer;
    s->samples -= blockstodecode;

    if (s->samples == 0) {
        ALOGD("ape decoder finish %d frames for total %d frames\n",
            s->curr_frame+1,s->totalframes);
        range_dec_normalize(s); //range_done_decoding, add to be same as mtk ape dec flow
        if (s->fileversion <= 3950) {
            switch (s->byte_offset) {
                case 0:
                    s->byte_offset = 2;
                    break;
                case 1:
                    s->byte_offset = 3;
                    break;
                case 2:
                    *bytes_consumed -= 4;
                    s->byte_offset = 0;
                    break;
                case 3:
                    *bytes_consumed -= 4;
                    s->byte_offset = 1;
                    break;
                default:
                    return APE_ERR_EOS;
            }
        }
        s->curr_frame++;
        s->decode_state = APE_STATE_FRAME_INIT;
    }

    *bytes_consumed = (s->ptr - s->data)&0xfffffffc; //ptr will go
    s->byte_offset = (s->ptr - s->data)%4;
    ALOGD("ape dec end %d *bytes_consumed:%d, *bytes_produced %d, samples:%d, offset:%d\n",
        DecCnt,*bytes_consumed,*bytes_produced,s->samples,s->byte_offset);
    return 0;
}

