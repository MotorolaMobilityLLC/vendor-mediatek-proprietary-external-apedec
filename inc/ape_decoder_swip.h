/*
* Copyright (c) 2004 Michael Niedermayer <michaelni@gmx.at>
* Copyright (c) 2016 Alexandra Hájková
* Copyright (c) 2020 Mediatek Inc.
*
*
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

#ifndef APE_DECODER_SWIP_H
#define APE_DECODER_SWIP_H

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "APE_DEC"

//#define DEBUG
//#define VERBOSE
#ifndef DEBUG
#ifdef ALOGD
#undef ALOGD
#endif
#ifdef ALOGV
#undef ALOGV
#endif
#define ALOGD(...)
#define ALOGV(...)
#endif

#define MIN(a,b) ((a) > (b) ? (b) : (a))
#define MAX(a,b) ((a) > (b) ? (a) : (b))

#define BSWAP16C(x) (((x) << 8 & 0xff00)  | ((x) >> 8 & 0x00ff))
#define BSWAP32C(x) (BSWAP16C(x) << 16 | BSWAP16C((x) >> 16))

#ifndef INT_MAX
#define INT_MAX 2147483647
#endif

#ifndef LOG2
#define LOG2(x) (31 - __builtin_clz((x)|1))
#endif

#define uint32_t unsigned int
#define uint16_t unsigned short
#define uint8_t  unsigned char
#define int32_t signed int
#define int16_t signed short
#define int8_t  signed char

#ifndef emms_c
#   define emms_c() do {} while(0)
#endif

/*
 * Safe bitstream reading:
 * optionally, the get_bits API can check to ensure that we
 * don't read past input buffer boundaries. This is protected
 * with CONFIG_SAFE_BITSTREAM_READER at the global level, and
 * then below that with UNCHECKED_BITSTREAM_READER at the per-
 * decoder level. This means that decoders that check internally
 * can "#define UNCHECKED_BITSTREAM_READER 1" to disable
 * overread checks.
 * Boundary checking causes a minor performance penalty so for
 * applications that won't want/need this, it can be disabled
 * globally using "#define CONFIG_SAFE_BITSTREAM_READER 0".
 */
#ifndef UNCHECKED_BITSTREAM_READER
#define UNCHECKED_BITSTREAM_READER !CONFIG_SAFE_BITSTREAM_READER
#endif

#ifndef CACHED_BITSTREAM_READER
#define CACHED_BITSTREAM_READER 0
#endif

typedef struct GetBitContext {
    const uint8_t *buffer, *buffer_end;
#if CACHED_BITSTREAM_READER
    uint64_t cache;
    unsigned bits_left;
#endif
    int index;
    int size_in_bits;
    int size_in_bits_plus8;
} GetBitContext;

static inline unsigned int get_bits(GetBitContext *s, int n);
static inline void skip_bits(GetBitContext *s, int n);
static inline unsigned int show_bits(GetBitContext *s, int n);

/**
 * Clip a signed integer value into the -32768,32767 range.
 * @param a value to clip
 * @return clipped value
 */
static inline int16_t clip_int16(int a)
{
    if ((a+0x8000U) & ~0xFFFF)
        return (a>>31) ^ 0x7FFF;
    else
        return a;
}

static inline int clip_c(int a, int amin, int amax)
{
    if (a < amin)
        return amin;
    else if (a > amax)
        return amax;
    else
        return a;
}

#if CACHED_BITSTREAM_READER
#   define MIN_CACHE_BITS 64
#elif defined LONG_BITSTREAM_READER
#   define MIN_CACHE_BITS 32
#else
#   define MIN_CACHE_BITS 25
#endif

#if !CACHED_BITSTREAM_READER

#define OPEN_READER_NOSIZE(name, gb)            \
    unsigned int name ## _index = (gb)->index;  \
    unsigned int name ## _cache

#if UNCHECKED_BITSTREAM_READER
#define OPEN_READER(name, gb) OPEN_READER_NOSIZE(name, gb)

#define BITS_AVAILABLE(name, gb) 1
#else
#define OPEN_READER(name, gb)                   \
    OPEN_READER_NOSIZE(name, gb);               \
    unsigned int name ## _size_plus8 = (gb)->size_in_bits_plus8

#define BITS_AVAILABLE(name, gb) name ## _index < name ## _size_plus8
#endif

#define CLOSE_READER(name, gb) (gb)->index = name ## _index

#define AV_INPUT_BUFFER_PADDING_SIZE 64

#ifndef AV_RB32
#   define AV_RB32(x)                                \
    (((uint32_t)((const uint8_t*)(x))[0] << 24) |    \
               (((const uint8_t*)(x))[1] << 16) |    \
               (((const uint8_t*)(x))[2] <<  8) |    \
                ((const uint8_t*)(x))[3])
#endif

#ifndef AV_RL32
#   define AV_RL32(x)                                \
    (((uint32_t)((const uint8_t*)(x))[3] << 24) |    \
               (((const uint8_t*)(x))[2] << 16) |    \
               (((const uint8_t*)(x))[1] <<  8) |    \
                ((const uint8_t*)(x))[0])
#endif

#ifndef NEG_SSR32
#   define NEG_SSR32(a,s) ((( int32_t)(a))>>(32-(s)))
#endif

#ifndef NEG_USR32
#   define NEG_USR32(a,s) (((uint32_t)(a))>>(32-(s)))
#endif

#ifndef FILTER_HISTORY_SIZE
#define FILTER_HISTORY_SIZE 512
#endif

#ifndef PREDICTOR_HISTORY_SIZE
#define PREDICTOR_HISTORY_SIZE 512
#endif

# ifdef LONG_BITSTREAM_READER

# define UPDATE_CACHE_LE(name, gb) name ## _cache = \
      AV_RL64((gb)->buffer + (name ## _index >> 3)) >> (name ## _index & 7)

# define UPDATE_CACHE_BE(name, gb) name ## _cache = \
      AV_RB64((gb)->buffer + (name ## _index >> 3)) >> (32 - (name ## _index & 7))

#else

# define UPDATE_CACHE_LE(name, gb) name ## _cache = \
      AV_RL32((gb)->buffer + (name ## _index >> 3)) >> (name ## _index & 7)

# define UPDATE_CACHE_BE(name, gb) name ## _cache = \
      AV_RB32((gb)->buffer + (name ## _index >> 3)) << (name ## _index & 7)

#endif


#ifdef BITSTREAM_READER_LE

# define UPDATE_CACHE(name, gb) UPDATE_CACHE_LE(name, gb)

# define SKIP_CACHE(name, gb, num) name ## _cache >>= (num)

#else

# define UPDATE_CACHE(name, gb) UPDATE_CACHE_BE(name, gb)

# define SKIP_CACHE(name, gb, num) name ## _cache <<= (num)

#endif

#if UNCHECKED_BITSTREAM_READER
#   define SKIP_COUNTER(name, gb, num) name ## _index += (num)
#else
#   define SKIP_COUNTER(name, gb, num) \
    name ## _index = FFMIN(name ## _size_plus8, name ## _index + (num))
#endif

#define BITS_LEFT(name, gb) ((int)((gb)->size_in_bits - name ## _index))

#define SKIP_BITS(name, gb, num)                \
    do {                                        \
        SKIP_CACHE(name, gb, num);              \
        SKIP_COUNTER(name, gb, num);            \
    } while (0)

#define LAST_SKIP_BITS(name, gb, num) SKIP_COUNTER(name, gb, num)

#define SHOW_UBITS_LE(name, gb, num) zero_extend(name ## _cache, num)
#define SHOW_SBITS_LE(name, gb, num) sign_extend(name ## _cache, num)

#define SHOW_UBITS_BE(name, gb, num) NEG_USR32(name ## _cache, num)
#define SHOW_SBITS_BE(name, gb, num) NEG_SSR32(name ## _cache, num)

#ifdef BITSTREAM_READER_LE
#   define SHOW_UBITS(name, gb, num) SHOW_UBITS_LE(name, gb, num)
#   define SHOW_SBITS(name, gb, num) SHOW_SBITS_LE(name, gb, num)
#else
#   define SHOW_UBITS(name, gb, num) SHOW_UBITS_BE(name, gb, num)
#   define SHOW_SBITS(name, gb, num) SHOW_SBITS_BE(name, gb, num)
#endif

#define GET_CACHE(name, gb) ((uint32_t) name ## _cache)

#endif

static inline int get_bits_count(const GetBitContext *s)
{
#if CACHED_BITSTREAM_READER
    return s->index - s->bits_left;
#else
    return s->index;
#endif
}

#if CACHED_BITSTREAM_READER
static inline void refill_32(GetBitContext *s, int is_le)
{
#if !UNCHECKED_BITSTREAM_READER
    if (s->index >> 3 >= s->buffer_end - s->buffer)
        return;
#endif

    if (is_le)
    s->cache       = (uint64_t)AV_RL32(s->buffer + (s->index >> 3)) << s->bits_left | s->cache;
    else
    s->cache       = s->cache | (uint64_t)AV_RB32(s->buffer + (s->index >> 3)) << (32 - s->bits_left);
    s->index     += 32;
    s->bits_left += 32;
}

static inline void refill_64(GetBitContext *s, int is_le)
{
#if !UNCHECKED_BITSTREAM_READER
    if (s->index >> 3 >= s->buffer_end - s->buffer)
        return;
#endif

    if (is_le)
    s->cache = AV_RL64(s->buffer + (s->index >> 3));
    else
    s->cache = AV_RB64(s->buffer + (s->index >> 3));
    s->index += 64;
    s->bits_left = 64;
}

static inline uint64_t get_val(GetBitContext *s, unsigned n, int is_le)
{
    uint64_t ret;
    //assert(n>0 && n<=63);
    if (is_le) {
        ret = s->cache & ((UINT64_C(1) << n) - 1);
        s->cache >>= n;
    } else {
        ret = s->cache >> (64 - n);
        s->cache <<= n;
    }
    s->bits_left -= n;
    return ret;
}

static inline unsigned show_val(const GetBitContext *s, unsigned n)
{
#ifdef BITSTREAM_READER_LE
    return s->cache & ((UINT64_C(1) << n) - 1);
#else
    return s->cache >> (64 - n);
#endif
}
#endif

/**
 * Skips the specified number of bits.
 * @param n the number of bits to skip,
 *          For the UNCHECKED_BITSTREAM_READER this must not cause the distance
 *          from the start to overflow int32_t. Staying within the bitstream + padding
 *          is sufficient, too.
 */
static inline void skip_bits_long(GetBitContext *s, int n)
{
#if CACHED_BITSTREAM_READER
    skip_bits(s, n);
#else
#if UNCHECKED_BITSTREAM_READER
    s->index += n;
#else
    s->index += clip_c(n, -s->index, s->size_in_bits_plus8 - s->index);
#endif
#endif
}

#if CACHED_BITSTREAM_READER
static inline void skip_remaining(GetBitContext *s, unsigned n)
{
#ifdef BITSTREAM_READER_LE
    s->cache >>= n;
#else
    s->cache <<= n;
#endif
    s->bits_left -= n;
}
#endif

static inline unsigned int get_bits1(GetBitContext *s)
{
#if CACHED_BITSTREAM_READER
    if (!s->bits_left)
#ifdef BITSTREAM_READER_LE
        refill_64(s, 1);
#else
        refill_64(s, 0);
#endif

#ifdef BITSTREAM_READER_LE
    return get_val(s, 1, 1);
#else
    return get_val(s, 1, 0);
#endif
#else
    unsigned int index = s->index;
    uint8_t result     = s->buffer[index >> 3];
#ifdef BITSTREAM_READER_LE
    result >>= index & 7;
    result  &= 1;
#else
    result <<= index & 7;
    result >>= 8 - 1;
#endif
#if !UNCHECKED_BITSTREAM_READER
    if (s->index < s->size_in_bits_plus8)
#endif
        index++;
    s->index = index;

    return result;
#endif
}

/**
 * Read 0-32 bits.
 */
static inline unsigned int get_bits_long(GetBitContext *s, int n)
{
    //assert(n>=0 && n<=32);
    if (!n) {
        return 0;
#if CACHED_BITSTREAM_READER
    }
    return get_bits(s, n);
#else
    } else if (n <= MIN_CACHE_BITS) {
        return get_bits(s, n);
    } else {
#ifdef BITSTREAM_READER_LE
        unsigned ret = get_bits(s, 16);
        return ret | (get_bits(s, n - 16) << 16);
#else
        unsigned ret = get_bits(s, 16) << (n - 16);
        return ret | get_bits(s, n - 16);
#endif
    }
#endif
}

/**
 * Read 0-64 bits.
 */
static inline uint64_t get_bits64(GetBitContext *s, int n)
{
    if (n <= 32) {
        return get_bits_long(s, n);
    } else {
#ifdef BITSTREAM_READER_LE
        uint64_t ret = get_bits_long(s, 32);
        return ret | (uint64_t) get_bits_long(s, n - 32) << 32;
#else
        uint64_t ret = (uint64_t) get_bits_long(s, n - 32) << 32;
        return ret | get_bits_long(s, 32);
#endif
    }
}

static inline int get_bits_left(GetBitContext *gb)
{
    return gb->size_in_bits - get_bits_count(gb);
}

/**
 * Read 1-25 bits.
 */
static inline unsigned int get_bits(GetBitContext *s, int n)
{
    register unsigned int tmp;
#if CACHED_BITSTREAM_READER

    //assert(n>0 && n<=32);
    if (n > s->bits_left) {
#ifdef BITSTREAM_READER_LE
        refill_32(s, 1);
#else
        refill_32(s, 0);
#endif
        if (s->bits_left < 32)
            s->bits_left = n;
    }

#ifdef BITSTREAM_READER_LE
    tmp = get_val(s, n, 1);
#else
    tmp = get_val(s, n, 0);
#endif
#else
    OPEN_READER(re, s);
    //assert(n>0 && n<=25);
    UPDATE_CACHE(re, s);
    tmp = SHOW_UBITS(re, s, n);
    LAST_SKIP_BITS(re, s, n);
    CLOSE_READER(re, s);
#endif
    //assert(tmp < UINT64_C(1) << n);
    return tmp;
}

static inline int init_get_bits_xe(GetBitContext *s, const uint8_t *buffer,
                                   int bit_size, int is_le)
{
    int buffer_size;
    int ret = 0;

    (void)(is_le);

    if (bit_size >= INT_MAX - MAX(7, AV_INPUT_BUFFER_PADDING_SIZE*8) || bit_size < 0 || !buffer) {
        bit_size    = 0;
        buffer      = NULL;
        ret         = APE_ERR_EOS;
    }

    buffer_size = (bit_size + 7) >> 3;

    s->buffer             = buffer;
    s->size_in_bits       = bit_size;
    s->size_in_bits_plus8 = bit_size + 8;
    s->buffer_end         = buffer + buffer_size;
    s->index              = 0;

#if CACHED_BITSTREAM_READER
    s->cache              = 0;
    s->bits_left          = 0;
    refill_64(s, is_le);
#endif

    return ret;
}

/**
 * Initialize GetBitContext.
 * @param buffer bitstream buffer, must be AV_INPUT_BUFFER_PADDING_SIZE bytes
 *        larger than the actual read bits because some optimized bitstream
 *        readers read 32 or 64 bit at once and could read over the end
 * @param bit_size the size of the buffer in bits
 * @return 0 on success, AVERROR_INVALIDDATA if the buffer_size would overflow.
 */
static inline int init_get_bits(GetBitContext *s, const uint8_t *buffer,
                                int bit_size)
{
#ifdef BITSTREAM_READER_LE
    return init_get_bits_xe(s, buffer, bit_size, 1);
#else
    return init_get_bits_xe(s, buffer, bit_size, 0);
#endif
}

/**
 * Initialize GetBitContext.
 * @param buffer bitstream buffer, must be AV_INPUT_BUFFER_PADDING_SIZE bytes
 *        larger than the actual read bits because some optimized bitstream
 *        readers read 32 or 64 bit at once and could read over the end
 * @param byte_size the size of the buffer in bytes
 * @return 0 on success, AVERROR_INVALIDDATA if the buffer_size would overflow.
 */
static inline int init_get_bits8(GetBitContext *s, const uint8_t *buffer,
                                 int byte_size)
{
    if (byte_size > INT_MAX / 8 || byte_size < 0)
        byte_size = -1;
    return init_get_bits(s, buffer, byte_size * 8);
}

/**
 * Get unary code of limited length
 * @param gb GetBitContext
 * @param[in] stop The bitstop value (unary code of 1's or 0's)
 * @param[in] len Maximum length
 * @return unary 0 based code index. This is also the length in bits of the
 * code excluding the stop bit.
 * (in case len=1)
 * 1            0
 * 0            1
 * (in case len=2)
 * 1            0
 * 01           1
 * 00           2
 * (in case len=3)
 * 1            0
 * 01           1
 * 001          2
 * 000          3
 */
static inline int get_unary(GetBitContext *gb, int stop, int len)
{
    int i;

    for(i = 0; i < len && get_bits1(gb) != stop; i++);
    return i;
}

#endif /* APE_DECODER_SWIP_H */
