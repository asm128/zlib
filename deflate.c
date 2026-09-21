/* deflate.c -- compress data using the deflation algorithm
 * Copyright (C) 1995-2017 Jean-loup Gailly and Mark Adler
 * For conditions of distribution and use, see copyright notice in zlib.h
 */

/*
 *  ALGORITHM
 *
 *      The "deflation" process depends on being able to identify portions
 *      of the input text which are identical to earlier input (within a
 *      sliding window trailing behind the input currently being processed).
 *
 *      The most straightforward technique turns out to be the fastest for
 *      most input files: try all possible matches and select the longest.
 *      The key feature of this algorithm is that insertions into the string
 *      dictionary are very simple and thus fast, and deletions are avoided
 *      completely. Insertions are performed at each input character, whereas
 *      string matches are performed only when the previous match ends. So it
 *      is preferable to spend more time in matches to allow very fast string
 *      insertions and avoid deletions. The matching algorithm for small
 *      strings is inspired from that of Rabin & Karp. A brute force approach
 *      is used to find longer strings when a small match has been found.
 *      A similar algorithm is used in comic (by Jan-Mark Wams) and freeze
 *      (by Leonid Broukhis).
 *         A previous version of this file used a more sophisticated algorithm
 *      (by Fiala and Greene) which is guaranteed to run in linear amortized
 *      time, but has a larger average cost, uses more memory and is patented.
 *      However the F&G algorithm may be faster for some highly redundant
 *      files if the parameter max_chain_length (described below) is too large.
 *
 *  ACKNOWLEDGEMENTS
 *
 *      The idea of lazy evaluation of matches is due to Jan-Mark Wams, and
 *      I found it in 'freeze' written by Leonid Broukhis.
 *      Thanks to many people for bug reports and testing.
 *
 *  REFERENCES
 *
 *      Deutsch, L.P.,"DEFLATE Compressed Data Format Specification".
 *      Available in http://tools.ietf.org/html/rfc1951
 *
 *      A description of the Rabin and Karp algorithm is given in the book
 *         "Algorithms" by R. Sedgewick, Addison-Wesley, p252.
 *
 *      Fiala,E.R., and Greene,D.H.
 *         Data Compression with Finite Windows, Comm.ACM, 32,4 (1989) 490-595
 *
 */

/* @(#) $Id$ */

#include "deflate.h"

const char deflate_copyright[] =
   " deflate 1.2.11 Copyright 1995-2017 Jean-loup Gailly and Mark Adler ";
/*
  If you use the zlib library in a product, an acknowledgment is welcome
  in the documentation of your product. If for some reason you cannot
  include such an acknowledgment, I would appreciate that you keep this
  copyright string in the executable of your product.
 */

/* ===========================================================================
 *  Function prototypes.
 */
typedef enum {
    need_more,      /* block not completed, need more input or more output */
    block_done,     /* block flush performed */
    finish_started, /* finish started, need only more output at next deflate */
    finish_done     /* finish done, accept no more input or output */
} block_state;

typedef block_state (*compress_func) OF((deflate_state *deflateState, int flushMode));
/* Compression function. Returns the block state after the call. */

local int deflateStateCheck      OF((z_streamp stream));
local void slide_hash     OF((deflate_state *deflateState));
local void fill_window    OF((deflate_state *deflateState));
local block_state deflate_stored OF((deflate_state *deflateState, int flushMode));
local block_state deflate_fast   OF((deflate_state *deflateState, int flushMode));
#ifndef FASTEST
local block_state deflate_slow   OF((deflate_state *deflateState, int flushMode));
#endif
local block_state deflate_rle    OF((deflate_state *deflateState, int flushMode));
local block_state deflate_huff   OF((deflate_state *deflateState, int flushMode));
local void lm_init        OF((deflate_state *deflateState));
local void putShortMSB    OF((deflate_state *deflateState, uInt b));
local void flush_pending  OF((z_streamp stream));
local unsigned read_buf   OF((z_streamp stream, Bytef *buf, unsigned size));
#ifdef ASMV
#  pragma message("Assembler code may have bugs -- use at your own risk")
      void match_init OF((void)); /* asm code initialization */
      uInt longest_match  OF((deflate_state *deflateState, IPos currentMatchPosition));
#else
local uInt longest_match  OF((deflate_state *deflateState, IPos currentMatchPosition));
#endif

#ifdef ZLIB_DEBUG
local  void check_match OF((deflate_state *deflateState, IPos start, IPos match,
                            int length));
#endif

/* ===========================================================================
 * Local data
 */

#define NIL 0
/* Tail of hash chains */

#ifndef TOO_FAR
#  define TOO_FAR 4096
#endif
/* Matches of length 3 are discarded if their distance exceeds TOO_FAR */

/* Values for max_lazy_match, good_match and max_chain_length, depending on
 * the desired pack level (0..9). The values given below have been tuned to
 * exclude worst case performance for pathological files. Better values may be
 * found for specific files.
 */
typedef struct config_s {
   ush good_length; /* reduce lazy search above this match length */
   ush max_lazy;    /* do not perform lazy search above this match length */
   ush nice_length; /* quit search above this match length */
   ush max_chain;
   compress_func func;
} config;

#ifdef FASTEST
local const config configuration_table[2] = {
/*      good lazy nice chain */
/* 0 */ {0,    0,  0,    0, deflate_stored},  /* store only */
/* 1 */ {4,    4,  8,    4, deflate_fast}}; /* max speed, no lazy matches */
#else
local const config configuration_table[10] = {
/*      good lazy nice chain */
/* 0 */ {0,    0,  0,    0, deflate_stored},  /* store only */
/* 1 */ {4,    4,  8,    4, deflate_fast}, /* max speed, no lazy matches */
/* 2 */ {4,    5, 16,    8, deflate_fast},
/* 3 */ {4,    6, 32,   32, deflate_fast},

/* 4 */ {4,    4, 16,   16, deflate_slow},  /* lazy matches */
/* 5 */ {8,   16, 32,   32, deflate_slow},
/* 6 */ {8,   16, 128, 128, deflate_slow},
/* 7 */ {8,   32, 128, 256, deflate_slow},
/* 8 */ {32, 128, 258, 1024, deflate_slow},
/* 9 */ {32, 258, 258, 4096, deflate_slow}}; /* max compression */
#endif

/* Note: the deflate() code requires max_lazy >= MIN_MATCH and max_chain >= 4
 * For deflate_fast() (levels <= 3) good is ignored and lazy has a different
 * meaning.
 */

/* rank Z_BLOCK between Z_NO_FLUSH and Z_PARTIAL_FLUSH */
#define RANK(f) (((f) * 2) - ((f) > 4 ? 9 : 0))

/* ===========================================================================
 * Update a hash value with the given input byte
 * IN  assertion: all calls to UPDATE_HASH are made with consecutive input
 *    characters, so that a running hash key can be computed from the previous
 *    key instead of complete recalculation each time.
 */
#define UPDATE_HASH(deflateState,h,c) (h = (((h)<<deflateState->hash_shift) ^ (c)) & deflateState->hash_mask)


/* ===========================================================================
 * Insert string str in the dictionary and set match_head to the previous head
 * of the hash chain (the most recent string with same hash key). Return
 * the previous length of the hash chain.
 * If this file is compiled with -DFASTEST, the compression level is forced
 * to 1, and no hash chains are maintained.
 * IN  assertion: all calls to INSERT_STRING are made with consecutive input
 *    characters and the first MIN_MATCH bytes of str are valid (except for
 *    the last MIN_MATCH-1 bytes of the input file).
 */
#ifdef FASTEST
#define INSERT_STRING(deflateState, str, match_head) \
   (UPDATE_HASH(deflateState, deflateState->ins_h, deflateState->window[(str) + (MIN_MATCH-1)]), \
    match_head = deflateState->head[deflateState->ins_h], \
    deflateState->head[deflateState->ins_h] = (Pos)(str))
#else
#define INSERT_STRING(deflateState, str, match_head) \
   (UPDATE_HASH(deflateState, deflateState->ins_h, deflateState->window[(str) + (MIN_MATCH-1)]), \
    match_head = deflateState->prev[(str) & deflateState->w_mask] = deflateState->head[deflateState->ins_h], \
    deflateState->head[deflateState->ins_h] = (Pos)(str))
#endif

/* ===========================================================================
 * Initialize the hash table (avoiding 64K overflow for 16 bit systems).
 * prev[] will be initialized on the fly.
 */
#define CLEAR_HASH(deflateState) \
    deflateState->head[deflateState->hash_size-1] = NIL; \
    zmemzero((Bytef *)deflateState->head, (unsigned)(deflateState->hash_size-1)*sizeof(*deflateState->head));

/* ===========================================================================
 * Slide the hash table when sliding the window down (could be avoided with 32
 * bit values at the expense of memory usage). We slide even when level == 0 to
 * keep the hash table consistent if we switch back to level > 0 later.
 */
local void slide_hash(deflateState)
    deflate_state *deflateState;
{
    unsigned entriesRemaining, matchPosition;
    Posf *hashEntry;
    uInt windowSize = deflateState->w_size;

    entriesRemaining = deflateState->hash_size;
    hashEntry = &deflateState->head[entriesRemaining];
    do {
        matchPosition = *--hashEntry;
        *hashEntry = (Pos)(matchPosition >= windowSize ? matchPosition - windowSize : NIL);
    } while (--entriesRemaining);
    entriesRemaining = windowSize;
#ifndef FASTEST
    hashEntry = &deflateState->prev[entriesRemaining];
    do {
        matchPosition = *--hashEntry;
        *hashEntry = (Pos)(matchPosition >= windowSize ? matchPosition - windowSize : NIL);
        /* If entriesRemaining is not on any hash chain, prev[entriesRemaining] is garbage but
         * its value will never be used.
         */
    } while (--entriesRemaining);
#endif
}

/* ========================================================================= */
int ZEXPORT deflateInit_(stream, level, version, streamSize)
    z_streamp stream;
    int level;
    const char *version;
    int streamSize;
{
    return deflateInit2_(stream, level, Z_DEFLATED, MAX_WBITS, DEF_MEM_LEVEL,
                         Z_DEFAULT_STRATEGY, version, streamSize);
    /* To do: ignore stream->next_in if we use it as window */
}

/* ========================================================================= */
int ZEXPORT deflateInit2_(stream, level, method, windowBits, memoryLevel, strategy,
                  version, streamSize)
    z_streamp stream;
    int  level;
    int  method;
    int  windowBits;
    int  memoryLevel;
    int  strategy;
    const char *version;
    int streamSize;
{
    deflate_state *deflateState;
    int wrapperMode = 1;
    static const char my_version[] = ZLIB_VERSION;

    ushf *bufferOverlay;
    /* We bufferOverlay pending_buf and d_buf+l_buf. This works since the average
     * output size for (length,distance) codes is <= 24 bits.
     */

    if (version == Z_NULL || version[0] != my_version[0] ||
        streamSize != sizeof(z_stream)) {
        return Z_VERSION_ERROR;
    }
    if (stream == Z_NULL) return Z_STREAM_ERROR;

    stream->msg = Z_NULL;
    if (stream->zalloc == (alloc_func)0) {
#ifdef Z_SOLO
        return Z_STREAM_ERROR;
#else
        stream->zalloc = zcalloc;
        stream->opaque = (voidpf)0;
#endif
    }
    if (stream->zfree == (free_func)0)
#ifdef Z_SOLO
        return Z_STREAM_ERROR;
#else
        stream->zfree = zcfree;
#endif

#ifdef FASTEST
    if (level != 0) level = 1;
#else
    if (level == Z_DEFAULT_COMPRESSION) level = 6;
#endif

    if (windowBits < 0) { /* suppress zlib wrapper */
        wrapperMode = 0;
        windowBits = -windowBits;
    }
#ifdef GZIP
    else if (windowBits > 15) {
        wrapperMode = 2;       /* write gzip wrapper instead */
        windowBits -= 16;
    }
#endif
    if (memoryLevel < 1 || memoryLevel > MAX_MEM_LEVEL || method != Z_DEFLATED ||
        windowBits < 8 || windowBits > 15 || level < 0 || level > 9 ||
        strategy < 0 || strategy > Z_FIXED || (windowBits == 8 && wrapperMode != 1)) {
        return Z_STREAM_ERROR;
    }
    if (windowBits == 8) windowBits = 9;  /* until 256-byte window bug fixed */
    deflateState = (deflate_state *) ZALLOC(stream, 1, sizeof(deflate_state));
    if (deflateState == Z_NULL) return Z_MEM_ERROR;
    stream->state = (struct internal_state FAR *)deflateState;
    deflateState->strm = stream;
    deflateState->status = INIT_STATE;     /* to pass state test in deflateReset() */

    deflateState->wrap = wrapperMode;
    deflateState->gzhead = Z_NULL;
    deflateState->w_bits = (uInt)windowBits;
    deflateState->w_size = 1 << deflateState->w_bits;
    deflateState->w_mask = deflateState->w_size - 1;

    deflateState->hash_bits = (uInt)memoryLevel + 7;
    deflateState->hash_size = 1 << deflateState->hash_bits;
    deflateState->hash_mask = deflateState->hash_size - 1;
    deflateState->hash_shift =  ((deflateState->hash_bits+MIN_MATCH-1)/MIN_MATCH);

    deflateState->window = (Bytef *) ZALLOC(stream, deflateState->w_size, 2*sizeof(Byte));
    deflateState->prev   = (Posf *)  ZALLOC(stream, deflateState->w_size, sizeof(Pos));
    deflateState->head   = (Posf *)  ZALLOC(stream, deflateState->hash_size, sizeof(Pos));

    deflateState->high_water = 0;      /* nothing written to deflateState->window yet */

    deflateState->lit_bufsize = 1 << (memoryLevel + 6); /* 16K elements by default */

    bufferOverlay = (ushf *) ZALLOC(stream, deflateState->lit_bufsize, sizeof(ush)+2);
    deflateState->pending_buf = (uchf *) bufferOverlay;
    deflateState->pending_buf_size = (ulg)deflateState->lit_bufsize * (sizeof(ush)+2L);

    if (deflateState->window == Z_NULL || deflateState->prev == Z_NULL || deflateState->head == Z_NULL ||
        deflateState->pending_buf == Z_NULL) {
        deflateState->status = FINISH_STATE;
        stream->msg = ERR_MSG(Z_MEM_ERROR);
        deflateEnd (stream);
        return Z_MEM_ERROR;
    }
    deflateState->d_buf = bufferOverlay + deflateState->lit_bufsize/sizeof(ush);
    deflateState->l_buf = deflateState->pending_buf + (1+sizeof(ush))*deflateState->lit_bufsize;

    deflateState->level = level;
    deflateState->strategy = strategy;
    deflateState->method = (Byte)method;

    return deflateReset(stream);
}

/* =========================================================================
 * Check for a valid deflate stream state. Return 0 if ok, 1 if not.
 */
local int deflateStateCheck (stream)
    z_streamp stream;
{
    deflate_state *deflateState;
    if (stream == Z_NULL ||
        stream->zalloc == (alloc_func)0 || stream->zfree == (free_func)0)
        return 1;
    deflateState = stream->state;
    if (deflateState == Z_NULL || deflateState->strm != stream || (deflateState->status != INIT_STATE &&
#ifdef GZIP
                                           deflateState->status != GZIP_STATE &&
#endif
                                           deflateState->status != EXTRA_STATE &&
                                           deflateState->status != NAME_STATE &&
                                           deflateState->status != COMMENT_STATE &&
                                           deflateState->status != HCRC_STATE &&
                                           deflateState->status != BUSY_STATE &&
                                           deflateState->status != FINISH_STATE))
        return 1;
    return 0;
}

/* ========================================================================= */
int ZEXPORT deflateSetDictionary (stream, dictionary, dictionaryLength)
    z_streamp stream;
    const Bytef *dictionary;
    uInt  dictionaryLength;
{
    deflate_state *deflateState;
    uInt insertPosition, positionsRemaining;
    int wrapperMode;
    unsigned savedInputAvailable;
    z_const unsigned char *savedInputNext;

    if (deflateStateCheck(stream) || dictionary == Z_NULL)
        return Z_STREAM_ERROR;
    deflateState = stream->state;
    wrapperMode = deflateState->wrap;
    if (wrapperMode == 2 || (wrapperMode == 1 && deflateState->status != INIT_STATE) || deflateState->lookahead)
        return Z_STREAM_ERROR;

    /* when using zlib wrappers, compute Adler-32 for provided dictionary */
    if (wrapperMode == 1)
        stream->adler = adler32(stream->adler, dictionary, dictionaryLength);
    deflateState->wrap = 0;                    /* avoid computing Adler-32 in read_buf */

    /* if dictionary would fill window, just replace the history */
    if (dictionaryLength >= deflateState->w_size) {
        if (wrapperMode == 0) {            /* already empty otherwise */
            CLEAR_HASH(deflateState);
            deflateState->strstart = 0;
            deflateState->block_start = 0L;
            deflateState->insert = 0;
        }
        dictionary += dictionaryLength - deflateState->w_size;  /* use the tail */
        dictionaryLength = deflateState->w_size;
    }

    /* insert dictionary into window and hash */
    savedInputAvailable = stream->avail_in;
    savedInputNext = stream->next_in;
    stream->avail_in = dictionaryLength;
    stream->next_in = (z_const Bytef *)dictionary;
    fill_window(deflateState);
    while (deflateState->lookahead >= MIN_MATCH) {
        insertPosition = deflateState->strstart;
        positionsRemaining = deflateState->lookahead - (MIN_MATCH-1);
        do {
            UPDATE_HASH(deflateState, deflateState->ins_h, deflateState->window[insertPosition + MIN_MATCH-1]);
#ifndef FASTEST
            deflateState->prev[insertPosition & deflateState->w_mask] = deflateState->head[deflateState->ins_h];
#endif
            deflateState->head[deflateState->ins_h] = (Pos)insertPosition;
            insertPosition++;
        } while (--positionsRemaining);
        deflateState->strstart = insertPosition;
        deflateState->lookahead = MIN_MATCH-1;
        fill_window(deflateState);
    }
    deflateState->strstart += deflateState->lookahead;
    deflateState->block_start = (long)deflateState->strstart;
    deflateState->insert = deflateState->lookahead;
    deflateState->lookahead = 0;
    deflateState->match_length = deflateState->prev_length = MIN_MATCH-1;
    deflateState->match_available = 0;
    stream->next_in = savedInputNext;
    stream->avail_in = savedInputAvailable;
    deflateState->wrap = wrapperMode;
    return Z_OK;
}

/* ========================================================================= */
int ZEXPORT deflateGetDictionary (stream, dictionary, dictionaryLength)
    z_streamp stream;
    Bytef *dictionary;
    uInt  *dictionaryLength;
{
    deflate_state *deflateState;
    uInt dictionaryBytes;

    if (deflateStateCheck(stream))
        return Z_STREAM_ERROR;
    deflateState = stream->state;
    dictionaryBytes = deflateState->strstart + deflateState->lookahead;
    if (dictionaryBytes > deflateState->w_size)
        dictionaryBytes = deflateState->w_size;
    if (dictionary != Z_NULL && dictionaryBytes)
        zmemcpy(dictionary, deflateState->window + deflateState->strstart + deflateState->lookahead - dictionaryBytes, dictionaryBytes);
    if (dictionaryLength != Z_NULL)
        *dictionaryLength = dictionaryBytes;
    return Z_OK;
}

/* ========================================================================= */
int ZEXPORT deflateResetKeep (stream)
    z_streamp stream;
{
    deflate_state *deflateState;

    if (deflateStateCheck(stream)) {
        return Z_STREAM_ERROR;
    }

    stream->total_in = stream->total_out = 0;
    stream->msg = Z_NULL; /* use zfree if we ever allocate msg dynamically */
    stream->data_type = Z_UNKNOWN;

    deflateState = (deflate_state *)stream->state;
    deflateState->pending = 0;
    deflateState->pending_out = deflateState->pending_buf;

    if (deflateState->wrap < 0) {
        deflateState->wrap = -deflateState->wrap; /* was made negative by deflate(..., Z_FINISH); */
    }
    deflateState->status =
#ifdef GZIP
        deflateState->wrap == 2 ? GZIP_STATE :
#endif
        deflateState->wrap ? INIT_STATE : BUSY_STATE;
    stream->adler =
#ifdef GZIP
        deflateState->wrap == 2 ? crc32(0L, Z_NULL, 0) :
#endif
        adler32(0L, Z_NULL, 0);
    deflateState->last_flush = Z_NO_FLUSH;

    _tr_init(deflateState);

    return Z_OK;
}

/* ========================================================================= */
int ZEXPORT deflateReset (stream)
    z_streamp stream;
{
    int resultCode;

    resultCode = deflateResetKeep(stream);
    if (resultCode == Z_OK)
        lm_init(stream->state);
    return resultCode;
}

/* ========================================================================= */
int ZEXPORT deflateSetHeader (stream, gzipHeader)
    z_streamp stream;
    gz_headerp gzipHeader;
{
    if (deflateStateCheck(stream) || stream->state->wrap != 2)
        return Z_STREAM_ERROR;
    stream->state->gzhead = gzipHeader;
    return Z_OK;
}

/* ========================================================================= */
int ZEXPORT deflatePending (stream, pending, bits)
    unsigned *pending;
    int *bits;
    z_streamp stream;
{
    if (deflateStateCheck(stream)) return Z_STREAM_ERROR;
    if (pending != Z_NULL)
        *pending = stream->state->pending;
    if (bits != Z_NULL)
        *bits = stream->state->bi_valid;
    return Z_OK;
}

/* ========================================================================= */
int ZEXPORT deflatePrime (stream, bits, value)
    z_streamp stream;
    int bits;
    int value;
{
    deflate_state *deflateState;
    int bitsToInsert;

    if (deflateStateCheck(stream)) return Z_STREAM_ERROR;
    deflateState = stream->state;
    if ((Bytef *)(deflateState->d_buf) < deflateState->pending_out + ((Buf_size + 7) >> 3))
        return Z_BUF_ERROR;
    do {
        bitsToInsert = Buf_size - deflateState->bi_valid;
        if (bitsToInsert > bits)
            bitsToInsert = bits;
        deflateState->bi_buf |= (ush)((value & ((1 << bitsToInsert) - 1)) << deflateState->bi_valid);
        deflateState->bi_valid += bitsToInsert;
        _tr_flush_bits(deflateState);
        value >>= bitsToInsert;
        bits -= bitsToInsert;
    } while (bits);
    return Z_OK;
}

/* ========================================================================= */
int ZEXPORT deflateParams(stream, level, strategy)
    z_streamp stream;
    int level;
    int strategy;
{
    deflate_state *deflateState;
    compress_func compressionFunction;

    if (deflateStateCheck(stream)) return Z_STREAM_ERROR;
    deflateState = stream->state;

#ifdef FASTEST
    if (level != 0) level = 1;
#else
    if (level == Z_DEFAULT_COMPRESSION) level = 6;
#endif
    if (level < 0 || level > 9 || strategy < 0 || strategy > Z_FIXED) {
        return Z_STREAM_ERROR;
    }
    compressionFunction = configuration_table[deflateState->level].func;

    if ((strategy != deflateState->strategy || compressionFunction != configuration_table[level].func) &&
        deflateState->high_water) {
        /* Flush the last buffer: */
        int resultCode = deflate(stream, Z_BLOCK);
        if (resultCode == Z_STREAM_ERROR)
            return resultCode;
        if (stream->avail_out == 0)
            return Z_BUF_ERROR;
    }
    if (deflateState->level != level) {
        if (deflateState->level == 0 && deflateState->matches != 0) {
            if (deflateState->matches == 1)
                slide_hash(deflateState);
            else
                CLEAR_HASH(deflateState);
            deflateState->matches = 0;
        }
        deflateState->level = level;
        deflateState->max_lazy_match   = configuration_table[level].max_lazy;
        deflateState->good_match       = configuration_table[level].good_length;
        deflateState->nice_match       = configuration_table[level].nice_length;
        deflateState->max_chain_length = configuration_table[level].max_chain;
    }
    deflateState->strategy = strategy;
    return Z_OK;
}

/* ========================================================================= */
int ZEXPORT deflateTune(stream, goodMatchLength, maximumLazyMatch, sufficientMatchLength, maximumChainLength)
    z_streamp stream;
    int goodMatchLength;
    int maximumLazyMatch;
    int sufficientMatchLength;
    int maximumChainLength;
{
    deflate_state *deflateState;

    if (deflateStateCheck(stream)) return Z_STREAM_ERROR;
    deflateState = stream->state;
    deflateState->good_match = (uInt)goodMatchLength;
    deflateState->max_lazy_match = (uInt)maximumLazyMatch;
    deflateState->nice_match = sufficientMatchLength;
    deflateState->max_chain_length = (uInt)maximumChainLength;
    return Z_OK;
}

/* =========================================================================
 * For the default windowBits of 15 and memoryLevel of 8, this function returns
 * a close to exact, as well as small, upper bound on the compressed size.
 * They are coded as constants here for a reason--if the #define's are
 * changed, then this function needs to be changed as well.  The return
 * value for 15 and 8 only works for those exact settings.
 *
 * For any setting other than those defaults for windowBits and memoryLevel,
 * the value returned is a conservative worst case for the maximum expansion
 * resulting from using fixed blocks instead of stored blocks, which deflate
 * can emit on compressed data for some combinations of the parameters.
 *
 * This function could be more sophisticated to provide closer upper bounds for
 * every combination of windowBits and memoryLevel.  But even the conservative
 * upper bound of about 14% expansion does not seem onerous for output buffer
 * allocation.
 */
uLong ZEXPORT deflateBound(stream, sourceLen)
    z_streamp stream;
    uLong sourceLen;
{
    deflate_state *deflateState;
    uLong compressedSizeBound, wrapperSize;

    /* conservative upper bound for compressed data */
    compressedSizeBound = sourceLen +
              ((sourceLen + 7) >> 3) + ((sourceLen + 63) >> 6) + 5;

    /* if can't get parameters, return conservative bound plus zlib wrapper */
    if (deflateStateCheck(stream))
        return compressedSizeBound + 6;

    /* compute wrapper length */
    deflateState = stream->state;
    switch (deflateState->wrap) {
    case 0:                                 /* raw deflate */
        wrapperSize = 0;
        break;
    case 1:                                 /* zlib wrapper */
        wrapperSize = 6 + (deflateState->strstart ? 4 : 0);
        break;
#ifdef GZIP
    case 2:                                 /* gzip wrapper */
        wrapperSize = 18;
        if (deflateState->gzhead != Z_NULL) {          /* user-supplied gzip header */
            Bytef *headerText;
            if (deflateState->gzhead->extra != Z_NULL)
                wrapperSize += 2 + deflateState->gzhead->extra_len;
            headerText = deflateState->gzhead->name;
            if (headerText != Z_NULL)
                do {
                    wrapperSize++;
                } while (*headerText++);
            headerText = deflateState->gzhead->comment;
            if (headerText != Z_NULL)
                do {
                    wrapperSize++;
                } while (*headerText++);
            if (deflateState->gzhead->hcrc)
                wrapperSize += 2;
        }
        break;
#endif
    default:                                /* for compiler happiness */
        wrapperSize = 6;
    }

    /* if not default parameters, return conservative bound */
    if (deflateState->w_bits != 15 || deflateState->hash_bits != 8 + 7)
        return compressedSizeBound + wrapperSize;

    /* default settings: return tight bound for that case */
    return sourceLen + (sourceLen >> 12) + (sourceLen >> 14) +
           (sourceLen >> 25) + 13 - 6 + wrapperSize;
}

/* =========================================================================
 * Put a short in the pending buffer. The 16-bit value is put in MSB order.
 * IN assertion: the stream state is correct and there is enough room in
 * pending_buf.
 */
local void putShortMSB (deflateState, shortValue)
    deflate_state *deflateState;
    uInt shortValue;
{
    put_byte(deflateState, (Byte)(shortValue >> 8));
    put_byte(deflateState, (Byte)(shortValue & 0xff));
}

/* =========================================================================
 * Flush as much pending output as possible. All deflate() output, except for
 * some deflate_stored() output, goes through this function so some
 * applications may wish to modify it to avoid allocating a large
 * stream->next_out buffer and copying into it. (See also read_buf()).
 */
local void flush_pending(stream)
    z_streamp stream;
{
    unsigned bytesToFlush;
    deflate_state *deflateState = stream->state;

    _tr_flush_bits(deflateState);
    bytesToFlush = deflateState->pending;
    if (bytesToFlush > stream->avail_out) bytesToFlush = stream->avail_out;
    if (bytesToFlush == 0) return;

    zmemcpy(stream->next_out, deflateState->pending_out, bytesToFlush);
    stream->next_out  += bytesToFlush;
    deflateState->pending_out  += bytesToFlush;
    stream->total_out += bytesToFlush;
    stream->avail_out -= bytesToFlush;
    deflateState->pending      -= bytesToFlush;
    if (deflateState->pending == 0) {
        deflateState->pending_out = deflateState->pending_buf;
    }
}

/* ===========================================================================
 * Update the header CRC with the bytes deflateState->pending_buf[beg..deflateState->pending - 1].
 */
#define HCRC_UPDATE(checksumStart) \
    do { \
        if (deflateState->gzhead->hcrc && deflateState->pending > (checksumStart)) \
            stream->adler = crc32(stream->adler, deflateState->pending_buf + (checksumStart), \
                                deflateState->pending - (checksumStart)); \
    } while (0)

/* ========================================================================= */
int ZEXPORT deflate (stream, flushMode)
    z_streamp stream;
    int flushMode;
{
    int previousFlush; /* value of flush param for previous deflate call */
    deflate_state *deflateState;

    if (deflateStateCheck(stream) || flushMode > Z_BLOCK || flushMode < 0) {
        return Z_STREAM_ERROR;
    }
    deflateState = stream->state;

    if (stream->next_out == Z_NULL ||
        (stream->avail_in != 0 && stream->next_in == Z_NULL) ||
        (deflateState->status == FINISH_STATE && flushMode != Z_FINISH)) {
        ERR_RETURN(stream, Z_STREAM_ERROR);
    }
    if (stream->avail_out == 0) ERR_RETURN(stream, Z_BUF_ERROR);

    previousFlush = deflateState->last_flush;
    deflateState->last_flush = flushMode;

    /* Flush as much pending output as possible */
    if (deflateState->pending != 0) {
        flush_pending(stream);
        if (stream->avail_out == 0) {
            /* Since avail_out is 0, deflate will be called again with
             * more output space, but possibly with both pending and
             * avail_in equal to zero. There won't be anything to do,
             * but this is not an error situation so make sure we
             * return OK instead of BUF_ERROR at next call of deflate:
             */
            deflateState->last_flush = -1;
            return Z_OK;
        }

    /* Make sure there is something to do and avoid duplicate consecutive
     * flushes. For repeated and useless calls with Z_FINISH, we keep
     * returning Z_STREAM_END instead of Z_BUF_ERROR.
     */
    } else if (stream->avail_in == 0 && RANK(flushMode) <= RANK(previousFlush) &&
               flushMode != Z_FINISH) {
        ERR_RETURN(stream, Z_BUF_ERROR);
    }

    /* User must not provide more input after the first FINISH: */
    if (deflateState->status == FINISH_STATE && stream->avail_in != 0) {
        ERR_RETURN(stream, Z_BUF_ERROR);
    }

    /* Write the header */
    if (deflateState->status == INIT_STATE) {
        /* zlib header */
        uInt header = (Z_DEFLATED + ((deflateState->w_bits-8)<<4)) << 8;
        uInt compressionLevelFlags;

        if (deflateState->strategy >= Z_HUFFMAN_ONLY || deflateState->level < 2)
            compressionLevelFlags = 0;
        else if (deflateState->level < 6)
            compressionLevelFlags = 1;
        else if (deflateState->level == 6)
            compressionLevelFlags = 2;
        else
            compressionLevelFlags = 3;
        header |= (compressionLevelFlags << 6);
        if (deflateState->strstart != 0) header |= PRESET_DICT;
        header += 31 - (header % 31);

        putShortMSB(deflateState, header);

        /* Save the adler32 of the preset dictionary: */
        if (deflateState->strstart != 0) {
            putShortMSB(deflateState, (uInt)(stream->adler >> 16));
            putShortMSB(deflateState, (uInt)(stream->adler & 0xffff));
        }
        stream->adler = adler32(0L, Z_NULL, 0);
        deflateState->status = BUSY_STATE;

        /* Compression must start with an empty pending buffer */
        flush_pending(stream);
        if (deflateState->pending != 0) {
            deflateState->last_flush = -1;
            return Z_OK;
        }
    }
#ifdef GZIP
    if (deflateState->status == GZIP_STATE) {
        /* gzip header */
        stream->adler = crc32(0L, Z_NULL, 0);
        put_byte(deflateState, 31);
        put_byte(deflateState, 139);
        put_byte(deflateState, 8);
        if (deflateState->gzhead == Z_NULL) {
            put_byte(deflateState, 0);
            put_byte(deflateState, 0);
            put_byte(deflateState, 0);
            put_byte(deflateState, 0);
            put_byte(deflateState, 0);
            put_byte(deflateState, deflateState->level == 9 ? 2 :
                     (deflateState->strategy >= Z_HUFFMAN_ONLY || deflateState->level < 2 ?
                      4 : 0));
            put_byte(deflateState, OS_CODE);
            deflateState->status = BUSY_STATE;

            /* Compression must start with an empty pending buffer */
            flush_pending(stream);
            if (deflateState->pending != 0) {
                deflateState->last_flush = -1;
                return Z_OK;
            }
        }
        else {
            put_byte(deflateState, (deflateState->gzhead->text ? 1 : 0) +
                     (deflateState->gzhead->hcrc ? 2 : 0) +
                     (deflateState->gzhead->extra == Z_NULL ? 0 : 4) +
                     (deflateState->gzhead->name == Z_NULL ? 0 : 8) +
                     (deflateState->gzhead->comment == Z_NULL ? 0 : 16)
                     );
            put_byte(deflateState, (Byte)(deflateState->gzhead->time & 0xff));
            put_byte(deflateState, (Byte)((deflateState->gzhead->time >> 8) & 0xff));
            put_byte(deflateState, (Byte)((deflateState->gzhead->time >> 16) & 0xff));
            put_byte(deflateState, (Byte)((deflateState->gzhead->time >> 24) & 0xff));
            put_byte(deflateState, deflateState->level == 9 ? 2 :
                     (deflateState->strategy >= Z_HUFFMAN_ONLY || deflateState->level < 2 ?
                      4 : 0));
            put_byte(deflateState, deflateState->gzhead->os & 0xff);
            if (deflateState->gzhead->extra != Z_NULL) {
                put_byte(deflateState, deflateState->gzhead->extra_len & 0xff);
                put_byte(deflateState, (deflateState->gzhead->extra_len >> 8) & 0xff);
            }
            if (deflateState->gzhead->hcrc)
                stream->adler = crc32(stream->adler, deflateState->pending_buf,
                                    deflateState->pending);
            deflateState->gzindex = 0;
            deflateState->status = EXTRA_STATE;
        }
    }
    if (deflateState->status == EXTRA_STATE) {
        if (deflateState->gzhead->extra != Z_NULL) {
            ulg checksumStart = deflateState->pending;   /* start of bytes to update crc */
            uInt extraBytesRemaining = (deflateState->gzhead->extra_len & 0xffff) - deflateState->gzindex;
            while (deflateState->pending + extraBytesRemaining > deflateState->pending_buf_size) {
                uInt extraBytesToCopy = deflateState->pending_buf_size - deflateState->pending;
                zmemcpy(deflateState->pending_buf + deflateState->pending,
                        deflateState->gzhead->extra + deflateState->gzindex, extraBytesToCopy);
                deflateState->pending = deflateState->pending_buf_size;
                HCRC_UPDATE(checksumStart);
                deflateState->gzindex += extraBytesToCopy;
                flush_pending(stream);
                if (deflateState->pending != 0) {
                    deflateState->last_flush = -1;
                    return Z_OK;
                }
                checksumStart = 0;
                extraBytesRemaining -= extraBytesToCopy;
            }
            zmemcpy(deflateState->pending_buf + deflateState->pending,
                    deflateState->gzhead->extra + deflateState->gzindex, extraBytesRemaining);
            deflateState->pending += extraBytesRemaining;
            HCRC_UPDATE(checksumStart);
            deflateState->gzindex = 0;
        }
        deflateState->status = NAME_STATE;
    }
    if (deflateState->status == NAME_STATE) {
        if (deflateState->gzhead->name != Z_NULL) {
            ulg checksumStart = deflateState->pending;   /* start of bytes to update crc */
            int headerByte;
            do {
                if (deflateState->pending == deflateState->pending_buf_size) {
                    HCRC_UPDATE(checksumStart);
                    flush_pending(stream);
                    if (deflateState->pending != 0) {
                        deflateState->last_flush = -1;
                        return Z_OK;
                    }
                    checksumStart = 0;
                }
                headerByte = deflateState->gzhead->name[deflateState->gzindex++];
                put_byte(deflateState, headerByte);
            } while (headerByte != 0);
            HCRC_UPDATE(checksumStart);
            deflateState->gzindex = 0;
        }
        deflateState->status = COMMENT_STATE;
    }
    if (deflateState->status == COMMENT_STATE) {
        if (deflateState->gzhead->comment != Z_NULL) {
            ulg checksumStart = deflateState->pending;   /* start of bytes to update crc */
            int headerByte;
            do {
                if (deflateState->pending == deflateState->pending_buf_size) {
                    HCRC_UPDATE(checksumStart);
                    flush_pending(stream);
                    if (deflateState->pending != 0) {
                        deflateState->last_flush = -1;
                        return Z_OK;
                    }
                    checksumStart = 0;
                }
                headerByte = deflateState->gzhead->comment[deflateState->gzindex++];
                put_byte(deflateState, headerByte);
            } while (headerByte != 0);
            HCRC_UPDATE(checksumStart);
        }
        deflateState->status = HCRC_STATE;
    }
    if (deflateState->status == HCRC_STATE) {
        if (deflateState->gzhead->hcrc) {
            if (deflateState->pending + 2 > deflateState->pending_buf_size) {
                flush_pending(stream);
                if (deflateState->pending != 0) {
                    deflateState->last_flush = -1;
                    return Z_OK;
                }
            }
            put_byte(deflateState, (Byte)(stream->adler & 0xff));
            put_byte(deflateState, (Byte)((stream->adler >> 8) & 0xff));
            stream->adler = crc32(0L, Z_NULL, 0);
        }
        deflateState->status = BUSY_STATE;

        /* Compression must start with an empty pending buffer */
        flush_pending(stream);
        if (deflateState->pending != 0) {
            deflateState->last_flush = -1;
            return Z_OK;
        }
    }
#endif

    /* Start a new block or continue the current one.
     */
    if (stream->avail_in != 0 || deflateState->lookahead != 0 ||
        (flushMode != Z_NO_FLUSH && deflateState->status != FINISH_STATE)) {
        block_state blockResult;

        blockResult = deflateState->level == 0 ? deflate_stored(deflateState, flushMode) :
                 deflateState->strategy == Z_HUFFMAN_ONLY ? deflate_huff(deflateState, flushMode) :
                 deflateState->strategy == Z_RLE ? deflate_rle(deflateState, flushMode) :
                 (*(configuration_table[deflateState->level].func))(deflateState, flushMode);

        if (blockResult == finish_started || blockResult == finish_done) {
            deflateState->status = FINISH_STATE;
        }
        if (blockResult == need_more || blockResult == finish_started) {
            if (stream->avail_out == 0) {
                deflateState->last_flush = -1; /* avoid BUF_ERROR next call, see above */
            }
            return Z_OK;
            /* If flush != Z_NO_FLUSH && avail_out == 0, the next call
             * of deflate should use the same flush parameter to make sure
             * that the flush is complete. So we don't have to output an
             * empty block here, this will be done at next call. This also
             * ensures that for a very small output buffer, we emit at most
             * one empty block.
             */
        }
        if (blockResult == block_done) {
            if (flushMode == Z_PARTIAL_FLUSH) {
                _tr_align(deflateState);
            } else if (flushMode != Z_BLOCK) { /* FULL_FLUSH or SYNC_FLUSH */
                _tr_stored_block(deflateState, (char*)0, 0L, 0);
                /* For a full flush, this empty block will be recognized
                 * as a special marker by inflate_sync().
                 */
                if (flushMode == Z_FULL_FLUSH) {
                    CLEAR_HASH(deflateState);             /* forget history */
                    if (deflateState->lookahead == 0) {
                        deflateState->strstart = 0;
                        deflateState->block_start = 0L;
                        deflateState->insert = 0;
                    }
                }
            }
            flush_pending(stream);
            if (stream->avail_out == 0) {
              deflateState->last_flush = -1; /* avoid BUF_ERROR at next call, see above */
              return Z_OK;
            }
        }
    }

    if (flushMode != Z_FINISH) return Z_OK;
    if (deflateState->wrap <= 0) return Z_STREAM_END;

    /* Write the trailer */
#ifdef GZIP
    if (deflateState->wrap == 2) {
        put_byte(deflateState, (Byte)(stream->adler & 0xff));
        put_byte(deflateState, (Byte)((stream->adler >> 8) & 0xff));
        put_byte(deflateState, (Byte)((stream->adler >> 16) & 0xff));
        put_byte(deflateState, (Byte)((stream->adler >> 24) & 0xff));
        put_byte(deflateState, (Byte)(stream->total_in & 0xff));
        put_byte(deflateState, (Byte)((stream->total_in >> 8) & 0xff));
        put_byte(deflateState, (Byte)((stream->total_in >> 16) & 0xff));
        put_byte(deflateState, (Byte)((stream->total_in >> 24) & 0xff));
    }
    else
#endif
    {
        putShortMSB(deflateState, (uInt)(stream->adler >> 16));
        putShortMSB(deflateState, (uInt)(stream->adler & 0xffff));
    }
    flush_pending(stream);
    /* If avail_out is zero, the application will call deflate again
     * to flush the rest.
     */
    if (deflateState->wrap > 0) deflateState->wrap = -deflateState->wrap; /* write the trailer only once! */
    return deflateState->pending != 0 ? Z_OK : Z_STREAM_END;
}

/* ========================================================================= */
int ZEXPORT deflateEnd (stream)
    z_streamp stream;
{
    int status;

    if (deflateStateCheck(stream)) return Z_STREAM_ERROR;

    status = stream->state->status;

    /* Deallocate in reverse order of allocations: */
    TRY_FREE(stream, stream->state->pending_buf);
    TRY_FREE(stream, stream->state->head);
    TRY_FREE(stream, stream->state->prev);
    TRY_FREE(stream, stream->state->window);

    ZFREE(stream, stream->state);
    stream->state = Z_NULL;

    return status == BUSY_STATE ? Z_DATA_ERROR : Z_OK;
}

/* =========================================================================
 * Copy the sourceStream state to the destination state.
 * To simplify the sourceStream, this is not supported for 16-bit MSDOS (which
 * doesn't have enough memory anyway to duplicate compression states).
 */
int ZEXPORT deflateCopy (destinationStream, sourceStream)
    z_streamp destinationStream;
    z_streamp sourceStream;
{
#ifdef MAXSEG_64K
    return Z_STREAM_ERROR;
#else
    deflate_state *destinationState;
    deflate_state *sourceState;
    ushf *bufferOverlay;


    if (deflateStateCheck(sourceStream) || destinationStream == Z_NULL) {
        return Z_STREAM_ERROR;
    }

    sourceState = sourceStream->state;

    zmemcpy((voidpf)destinationStream, (voidpf)sourceStream, sizeof(z_stream));

    destinationState = (deflate_state *) ZALLOC(destinationStream, 1, sizeof(deflate_state));
    if (destinationState == Z_NULL) return Z_MEM_ERROR;
    destinationStream->state = (struct internal_state FAR *) destinationState;
    zmemcpy((voidpf)destinationState, (voidpf)sourceState, sizeof(deflate_state));
    destinationState->strm = destinationStream;

    destinationState->window = (Bytef *) ZALLOC(destinationStream, destinationState->w_size, 2*sizeof(Byte));
    destinationState->prev   = (Posf *)  ZALLOC(destinationStream, destinationState->w_size, sizeof(Pos));
    destinationState->head   = (Posf *)  ZALLOC(destinationStream, destinationState->hash_size, sizeof(Pos));
    bufferOverlay = (ushf *) ZALLOC(destinationStream, destinationState->lit_bufsize, sizeof(ush)+2);
    destinationState->pending_buf = (uchf *) bufferOverlay;

    if (destinationState->window == Z_NULL || destinationState->prev == Z_NULL || destinationState->head == Z_NULL ||
        destinationState->pending_buf == Z_NULL) {
        deflateEnd (destinationStream);
        return Z_MEM_ERROR;
    }
    /* following zmemcpy do not work for 16-bit MSDOS */
    zmemcpy(destinationState->window, sourceState->window, destinationState->w_size * 2 * sizeof(Byte));
    zmemcpy((voidpf)destinationState->prev, (voidpf)sourceState->prev, destinationState->w_size * sizeof(Pos));
    zmemcpy((voidpf)destinationState->head, (voidpf)sourceState->head, destinationState->hash_size * sizeof(Pos));
    zmemcpy(destinationState->pending_buf, sourceState->pending_buf, (uInt)destinationState->pending_buf_size);

    destinationState->pending_out = destinationState->pending_buf + (sourceState->pending_out - sourceState->pending_buf);
    destinationState->d_buf = bufferOverlay + destinationState->lit_bufsize/sizeof(ush);
    destinationState->l_buf = destinationState->pending_buf + (1+sizeof(ush))*destinationState->lit_bufsize;

    destinationState->l_desc.dyn_tree = destinationState->dyn_ltree;
    destinationState->d_desc.dyn_tree = destinationState->dyn_dtree;
    destinationState->bl_desc.dyn_tree = destinationState->bl_tree;

    return Z_OK;
#endif /* MAXSEG_64K */
}

/* ===========================================================================
 * Read a new buffer from the current input stream, update the adler32
 * and total number of bytes read.  All deflate() input goes through
 * this function so some applications may wish to modify it to avoid
 * allocating a large stream->next_in buffer and copying from it.
 * (See also flush_pending()).
 */
local unsigned read_buf(stream, destination, destinationCapacity)
    z_streamp stream;
    Bytef *destination;
    unsigned destinationCapacity;
{
    unsigned bytesToRead = stream->avail_in;

    if (bytesToRead > destinationCapacity) bytesToRead = destinationCapacity;
    if (bytesToRead == 0) return 0;

    stream->avail_in  -= bytesToRead;

    zmemcpy(destination, stream->next_in, bytesToRead);
    if (stream->state->wrap == 1) {
        stream->adler = adler32(stream->adler, destination, bytesToRead);
    }
#ifdef GZIP
    else if (stream->state->wrap == 2) {
        stream->adler = crc32(stream->adler, destination, bytesToRead);
    }
#endif
    stream->next_in  += bytesToRead;
    stream->total_in += bytesToRead;

    return bytesToRead;
}

/* ===========================================================================
 * Initialize the "longest match" routines for a new zlib stream
 */
local void lm_init (deflateState)
    deflate_state *deflateState;
{
    deflateState->window_size = (ulg)2L*deflateState->w_size;

    CLEAR_HASH(deflateState);

    /* Set the default configuration parameters:
     */
    deflateState->max_lazy_match   = configuration_table[deflateState->level].max_lazy;
    deflateState->good_match       = configuration_table[deflateState->level].good_length;
    deflateState->nice_match       = configuration_table[deflateState->level].nice_length;
    deflateState->max_chain_length = configuration_table[deflateState->level].max_chain;

    deflateState->strstart = 0;
    deflateState->block_start = 0L;
    deflateState->lookahead = 0;
    deflateState->insert = 0;
    deflateState->match_length = deflateState->prev_length = MIN_MATCH-1;
    deflateState->match_available = 0;
    deflateState->ins_h = 0;
#ifndef FASTEST
#ifdef ASMV
    match_init(); /* initialize the asm code */
#endif
#endif
}

#ifndef FASTEST
/* ===========================================================================
 * Set match_start to the longest match starting at the given string and
 * return its length. Matches shorter or equal to prev_length are discarded,
 * in which case the result is equal to prev_length and match_start is
 * garbage.
 * IN assertions: currentMatchPosition is the head of the hash chain for the current
 *   string (strstart) and its distance is <= MAX_DIST, and prev_length >= 1
 * OUT assertion: the match length is not greater than deflateState->lookahead.
 */
#ifndef ASMV
/* For 80x86 and 680x0, an optimized version will be provided in match.asm or
 * match.S. The code will be functionally equivalent.
 */
local uInt longest_match(deflateState, currentMatchPosition)
    deflate_state *deflateState;
    IPos currentMatchPosition;                             /* current match */
{
    unsigned chainSearchRemaining = deflateState->max_chain_length;/* max hash chain length */
    register Bytef *scanNext = deflateState->window + deflateState->strstart; /* current string */
    register Bytef *matchNext;                      /* matched string */
    register int matchLength;                           /* length of current match */
    int bestMatchLength = (int)deflateState->prev_length;         /* best match length so far */
    int sufficientMatchLength = deflateState->nice_match;             /* stop if match long enough */
    IPos limit = deflateState->strstart > (IPos)MAX_DIST(deflateState) ?
        deflateState->strstart - (IPos)MAX_DIST(deflateState) : NIL;
    /* Stop when currentMatchPosition becomes <= limit. To simplify the code,
     * we prevent matches with the string of window index 0.
     */
    Posf *previousPositions = deflateState->prev;
    uInt windowMask = deflateState->w_mask;

#ifdef UNALIGNED_OK
    /* Compare two bytes at a time. Note: this is not always beneficial.
     * Try with and without -DUNALIGNED_OK to check.
     */
    register Bytef *scanEnd = deflateState->window + deflateState->strstart + MAX_MATCH - 1;
    register ush scanStartWord = *(ushf*)scanNext;
    register ush scanEndValue   = *(ushf*)(scanNext+bestMatchLength-1);
#else
    register Bytef *scanEnd = deflateState->window + deflateState->strstart + MAX_MATCH;
    register Byte scanPreviousEndByte  = scanNext[bestMatchLength-1];
    register Byte scanEndValue   = scanNext[bestMatchLength];
#endif

    /* The code is optimized for HASH_BITS >= 8 and MAX_MATCH-2 multiple of 16.
     * It is easy to get rid of this optimization if necessary.
     */
    Assert(deflateState->hash_bits >= 8 && MAX_MATCH == 258, "Code too clever");

    /* Do not waste too much time if we already have a good match: */
    if (deflateState->prev_length >= deflateState->good_match) {
        chainSearchRemaining >>= 2;
    }
    /* Do not look for matches beyond the end of the input. This is necessary
     * to make deflate deterministic.
     */
    if ((uInt)sufficientMatchLength > deflateState->lookahead) sufficientMatchLength = (int)deflateState->lookahead;

    Assert((ulg)deflateState->strstart <= deflateState->window_size-MIN_LOOKAHEAD, "need lookahead");

    do {
        Assert(currentMatchPosition < deflateState->strstart, "no future");
        matchNext = deflateState->window + currentMatchPosition;

        /* Skip to next match if the match length cannot increase
         * or if the match length is less than 2.  Note that the checks below
         * for insufficient lookahead only occur occasionally for performance
         * reasons.  Therefore uninitialized memory will be accessed, and
         * conditional jumps will be made that depend on those values.
         * However the length of the match is limited to the lookahead, so
         * the output of deflate is not affected by the uninitialized values.
         */
#if (defined(UNALIGNED_OK) && MAX_MATCH == 258)
        /* This code assumes sizeof(unsigned short) == 2. Do not use
         * UNALIGNED_OK if your compiler uses a different size.
         */
        if (*(ushf*)(matchNext+bestMatchLength-1) != scanEndValue ||
            *(ushf*)matchNext != scanStartWord) continue;

        /* It is not necessary to compare scan[2] and match[2] since they are
         * always equal when the other bytes match, given that the hash keys
         * are equal and that HASH_BITS >= 8. Compare 2 bytes at a time at
         * strstart+3, +5, ... up to strstart+257. We check for insufficient
         * lookahead only every 4th comparison; the 128th check will be made
         * at strstart+257. If MAX_MATCH-2 is not a multiple of 8, it is
         * necessary to put more guard bytes at the end of the window, or
         * to check more often for insufficient lookahead.
         */
        Assert(scanNext[2] == matchNext[2], "scan[2]?");
        scanNext++, matchNext++;
        do {
        } while (*(ushf*)(scanNext+=2) == *(ushf*)(matchNext+=2) &&
                 *(ushf*)(scanNext+=2) == *(ushf*)(matchNext+=2) &&
                 *(ushf*)(scanNext+=2) == *(ushf*)(matchNext+=2) &&
                 *(ushf*)(scanNext+=2) == *(ushf*)(matchNext+=2) &&
                 scanNext < scanEnd);
        /* The funny "do {}" generates better code on most compilers */

        /* Here, scan <= window+strstart+257 */
        Assert(scanNext <= deflateState->window+(unsigned)(deflateState->window_size-1), "wild scan");
        if (*scanNext == *matchNext) scanNext++;

        matchLength = (MAX_MATCH - 1) - (int)(scanEnd-scanNext);
        scanNext = scanEnd - (MAX_MATCH-1);

#else /* UNALIGNED_OK */

        if (matchNext[bestMatchLength]   != scanEndValue  ||
            matchNext[bestMatchLength-1] != scanPreviousEndByte ||
            *matchNext            != *scanNext     ||
            *++matchNext          != scanNext[1])      continue;

        /* The check at bestMatchLength-1 can be removed because it will be made
         * again later. (This heuristic is not always a win.)
         * It is not necessary to compare scan[2] and match[2] since they
         * are always equal when the other bytes match, given that
         * the hash keys are equal and that HASH_BITS >= 8.
         */
        scanNext += 2, matchNext++;
        Assert(*scanNext == *matchNext, "match[2]?");

        /* We check for insufficient lookahead only every 8th comparison;
         * the 256th check will be made at strstart+258.
         */
        do {
        } while (*++scanNext == *++matchNext && *++scanNext == *++matchNext &&
                 *++scanNext == *++matchNext && *++scanNext == *++matchNext &&
                 *++scanNext == *++matchNext && *++scanNext == *++matchNext &&
                 *++scanNext == *++matchNext && *++scanNext == *++matchNext &&
                 scanNext < scanEnd);

        Assert(scanNext <= deflateState->window+(unsigned)(deflateState->window_size-1), "wild scan");

        matchLength = MAX_MATCH - (int)(scanEnd - scanNext);
        scanNext = scanEnd - MAX_MATCH;

#endif /* UNALIGNED_OK */

        if (matchLength > bestMatchLength) {
            deflateState->match_start = currentMatchPosition;
            bestMatchLength = matchLength;
            if (matchLength >= sufficientMatchLength) break;
#ifdef UNALIGNED_OK
            scanEndValue = *(ushf*)(scanNext+bestMatchLength-1);
#else
            scanPreviousEndByte  = scanNext[bestMatchLength-1];
            scanEndValue   = scanNext[bestMatchLength];
#endif
        }
    } while ((currentMatchPosition = previousPositions[currentMatchPosition & windowMask]) > limit
             && --chainSearchRemaining != 0);

    if ((uInt)bestMatchLength <= deflateState->lookahead) return (uInt)bestMatchLength;
    return deflateState->lookahead;
}
#endif /* ASMV */

#else /* FASTEST */

/* ---------------------------------------------------------------------------
 * Optimized version for FASTEST only
 */
local uInt longest_match(deflateState, currentMatchPosition)
    deflate_state *deflateState;
    IPos currentMatchPosition;                             /* current match */
{
    register Bytef *scanNext = deflateState->window + deflateState->strstart; /* current string */
    register Bytef *matchNext;                       /* matched string */
    register int matchLength;                           /* length of current match */
    register Bytef *scanEnd = deflateState->window + deflateState->strstart + MAX_MATCH;

    /* The code is optimized for HASH_BITS >= 8 and MAX_MATCH-2 multiple of 16.
     * It is easy to get rid of this optimization if necessary.
     */
    Assert(deflateState->hash_bits >= 8 && MAX_MATCH == 258, "Code too clever");

    Assert((ulg)deflateState->strstart <= deflateState->window_size-MIN_LOOKAHEAD, "need lookahead");

    Assert(currentMatchPosition < deflateState->strstart, "no future");

    matchNext = deflateState->window + currentMatchPosition;

    /* Return failure if the match length is less than 2:
     */
    if (matchNext[0] != scanNext[0] || matchNext[1] != scanNext[1]) return MIN_MATCH-1;

    /* The check at bestMatchLength-1 can be removed because it will be made
     * again later. (This heuristic is not always a win.)
     * It is not necessary to compare scan[2] and match[2] since they
     * are always equal when the other bytes match, given that
     * the hash keys are equal and that HASH_BITS >= 8.
     */
    scanNext += 2, matchNext += 2;
    Assert(*scanNext == *matchNext, "match[2]?");

    /* We check for insufficient lookahead only every 8th comparison;
     * the 256th check will be made at strstart+258.
     */
    do {
    } while (*++scanNext == *++matchNext && *++scanNext == *++matchNext &&
             *++scanNext == *++matchNext && *++scanNext == *++matchNext &&
             *++scanNext == *++matchNext && *++scanNext == *++matchNext &&
             *++scanNext == *++matchNext && *++scanNext == *++matchNext &&
             scanNext < scanEnd);

    Assert(scanNext <= deflateState->window+(unsigned)(deflateState->window_size-1), "wild scan");

    matchLength = MAX_MATCH - (int)(scanEnd - scanNext);

    if (matchLength < MIN_MATCH) return MIN_MATCH - 1;

    deflateState->match_start = currentMatchPosition;
    return (uInt)matchLength <= deflateState->lookahead ? (uInt)matchLength : deflateState->lookahead;
}

#endif /* FASTEST */

#ifdef ZLIB_DEBUG

#define EQUAL 0
/* result of memcmp for equal strings */

/* ===========================================================================
 * Check that the match at match_start is indeed a match.
 */
local void check_match(deflateState, sourcePosition, matchPosition, matchLength)
    deflate_state *deflateState;
    IPos sourcePosition, matchPosition;
    int matchLength;
{
    /* check that the match is indeed a match */
    if (zmemcmp(deflateState->window + matchPosition,
                deflateState->window + sourcePosition, matchLength) != EQUAL) {
        fprintf(stderr, " start %u, match %u, length %d\n",
                sourcePosition, matchPosition, matchLength);
        do {
            fprintf(stderr, "%c%c", deflateState->window[matchPosition++], deflateState->window[sourcePosition++]);
        } while (--matchLength != 0);
        z_error("invalid match");
    }
    if (z_verbose > 1) {
        fprintf(stderr,"\\[%d,%d]", sourcePosition-matchPosition, matchLength);
        do { putc(deflateState->window[sourcePosition++], stderr); } while (--matchLength != 0);
    }
}
#else
#  define check_match(deflateState, sourcePosition, matchPosition, matchLength)
#endif /* ZLIB_DEBUG */

/* ===========================================================================
 * Fill the window when the lookahead becomes insufficient.
 * Updates strstart and lookahead.
 *
 * IN assertion: lookahead < MIN_LOOKAHEAD
 * OUT assertions: strstart <= window_size-MIN_LOOKAHEAD
 *    At least one byte has been read, or avail_in == 0; reads are
 *    performed for at least two bytes (required for the zip translate_eol
 *    option -- not supported here).
 */
local void fill_window(deflateState)
    deflate_state *deflateState;
{
    unsigned bytesRead;
    unsigned windowSpaceAvailable;    /* Amount of free space at the end of the window. */
    uInt windowSize = deflateState->w_size;

    Assert(deflateState->lookahead < MIN_LOOKAHEAD, "already enough lookahead");

    do {
        windowSpaceAvailable = (unsigned)(deflateState->window_size -(ulg)deflateState->lookahead -(ulg)deflateState->strstart);

        /* Deal with !@#$% 64K limit: */
        if (sizeof(int) <= 2) {
            if (windowSpaceAvailable == 0 && deflateState->strstart == 0 && deflateState->lookahead == 0) {
                windowSpaceAvailable = windowSize;

            } else if (windowSpaceAvailable == (unsigned)(-1)) {
                /* Very unlikely, but possible on 16 bit machine if
                 * strstart == 0 && lookahead == 1 (input done a byte at time)
                 */
                windowSpaceAvailable--;
            }
        }

        /* If the window is almost full and there is insufficient lookahead,
         * move the upper half to the lower one to make room in the upper half.
         */
        if (deflateState->strstart >= windowSize+MAX_DIST(deflateState)) {

            zmemcpy(deflateState->window, deflateState->window+windowSize, (unsigned)windowSize - windowSpaceAvailable);
            deflateState->match_start -= windowSize;
            deflateState->strstart    -= windowSize; /* we now have strstart >= MAX_DIST */
            deflateState->block_start -= (long) windowSize;
            slide_hash(deflateState);
            windowSpaceAvailable += windowSize;
        }
        if (deflateState->strm->avail_in == 0) break;

        /* If there was no sliding:
         *    strstart <= WSIZE+MAX_DIST-1 && lookahead <= MIN_LOOKAHEAD - 1 &&
         *    more == window_size - lookahead - strstart
         * => more >= window_size - (MIN_LOOKAHEAD-1 + WSIZE + MAX_DIST-1)
         * => more >= window_size - 2*WSIZE + 2
         * In the BIG_MEM or MMAP case (not yet supported),
         *   window_size == input_size + MIN_LOOKAHEAD  &&
         *   strstart + deflateState->lookahead <= input_size => more >= MIN_LOOKAHEAD.
         * Otherwise, window_size == 2*WSIZE so more >= 2.
         * If there was sliding, more >= WSIZE. So in all cases, more >= 2.
         */
        Assert(windowSpaceAvailable >= 2, "more < 2");

        bytesRead = read_buf(deflateState->strm, deflateState->window + deflateState->strstart + deflateState->lookahead, windowSpaceAvailable);
        deflateState->lookahead += bytesRead;

        /* Initialize the hash value now that we have some input: */
        if (deflateState->lookahead + deflateState->insert >= MIN_MATCH) {
            uInt insertPosition = deflateState->strstart - deflateState->insert;
            deflateState->ins_h = deflateState->window[insertPosition];
            UPDATE_HASH(deflateState, deflateState->ins_h, deflateState->window[insertPosition + 1]);
#if MIN_MATCH != 3
            Call UPDATE_HASH() MIN_MATCH-3 windowSpaceAvailable times
#endif
            while (deflateState->insert) {
                UPDATE_HASH(deflateState, deflateState->ins_h, deflateState->window[insertPosition + MIN_MATCH-1]);
#ifndef FASTEST
                deflateState->prev[insertPosition & deflateState->w_mask] = deflateState->head[deflateState->ins_h];
#endif
                deflateState->head[deflateState->ins_h] = (Pos)insertPosition;
                insertPosition++;
                deflateState->insert--;
                if (deflateState->lookahead + deflateState->insert < MIN_MATCH)
                    break;
            }
        }
        /* If the whole input has less than MIN_MATCH bytes, ins_h is garbage,
         * but this is not important since only literal bytes will be emitted.
         */

    } while (deflateState->lookahead < MIN_LOOKAHEAD && deflateState->strm->avail_in != 0);

    /* If the WIN_INIT bytes after the end of the current data have never been
     * written, then zero those bytes in order to avoid memory check reports of
     * the use of uninitialized (or uninitialised as Julian writes) bytes by
     * the longest match routines.  Update the high water mark for the next
     * time through here.  WIN_INIT is set to MAX_MATCH since the longest match
     * routines allow scanning to strstart + MAX_MATCH, ignoring lookahead.
     */
    if (deflateState->high_water < deflateState->window_size) {
        ulg dataEndPosition = deflateState->strstart + (ulg)(deflateState->lookahead);
        ulg bytesToInitialize;

        if (deflateState->high_water < dataEndPosition) {
            /* Previous high water mark below current data -- zero WIN_INIT
             * bytes or up to end of window, whichever is less.
             */
            bytesToInitialize = deflateState->window_size - dataEndPosition;
            if (bytesToInitialize > WIN_INIT)
                bytesToInitialize = WIN_INIT;
            zmemzero(deflateState->window + dataEndPosition, (unsigned)bytesToInitialize);
            deflateState->high_water = dataEndPosition + bytesToInitialize;
        }
        else if (deflateState->high_water < (ulg)dataEndPosition + WIN_INIT) {
            /* High water mark at or above current data, but below current data
             * plus WIN_INIT -- zero out to current data plus WIN_INIT, or up
             * to end of window, whichever is less.
             */
            bytesToInitialize = (ulg)dataEndPosition + WIN_INIT - deflateState->high_water;
            if (bytesToInitialize > deflateState->window_size - deflateState->high_water)
                bytesToInitialize = deflateState->window_size - deflateState->high_water;
            zmemzero(deflateState->window + deflateState->high_water, (unsigned)bytesToInitialize);
            deflateState->high_water += bytesToInitialize;
        }
    }

    Assert((ulg)deflateState->strstart <= deflateState->window_size - MIN_LOOKAHEAD,
           "not enough room for search");
}

/* ===========================================================================
 * Flush the current block, with given end-of-file flag.
 * IN assertion: strstart is set to the end of the current match.
 */
#define FLUSH_BLOCK_ONLY(deflateState, last) { \
   _tr_flush_block(deflateState, (deflateState->block_start >= 0L ? \
                   (charf *)&deflateState->window[(unsigned)deflateState->block_start] : \
                   (charf *)Z_NULL), \
                (ulg)((long)deflateState->strstart - deflateState->block_start), \
                (last)); \
   deflateState->block_start = deflateState->strstart; \
   flush_pending(deflateState->strm); \
   Tracev((stderr,"[FLUSH]")); \
}

/* Same but force premature exit if necessary. */
#define FLUSH_BLOCK(deflateState, last) { \
   FLUSH_BLOCK_ONLY(deflateState, last); \
   if (deflateState->strm->avail_out == 0) return (last) ? finish_started : need_more; \
}

/* Maximum stored block length in deflate format (not including header). */
#define MAX_STORED 65535

/* Minimum of a and b. */
#define MIN(a, b) ((a) > (b) ? (b) : (a))

/* ===========================================================================
 * Copy without compression as much as possible from the input stream, return
 * the current block state.
 *
 * In case deflateParams() is used to later switch to a non-zero compression
 * level, deflateState->matches (otherwise unused when storing) keeps track of the number
 * of hash table slides to perform. If deflateState->matches is 1, then one hash table
 * slide will be done when switching. If deflateState->matches is 2, the maximum value
 * allowed here, then the hash table will be cleared, since two or more slides
 * is the same as a clear.
 *
 * deflate_stored() is written to minimize the number of times an input byte is
 * copied. It is most efficient with large input and output buffers, which
 * maximizes the opportunites to have a single copy from next_in to next_out.
 */
local block_state deflate_stored(deflateState, flushMode)
    deflate_state *deflateState;
    int flushMode;
{
    /* Smallest worthy block size when not flushing or finishing. By default
     * this is 32K. This can be as small as 507 bytes for memoryLevel == 1. For
     * large input and output buffers, the stored block size will be larger.
     */
    unsigned minimumBlockSize = MIN(deflateState->pending_buf_size - 5, deflateState->w_size);

    /* Copy as many minimumBlockSize or larger stored blocks directly to next_out as
     * possible. If flushing, copy the remaining available input to next_out as
     * stored blocks, if there is enough space.
     */
    unsigned storedBlockLength, windowBytesRemaining, bufferByteCount, isLastBlock = 0;
    unsigned inputBytesCopied = deflateState->strm->avail_in;
    do {
        /* Set storedBlockLength to the maximum size block that we can copy directly with the
         * available input data and output space. Set left to how much of that
         * would be copied from what's left in the window.
         */
        storedBlockLength = MAX_STORED;       /* maximum deflate stored block length */
        bufferByteCount = (deflateState->bi_valid + 42) >> 3;         /* number of header bytes */
        if (deflateState->strm->avail_out < bufferByteCount)          /* need room for header */
            break;
            /* maximum stored block length that will fit in avail_out: */
        bufferByteCount = deflateState->strm->avail_out - bufferByteCount;
        windowBytesRemaining = deflateState->strstart - deflateState->block_start;    /* bytes left in window */
        if (storedBlockLength > (ulg)windowBytesRemaining + deflateState->strm->avail_in)
            storedBlockLength = windowBytesRemaining + deflateState->strm->avail_in;     /* limit storedBlockLength to the input */
        if (storedBlockLength > bufferByteCount)
            storedBlockLength = bufferByteCount;                         /* limit storedBlockLength to the output */

        /* If the stored block would be less than minimumBlockSize in length, or if
         * unable to copy all of the available input when flushing, then try
         * copying to the window and the pending buffer instead. Also don't
         * write an empty block when flushing -- deflate() does that.
         */
        if (storedBlockLength < minimumBlockSize && ((storedBlockLength == 0 && flushMode != Z_FINISH) ||
                                flushMode == Z_NO_FLUSH ||
                                storedBlockLength != windowBytesRemaining + deflateState->strm->avail_in))
            break;

        /* Make a dummy stored block in pending to get the header bytes,
         * including any pending bits. This also updates the debugging counts.
         */
        isLastBlock = flushMode == Z_FINISH && storedBlockLength == windowBytesRemaining + deflateState->strm->avail_in ? 1 : 0;
        _tr_stored_block(deflateState, (char *)0, 0L, isLastBlock);

        /* Replace the lengths in the dummy stored block with storedBlockLength. */
        deflateState->pending_buf[deflateState->pending - 4] = storedBlockLength;
        deflateState->pending_buf[deflateState->pending - 3] = storedBlockLength >> 8;
        deflateState->pending_buf[deflateState->pending - 2] = ~storedBlockLength;
        deflateState->pending_buf[deflateState->pending - 1] = ~storedBlockLength >> 8;

        /* Write the stored block header bytes. */
        flush_pending(deflateState->strm);

#ifdef ZLIB_DEBUG
        /* Update debugging counts for the data about to be copied. */
        deflateState->compressed_len += storedBlockLength << 3;
        deflateState->bits_sent += storedBlockLength << 3;
#endif

        /* Copy uncompressed bytes from the window to next_out. */
        if (windowBytesRemaining) {
            if (windowBytesRemaining > storedBlockLength)
                windowBytesRemaining = storedBlockLength;
            zmemcpy(deflateState->strm->next_out, deflateState->window + deflateState->block_start, windowBytesRemaining);
            deflateState->strm->next_out += windowBytesRemaining;
            deflateState->strm->avail_out -= windowBytesRemaining;
            deflateState->strm->total_out += windowBytesRemaining;
            deflateState->block_start += windowBytesRemaining;
            storedBlockLength -= windowBytesRemaining;
        }

        /* Copy uncompressed bytes directly from next_in to next_out, updating
         * the check value.
         */
        if (storedBlockLength) {
            read_buf(deflateState->strm, deflateState->strm->next_out, storedBlockLength);
            deflateState->strm->next_out += storedBlockLength;
            deflateState->strm->avail_out -= storedBlockLength;
            deflateState->strm->total_out += storedBlockLength;
        }
    } while (isLastBlock == 0);

    /* Update the sliding window with the last deflateState->w_size bytes of the copied
     * data, or append all of the copied data to the existing window if less
     * than deflateState->w_size bytes were copied. Also update the number of bytes to
     * insert in the hash tables, in the event that deflateParams() switches to
     * a non-zero compression level.
     */
    inputBytesCopied -= deflateState->strm->avail_in;      /* number of input bytes directly copied */
    if (inputBytesCopied) {
        /* If any input was used, then no unused input remains in the window,
         * therefore deflateState->block_start == deflateState->strstart.
         */
        if (inputBytesCopied >= deflateState->w_size) {    /* supplant the previous history */
            deflateState->matches = 2;         /* clear hash */
            zmemcpy(deflateState->window, deflateState->strm->next_in - deflateState->w_size, deflateState->w_size);
            deflateState->strstart = deflateState->w_size;
        }
        else {
            if (deflateState->window_size - deflateState->strstart <= inputBytesCopied) {
                /* Slide the window down. */
                deflateState->strstart -= deflateState->w_size;
                zmemcpy(deflateState->window, deflateState->window + deflateState->w_size, deflateState->strstart);
                if (deflateState->matches < 2)
                    deflateState->matches++;   /* add a pending slide_hash() */
            }
            zmemcpy(deflateState->window + deflateState->strstart, deflateState->strm->next_in - inputBytesCopied, inputBytesCopied);
            deflateState->strstart += inputBytesCopied;
        }
        deflateState->block_start = deflateState->strstart;
        deflateState->insert += MIN(inputBytesCopied, deflateState->w_size - deflateState->insert);
    }
    if (deflateState->high_water < deflateState->strstart)
        deflateState->high_water = deflateState->strstart;

    /* If the last block was written to next_out, then done. */
    if (isLastBlock)
        return finish_done;

    /* If flushing and all input has been consumed, then done. */
    if (flushMode != Z_NO_FLUSH && flushMode != Z_FINISH &&
        deflateState->strm->avail_in == 0 && (long)deflateState->strstart == deflateState->block_start)
        return block_done;

    /* Fill the window with any remaining input. */
    bufferByteCount = deflateState->window_size - deflateState->strstart - 1;
    if (deflateState->strm->avail_in > bufferByteCount && deflateState->block_start >= (long)deflateState->w_size) {
        /* Slide the window down. */
        deflateState->block_start -= deflateState->w_size;
        deflateState->strstart -= deflateState->w_size;
        zmemcpy(deflateState->window, deflateState->window + deflateState->w_size, deflateState->strstart);
        if (deflateState->matches < 2)
            deflateState->matches++;           /* add a pending slide_hash() */
        bufferByteCount += deflateState->w_size;          /* more space now */
    }
    if (bufferByteCount > deflateState->strm->avail_in)
        bufferByteCount = deflateState->strm->avail_in;
    if (bufferByteCount) {
        read_buf(deflateState->strm, deflateState->window + deflateState->strstart, bufferByteCount);
        deflateState->strstart += bufferByteCount;
    }
    if (deflateState->high_water < deflateState->strstart)
        deflateState->high_water = deflateState->strstart;

    /* There was not enough avail_out to write a complete worthy or flushed
     * stored block to next_out. Write a stored block to pending instead, if we
     * have enough input for a worthy block, or if flushing and there is enough
     * room for the remaining input as a stored block in the pending buffer.
     */
    bufferByteCount = (deflateState->bi_valid + 42) >> 3;         /* number of header bytes */
        /* maximum stored block length that will fit in pending: */
    bufferByteCount = MIN(deflateState->pending_buf_size - bufferByteCount, MAX_STORED);
    minimumBlockSize = MIN(bufferByteCount, deflateState->w_size);
    windowBytesRemaining = deflateState->strstart - deflateState->block_start;
    if (windowBytesRemaining >= minimumBlockSize ||
        ((windowBytesRemaining || flushMode == Z_FINISH) && flushMode != Z_NO_FLUSH &&
         deflateState->strm->avail_in == 0 && windowBytesRemaining <= bufferByteCount)) {
        storedBlockLength = MIN(windowBytesRemaining, bufferByteCount);
        isLastBlock = flushMode == Z_FINISH && deflateState->strm->avail_in == 0 &&
               storedBlockLength == windowBytesRemaining ? 1 : 0;
        _tr_stored_block(deflateState, (charf *)deflateState->window + deflateState->block_start, storedBlockLength, isLastBlock);
        deflateState->block_start += storedBlockLength;
        flush_pending(deflateState->strm);
    }

    /* We've done all we can with the available input and output. */
    return isLastBlock ? finish_started : need_more;
}

/* ===========================================================================
 * Compress as much as possible from the input stream, return the current
 * block state.
 * This function does not perform lazy evaluation of matches and inserts
 * new strings in the dictionary only for unmatched strings or for short
 * matches. It is used only for the fast compression options.
 */
local block_state deflate_fast(deflateState, flushMode)
    deflate_state *deflateState;
    int flushMode;
{
    IPos hashChainHead;       /* head of the hash chain */
    int shouldFlushBlock;           /* set if current block must be flushed */

    for (;;) {
        /* Make sure that we always have enough lookahead, except
         * at the end of the input file. We need MAX_MATCH bytes
         * for the next match, plus MIN_MATCH bytes to insert the
         * string following the next match.
         */
        if (deflateState->lookahead < MIN_LOOKAHEAD) {
            fill_window(deflateState);
            if (deflateState->lookahead < MIN_LOOKAHEAD && flushMode == Z_NO_FLUSH) {
                return need_more;
            }
            if (deflateState->lookahead == 0) break; /* flush the current block */
        }

        /* Insert the string window[strstart .. strstart+2] in the
         * dictionary, and set hashChainHead to the head of the hash chain:
         */
        hashChainHead = NIL;
        if (deflateState->lookahead >= MIN_MATCH) {
            INSERT_STRING(deflateState, deflateState->strstart, hashChainHead);
        }

        /* Find the longest match, discarding those <= prev_length.
         * At this point we have always match_length < MIN_MATCH
         */
        if (hashChainHead != NIL && deflateState->strstart - hashChainHead <= MAX_DIST(deflateState)) {
            /* To simplify the code, we prevent matches with the string
             * of window index 0 (in particular we have to avoid a match
             * of the string with itself at the start of the input file).
             */
            deflateState->match_length = longest_match (deflateState, hashChainHead);
            /* longest_match() sets match_start */
        }
        if (deflateState->match_length >= MIN_MATCH) {
            check_match(deflateState, deflateState->strstart, deflateState->match_start, deflateState->match_length);

            _tr_tally_dist(deflateState, deflateState->strstart - deflateState->match_start,
                           deflateState->match_length - MIN_MATCH, shouldFlushBlock);

            deflateState->lookahead -= deflateState->match_length;

            /* Insert new strings in the hash table only if the match length
             * is not too large. This saves time but degrades compression.
             */
#ifndef FASTEST
            if (deflateState->match_length <= deflateState->max_insert_length &&
                deflateState->lookahead >= MIN_MATCH) {
                deflateState->match_length--; /* string at strstart already in table */
                do {
                    deflateState->strstart++;
                    INSERT_STRING(deflateState, deflateState->strstart, hashChainHead);
                    /* strstart never exceeds WSIZE-MAX_MATCH, so there are
                     * always MIN_MATCH bytes ahead.
                     */
                } while (--deflateState->match_length != 0);
                deflateState->strstart++;
            } else
#endif
            {
                deflateState->strstart += deflateState->match_length;
                deflateState->match_length = 0;
                deflateState->ins_h = deflateState->window[deflateState->strstart];
                UPDATE_HASH(deflateState, deflateState->ins_h, deflateState->window[deflateState->strstart+1]);
#if MIN_MATCH != 3
                Call UPDATE_HASH() MIN_MATCH-3 more times
#endif
                /* If lookahead < MIN_MATCH, ins_h is garbage, but it does not
                 * matter since it will be recomputed at next deflate call.
                 */
            }
        } else {
            /* No match, output a literal byte */
            Tracevv((stderr,"%c", deflateState->window[deflateState->strstart]));
            _tr_tally_lit (deflateState, deflateState->window[deflateState->strstart], shouldFlushBlock);
            deflateState->lookahead--;
            deflateState->strstart++;
        }
        if (shouldFlushBlock) FLUSH_BLOCK(deflateState, 0);
    }
    deflateState->insert = deflateState->strstart < MIN_MATCH-1 ? deflateState->strstart : MIN_MATCH-1;
    if (flushMode == Z_FINISH) {
        FLUSH_BLOCK(deflateState, 1);
        return finish_done;
    }
    if (deflateState->last_lit)
        FLUSH_BLOCK(deflateState, 0);
    return block_done;
}

#ifndef FASTEST
/* ===========================================================================
 * Same as above, but achieves better compression. We use a lazy
 * evaluation for matches: a match is finally adopted only if there is
 * no better match at the next window position.
 */
local block_state deflate_slow(deflateState, flushMode)
    deflate_state *deflateState;
    int flushMode;
{
    IPos hashChainHead;          /* head of hash chain */
    int shouldFlushBlock;              /* set if current block must be flushed */

    /* Process the input block. */
    for (;;) {
        /* Make sure that we always have enough lookahead, except
         * at the end of the input file. We need MAX_MATCH bytes
         * for the next match, plus MIN_MATCH bytes to insert the
         * string following the next match.
         */
        if (deflateState->lookahead < MIN_LOOKAHEAD) {
            fill_window(deflateState);
            if (deflateState->lookahead < MIN_LOOKAHEAD && flushMode == Z_NO_FLUSH) {
                return need_more;
            }
            if (deflateState->lookahead == 0) break; /* flush the current block */
        }

        /* Insert the string window[strstart .. strstart+2] in the
         * dictionary, and set hashChainHead to the head of the hash chain:
         */
        hashChainHead = NIL;
        if (deflateState->lookahead >= MIN_MATCH) {
            INSERT_STRING(deflateState, deflateState->strstart, hashChainHead);
        }

        /* Find the longest match, discarding those <= prev_length.
         */
        deflateState->prev_length = deflateState->match_length, deflateState->prev_match = deflateState->match_start;
        deflateState->match_length = MIN_MATCH-1;

        if (hashChainHead != NIL && deflateState->prev_length < deflateState->max_lazy_match &&
            deflateState->strstart - hashChainHead <= MAX_DIST(deflateState)) {
            /* To simplify the code, we prevent matches with the string
             * of window index 0 (in particular we have to avoid a match
             * of the string with itself at the start of the input file).
             */
            deflateState->match_length = longest_match (deflateState, hashChainHead);
            /* longest_match() sets match_start */

            if (deflateState->match_length <= 5 && (deflateState->strategy == Z_FILTERED
#if TOO_FAR <= 32767
                || (deflateState->match_length == MIN_MATCH &&
                    deflateState->strstart - deflateState->match_start > TOO_FAR)
#endif
                )) {

                /* If prev_match is also MIN_MATCH, match_start is garbage
                 * but we will ignore the current match anyway.
                 */
                deflateState->match_length = MIN_MATCH-1;
            }
        }
        /* If there was a match at the previous step and the current
         * match is not better, output the previous match:
         */
        if (deflateState->prev_length >= MIN_MATCH && deflateState->match_length <= deflateState->prev_length) {
            uInt lastInsertPosition = deflateState->strstart + deflateState->lookahead - MIN_MATCH;
            /* Do not insert strings in hash table beyond this. */

            check_match(deflateState, deflateState->strstart-1, deflateState->prev_match, deflateState->prev_length);

            _tr_tally_dist(deflateState, deflateState->strstart -1 - deflateState->prev_match,
                           deflateState->prev_length - MIN_MATCH, shouldFlushBlock);

            /* Insert in hash table all strings up to the end of the match.
             * strstart-1 and strstart are already inserted. If there is not
             * enough lookahead, the last two strings are not inserted in
             * the hash table.
             */
            deflateState->lookahead -= deflateState->prev_length-1;
            deflateState->prev_length -= 2;
            do {
                if (++deflateState->strstart <= lastInsertPosition) {
                    INSERT_STRING(deflateState, deflateState->strstart, hashChainHead);
                }
            } while (--deflateState->prev_length != 0);
            deflateState->match_available = 0;
            deflateState->match_length = MIN_MATCH-1;
            deflateState->strstart++;

            if (shouldFlushBlock) FLUSH_BLOCK(deflateState, 0);

        } else if (deflateState->match_available) {
            /* If there was no match at the previous position, output a
             * single literal. If there was a match but the current match
             * is longer, truncate the previous match to a single literal.
             */
            Tracevv((stderr,"%c", deflateState->window[deflateState->strstart-1]));
            _tr_tally_lit(deflateState, deflateState->window[deflateState->strstart-1], shouldFlushBlock);
            if (shouldFlushBlock) {
                FLUSH_BLOCK_ONLY(deflateState, 0);
            }
            deflateState->strstart++;
            deflateState->lookahead--;
            if (deflateState->strm->avail_out == 0) return need_more;
        } else {
            /* There is no previous match to compare with, wait for
             * the next step to decide.
             */
            deflateState->match_available = 1;
            deflateState->strstart++;
            deflateState->lookahead--;
        }
    }
    Assert (flushMode != Z_NO_FLUSH, "no flush?");
    if (deflateState->match_available) {
        Tracevv((stderr,"%c", deflateState->window[deflateState->strstart-1]));
        _tr_tally_lit(deflateState, deflateState->window[deflateState->strstart-1], shouldFlushBlock);
        deflateState->match_available = 0;
    }
    deflateState->insert = deflateState->strstart < MIN_MATCH-1 ? deflateState->strstart : MIN_MATCH-1;
    if (flushMode == Z_FINISH) {
        FLUSH_BLOCK(deflateState, 1);
        return finish_done;
    }
    if (deflateState->last_lit)
        FLUSH_BLOCK(deflateState, 0);
    return block_done;
}
#endif /* FASTEST */

/* ===========================================================================
 * For Z_RLE, simply look for runs of bytes, generate matches only of distance
 * one.  Do not maintain a hash table.  (It will be regenerated if this run of
 * deflate switches away from Z_RLE.)
 */
local block_state deflate_rle(deflateState, flushMode)
    deflate_state *deflateState;
    int flushMode;
{
    int shouldFlushBlock;             /* set if current block must be flushed */
    uInt repeatedByte;              /* byte at distance one to match */
    Bytef *scanNext, *scanEnd;   /* scan goes up to scanEnd for length of run */

    for (;;) {
        /* Make sure that we always have enough lookahead, except
         * at the end of the input file. We need MAX_MATCH bytes
         * for the longest run, plus one for the unrolled loop.
         */
        if (deflateState->lookahead <= MAX_MATCH) {
            fill_window(deflateState);
            if (deflateState->lookahead <= MAX_MATCH && flushMode == Z_NO_FLUSH) {
                return need_more;
            }
            if (deflateState->lookahead == 0) break; /* flush the current block */
        }

        /* See how many times the previous byte repeats */
        deflateState->match_length = 0;
        if (deflateState->lookahead >= MIN_MATCH && deflateState->strstart > 0) {
            scanNext = deflateState->window + deflateState->strstart - 1;
            repeatedByte = *scanNext;
            if (repeatedByte == *++scanNext && repeatedByte == *++scanNext && repeatedByte == *++scanNext) {
                scanEnd = deflateState->window + deflateState->strstart + MAX_MATCH;
                do {
                } while (repeatedByte == *++scanNext && repeatedByte == *++scanNext &&
                         repeatedByte == *++scanNext && repeatedByte == *++scanNext &&
                         repeatedByte == *++scanNext && repeatedByte == *++scanNext &&
                         repeatedByte == *++scanNext && repeatedByte == *++scanNext &&
                         scanNext < scanEnd);
                deflateState->match_length = MAX_MATCH - (uInt)(scanEnd - scanNext);
                if (deflateState->match_length > deflateState->lookahead)
                    deflateState->match_length = deflateState->lookahead;
            }
            Assert(scanNext <= deflateState->window+(uInt)(deflateState->window_size-1), "wild scan");
        }

        /* Emit match if have run of MIN_MATCH or longer, else emit literal */
        if (deflateState->match_length >= MIN_MATCH) {
            check_match(deflateState, deflateState->strstart, deflateState->strstart - 1, deflateState->match_length);

            _tr_tally_dist(deflateState, 1, deflateState->match_length - MIN_MATCH, shouldFlushBlock);

            deflateState->lookahead -= deflateState->match_length;
            deflateState->strstart += deflateState->match_length;
            deflateState->match_length = 0;
        } else {
            /* No match, output a literal byte */
            Tracevv((stderr,"%c", deflateState->window[deflateState->strstart]));
            _tr_tally_lit (deflateState, deflateState->window[deflateState->strstart], shouldFlushBlock);
            deflateState->lookahead--;
            deflateState->strstart++;
        }
        if (shouldFlushBlock) FLUSH_BLOCK(deflateState, 0);
    }
    deflateState->insert = 0;
    if (flushMode == Z_FINISH) {
        FLUSH_BLOCK(deflateState, 1);
        return finish_done;
    }
    if (deflateState->last_lit)
        FLUSH_BLOCK(deflateState, 0);
    return block_done;
}

/* ===========================================================================
 * For Z_HUFFMAN_ONLY, do not look for matches.  Do not maintain a hash table.
 * (It will be regenerated if this run of deflate switches away from Huffman.)
 */
local block_state deflate_huff(deflateState, flushMode)
    deflate_state *deflateState;
    int flushMode;
{
    int shouldFlushBlock;             /* set if current block must be flushed */

    for (;;) {
        /* Make sure that we have a literal to write. */
        if (deflateState->lookahead == 0) {
            fill_window(deflateState);
            if (deflateState->lookahead == 0) {
                if (flushMode == Z_NO_FLUSH)
                    return need_more;
                break;      /* flush the current block */
            }
        }

        /* Output a literal byte */
        deflateState->match_length = 0;
        Tracevv((stderr,"%c", deflateState->window[deflateState->strstart]));
        _tr_tally_lit (deflateState, deflateState->window[deflateState->strstart], shouldFlushBlock);
        deflateState->lookahead--;
        deflateState->strstart++;
        if (shouldFlushBlock) FLUSH_BLOCK(deflateState, 0);
    }
    deflateState->insert = 0;
    if (flushMode == Z_FINISH) {
        FLUSH_BLOCK(deflateState, 1);
        return finish_done;
    }
    if (deflateState->last_lit)
        FLUSH_BLOCK(deflateState, 0);
    return block_done;
}
