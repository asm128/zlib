/* inflate.c -- zlib decompression
 * Copyright (C) 1995-2016 Mark Adler
 * For conditions of distribution and use, see copyright notice in zlib.h
 */

/*
 * Change history:
 *
 * 1.2.beta0    24 Nov 2002
 * - First version -- complete rewrite of inflate to simplify code, avoid
 *   creation of window when not needed, minimize use of window when it is
 *   needed, make inffast.c even faster, implement gzip decoding, and to
 *   improve code readability and style over the previous zlib inflate code
 *
 * 1.2.beta1    25 Nov 2002
 * - Use pointers for available input and output checking in inffast.c
 * - Remove input and output counters in inffast.c
 * - Change inffast.c entry and loop from avail_in >= 7 to >= 6
 * - Remove unnecessary second byte pull from length extra in inffast.c
 * - Unroll direct copy to three copies per loop in inffast.c
 *
 * 1.2.beta2    4 Dec 2002
 * - Change external routine names to reduce potential conflicts
 * - Correct filename to inffixed.h for fixed tables in inflate.c
 * - Make headerChecksumBytes[] unsigned char to match parameter type in inflate.c
 * - Change stream->next_out[-inflateState->offset] to *(stream->next_out - inflateState->offset)
 *   to avoid negation problem on Alphas (64 bit) in inflate.c
 *
 * 1.2.beta3    22 Dec 2002
 * - Add comments on inflateState->bits assertion in inffast.c
 * - Add comments on op field in inftrees.h
 * - Fix bug in reuse of allocated window after inflateReset()
 * - Remove bit fields--back to byte structure for speed
 * - Remove distance extra == 0 check in inflate_fast()--only helps for lengths
 * - Change post-increments to pre-increments in inflate_fast(), PPC biased?
 * - Add compile time option, POSTINC, to use post-increments instead (Intel?)
 * - Make MATCH copy in inflate() much faster for when inflate_fast() not used
 * - Use local copies of stream next and avail values, as well as local bit
 *   buffer and bit count in inflate()--for speed when inflate_fast() not used
 *
 * 1.2.beta4    1 Jan 2003
 * - Split ptr - 257 statements in inflate_table() to avoid compiler warnings
 * - Move a comment on output buffer sizes from inffast.c to inflate.c
 * - Add comments in inffast.c to introduce the inflate_fast() routine
 * - Rearrange window copies in inflate_fast() for speed and simplification
 * - Unroll last copy for window match in inflate_fast()
 * - Use local copies of window variables in inflate_fast() for speed
 * - Pull out common wnext == 0 case for speed in inflate_fast()
 * - Make op and len in inflate_fast() unsigned for consistency
 * - Add FAR to lcode and dcode declarations in inflate_fast()
 * - Simplified bad distance check in inflate_fast()
 * - Added inflateBackInit(), inflateBack(), and inflateBackEnd() in new
 *   sourceStream file infback.c to provide a call-back interface to inflate for
 *   programs like gzip and unzip -- uses window as output buffer to avoid
 *   window copying
 *
 * 1.2.beta5    1 Jan 2003
 * - Improved inflateBack() interface to allow the caller to provide initial
 *   input in stream.
 * - Fixed stored blocks bug in inflateBack()
 *
 * 1.2.beta6    4 Jan 2003
 * - Added comments in inffast.c on effectiveness of POSTINC
 * - Typecasting all around to reduce compiler warnings
 * - Changed loops from while (1) or do {} while (1) to for (;;), again to
 *   make compilers happy
 * - Changed type of window in inflateBackInit() to unsigned char *
 *
 * 1.2.beta7    27 Jan 2003
 * - Changed many types to unsigned or unsigned short to avoid warnings
 * - Added inflateCopy() function
 *
 * 1.2.0        9 Mar 2003
 * - Changed inflateBack() interface to provide separate opaque descriptors
 *   for the in() and out() functions
 * - Changed inflateBack() argument and in_func typedef to swap the length
 *   and buffer address return values for the input function
 * - Check next_in and next_out for Z_NULL on entry to inflate()
 *
 * The history for versions after 1.2.0 are in ChangeLog in zlib distribution.
 */

#include "zutil.h"
#include "inftrees.h"
#include "inflate.h"
#include "inffast.h"

#ifdef MAKEFIXED
#  ifndef BUILDFIXED
#    define BUILDFIXED
#  endif
#endif

/* function prototypes */
local int inflateStateCheck OF((z_streamp stream));
local void fixedtables OF((struct inflate_state FAR *inflateState));
local int updatewindow OF((z_streamp stream, const unsigned char FAR *end,
                           unsigned copyLength));
#ifdef BUILDFIXED
   void makefixed OF((void));
#endif
local unsigned syncsearch OF((unsigned FAR *inputBytesAvailable, const unsigned char FAR *buf,
                              unsigned len));

local int inflateStateCheck(stream)
z_streamp stream;
{
    struct inflate_state FAR *inflateState;
    if (stream == Z_NULL ||
        stream->zalloc == (alloc_func)0 || stream->zfree == (free_func)0)
        return 1;
    inflateState = (struct inflate_state FAR *)stream->state;
    if (inflateState == Z_NULL || inflateState->strm != stream ||
        inflateState->mode < HEAD || inflateState->mode > SYNC)
        return 1;
    return 0;
}

int ZEXPORT inflateResetKeep(stream)
z_streamp stream;
{
    struct inflate_state FAR *inflateState;

    if (inflateStateCheck(stream)) return Z_STREAM_ERROR;
    inflateState = (struct inflate_state FAR *)stream->state;
    stream->total_in = stream->total_out = inflateState->total = 0;
    stream->msg = Z_NULL;
    if (inflateState->wrap)        /* to support ill-conceived Java test suite */
        stream->adler = inflateState->wrap & 1;
    inflateState->mode = HEAD;
    inflateState->last = 0;
    inflateState->havedict = 0;
    inflateState->dmax = 32768U;
    inflateState->head = Z_NULL;
    inflateState->hold = 0;
    inflateState->bits = 0;
    inflateState->lencode = inflateState->distcode = inflateState->next = inflateState->codes;
    inflateState->sane = 1;
    inflateState->back = -1;
    Tracev((stderr, "inflate: reset\n"));
    return Z_OK;
}

int ZEXPORT inflateReset(stream)
z_streamp stream;
{
    struct inflate_state FAR *inflateState;

    if (inflateStateCheck(stream)) return Z_STREAM_ERROR;
    inflateState = (struct inflate_state FAR *)stream->state;
    inflateState->wsize = 0;
    inflateState->whave = 0;
    inflateState->wnext = 0;
    return inflateResetKeep(stream);
}

int ZEXPORT inflateReset2(stream, windowBits)
z_streamp stream;
int windowBits;
{
    int wrapperMode;
    struct inflate_state FAR *inflateState;

    /* get the state */
    if (inflateStateCheck(stream)) return Z_STREAM_ERROR;
    inflateState = (struct inflate_state FAR *)stream->state;

    /* extract wrap request from windowBits parameter */
    if (windowBits < 0) {
        wrapperMode = 0;
        windowBits = -windowBits;
    }
    else {
        wrapperMode = (windowBits >> 4) + 5;
#ifdef GUNZIP
        if (windowBits < 48)
            windowBits &= 15;
#endif
    }

    /* set number of window bits, free window if different */
    if (windowBits && (windowBits < 8 || windowBits > 15))
        return Z_STREAM_ERROR;
    if (inflateState->window != Z_NULL && inflateState->wbits != (unsigned)windowBits) {
        ZFREE(stream, inflateState->window);
        inflateState->window = Z_NULL;
    }

    /* update state and reset the rest of it */
    inflateState->wrap = wrapperMode;
    inflateState->wbits = (unsigned)windowBits;
    return inflateReset(stream);
}

int ZEXPORT inflateInit2_(stream, windowBits, version, stream_size)
z_streamp stream;
int windowBits;
const char *version;
int stream_size;
{
    int resultCode;
    struct inflate_state FAR *inflateState;

    if (version == Z_NULL || version[0] != ZLIB_VERSION[0] ||
        stream_size != (int)(sizeof(z_stream)))
        return Z_VERSION_ERROR;
    if (stream == Z_NULL) return Z_STREAM_ERROR;
    stream->msg = Z_NULL;                 /* in case we return an error */
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
    inflateState = (struct inflate_state FAR *)
            ZALLOC(stream, 1, sizeof(struct inflate_state));
    if (inflateState == Z_NULL) return Z_MEM_ERROR;
    Tracev((stderr, "inflate: allocated\n"));
    stream->state = (struct internal_state FAR *)inflateState;
    inflateState->strm = stream;
    inflateState->window = Z_NULL;
    inflateState->mode = HEAD;     /* to pass state test in inflateReset2() */
    resultCode = inflateReset2(stream, windowBits);
    if (resultCode != Z_OK) {
        ZFREE(stream, inflateState);
        stream->state = Z_NULL;
    }
    return resultCode;
}

int ZEXPORT inflateInit_(stream, version, stream_size)
z_streamp stream;
const char *version;
int stream_size;
{
    return inflateInit2_(stream, DEF_WBITS, version, stream_size);
}

int ZEXPORT inflatePrime(stream, bitCount, value)
z_streamp stream;
int bitCount;
int value;
{
    struct inflate_state FAR *inflateState;

    if (inflateStateCheck(stream)) return Z_STREAM_ERROR;
    inflateState = (struct inflate_state FAR *)stream->state;
    if (bitCount < 0) {
        inflateState->hold = 0;
        inflateState->bits = 0;
        return Z_OK;
    }
    if (bitCount > 16 || inflateState->bits + (uInt)bitCount > 32) return Z_STREAM_ERROR;
    value &= (1L << bitCount) - 1;
    inflateState->hold += (unsigned)value << inflateState->bits;
    inflateState->bits += (uInt)bitCount;
    return Z_OK;
}

/*
   Return state with length and distance decoding tables and index sizes set to
   fixed code decoding.  Normally this returns fixed tables from inffixed.h.
   If BUILDFIXED is defined, then instead this routine builds the tables the
   first time it's called, and returns those tables the first time and
   thereafter.  This reduces the size of the code by about 2K bytes, in
   exchange for a little execution time.  However, BUILDFIXED should not be
   used for threaded applications, since the rewriting of the tables and fixedTablesUninitialized
   may not be thread-safe.
 */
local void fixedtables(inflateState)
struct inflate_state FAR *inflateState;
{
#ifdef BUILDFIXED
    static int fixedTablesUninitialized = 1;
    static code *lenfix, *distfix;
    static code fixed[544];

    /* build fixed huffman tables if first call (may not be thread safe) */
    if (fixedTablesUninitialized) {
        unsigned iSymbol, bitCount;
        static code *nextTableEntry;

        /* literal/length table */
        iSymbol = 0;
        while (iSymbol < 144) inflateState->lens[iSymbol++] = 8;
        while (iSymbol < 256) inflateState->lens[iSymbol++] = 9;
        while (iSymbol < 280) inflateState->lens[iSymbol++] = 7;
        while (iSymbol < 288) inflateState->lens[iSymbol++] = 8;
        nextTableEntry = fixed;
        lenfix = nextTableEntry;
        bitCount = 9;
        inflate_table(LENS, inflateState->lens, 288, &(nextTableEntry), &(bitCount), inflateState->work);

        /* distance table */
        iSymbol = 0;
        while (iSymbol < 32) inflateState->lens[iSymbol++] = 5;
        distfix = nextTableEntry;
        bitCount = 5;
        inflate_table(DISTS, inflateState->lens, 32, &(nextTableEntry), &(bitCount), inflateState->work);

        /* do this just once */
        fixedTablesUninitialized = 0;
    }
#else /* !BUILDFIXED */
#   include "inffixed.h"
#endif /* BUILDFIXED */
    inflateState->lencode = lenfix;
    inflateState->lenbits = 9;
    inflateState->distcode = distfix;
    inflateState->distbits = 5;
}

#ifdef MAKEFIXED
#include <stdio.h>

/*
   Write out the inffixed.h that is #include'd above.  Defining MAKEFIXED also
   defines BUILDFIXED, so the tables are built on the fly.  makefixed() writes
   those tables to stdout, which would be piped to inffixed.h.  A small program
   can simply call makefixed to do this:

    void makefixed(void);

    int main(void)
    {
        makefixed();
        return 0;
    }

   Then that can be linked with zlib built with MAKEFIXED defined and run:

    a.out > inffixed.h
 */
void makefixed()
{
    unsigned low, size;
    struct inflate_state inflateState;

    fixedtables(&inflateState);
    puts("    /* inffixed.h -- table for decoding fixed codes");
    puts("     * Generated automatically by makefixed().");
    puts("     */");
    puts("");
    puts("    /* WARNING: this file should *not* be used by applications.");
    puts("       It is part of the implementation of this library and is");
    puts("       subject to change. Applications should only use zlib.h.");
    puts("     */");
    puts("");
    size = 1U << 9;
    printf("    static const code lenfix[%u] = {", size);
    low = 0;
    for (;;) {
        if ((low % 7) == 0) printf("\n        ");
        printf("{%u,%u,%d}", (low & 127) == 99 ? 64 : inflateState.lencode[low].op,
               inflateState.lencode[low].bits, inflateState.lencode[low].val);
        if (++low == size) break;
        putchar(',');
    }
    puts("\n    };");
    size = 1U << 5;
    printf("\n    static const code distfix[%u] = {", size);
    low = 0;
    for (;;) {
        if ((low % 6) == 0) printf("\n        ");
        printf("{%u,%u,%d}", inflateState.distcode[low].op, inflateState.distcode[low].bits,
               inflateState.distcode[low].val);
        if (++low == size) break;
        putchar(',');
    }
    puts("\n    };");
}
#endif /* MAKEFIXED */

/*
   Update the window with the last windowSize (normally 32K) bytes written before
   returning.  If window does not exist yet, create it.  This is only called
   when a window is already in use, or when output has been written during this
   inflate call, but the end of the deflate stream has not been reached yet.
   It is also called to create a window for dictionary data when a dictionary
   is loaded.

   Providing output buffers larger than 32K to inflate() should provide a speed
   advantage, since only the last 32K of output is copied to the sliding window
   upon return from inflate(), and since all distances after the first 32K of
   output will fall in the output data, making match copies simpler and faster.
   The advantage may be dependent on the size of the processor's data caches.
 */
local int updatewindow(stream, end, copyLength)
z_streamp stream;
const Bytef *end;
unsigned copyLength;
{
    struct inflate_state FAR *inflateState;
    unsigned windowTailBytes;

    inflateState = (struct inflate_state FAR *)stream->state;

    /* if it hasn't been done already, allocate space for the window */
    if (inflateState->window == Z_NULL) {
        inflateState->window = (unsigned char FAR *)
                        ZALLOC(stream, 1U << inflateState->wbits,
                               sizeof(unsigned char));
        if (inflateState->window == Z_NULL) return 1;
    }

    /* if window not in use yet, initialize */
    if (inflateState->wsize == 0) {
        inflateState->wsize = 1U << inflateState->wbits;
        inflateState->wnext = 0;
        inflateState->whave = 0;
    }

    /* copy inflateState->wsize or less output bytes into the circular window */
    if (copyLength >= inflateState->wsize) {
        zmemcpy(inflateState->window, end - inflateState->wsize, inflateState->wsize);
        inflateState->wnext = 0;
        inflateState->whave = inflateState->wsize;
    }
    else {
        windowTailBytes = inflateState->wsize - inflateState->wnext;
        if (windowTailBytes > copyLength) windowTailBytes = copyLength;
        zmemcpy(inflateState->window + inflateState->wnext, end - copyLength, windowTailBytes);
        copyLength -= windowTailBytes;
        if (copyLength) {
            zmemcpy(inflateState->window, end - copyLength, copyLength);
            inflateState->wnext = copyLength;
            inflateState->whave = inflateState->wsize;
        }
        else {
            inflateState->wnext += windowTailBytes;
            if (inflateState->wnext == inflateState->wsize) inflateState->wnext = 0;
            if (inflateState->whave < inflateState->wsize) inflateState->whave += windowTailBytes;
        }
    }
    return 0;
}

/* Macros for inflate(): */

/* check function to use adler32() for zlib or crc32() for gzip */
#ifdef GUNZIP
#  define UPDATE(check, buf, len) \
    (inflateState->flags ? crc32(check, buf, len) : adler32(check, buf, len))
#else
#  define UPDATE(check, buf, len) adler32(check, buf, len)
#endif

/* check macros for header crc */
#ifdef GUNZIP
#  define CRC2(check, word) \
    do { \
        headerChecksumBytes[0] = (unsigned char)(word); \
        headerChecksumBytes[1] = (unsigned char)((word) >> 8); \
        check = crc32(check, headerChecksumBytes, 2); \
    } while (0)

#  define CRC4(check, word) \
    do { \
        headerChecksumBytes[0] = (unsigned char)(word); \
        headerChecksumBytes[1] = (unsigned char)((word) >> 8); \
        headerChecksumBytes[2] = (unsigned char)((word) >> 16); \
        headerChecksumBytes[3] = (unsigned char)((word) >> 24); \
        check = crc32(check, headerChecksumBytes, 4); \
    } while (0)
#endif

/* Load registers with state in inflate() for speed */
#define LOAD() \
    do { \
        outputNext = stream->next_out; \
        outputBytesAvailable = stream->avail_out; \
        inputNext = stream->next_in; \
        inputBytesAvailable = stream->avail_in; \
        bitBuffer = inflateState->hold; \
        bitCount = inflateState->bits; \
    } while (0)

/* Restore state from registers in inflate() */
#define RESTORE() \
    do { \
        stream->next_out = outputNext; \
        stream->avail_out = outputBytesAvailable; \
        stream->next_in = inputNext; \
        stream->avail_in = inputBytesAvailable; \
        inflateState->hold = bitBuffer; \
        inflateState->bits = bitCount; \
    } while (0)

/* Clear the input bit accumulator */
#define INITBITS() \
    do { \
        bitBuffer = 0; \
        bitCount = 0; \
    } while (0)

/* Get a byte of input into the bit accumulator, or return from inflate()
   if there is no input available. */
#define PULLBYTE() \
    do { \
        if (inputBytesAvailable == 0) goto inf_leave; \
        inputBytesAvailable--; \
        bitBuffer += (unsigned long)(*inputNext++) << bitCount; \
        bitCount += 8; \
    } while (0)

/* Assure that there are at least n bits in the bit accumulator.  If there is
   not enough available input to do that, then return from inflate(). */
#define NEEDBITS(n) \
    do { \
        while (bitCount < (unsigned)(n)) \
            PULLBYTE(); \
    } while (0)

/* Return the low n bits of the bit accumulator (n < 16) */
#define BITS(n) \
    ((unsigned)bitBuffer & ((1U << (n)) - 1))

/* Remove n bits from the bit accumulator */
#define DROPBITS(n) \
    do { \
        bitBuffer >>= (n); \
        bitCount -= (unsigned)(n); \
    } while (0)

/* Remove zero to seven bits as needed to go to a byte boundary */
#define BYTEBITS() \
    do { \
        bitBuffer >>= bitCount & 7; \
        bitCount -= bitCount & 7; \
    } while (0)

/*
   inflate() uses a state machine to process as much input data and generate as
   much output data as possible before returning.  The state machine is
   structured roughly as follows:

    for (;;) switch (state) {
    ...
    case STATEn:
        if (not enough input data or output space to make progress)
            return;
        ... make progress ...
        state = STATEm;
        break;
    ...
    }

   so when inflate() is called again, the same case is attempted again, and
   if the appropriate resources are provided, the machine proceeds to the
   next state.  The NEEDBITS() macro is usually the way the state evaluates
   whether it can proceed or should return.  NEEDBITS() does the return if
   the requested bits are not available.  The typical use of the BITS macros
   is:

        NEEDBITS(n);
        ... do something with BITS(n) ...
        DROPBITS(n);

   where NEEDBITS(n) either returns from inflate() if there isn't enough
   input left to load n bits into the accumulator, or it continues.  BITS(n)
   gives the low n bits in the accumulator.  When done, DROPBITS(n) drops
   the low n bits off the accumulator.  INITBITS() clears the accumulator
   and sets the number of available bits to zero.  BYTEBITS() discards just
   enough bits to put the accumulator on a byte boundary.  After BYTEBITS()
   and a NEEDBITS(8), then BITS(8) would return the next byte in the stream.

   NEEDBITS(n) uses PULLBYTE() to get an available byte of input, or to return
   if there is no input available.  The decoding of variable length codes uses
   PULLBYTE() directly in order to pull just enough bytes to decode the next
   code, and no more.

   Some states loop until they get enough input, making sure that enough
   state information is maintained to continue the loop where it left off
   if NEEDBITS() returns in the loop.  For example, want, need, and keep
   would all have to actually be part of the saved state in case NEEDBITS()
   returns:

    case STATEw:
        while (want < need) {
            NEEDBITS(n);
            keep[want++] = BITS(n);
            DROPBITS(n);
        }
        state = STATEx;
    case STATEx:

   As shown above, if the next state is also the next case, then the break
   is omitted.

   A state may also return if there is not enough output space available to
   complete that state.  Those states are copying stored data, writing a
   literal byte, and copying a matching string.

   When returning, a "goto inf_leave" is used to update the total counters,
   update the check value, and determine whether any progress has been made
   during that inflate() call in order to return the proper return code.
   Progress is defined as a change in either stream->avail_in or stream->avail_out.
   When there is a window, goto inf_leave will update the window with the last
   output written.  If a goto inf_leave occurs in the middle of decompression
   and there is no window currently, goto inf_leave will create one and copy
   output to the window for the next call of inflate().

   In this implementation, the flush parameter of inflate() only affects the
   return code (per zlib.h).  inflate() always writes as much as possible to
   stream->next_out, given the space available and the provided input--the effect
   documented in zlib.h of Z_SYNC_FLUSH.  Furthermore, inflate() always defers
   the allocation of and copying into a sliding window until necessary, which
   provides the effect documented in zlib.h for Z_FINISH when the entire input
   stream available.  So the only thing the flush parameter actually does is:
   when flush is set to Z_FINISH, inflate() cannot return Z_OK.  Instead it
   will return Z_BUF_ERROR if it has not reached the end of the stream.
 */

int ZEXPORT inflate(stream, flushMode)
z_streamp stream;
int flushMode;
{
    struct inflate_state FAR *inflateState;
    z_const unsigned char FAR *inputNext;    /* next input */
    unsigned char FAR *outputNext;     /* next output */
    unsigned inputBytesAvailable, outputBytesAvailable;        /* available input and output */
    unsigned long bitBuffer;         /* bit buffer */
    unsigned bitCount;              /* bits in bit buffer */
    unsigned inputByteCount, outputByteCount;           /* save starting available input and output */
    unsigned copyLength;              /* number of stored or match bytes to copy */
    unsigned char FAR *matchSource;    /* where to copy match bytes from */
    code currentEntry;                  /* current decoding table entry */
    code parentEntry;                  /* parent table entry */
    unsigned decodeLength;               /* length to copy for repeats, bits to drop */
    int resultCode;                    /* return code */
#ifdef GUNZIP
    unsigned char headerChecksumBytes[4];      /* buffer for gzip header crc calculation */
#endif
    static const unsigned short order[19] = /* permutation of code lengths */
        {16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15};

    if (inflateStateCheck(stream) || stream->next_out == Z_NULL ||
        (stream->next_in == Z_NULL && stream->avail_in != 0))
        return Z_STREAM_ERROR;

    inflateState = (struct inflate_state FAR *)stream->state;
    if (inflateState->mode == TYPE) inflateState->mode = TYPEDO;      /* skip check */
    LOAD();
    inputByteCount = inputBytesAvailable;
    outputByteCount = outputBytesAvailable;
    resultCode = Z_OK;
    for (;;)
        switch (inflateState->mode) {
        case HEAD:
            if (inflateState->wrap == 0) {
                inflateState->mode = TYPEDO;
                break;
            }
            NEEDBITS(16);
#ifdef GUNZIP
            if ((inflateState->wrap & 2) && bitBuffer == 0x8b1f) {  /* gzip header */
                if (inflateState->wbits == 0)
                    inflateState->wbits = 15;
                inflateState->check = crc32(0L, Z_NULL, 0);
                CRC2(inflateState->check, bitBuffer);
                INITBITS();
                inflateState->mode = FLAGS;
                break;
            }
            inflateState->flags = 0;           /* expect zlib header */
            if (inflateState->head != Z_NULL)
                inflateState->head->done = -1;
            if (!(inflateState->wrap & 1) ||   /* check if zlib header allowed */
#else
            if (
#endif
                ((BITS(8) << 8) + (bitBuffer >> 8)) % 31) {
                stream->msg = (char *)"incorrect header check";
                inflateState->mode = BAD;
                break;
            }
            if (BITS(4) != Z_DEFLATED) {
                stream->msg = (char *)"unknown compression method";
                inflateState->mode = BAD;
                break;
            }
            DROPBITS(4);
            decodeLength = BITS(4) + 8;
            if (inflateState->wbits == 0)
                inflateState->wbits = decodeLength;
            if (decodeLength > 15 || decodeLength > inflateState->wbits) {
                stream->msg = (char *)"invalid window size";
                inflateState->mode = BAD;
                break;
            }
            inflateState->dmax = 1U << decodeLength;
            Tracev((stderr, "inflate:   zlib header ok\n"));
            stream->adler = inflateState->check = adler32(0L, Z_NULL, 0);
            inflateState->mode = bitBuffer & 0x200 ? DICTID : TYPE;
            INITBITS();
            break;
#ifdef GUNZIP
        case FLAGS:
            NEEDBITS(16);
            inflateState->flags = (int)(bitBuffer);
            if ((inflateState->flags & 0xff) != Z_DEFLATED) {
                stream->msg = (char *)"unknown compression method";
                inflateState->mode = BAD;
                break;
            }
            if (inflateState->flags & 0xe000) {
                stream->msg = (char *)"unknown header flags set";
                inflateState->mode = BAD;
                break;
            }
            if (inflateState->head != Z_NULL)
                inflateState->head->text = (int)((bitBuffer >> 8) & 1);
            if ((inflateState->flags & 0x0200) && (inflateState->wrap & 4))
                CRC2(inflateState->check, bitBuffer);
            INITBITS();
            inflateState->mode = TIME;
        case TIME:
            NEEDBITS(32);
            if (inflateState->head != Z_NULL)
                inflateState->head->time = bitBuffer;
            if ((inflateState->flags & 0x0200) && (inflateState->wrap & 4))
                CRC4(inflateState->check, bitBuffer);
            INITBITS();
            inflateState->mode = OS;
        case OS:
            NEEDBITS(16);
            if (inflateState->head != Z_NULL) {
                inflateState->head->xflags = (int)(bitBuffer & 0xff);
                inflateState->head->os = (int)(bitBuffer >> 8);
            }
            if ((inflateState->flags & 0x0200) && (inflateState->wrap & 4))
                CRC2(inflateState->check, bitBuffer);
            INITBITS();
            inflateState->mode = EXLEN;
        case EXLEN:
            if (inflateState->flags & 0x0400) {
                NEEDBITS(16);
                inflateState->length = (unsigned)(bitBuffer);
                if (inflateState->head != Z_NULL)
                    inflateState->head->extra_len = (unsigned)bitBuffer;
                if ((inflateState->flags & 0x0200) && (inflateState->wrap & 4))
                    CRC2(inflateState->check, bitBuffer);
                INITBITS();
            }
            else if (inflateState->head != Z_NULL)
                inflateState->head->extra = Z_NULL;
            inflateState->mode = EXTRA;
        case EXTRA:
            if (inflateState->flags & 0x0400) {
                copyLength = inflateState->length;
                if (copyLength > inputBytesAvailable) copyLength = inputBytesAvailable;
                if (copyLength) {
                    if (inflateState->head != Z_NULL &&
                        inflateState->head->extra != Z_NULL) {
                        decodeLength = inflateState->head->extra_len - inflateState->length;
                        zmemcpy(inflateState->head->extra + decodeLength, inputNext,
                                decodeLength + copyLength > inflateState->head->extra_max ?
                                inflateState->head->extra_max - decodeLength : copyLength);
                    }
                    if ((inflateState->flags & 0x0200) && (inflateState->wrap & 4))
                        inflateState->check = crc32(inflateState->check, inputNext, copyLength);
                    inputBytesAvailable -= copyLength;
                    inputNext += copyLength;
                    inflateState->length -= copyLength;
                }
                if (inflateState->length) goto inf_leave;
            }
            inflateState->length = 0;
            inflateState->mode = NAME;
        case NAME:
            if (inflateState->flags & 0x0800) {
                if (inputBytesAvailable == 0) goto inf_leave;
                copyLength = 0;
                do {
                    decodeLength = (unsigned)(inputNext[copyLength++]);
                    if (inflateState->head != Z_NULL &&
                            inflateState->head->name != Z_NULL &&
                            inflateState->length < inflateState->head->name_max)
                        inflateState->head->name[inflateState->length++] = (Bytef)decodeLength;
                } while (decodeLength && copyLength < inputBytesAvailable);
                if ((inflateState->flags & 0x0200) && (inflateState->wrap & 4))
                    inflateState->check = crc32(inflateState->check, inputNext, copyLength);
                inputBytesAvailable -= copyLength;
                inputNext += copyLength;
                if (decodeLength) goto inf_leave;
            }
            else if (inflateState->head != Z_NULL)
                inflateState->head->name = Z_NULL;
            inflateState->length = 0;
            inflateState->mode = COMMENT;
        case COMMENT:
            if (inflateState->flags & 0x1000) {
                if (inputBytesAvailable == 0) goto inf_leave;
                copyLength = 0;
                do {
                    decodeLength = (unsigned)(inputNext[copyLength++]);
                    if (inflateState->head != Z_NULL &&
                            inflateState->head->comment != Z_NULL &&
                            inflateState->length < inflateState->head->comm_max)
                        inflateState->head->comment[inflateState->length++] = (Bytef)decodeLength;
                } while (decodeLength && copyLength < inputBytesAvailable);
                if ((inflateState->flags & 0x0200) && (inflateState->wrap & 4))
                    inflateState->check = crc32(inflateState->check, inputNext, copyLength);
                inputBytesAvailable -= copyLength;
                inputNext += copyLength;
                if (decodeLength) goto inf_leave;
            }
            else if (inflateState->head != Z_NULL)
                inflateState->head->comment = Z_NULL;
            inflateState->mode = HCRC;
        case HCRC:
            if (inflateState->flags & 0x0200) {
                NEEDBITS(16);
                if ((inflateState->wrap & 4) && bitBuffer != (inflateState->check & 0xffff)) {
                    stream->msg = (char *)"header crc mismatch";
                    inflateState->mode = BAD;
                    break;
                }
                INITBITS();
            }
            if (inflateState->head != Z_NULL) {
                inflateState->head->hcrc = (int)((inflateState->flags >> 9) & 1);
                inflateState->head->done = 1;
            }
            stream->adler = inflateState->check = crc32(0L, Z_NULL, 0);
            inflateState->mode = TYPE;
            break;
#endif
        case DICTID:
            NEEDBITS(32);
            stream->adler = inflateState->check = ZSWAP32(bitBuffer);
            INITBITS();
            inflateState->mode = DICT;
        case DICT:
            if (inflateState->havedict == 0) {
                RESTORE();
                return Z_NEED_DICT;
            }
            stream->adler = inflateState->check = adler32(0L, Z_NULL, 0);
            inflateState->mode = TYPE;
        case TYPE:
            if (flushMode == Z_BLOCK || flushMode == Z_TREES) goto inf_leave;
        case TYPEDO:
            if (inflateState->last) {
                BYTEBITS();
                inflateState->mode = CHECK;
                break;
            }
            NEEDBITS(3);
            inflateState->last = BITS(1);
            DROPBITS(1);
            switch (BITS(2)) {
            case 0:                             /* stored block */
                Tracev((stderr, "inflate:     stored block%s\n",
                        inflateState->last ? " (last)" : ""));
                inflateState->mode = STORED;
                break;
            case 1:                             /* fixed block */
                fixedtables(inflateState);
                Tracev((stderr, "inflate:     fixed codes block%s\n",
                        inflateState->last ? " (last)" : ""));
                inflateState->mode = LEN_;             /* decode codes */
                if (flushMode == Z_TREES) {
                    DROPBITS(2);
                    goto inf_leave;
                }
                break;
            case 2:                             /* dynamic block */
                Tracev((stderr, "inflate:     dynamic codes block%s\n",
                        inflateState->last ? " (last)" : ""));
                inflateState->mode = TABLE;
                break;
            case 3:
                stream->msg = (char *)"invalid block type";
                inflateState->mode = BAD;
            }
            DROPBITS(2);
            break;
        case STORED:
            BYTEBITS();                         /* go to byte boundary */
            NEEDBITS(32);
            if ((bitBuffer & 0xffff) != ((bitBuffer >> 16) ^ 0xffff)) {
                stream->msg = (char *)"invalid stored block lengths";
                inflateState->mode = BAD;
                break;
            }
            inflateState->length = (unsigned)bitBuffer & 0xffff;
            Tracev((stderr, "inflate:       stored length %u\n",
                    inflateState->length));
            INITBITS();
            inflateState->mode = COPY_;
            if (flushMode == Z_TREES) goto inf_leave;
        case COPY_:
            inflateState->mode = COPY;
        case COPY:
            copyLength = inflateState->length;
            if (copyLength) {
                if (copyLength > inputBytesAvailable) copyLength = inputBytesAvailable;
                if (copyLength > outputBytesAvailable) copyLength = outputBytesAvailable;
                if (copyLength == 0) goto inf_leave;
                zmemcpy(outputNext, inputNext, copyLength);
                inputBytesAvailable -= copyLength;
                inputNext += copyLength;
                outputBytesAvailable -= copyLength;
                outputNext += copyLength;
                inflateState->length -= copyLength;
                break;
            }
            Tracev((stderr, "inflate:       stored end\n"));
            inflateState->mode = TYPE;
            break;
        case TABLE:
            NEEDBITS(14);
            inflateState->nlen = BITS(5) + 257;
            DROPBITS(5);
            inflateState->ndist = BITS(5) + 1;
            DROPBITS(5);
            inflateState->ncode = BITS(4) + 4;
            DROPBITS(4);
#ifndef PKZIP_BUG_WORKAROUND
            if (inflateState->nlen > 286 || inflateState->ndist > 30) {
                stream->msg = (char *)"too many length or distance symbols";
                inflateState->mode = BAD;
                break;
            }
#endif
            Tracev((stderr, "inflate:       table sizes ok\n"));
            inflateState->have = 0;
            inflateState->mode = LENLENS;
        case LENLENS:
            while (inflateState->have < inflateState->ncode) {
                NEEDBITS(3);
                inflateState->lens[order[inflateState->have++]] = (unsigned short)BITS(3);
                DROPBITS(3);
            }
            while (inflateState->have < 19)
                inflateState->lens[order[inflateState->have++]] = 0;
            inflateState->next = inflateState->codes;
            inflateState->lencode = (const code FAR *)(inflateState->next);
            inflateState->lenbits = 7;
            resultCode = inflate_table(CODES, inflateState->lens, 19, &(inflateState->next),
                                &(inflateState->lenbits), inflateState->work);
            if (resultCode) {
                stream->msg = (char *)"invalid code lengths set";
                inflateState->mode = BAD;
                break;
            }
            Tracev((stderr, "inflate:       code lengths ok\n"));
            inflateState->have = 0;
            inflateState->mode = CODELENS;
        case CODELENS:
            while (inflateState->have < inflateState->nlen + inflateState->ndist) {
                for (;;) {
                    currentEntry = inflateState->lencode[BITS(inflateState->lenbits)];
                    if ((unsigned)(currentEntry.bits) <= bitCount) break;
                    PULLBYTE();
                }
                if (currentEntry.val < 16) {
                    DROPBITS(currentEntry.bits);
                    inflateState->lens[inflateState->have++] = currentEntry.val;
                }
                else {
                    if (currentEntry.val == 16) {
                        NEEDBITS(currentEntry.bits + 2);
                        DROPBITS(currentEntry.bits);
                        if (inflateState->have == 0) {
                            stream->msg = (char *)"invalid bit length repeat";
                            inflateState->mode = BAD;
                            break;
                        }
                        decodeLength = inflateState->lens[inflateState->have - 1];
                        copyLength = 3 + BITS(2);
                        DROPBITS(2);
                    }
                    else if (currentEntry.val == 17) {
                        NEEDBITS(currentEntry.bits + 3);
                        DROPBITS(currentEntry.bits);
                        decodeLength = 0;
                        copyLength = 3 + BITS(3);
                        DROPBITS(3);
                    }
                    else {
                        NEEDBITS(currentEntry.bits + 7);
                        DROPBITS(currentEntry.bits);
                        decodeLength = 0;
                        copyLength = 11 + BITS(7);
                        DROPBITS(7);
                    }
                    if (inflateState->have + copyLength > inflateState->nlen + inflateState->ndist) {
                        stream->msg = (char *)"invalid bit length repeat";
                        inflateState->mode = BAD;
                        break;
                    }
                    while (copyLength--)
                        inflateState->lens[inflateState->have++] = (unsigned short)decodeLength;
                }
            }

            /* handle error breaks in while */
            if (inflateState->mode == BAD) break;

            /* check for end-of-block code (better have one) */
            if (inflateState->lens[256] == 0) {
                stream->msg = (char *)"invalid code -- missing end-of-block";
                inflateState->mode = BAD;
                break;
            }

            /* build code tables -- note: do not change the lenbits or distbits
               values here (9 and 6) without reading the comments in inftrees.h
               concerning the ENOUGH constants, which depend on those values */
            inflateState->next = inflateState->codes;
            inflateState->lencode = (const code FAR *)(inflateState->next);
            inflateState->lenbits = 9;
            resultCode = inflate_table(LENS, inflateState->lens, inflateState->nlen, &(inflateState->next),
                                &(inflateState->lenbits), inflateState->work);
            if (resultCode) {
                stream->msg = (char *)"invalid literal/lengths set";
                inflateState->mode = BAD;
                break;
            }
            inflateState->distcode = (const code FAR *)(inflateState->next);
            inflateState->distbits = 6;
            resultCode = inflate_table(DISTS, inflateState->lens + inflateState->nlen, inflateState->ndist,
                            &(inflateState->next), &(inflateState->distbits), inflateState->work);
            if (resultCode) {
                stream->msg = (char *)"invalid distances set";
                inflateState->mode = BAD;
                break;
            }
            Tracev((stderr, "inflate:       codes ok\n"));
            inflateState->mode = LEN_;
            if (flushMode == Z_TREES) goto inf_leave;
        case LEN_:
            inflateState->mode = LEN;
        case LEN:
            if (inputBytesAvailable >= 6 && outputBytesAvailable >= 258) {
                RESTORE();
                inflate_fast(stream, outputByteCount);
                LOAD();
                if (inflateState->mode == TYPE)
                    inflateState->back = -1;
                break;
            }
            inflateState->back = 0;
            for (;;) {
                currentEntry = inflateState->lencode[BITS(inflateState->lenbits)];
                if ((unsigned)(currentEntry.bits) <= bitCount) break;
                PULLBYTE();
            }
            if (currentEntry.op && (currentEntry.op & 0xf0) == 0) {
                parentEntry = currentEntry;
                for (;;) {
                    currentEntry = inflateState->lencode[parentEntry.val +
                            (BITS(parentEntry.bits + parentEntry.op) >> parentEntry.bits)];
                    if ((unsigned)(parentEntry.bits + currentEntry.bits) <= bitCount) break;
                    PULLBYTE();
                }
                DROPBITS(parentEntry.bits);
                inflateState->back += parentEntry.bits;
            }
            DROPBITS(currentEntry.bits);
            inflateState->back += currentEntry.bits;
            inflateState->length = (unsigned)currentEntry.val;
            if ((int)(currentEntry.op) == 0) {
                Tracevv((stderr, currentEntry.val >= 0x20 && currentEntry.val < 0x7f ?
                        "inflate:         literal '%c'\n" :
                        "inflate:         literal 0x%02x\n", currentEntry.val));
                inflateState->mode = LIT;
                break;
            }
            if (currentEntry.op & 32) {
                Tracevv((stderr, "inflate:         end of block\n"));
                inflateState->back = -1;
                inflateState->mode = TYPE;
                break;
            }
            if (currentEntry.op & 64) {
                stream->msg = (char *)"invalid literal/length code";
                inflateState->mode = BAD;
                break;
            }
            inflateState->extra = (unsigned)(currentEntry.op) & 15;
            inflateState->mode = LENEXT;
        case LENEXT:
            if (inflateState->extra) {
                NEEDBITS(inflateState->extra);
                inflateState->length += BITS(inflateState->extra);
                DROPBITS(inflateState->extra);
                inflateState->back += inflateState->extra;
            }
            Tracevv((stderr, "inflate:         length %u\n", inflateState->length));
            inflateState->was = inflateState->length;
            inflateState->mode = DIST;
        case DIST:
            for (;;) {
                currentEntry = inflateState->distcode[BITS(inflateState->distbits)];
                if ((unsigned)(currentEntry.bits) <= bitCount) break;
                PULLBYTE();
            }
            if ((currentEntry.op & 0xf0) == 0) {
                parentEntry = currentEntry;
                for (;;) {
                    currentEntry = inflateState->distcode[parentEntry.val +
                            (BITS(parentEntry.bits + parentEntry.op) >> parentEntry.bits)];
                    if ((unsigned)(parentEntry.bits + currentEntry.bits) <= bitCount) break;
                    PULLBYTE();
                }
                DROPBITS(parentEntry.bits);
                inflateState->back += parentEntry.bits;
            }
            DROPBITS(currentEntry.bits);
            inflateState->back += currentEntry.bits;
            if (currentEntry.op & 64) {
                stream->msg = (char *)"invalid distance code";
                inflateState->mode = BAD;
                break;
            }
            inflateState->offset = (unsigned)currentEntry.val;
            inflateState->extra = (unsigned)(currentEntry.op) & 15;
            inflateState->mode = DISTEXT;
        case DISTEXT:
            if (inflateState->extra) {
                NEEDBITS(inflateState->extra);
                inflateState->offset += BITS(inflateState->extra);
                DROPBITS(inflateState->extra);
                inflateState->back += inflateState->extra;
            }
#ifdef INFLATE_STRICT
            if (inflateState->offset > inflateState->dmax) {
                stream->msg = (char *)"invalid distance too far back";
                inflateState->mode = BAD;
                break;
            }
#endif
            Tracevv((stderr, "inflate:         distance %u\n", inflateState->offset));
            inflateState->mode = MATCH;
        case MATCH:
            if (outputBytesAvailable == 0) goto inf_leave;
            copyLength = outputByteCount - outputBytesAvailable;
            if (inflateState->offset > copyLength) {         /* copy from window */
                copyLength = inflateState->offset - copyLength;
                if (copyLength > inflateState->whave) {
                    if (inflateState->sane) {
                        stream->msg = (char *)"invalid distance too far back";
                        inflateState->mode = BAD;
                        break;
                    }
#ifdef INFLATE_ALLOW_INVALID_DISTANCE_TOOFAR_ARRR
                    Trace((stderr, "inflate.c too far\n"));
                    copyLength -= inflateState->whave;
                    if (copyLength > inflateState->length) copyLength = inflateState->length;
                    if (copyLength > outputBytesAvailable) copyLength = outputBytesAvailable;
                    outputBytesAvailable -= copyLength;
                    inflateState->length -= copyLength;
                    do {
                        *outputNext++ = 0;
                    } while (--copyLength);
                    if (inflateState->length == 0) inflateState->mode = LEN;
                    break;
#endif
                }
                if (copyLength > inflateState->wnext) {
                    copyLength -= inflateState->wnext;
                    matchSource = inflateState->window + (inflateState->wsize - copyLength);
                }
                else
                    matchSource = inflateState->window + (inflateState->wnext - copyLength);
                if (copyLength > inflateState->length) copyLength = inflateState->length;
            }
            else {                              /* copy from output */
                matchSource = outputNext - inflateState->offset;
                copyLength = inflateState->length;
            }
            if (copyLength > outputBytesAvailable) copyLength = outputBytesAvailable;
            outputBytesAvailable -= copyLength;
            inflateState->length -= copyLength;
            do {
                *outputNext++ = *matchSource++;
            } while (--copyLength);
            if (inflateState->length == 0) inflateState->mode = LEN;
            break;
        case LIT:
            if (outputBytesAvailable == 0) goto inf_leave;
            *outputNext++ = (unsigned char)(inflateState->length);
            outputBytesAvailable--;
            inflateState->mode = LEN;
            break;
        case CHECK:
            if (inflateState->wrap) {
                NEEDBITS(32);
                outputByteCount -= outputBytesAvailable;
                stream->total_out += outputByteCount;
                inflateState->total += outputByteCount;
                if ((inflateState->wrap & 4) && outputByteCount)
                    stream->adler = inflateState->check =
                        UPDATE(inflateState->check, outputNext - outputByteCount, outputByteCount);
                outputByteCount = outputBytesAvailable;
                if ((inflateState->wrap & 4) && (
#ifdef GUNZIP
                     inflateState->flags ? bitBuffer :
#endif
                     ZSWAP32(bitBuffer)) != inflateState->check) {
                    stream->msg = (char *)"incorrect data check";
                    inflateState->mode = BAD;
                    break;
                }
                INITBITS();
                Tracev((stderr, "inflate:   check matches trailer\n"));
            }
#ifdef GUNZIP
            inflateState->mode = LENGTH;
        case LENGTH:
            if (inflateState->wrap && inflateState->flags) {
                NEEDBITS(32);
                if (bitBuffer != (inflateState->total & 0xffffffffUL)) {
                    stream->msg = (char *)"incorrect length check";
                    inflateState->mode = BAD;
                    break;
                }
                INITBITS();
                Tracev((stderr, "inflate:   length matches trailer\n"));
            }
#endif
            inflateState->mode = DONE;
        case DONE:
            resultCode = Z_STREAM_END;
            goto inf_leave;
        case BAD:
            resultCode = Z_DATA_ERROR;
            goto inf_leave;
        case MEM:
            return Z_MEM_ERROR;
        case SYNC:
        default:
            return Z_STREAM_ERROR;
        }

    /*
       Return from inflate(), updating the total counts and the check value.
       If there was no progress during the inflate() call, return a buffer
       error.  Call updatewindow() to create and/or update the window state.
       Note: a memory error from inflate() is non-recoverable.
     */
  inf_leave:
    RESTORE();
    if (inflateState->wsize || (outputByteCount != stream->avail_out && inflateState->mode < BAD &&
            (inflateState->mode < CHECK || flushMode != Z_FINISH)))
        if (updatewindow(stream, stream->next_out, outputByteCount - stream->avail_out)) {
            inflateState->mode = MEM;
            return Z_MEM_ERROR;
        }
    inputByteCount -= stream->avail_in;
    outputByteCount -= stream->avail_out;
    stream->total_in += inputByteCount;
    stream->total_out += outputByteCount;
    inflateState->total += outputByteCount;
    if ((inflateState->wrap & 4) && outputByteCount)
        stream->adler = inflateState->check =
            UPDATE(inflateState->check, stream->next_out - outputByteCount, outputByteCount);
    stream->data_type = (int)inflateState->bits + (inflateState->last ? 64 : 0) +
                      (inflateState->mode == TYPE ? 128 : 0) +
                      (inflateState->mode == LEN_ || inflateState->mode == COPY_ ? 256 : 0);
    if (((inputByteCount == 0 && outputByteCount == 0) || flushMode == Z_FINISH) && resultCode == Z_OK)
        resultCode = Z_BUF_ERROR;
    return resultCode;
}

int ZEXPORT inflateEnd(stream)
z_streamp stream;
{
    struct inflate_state FAR *inflateState;
    if (inflateStateCheck(stream))
        return Z_STREAM_ERROR;
    inflateState = (struct inflate_state FAR *)stream->state;
    if (inflateState->window != Z_NULL) ZFREE(stream, inflateState->window);
    ZFREE(stream, stream->state);
    stream->state = Z_NULL;
    Tracev((stderr, "inflate: end\n"));
    return Z_OK;
}

int ZEXPORT inflateGetDictionary(stream, dictionary, dictionaryLength)
z_streamp stream;
Bytef *dictionary;
uInt *dictionaryLength;
{
    struct inflate_state FAR *inflateState;

    /* check state */
    if (inflateStateCheck(stream)) return Z_STREAM_ERROR;
    inflateState = (struct inflate_state FAR *)stream->state;

    /* copy dictionary */
    if (inflateState->whave && dictionary != Z_NULL) {
        zmemcpy(dictionary, inflateState->window + inflateState->wnext,
                inflateState->whave - inflateState->wnext);
        zmemcpy(dictionary + inflateState->whave - inflateState->wnext,
                inflateState->window, inflateState->wnext);
    }
    if (dictionaryLength != Z_NULL)
        *dictionaryLength = inflateState->whave;
    return Z_OK;
}

int ZEXPORT inflateSetDictionary(stream, dictionary, dictionaryLength)
z_streamp stream;
const Bytef *dictionary;
uInt dictionaryLength;
{
    struct inflate_state FAR *inflateState;
    unsigned long dictionaryChecksum;
    int resultCode;

    /* check state */
    if (inflateStateCheck(stream)) return Z_STREAM_ERROR;
    inflateState = (struct inflate_state FAR *)stream->state;
    if (inflateState->wrap != 0 && inflateState->mode != DICT)
        return Z_STREAM_ERROR;

    /* check for correct dictionary identifier */
    if (inflateState->mode == DICT) {
        dictionaryChecksum = adler32(0L, Z_NULL, 0);
        dictionaryChecksum = adler32(dictionaryChecksum, dictionary, dictionaryLength);
        if (dictionaryChecksum != inflateState->check)
            return Z_DATA_ERROR;
    }

    /* copy dictionary to window using updatewindow(), which will amend the
       existing dictionary if appropriate */
    resultCode = updatewindow(stream, dictionary + dictionaryLength, dictionaryLength);
    if (resultCode) {
        inflateState->mode = MEM;
        return Z_MEM_ERROR;
    }
    inflateState->havedict = 1;
    Tracev((stderr, "inflate:   dictionary set\n"));
    return Z_OK;
}

int ZEXPORT inflateGetHeader(stream, gzipHeader)
z_streamp stream;
gz_headerp gzipHeader;
{
    struct inflate_state FAR *inflateState;

    /* check state */
    if (inflateStateCheck(stream)) return Z_STREAM_ERROR;
    inflateState = (struct inflate_state FAR *)stream->state;
    if ((inflateState->wrap & 2) == 0) return Z_STREAM_ERROR;

    /* save header structure */
    inflateState->head = gzipHeader;
    gzipHeader->done = 0;
    return Z_OK;
}

/*
   Search buf[0..len-1] for the pattern: 0, 0, 0xff, 0xff.  Return when found
   or when out of input.  When called, *have is the number of pattern bytes
   found in order so far, in 0..3.  On return *have is updated to the new
   state.  If on return *have equals four, then the pattern was found and the
   return value is how many bytes were read including the last byte of the
   pattern.  If *have is less than four, then the pattern has not been found
   yet and the return value is len.  In the latter case, syncsearch() can be
   called again with more data and the *have state.  *have is initialized to
   zero for the first call.
 */
local unsigned syncsearch(matchedByteCount, inputBytes, inputLength)
unsigned FAR *matchedByteCount;
const unsigned char FAR *inputBytes;
unsigned inputLength;
{
    unsigned matchedBytes;
    unsigned iInputByte;

    matchedBytes = *matchedByteCount;
    iInputByte = 0;
    while (iInputByte < inputLength && matchedBytes < 4) {
        if ((int)(inputBytes[iInputByte]) == (matchedBytes < 2 ? 0 : 0xff))
            matchedBytes++;
        else if (inputBytes[iInputByte])
            matchedBytes = 0;
        else
            matchedBytes = 4 - matchedBytes;
        iInputByte++;
    }
    *matchedByteCount = matchedBytes;
    return iInputByte;
}

int ZEXPORT inflateSync(stream)
z_streamp stream;
{
    unsigned bytesSearched;               /* number of bytes to look at or looked at */
    unsigned long savedTotalInput, savedTotalOutput;      /* temporary to save total_in and total_out */
    unsigned char bufferedBytes[4];       /* to restore bit buffer to byte string */
    struct inflate_state FAR *inflateState;

    /* check parameters */
    if (inflateStateCheck(stream)) return Z_STREAM_ERROR;
    inflateState = (struct inflate_state FAR *)stream->state;
    if (stream->avail_in == 0 && inflateState->bits < 8) return Z_BUF_ERROR;

    /* if first time, start search in bit buffer */
    if (inflateState->mode != SYNC) {
        inflateState->mode = SYNC;
        inflateState->hold <<= inflateState->bits & 7;
        inflateState->bits -= inflateState->bits & 7;
        bytesSearched = 0;
        while (inflateState->bits >= 8) {
            bufferedBytes[bytesSearched++] = (unsigned char)(inflateState->hold);
            inflateState->hold >>= 8;
            inflateState->bits -= 8;
        }
        inflateState->have = 0;
        syncsearch(&(inflateState->have), bufferedBytes, bytesSearched);
    }

    /* search available input */
    bytesSearched = syncsearch(&(inflateState->have), stream->next_in, stream->avail_in);
    stream->avail_in -= bytesSearched;
    stream->next_in += bytesSearched;
    stream->total_in += bytesSearched;

    /* return no joy or set up to restart inflate() on a new block */
    if (inflateState->have != 4) return Z_DATA_ERROR;
    savedTotalInput = stream->total_in;  savedTotalOutput = stream->total_out;
    inflateReset(stream);
    stream->total_in = savedTotalInput;  stream->total_out = savedTotalOutput;
    inflateState->mode = TYPE;
    return Z_OK;
}

/*
   Returns true if inflate is currently at the end of a block generated by
   Z_SYNC_FLUSH or Z_FULL_FLUSH. This function is used by one PPP
   implementation to provide an additional safety check. PPP uses
   Z_SYNC_FLUSH but removes the length bytes of the resulting empty stored
   block. When decompressing, PPP checks that at the end of input packet,
   inflate is waiting for these length bytes.
 */
int ZEXPORT inflateSyncPoint(stream)
z_streamp stream;
{
    struct inflate_state FAR *inflateState;

    if (inflateStateCheck(stream)) return Z_STREAM_ERROR;
    inflateState = (struct inflate_state FAR *)stream->state;
    return inflateState->mode == STORED && inflateState->bits == 0;
}

int ZEXPORT inflateCopy(destinationStream, sourceStream)
z_streamp destinationStream;
z_streamp sourceStream;
{
    struct inflate_state FAR *inflateState;
    struct inflate_state FAR *destinationState;
    unsigned char FAR *window;
    unsigned windowSize;

    /* check input */
    if (inflateStateCheck(sourceStream) || destinationStream == Z_NULL)
        return Z_STREAM_ERROR;
    inflateState = (struct inflate_state FAR *)sourceStream->state;

    /* allocate space */
    destinationState = (struct inflate_state FAR *)
           ZALLOC(sourceStream, 1, sizeof(struct inflate_state));
    if (destinationState == Z_NULL) return Z_MEM_ERROR;
    window = Z_NULL;
    if (inflateState->window != Z_NULL) {
        window = (unsigned char FAR *)
                 ZALLOC(sourceStream, 1U << inflateState->wbits, sizeof(unsigned char));
        if (window == Z_NULL) {
            ZFREE(sourceStream, destinationState);
            return Z_MEM_ERROR;
        }
    }

    /* copy state */
    zmemcpy((voidpf)destinationStream, (voidpf)sourceStream, sizeof(z_stream));
    zmemcpy((voidpf)destinationState, (voidpf)inflateState, sizeof(struct inflate_state));
    destinationState->strm = destinationStream;
    if (inflateState->lencode >= inflateState->codes &&
        inflateState->lencode <= inflateState->codes + ENOUGH - 1) {
        destinationState->lencode = destinationState->codes + (inflateState->lencode - inflateState->codes);
        destinationState->distcode = destinationState->codes + (inflateState->distcode - inflateState->codes);
    }
    destinationState->next = destinationState->codes + (inflateState->next - inflateState->codes);
    if (window != Z_NULL) {
        windowSize = 1U << inflateState->wbits;
        zmemcpy(window, inflateState->window, windowSize);
    }
    destinationState->window = window;
    destinationStream->state = (struct internal_state FAR *)destinationState;
    return Z_OK;
}

int ZEXPORT inflateUndermine(stream, subvert)
z_streamp stream;
int subvert;
{
    struct inflate_state FAR *inflateState;

    if (inflateStateCheck(stream)) return Z_STREAM_ERROR;
    inflateState = (struct inflate_state FAR *)stream->state;
#ifdef INFLATE_ALLOW_INVALID_DISTANCE_TOOFAR_ARRR
    inflateState->sane = !subvert;
    return Z_OK;
#else
    (void)subvert;
    inflateState->sane = 1;
    return Z_DATA_ERROR;
#endif
}

int ZEXPORT inflateValidate(stream, check)
z_streamp stream;
int check;
{
    struct inflate_state FAR *inflateState;

    if (inflateStateCheck(stream)) return Z_STREAM_ERROR;
    inflateState = (struct inflate_state FAR *)stream->state;
    if (check)
        inflateState->wrap |= 4;
    else
        inflateState->wrap &= ~4;
    return Z_OK;
}

long ZEXPORT inflateMark(stream)
z_streamp stream;
{
    struct inflate_state FAR *inflateState;

    if (inflateStateCheck(stream))
        return -(1L << 16);
    inflateState = (struct inflate_state FAR *)stream->state;
    return (long)(((unsigned long)((long)inflateState->back)) << 16) +
        (inflateState->mode == COPY ? inflateState->length :
            (inflateState->mode == MATCH ? inflateState->was - inflateState->length : 0));
}

unsigned long ZEXPORT inflateCodesUsed(stream)
z_streamp stream;
{
    struct inflate_state FAR *inflateState;
    if (inflateStateCheck(stream)) return (unsigned long)-1;
    inflateState = (struct inflate_state FAR *)stream->state;
    return (unsigned long)(inflateState->next - inflateState->codes);
}
