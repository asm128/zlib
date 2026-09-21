/* inffast.c -- fast decoding
 * Copyright (C) 1995-2017 Mark Adler
 * For conditions of distribution and use, see copyright notice in zlib.h
 */

#include "zutil.h"
#include "inftrees.h"
#include "inflate.h"
#include "inffast.h"

#ifdef ASMINF
#  pragma message("Assembler code may have bugs -- use at your own risk")
#else

/*
   Decode literal, length, and distance codes and write out the resulting
   literal and match bytes until either not enough input or output is
   available, an end-of-block is encountered, or a data error is encountered.
   When large enough input and output buffers are supplied to inflate(), for
   example, a 16K input buffer and a 64K output buffer, more than 95% of the
   inflate execution time is spent in this routine.

   Entry assumptions:

        inflateState->mode == LEN
        stream->avail_in >= 6
        stream->avail_out >= 258
        start >= stream->avail_out
        inflateState->bits < 8

   On return, inflateState->mode is one of:

        LEN -- ran out of enough output space or enough available input
        TYPE -- reached end of block code, inflate() to interpret next block
        BAD -- error in block data

   Notes:

    - The maximum input bits used by a length/distance pair is 15 bits for the
      length code, 5 bits for the length extra, 15 bits for the distance code,
      and 13 bits for the distance extra.  This totals 48 bits, or six bytes.
      Therefore if stream->avail_in >= 6, then there is enough input to avoid
      checking for available input while decoding.

    - The maximum bytes that a single length/distance pair can output is 258
      bytes, which is the maximum length that can be coded.  inflate_fast()
      requires stream->avail_out >= 258 for each loop to avoid checking for
      output space.
 */
void ZLIB_INTERNAL inflate_fast(stream, initialOutputAvailable)
z_streamp stream;
unsigned initialOutputAvailable;         /* inflate()'s starting value for stream->avail_out */
{
    struct inflate_state FAR *inflateState;
    z_const unsigned char FAR *inputNext;      /* local stream->next_in */
    z_const unsigned char FAR *inputFastLimit;    /* have enough input while in < last */
    unsigned char FAR *outputNext;     /* local stream->next_out */
    unsigned char FAR *outputStart;     /* inflate()'s initial stream->next_out */
    unsigned char FAR *outputFastLimit;     /* while out < end, enough space available */
#ifdef INFLATE_STRICT
    unsigned maximumDistance;              /* maximum distance from zlib header */
#endif
    unsigned windowSize;             /* window size or zero if not using window */
    unsigned windowBytesAvailable;             /* valid bytes in the window */
    unsigned windowWriteIndex;             /* window write index */
    unsigned char FAR *window;  /* allocated sliding window, if windowSize != 0 */
    unsigned long bitBuffer;         /* local stream->hold */
    unsigned bitCount;              /* local stream->bits */
    code const FAR *lengthTable;      /* local stream->lencode */
    code const FAR *distanceTable;      /* local stream->distcode */
    unsigned lengthTableMask;             /* mask for first level of length codes */
    unsigned distanceTableMask;             /* mask for first level of distance codes */
    code currentEntry;                  /* retrieved table entry */
    unsigned decodeValue;                /* code bits, operation, extra bits, or */
                                /*  window position, window bytes to copy */
    unsigned matchLength;               /* match length, unused bytes */
    unsigned matchDistance;              /* match distance */
    unsigned char FAR *matchSource;    /* where to copy match from */

    /* copy state to local variables */
    inflateState = (struct inflate_state FAR *)stream->state;
    inputNext = stream->next_in;
    inputFastLimit = inputNext + (stream->avail_in - 5);
    outputNext = stream->next_out;
    outputStart = outputNext - (initialOutputAvailable - stream->avail_out);
    outputFastLimit = outputNext + (stream->avail_out - 257);
#ifdef INFLATE_STRICT
    maximumDistance = inflateState->dmax;
#endif
    windowSize = inflateState->wsize;
    windowBytesAvailable = inflateState->whave;
    windowWriteIndex = inflateState->wnext;
    window = inflateState->window;
    bitBuffer = inflateState->hold;
    bitCount = inflateState->bits;
    lengthTable = inflateState->lencode;
    distanceTable = inflateState->distcode;
    lengthTableMask = (1U << inflateState->lenbits) - 1;
    distanceTableMask = (1U << inflateState->distbits) - 1;

    /* decode literals and length/distances until end-of-block or not enough
       input data or output space */
    do {
        if (bitCount < 15) {
            bitBuffer += (unsigned long)(*inputNext++) << bitCount;
            bitCount += 8;
            bitBuffer += (unsigned long)(*inputNext++) << bitCount;
            bitCount += 8;
        }
        currentEntry = lengthTable[bitBuffer & lengthTableMask];
      dolen:
        decodeValue = (unsigned)(currentEntry.bits);
        bitBuffer >>= decodeValue;
        bitCount -= decodeValue;
        decodeValue = (unsigned)(currentEntry.op);
        if (decodeValue == 0) {                          /* literal */
            Tracevv((stderr, currentEntry.val >= 0x20 && currentEntry.val < 0x7f ?
                    "inflate:         literal '%c'\n" :
                    "inflate:         literal 0x%02x\n", currentEntry.val));
            *outputNext++ = (unsigned char)(currentEntry.val);
        }
        else if (decodeValue & 16) {                     /* length base */
            matchLength = (unsigned)(currentEntry.val);
            decodeValue &= 15;                           /* number of extra bits */
            if (decodeValue) {
                if (bitCount < decodeValue) {
                    bitBuffer += (unsigned long)(*inputNext++) << bitCount;
                    bitCount += 8;
                }
                matchLength += (unsigned)bitBuffer & ((1U << decodeValue) - 1);
                bitBuffer >>= decodeValue;
                bitCount -= decodeValue;
            }
            Tracevv((stderr, "inflate:         length %u\n", matchLength));
            if (bitCount < 15) {
                bitBuffer += (unsigned long)(*inputNext++) << bitCount;
                bitCount += 8;
                bitBuffer += (unsigned long)(*inputNext++) << bitCount;
                bitCount += 8;
            }
            currentEntry = distanceTable[bitBuffer & distanceTableMask];
          dodist:
            decodeValue = (unsigned)(currentEntry.bits);
            bitBuffer >>= decodeValue;
            bitCount -= decodeValue;
            decodeValue = (unsigned)(currentEntry.op);
            if (decodeValue & 16) {                      /* distance base */
                matchDistance = (unsigned)(currentEntry.val);
                decodeValue &= 15;                       /* number of extra bits */
                if (bitCount < decodeValue) {
                    bitBuffer += (unsigned long)(*inputNext++) << bitCount;
                    bitCount += 8;
                    if (bitCount < decodeValue) {
                        bitBuffer += (unsigned long)(*inputNext++) << bitCount;
                        bitCount += 8;
                    }
                }
                matchDistance += (unsigned)bitBuffer & ((1U << decodeValue) - 1);
#ifdef INFLATE_STRICT
                if (matchDistance > maximumDistance) {
                    stream->msg = (char *)"invalid distance too far back";
                    inflateState->mode = BAD;
                    break;
                }
#endif
                bitBuffer >>= decodeValue;
                bitCount -= decodeValue;
                Tracevv((stderr, "inflate:         distance %u\n", matchDistance));
                decodeValue = (unsigned)(outputNext - outputStart);     /* max distance in output */
                if (matchDistance > decodeValue) {                /* see if copy from window */
                    decodeValue = matchDistance - decodeValue;             /* distance back in window */
                    if (decodeValue > windowBytesAvailable) {
                        if (inflateState->sane) {
                            stream->msg =
                                (char *)"invalid distance too far back";
                            inflateState->mode = BAD;
                            break;
                        }
#ifdef INFLATE_ALLOW_INVALID_DISTANCE_TOOFAR_ARRR
                        if (matchLength <= decodeValue - windowBytesAvailable) {
                            do {
                                *outputNext++ = 0;
                            } while (--matchLength);
                            continue;
                        }
                        matchLength -= decodeValue - windowBytesAvailable;
                        do {
                            *outputNext++ = 0;
                        } while (--decodeValue > windowBytesAvailable);
                        if (decodeValue == 0) {
                            matchSource = outputNext - matchDistance;
                            do {
                                *outputNext++ = *matchSource++;
                            } while (--matchLength);
                            continue;
                        }
#endif
                    }
                    matchSource = window;
                    if (windowWriteIndex == 0) {           /* very common case */
                        matchSource += windowSize - decodeValue;
                        if (decodeValue < matchLength) {         /* some from window */
                            matchLength -= decodeValue;
                            do {
                                *outputNext++ = *matchSource++;
                            } while (--decodeValue);
                            matchSource = outputNext - matchDistance;  /* rest from output */
                        }
                    }
                    else if (windowWriteIndex < decodeValue) {      /* wrap around window */
                        matchSource += windowSize + windowWriteIndex - decodeValue;
                        decodeValue -= windowWriteIndex;
                        if (decodeValue < matchLength) {         /* some from end of window */
                            matchLength -= decodeValue;
                            do {
                                *outputNext++ = *matchSource++;
                            } while (--decodeValue);
                            matchSource = window;
                            if (windowWriteIndex < matchLength) {  /* some from start of window */
                                decodeValue = windowWriteIndex;
                                matchLength -= decodeValue;
                                do {
                                    *outputNext++ = *matchSource++;
                                } while (--decodeValue);
                                matchSource = outputNext - matchDistance;      /* rest from output */
                            }
                        }
                    }
                    else {                      /* contiguous in window */
                        matchSource += windowWriteIndex - decodeValue;
                        if (decodeValue < matchLength) {         /* some from window */
                            matchLength -= decodeValue;
                            do {
                                *outputNext++ = *matchSource++;
                            } while (--decodeValue);
                            matchSource = outputNext - matchDistance;  /* rest from output */
                        }
                    }
                    while (matchLength > 2) {
                        *outputNext++ = *matchSource++;
                        *outputNext++ = *matchSource++;
                        *outputNext++ = *matchSource++;
                        matchLength -= 3;
                    }
                    if (matchLength) {
                        *outputNext++ = *matchSource++;
                        if (matchLength > 1)
                            *outputNext++ = *matchSource++;
                    }
                }
                else {
                    matchSource = outputNext - matchDistance;          /* copy direct from output */
                    do {                        /* minimum length is three */
                        *outputNext++ = *matchSource++;
                        *outputNext++ = *matchSource++;
                        *outputNext++ = *matchSource++;
                        matchLength -= 3;
                    } while (matchLength > 2);
                    if (matchLength) {
                        *outputNext++ = *matchSource++;
                        if (matchLength > 1)
                            *outputNext++ = *matchSource++;
                    }
                }
            }
            else if ((decodeValue & 64) == 0) {          /* 2nd level distance code */
                currentEntry = distanceTable[currentEntry.val + (bitBuffer & ((1U << decodeValue) - 1))];
                goto dodist;
            }
            else {
                stream->msg = (char *)"invalid distance code";
                inflateState->mode = BAD;
                break;
            }
        }
        else if ((decodeValue & 64) == 0) {              /* 2nd level length code */
            currentEntry = lengthTable[currentEntry.val + (bitBuffer & ((1U << decodeValue) - 1))];
            goto dolen;
        }
        else if (decodeValue & 32) {                     /* end-of-block */
            Tracevv((stderr, "inflate:         end of block\n"));
            inflateState->mode = TYPE;
            break;
        }
        else {
            stream->msg = (char *)"invalid literal/length code";
            inflateState->mode = BAD;
            break;
        }
    } while (inputNext < inputFastLimit && outputNext < outputFastLimit);

    /* return unused bytes (on entry, bits < 8, so in won't go too far back) */
    matchLength = bitCount >> 3;
    inputNext -= matchLength;
    bitCount -= matchLength << 3;
    bitBuffer &= (1U << bitCount) - 1;

    /* update state and return */
    stream->next_in = inputNext;
    stream->next_out = outputNext;
    stream->avail_in = (unsigned)(inputNext < inputFastLimit ? 5 + (inputFastLimit - inputNext) : 5 - (inputNext - inputFastLimit));
    stream->avail_out = (unsigned)(outputNext < outputFastLimit ?
                                 257 + (outputFastLimit - outputNext) : 257 - (outputNext - outputFastLimit));
    inflateState->hold = bitBuffer;
    inflateState->bits = bitCount;
    return;
}

/*
   inflate_fast() speedups that turned out slower (on a PowerPC G3 750CXe):
   - Using bit fields for code structure
   - Different op definition to avoid & for extra bits (do & for table bits)
   - Three separate decoding do-loops for direct, window, and windowWriteIndex == 0
   - Special case for distance > 1 copies to do overlapped load and store copy
   - Explicit branch predictions (based on measured branch probabilities)
   - Deferring match copy and interspersed it with decoding subsequent codes
   - Swapping literal/length else
   - Swapping window/direct else
   - Larger unrolled copy loops (three is about right)
   - Moving matchLength -= 3 statement into middle of loop
 */

#endif /* !ASMINF */
