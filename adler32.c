/* adler32.c -- compute the Adler-32 checksum of a data stream
 * Copyright (C) 1995-2011, 2016 Mark Adler
 * For conditions of distribution and use, see copyright notice in zlib.h
 */

/* @(#) $Id$ */

#include "zutil.h"

static uLong adler32_combine_ OF((uLong firstChecksum, uLong secondChecksum, z_off64_t secondLength));

#define BASE 65521U     /* largest prime smaller than 65536 */
#define NMAX 5552
/* NMAX is the largest blocksRemaining such that 255n(blocksRemaining+1)/2 + (blocksRemaining+1)(BASE-1) <= 2^32-1 */

#define DO1(inputBytes,iByte)  {adler += (inputBytes)[iByte]; weightedByteSum += adler;}
#define DO2(inputBytes,iByte)  DO1(inputBytes,iByte); DO1(inputBytes,iByte+1);
#define DO4(inputBytes,iByte)  DO2(inputBytes,iByte); DO2(inputBytes,iByte+2);
#define DO8(inputBytes,iByte)  DO4(inputBytes,iByte); DO4(inputBytes,iByte+4);
#define DO16(inputBytes)   DO8(inputBytes,0); DO8(inputBytes,8);

/* use NO_DIVIDE if your processor does not do division in hardware --
   try it both ways to see which is faster */
#ifdef NO_DIVIDE
/* note that this assumes BASE is 65521, where 65536 % 65521 == 15
   (thank you to John Reiser for pointing this out) */
#  define CHOP(a) \
    do { \
        unsigned long upperBits = a >> 16; \
        a &= 0xffffUL; \
        a += (upperBits << 4) - upperBits; \
    } while (0)
#  define MOD28(a) \
    do { \
        CHOP(a); \
        if (a >= BASE) a -= BASE; \
    } while (0)
#  define MOD(a) \
    do { \
        CHOP(a); \
        MOD28(a); \
    } while (0)
#  define MOD63(a) \
    do { /* this assumes a is not negative */ \
        z_off64_t upperBits = a >> 32; \
        a &= 0xffffffffL; \
        a += (upperBits << 8) - (upperBits << 5) + upperBits; \
        upperBits = a >> 16; \
        a &= 0xffffL; \
        a += (upperBits << 4) - upperBits; \
        upperBits = a >> 16; \
        a &= 0xffffL; \
        a += (upperBits << 4) - upperBits; \
        if (a >= BASE) a -= BASE; \
    } while (0)
#else
#  define MOD(a) a %= BASE
#  define MOD28(a) a %= BASE
#  define MOD63(a) a %= BASE
#endif

/* ========================================================================= */
uLong ZEXPORT adler32_z
    ( uLong        adler
    , const Bytef  *inputBytes
    , z_size_t     inputLength
    )
{
    unsigned long weightedByteSum;
    uint32_t blocksRemaining;

    /* split Adler-32 into component sums */
    weightedByteSum = (adler >> 16) & 0xffff;
    adler &= 0xffff;

    /* in case user likes doing a byte at a time, keep it fast */
    if (inputLength == 1) {
        adler += inputBytes[0];
        if (adler >= BASE)
            adler -= BASE;
        weightedByteSum += adler;
        if (weightedByteSum >= BASE)
            weightedByteSum -= BASE;
        return adler | (weightedByteSum << 16);
    }

    /* initial Adler-32 value (deferred check for inputLength == 1 speed) */
    if (inputBytes == Z_NULL)
        return 1L;

    /* in case short lengths are provided, keep it somewhat fast */
    if (inputLength < 16) {
        while (inputLength--) {
            adler += *inputBytes++;
            weightedByteSum += adler;
        }
        if (adler >= BASE)
            adler -= BASE;
        MOD28(weightedByteSum);            /* only added so many BASE's */
        return adler | (weightedByteSum << 16);
    }

    /* do length NMAX blocks -- requires just one modulo operation */
    while (inputLength >= NMAX) {
        inputLength -= NMAX;
        blocksRemaining = NMAX / 16;          /* NMAX is divisible by 16 */
        do {
            DO16(inputBytes);          /* 16 sums unrolled */
            inputBytes += 16;
        } while (--blocksRemaining);
        MOD(adler);
        MOD(weightedByteSum);
    }

    /* do remaining bytes (less than NMAX, still just one modulo) */
    if (inputLength) {                  /* avoid modulos if none remaining */
        while (inputLength >= 16) {
            inputLength -= 16;
            DO16(inputBytes);
            inputBytes += 16;
        }
        while (inputLength--) {
            adler += *inputBytes++;
            weightedByteSum += adler;
        }
        MOD(adler);
        MOD(weightedByteSum);
    }

    /* return recombined sums */
    return adler | (weightedByteSum << 16);
}

/* ========================================================================= */
uLong ZEXPORT adler32
    ( uLong        adler
    , const Bytef  *inputBytes
    , uInt         inputLength
    )
{
    return adler32_z(adler, inputBytes, inputLength);
}

/* ========================================================================= */
static uLong adler32_combine_
    ( uLong      firstChecksum
    , uLong      secondChecksum
    , z_off64_t  secondLength
    )
{
    unsigned long byteSum;
    unsigned long weightedByteSum;
    uint32_t lengthRemainder;

    /* for negative inputLength, return invalid adler32 as a clue for debugging */
    if (secondLength < 0)
        return 0xffffffffUL;

    /* the derivation of this formula is left as an exercise for the reader */
    MOD63(secondLength);                /* assumes len2 >= 0 */
    lengthRemainder = (uint32_t)secondLength;
    byteSum = firstChecksum & 0xffff;
    weightedByteSum = lengthRemainder * byteSum;
    MOD(weightedByteSum);
    byteSum += (secondChecksum & 0xffff) + BASE - 1;
    weightedByteSum += ((firstChecksum >> 16) & 0xffff) + ((secondChecksum >> 16) & 0xffff) + BASE - lengthRemainder;
    if (byteSum >= BASE) byteSum -= BASE;
    if (byteSum >= BASE) byteSum -= BASE;
    if (weightedByteSum >= ((unsigned long)BASE << 1)) weightedByteSum -= ((unsigned long)BASE << 1);
    if (weightedByteSum >= BASE) weightedByteSum -= BASE;
    return byteSum | (weightedByteSum << 16);
}

/* ========================================================================= */
uLong ZEXPORT adler32_combine
    ( uLong    firstChecksum
    , uLong    secondChecksum
    , z_off_t  secondLength
    )
{
    return adler32_combine_(firstChecksum, secondChecksum, secondLength);
}

uLong ZEXPORT adler32_combine64
    ( uLong      firstChecksum
    , uLong      secondChecksum
    , z_off64_t  secondLength
    )
{
    return adler32_combine_(firstChecksum, secondChecksum, secondLength);
}
