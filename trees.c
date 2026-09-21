/* trees.c -- output deflated data using Huffman coding
 * Copyright (C) 1995-2017 Jean-loup Gailly
 * detect_data_type() function provided freely by Cosmin Truta, 2006
 * For conditions of distribution and use, see copyright notice in zlib.h
 */

/*
 *  ALGORITHM
 *
 *      The "deflation" process uses several Huffman trees. The more
 *      common source values are represented by shorter bit sequences.
 *
 *      Each code tree is stored in a compressed form which is itself
 * a Huffman encoding of the lengths of all the code strings (in
 * ascending order by source values).  The actual code strings are
 * reconstructed from the lengths in the inflate process, as described
 * in the deflate specification.
 *
 *  REFERENCES
 *
 *      Deutsch, L.P.,"'Deflate' Compressed Data Format Specification".
 *      Available in ftp.uu.net:/pub/archiving/zip/doc/deflate-1.1.doc
 *
 *      Storer, James A.
 *          Data Compression:  Methods and Theory, pp. 49-50.
 *          Computer Science Press, 1988.  ISBN 0-7167-8156-5.
 *
 *      Sedgewick, R.
 *          Algorithms, p290.
 *          Addison-Wesley, 1983. ISBN 0-201-06672-6.
 */

/* @(#) $Id$ */

/* #define GEN_TREES_H */

#include "deflate.h"

#ifdef ZLIB_DEBUG
#  include <ctype.h>
#endif

/* ===========================================================================
 * Constants
 */

#define MAX_BL_BITS 7
/* Bit length codes must not exceed MAX_BL_BITS bits */

#define END_BLOCK 256
/* end of block literal code */

#define REP_3_6      16
/* repeat previous bit length 3-6 times (2 bits of repeat count) */

#define REPZ_3_10    17
/* repeat a zero length 3-10 times  (3 bits of repeat count) */

#define REPZ_11_138  18
/* repeat a zero length 11-138 times  (7 bits of repeat count) */

local const int extra_lbits[LENGTH_CODES] /* extra bits for each length code */
   = {0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,0};

local const int extra_dbits[D_CODES] /* extra bits for each distance code */
   = {0,0,0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,10,10,11,11,12,12,13,13};

local const int extra_blbits[BL_CODES]/* extra bits for each bit length code */
   = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,3,7};

local const uch bl_order[BL_CODES]
   = {16,17,18,0,8,7,9,6,10,5,11,4,12,3,13,2,14,1,15};
/* The lengths of the bit length codes are sent in order of decreasing
 * probability, to avoid transmitting the lengths for unused bit length codes.
 */

/* ===========================================================================
 * Local data. These are initialized only once.
 */

#define DIST_CODE_LEN  512 /* see definition of array dist_code below */

#if defined(GEN_TREES_H) || !defined(STDC)
/* non ANSI compilers may not accept trees.h */

local ct_data static_ltree[L_CODES+2];
/* The static literal tree. Since the bit lengths are imposed, there is no
 * need for the L_CODES extra codes used during heap construction. However
 * The codes 286 and 287 are needed to build a canonical tree (see _tr_init
 * below).
 */

local ct_data static_dtree[D_CODES];
/* The static distance tree. (Actually a trivial tree since all codes use
 * 5 bits.)
 */

uch _dist_code[DIST_CODE_LEN];
/* Distance codes. The first 256 values correspond to the distances
 * 3 .. 258, the last 256 values correspond to the top 8 bits of
 * the 15 bit distances.
 */

uch _length_code[MAX_MATCH-MIN_MATCH+1];
/* length code for each normalized match length (0 == MIN_MATCH) */

local int base_length[LENGTH_CODES];
/* First normalized length for each code (0 = MIN_MATCH) */

local int base_dist[D_CODES];
/* First normalized distance for each code (0 = distance of 1) */

#else
#  include "trees.h"
#endif /* GEN_TREES_H */

struct static_tree_desc_s {
    const ct_data *static_tree;  /* static tree or NULL */
    const intf *extra_bits;      /* extra bits for each code or NULL */
    int     extra_base;          /* base index for extra_bits */
    int     elems;               /* max number of elements in the tree */
    int     max_length;          /* max bit length for the codes */
};

local const static_tree_desc  static_l_desc =
{static_ltree, extra_lbits, LITERALS+1, L_CODES, MAX_BITS};

local const static_tree_desc  static_d_desc =
{static_dtree, extra_dbits, 0,          D_CODES, MAX_BITS};

local const static_tree_desc  static_bl_desc =
{(const ct_data *)0, extra_blbits, 0,   BL_CODES, MAX_BL_BITS};

/* ===========================================================================
 * Local (static) routines in this file.
 */

local void tr_static_init OF((void));
local void init_block     OF((deflate_state *deflateState));
local void pqdownheap     OF((deflate_state *deflateState, ct_data *tree, int iHeapParent));
local void gen_bitlen     OF((deflate_state *deflateState, tree_desc *treeDescriptor));
local void gen_codes      OF((ct_data *tree, int lastUsedCode, ushf *bl_count));
local void build_tree     OF((deflate_state *deflateState, tree_desc *treeDescriptor));
local void scan_tree      OF((deflate_state *deflateState, ct_data *tree, int lastUsedCode));
local void send_tree      OF((deflate_state *deflateState, ct_data *tree, int lastUsedCode));
local int  build_bl_tree  OF((deflate_state *deflateState));
local void send_all_trees OF((deflate_state *deflateState, int literalCodeCount, int distanceCodeCount,
                              int bitLengthCodeCount));
local void compress_block OF((deflate_state *deflateState, const ct_data *literalTree,
                              const ct_data *distanceTree));
local int  detect_data_type OF((deflate_state *deflateState));
local unsigned bi_reverse OF((unsigned value, int length));
local void bi_windup      OF((deflate_state *deflateState));
local void bi_flush       OF((deflate_state *deflateState));

#ifdef GEN_TREES_H
local void gen_trees_header OF((void));
#endif

#ifndef ZLIB_DEBUG
#  define send_code(deflateState, c, tree) send_bits(deflateState, tree[c].Code, tree[c].Len)
   /* Send a code of the given tree. c and tree must not have side effects */

#else /* !ZLIB_DEBUG */
#  define send_code(deflateState, c, tree) \
     { if (z_verbose>2) fprintf(stderr,"\ncd %3d ",(c)); \
       send_bits(deflateState, tree[c].Code, tree[c].Len); }
#endif

/* ===========================================================================
 * Output a short LSB first on the stream.
 * IN assertion: there is enough room in pendingBuf.
 */
#define put_short(deflateState, w) { \
    put_byte(deflateState, (uch)((w) & 0xff)); \
    put_byte(deflateState, (uch)((ush)(w) >> 8)); \
}

/* ===========================================================================
 * Send a value on a given number of bits.
 * IN assertion: length <= 16 and value fits in length bits.
 */
#ifdef ZLIB_DEBUG
local void send_bits      OF((deflate_state *deflateState, int value, int length));

local void send_bits(deflateState, value, length)
    deflate_state *deflateState;
    int value;  /* value to send */
    int length; /* number of bits */
{
    Tracevv((stderr," l %2d v %4x ", length, value));
    Assert(length > 0 && length <= 15, "invalid length");
    deflateState->bits_sent += (ulg)length;

    /* If not enough room in bi_buf, use (valid) bits from bi_buf and
     * (16 - bi_valid) bits from value, leaving (width - (16-bi_valid))
     * unused bits in value.
     */
    if (deflateState->bi_valid > (int)Buf_size - length) {
        deflateState->bi_buf |= (ush)value << deflateState->bi_valid;
        put_short(deflateState, deflateState->bi_buf);
        deflateState->bi_buf = (ush)value >> (Buf_size - deflateState->bi_valid);
        deflateState->bi_valid += length - Buf_size;
    } else {
        deflateState->bi_buf |= (ush)value << deflateState->bi_valid;
        deflateState->bi_valid += length;
    }
}
#else /* !ZLIB_DEBUG */

#define send_bits(deflateState, value, length) \
{ int bitLength = length;\
  if (deflateState->bi_valid > (int)Buf_size - bitLength) {\
    int bitValue = (int)value;\
    deflateState->bi_buf |= (ush)bitValue << deflateState->bi_valid;\
    put_short(deflateState, deflateState->bi_buf);\
    deflateState->bi_buf = (ush)bitValue >> (Buf_size - deflateState->bi_valid);\
    deflateState->bi_valid += bitLength - Buf_size;\
  } else {\
    deflateState->bi_buf |= (ush)(value) << deflateState->bi_valid;\
    deflateState->bi_valid += bitLength;\
  }\
}
#endif /* ZLIB_DEBUG */


/* the arguments must not have side effects */

/* ===========================================================================
 * Initialize the various 'constant' tables.
 */
local void tr_static_init()
{
#if defined(GEN_TREES_H) || !defined(STDC)
    static int static_init_done = 0;
    int iNode;        /* iterates over tree elements */
    int bits;     /* bit counter */
    int length;   /* length value */
    int code;     /* code value */
    int matchDistance;     /* distance index */
    ush bl_count[MAX_BITS+1];
    /* number of codes at each bit length for an optimal tree */

    if (static_init_done) return;

    /* For some embedded targets, global variables are not initialized: */
#ifdef NO_INIT_GLOBAL_POINTERS
    static_l_desc.static_tree = static_ltree;
    static_l_desc.extra_bits = extra_lbits;
    static_d_desc.static_tree = static_dtree;
    static_d_desc.extra_bits = extra_dbits;
    static_bl_desc.extra_bits = extra_blbits;
#endif

    /* Initialize the mapping length (0..255) -> length code (0..28) */
    length = 0;
    for (code = 0; code < LENGTH_CODES-1; code++) {
        base_length[code] = length;
        for (iNode = 0; iNode < (1<<extra_lbits[code]); iNode++) {
            _length_code[length++] = (uch)code;
        }
    }
    Assert (length == 256, "tr_static_init: length != 256");
    /* Note that the length 255 (match length 258) can be represented
     * in two different ways: code 284 + 5 bits or code 285, so we
     * overwrite length_code[255] to use the best encoding:
     */
    _length_code[length-1] = (uch)code;

    /* Initialize the mapping dist (0..32K) -> dist code (0..29) */
    matchDistance = 0;
    for (code = 0 ; code < 16; code++) {
        base_dist[code] = matchDistance;
        for (iNode = 0; iNode < (1<<extra_dbits[code]); iNode++) {
            _dist_code[matchDistance++] = (uch)code;
        }
    }
    Assert (matchDistance == 256, "tr_static_init: dist != 256");
    matchDistance >>= 7; /* from now on, all distances are divided by 128 */
    for ( ; code < D_CODES; code++) {
        base_dist[code] = matchDistance << 7;
        for (iNode = 0; iNode < (1<<(extra_dbits[code]-7)); iNode++) {
            _dist_code[256 + matchDistance++] = (uch)code;
        }
    }
    Assert (matchDistance == 256, "tr_static_init: 256+dist != 512");

    /* Construct the codes of the static literal tree */
    for (bits = 0; bits <= MAX_BITS; bits++) bl_count[bits] = 0;
    iNode = 0;
    while (iNode <= 143) static_ltree[iNode++].Len = 8, bl_count[8]++;
    while (iNode <= 255) static_ltree[iNode++].Len = 9, bl_count[9]++;
    while (iNode <= 279) static_ltree[iNode++].Len = 7, bl_count[7]++;
    while (iNode <= 287) static_ltree[iNode++].Len = 8, bl_count[8]++;
    /* Codes 286 and 287 do not exist, but we must include them in the
     * tree construction to get a canonical Huffman tree (longest code
     * all ones)
     */
    gen_codes((ct_data *)static_ltree, L_CODES+1, bl_count);

    /* The static distance tree is trivial: */
    for (iNode = 0; iNode < D_CODES; iNode++) {
        static_dtree[iNode].Len = 5;
        static_dtree[iNode].Code = bi_reverse((unsigned)iNode, 5);
    }
    static_init_done = 1;

#  ifdef GEN_TREES_H
    gen_trees_header();
#  endif
#endif /* defined(GEN_TREES_H) || !defined(STDC) */
}

/* ===========================================================================
 * Genererate the file trees.h describing the static trees.
 */
#ifdef GEN_TREES_H
#  ifndef ZLIB_DEBUG
#    include <stdio.h>
#  endif

#  define SEPARATOR(i, isLastBlock, width) \
      ((i) == (isLastBlock)? "\n};\n\n" :    \
       ((i) % (width) == (width)-1 ? ",\n" : ", "))

void gen_trees_header()
{
    FILE *header = fopen("trees.h", "w");
    int i;

    Assert (header != NULL, "Can't open trees.h");
    fprintf(header,
            "/* header created automatically with -DGEN_TREES_H */\n\n");

    fprintf(header, "local const ct_data static_ltree[L_CODES+2] = {\n");
    for (i = 0; i < L_CODES+2; i++) {
        fprintf(header, "{{%3u},{%3u}}%s", static_ltree[i].Code,
                static_ltree[i].Len, SEPARATOR(i, L_CODES+1, 5));
    }

    fprintf(header, "local const ct_data static_dtree[D_CODES] = {\n");
    for (i = 0; i < D_CODES; i++) {
        fprintf(header, "{{%2u},{%2u}}%s", static_dtree[i].Code,
                static_dtree[i].Len, SEPARATOR(i, D_CODES-1, 5));
    }

    fprintf(header, "const uch ZLIB_INTERNAL _dist_code[DIST_CODE_LEN] = {\n");
    for (i = 0; i < DIST_CODE_LEN; i++) {
        fprintf(header, "%2u%s", _dist_code[i],
                SEPARATOR(i, DIST_CODE_LEN-1, 20));
    }

    fprintf(header,
        "const uch ZLIB_INTERNAL _length_code[MAX_MATCH-MIN_MATCH+1]= {\n");
    for (i = 0; i < MAX_MATCH-MIN_MATCH+1; i++) {
        fprintf(header, "%2u%s", _length_code[i],
                SEPARATOR(i, MAX_MATCH-MIN_MATCH, 20));
    }

    fprintf(header, "local const int base_length[LENGTH_CODES] = {\n");
    for (i = 0; i < LENGTH_CODES; i++) {
        fprintf(header, "%1u%s", base_length[i],
                SEPARATOR(i, LENGTH_CODES-1, 20));
    }

    fprintf(header, "local const int base_dist[D_CODES] = {\n");
    for (i = 0; i < D_CODES; i++) {
        fprintf(header, "%5u%s", base_dist[i],
                SEPARATOR(i, D_CODES-1, 10));
    }

    fclose(header);
}
#endif /* GEN_TREES_H */

/* ===========================================================================
 * Initialize the tree data structures for a new zlib stream.
 */
void ZLIB_INTERNAL _tr_init(deflateState)
    deflate_state *deflateState;
{
    tr_static_init();

    deflateState->l_desc.dyn_tree = deflateState->dyn_ltree;
    deflateState->l_desc.stat_desc = &static_l_desc;

    deflateState->d_desc.dyn_tree = deflateState->dyn_dtree;
    deflateState->d_desc.stat_desc = &static_d_desc;

    deflateState->bl_desc.dyn_tree = deflateState->bl_tree;
    deflateState->bl_desc.stat_desc = &static_bl_desc;

    deflateState->bi_buf = 0;
    deflateState->bi_valid = 0;
#ifdef ZLIB_DEBUG
    deflateState->compressed_len = 0L;
    deflateState->bits_sent = 0L;
#endif

    /* Initialize the first block of the first file: */
    init_block(deflateState);
}

/* ===========================================================================
 * Initialize a new block.
 */
local void init_block(deflateState)
    deflate_state *deflateState;
{
    int iNode; /* iterates over tree elements */

    /* Initialize the trees. */
    for (iNode = 0; iNode < L_CODES;  iNode++) deflateState->dyn_ltree[iNode].Freq = 0;
    for (iNode = 0; iNode < D_CODES;  iNode++) deflateState->dyn_dtree[iNode].Freq = 0;
    for (iNode = 0; iNode < BL_CODES; iNode++) deflateState->bl_tree[iNode].Freq = 0;

    deflateState->dyn_ltree[END_BLOCK].Freq = 1;
    deflateState->opt_len = deflateState->static_len = 0L;
    deflateState->last_lit = deflateState->matches = 0;
}

#define SMALLEST 1
/* Index within the heap array of least frequent node in the Huffman tree */


/* ===========================================================================
 * Remove the smallest element from the heap and recreate the heap with
 * one less element. Updates heap and heap_len.
 */
#define pqremove(deflateState, tree, top) \
{\
    top = deflateState->heap[SMALLEST]; \
    deflateState->heap[SMALLEST] = deflateState->heap[deflateState->heap_len--]; \
    pqdownheap(deflateState, tree, SMALLEST); \
}

/* ===========================================================================
 * Compares to subtrees, using the tree depth as tie breaker when
 * the subtrees have equal frequency. This minimizes the worst case length.
 */
#define smaller(tree, iNode, secondNode, depth) \
   (tree[iNode].Freq < tree[secondNode].Freq || \
   (tree[iNode].Freq == tree[secondNode].Freq && depth[iNode] <= depth[secondNode]))

/* ===========================================================================
 * Restore the heap property by moving down the tree starting at node iHeapParent,
 * exchanging a node with the smallest of its two sons if necessary, stopping
 * when the heap property is re-established (each father smaller than its
 * two sons).
 */
local void pqdownheap(deflateState, tree, iHeapParent)
    deflate_state *deflateState;
    ct_data *tree;  /* the tree to restore */
    int iHeapParent;               /* node to move down */
{
    int heapNode = deflateState->heap[iHeapParent];
    int iHeapChild = iHeapParent << 1;  /* left son of iHeapParent */
    while (iHeapChild <= deflateState->heap_len) {
        /* Set iHeapChild to the smallest of the two sons: */
        if (iHeapChild < deflateState->heap_len &&
            smaller(tree, deflateState->heap[iHeapChild+1], deflateState->heap[iHeapChild], deflateState->depth)) {
            iHeapChild++;
        }
        /* Exit if heapNode is smaller than both sons */
        if (smaller(tree, heapNode, deflateState->heap[iHeapChild], deflateState->depth)) break;

        /* Exchange heapNode with the smallest son */
        deflateState->heap[iHeapParent] = deflateState->heap[iHeapChild];  iHeapParent = iHeapChild;

        /* And continue down the tree, setting iHeapChild to the left son of iHeapParent */
        iHeapChild <<= 1;
    }
    deflateState->heap[iHeapParent] = heapNode;
}

/* ===========================================================================
 * Compute the optimal bit lengths for a tree and update the total bit length
 * for the current block.
 * IN assertion: the fields freq and dad are set, heap[heap_max] and
 *    above are the tree nodes sorted by increasing frequency.
 * OUT assertions: the field bitLength is set to the optimal bit length, the
 *     array bl_count contains the frequencies for each bit length.
 *     The length opt_len is updated; static_len is also updated if staticTree is
 *     not null.
 */
local void gen_bitlen(deflateState, treeDescriptor)
    deflate_state *deflateState;
    tree_desc *treeDescriptor;    /* the tree descriptor */
{
    ct_data *tree        = treeDescriptor->dyn_tree;
    int lastUsedCode         = treeDescriptor->max_code;
    const ct_data *staticTree = treeDescriptor->stat_desc->static_tree;
    const intf *extraBitsByCode    = treeDescriptor->stat_desc->extra_bits;
    int extraBitsBase             = treeDescriptor->stat_desc->extra_base;
    int maximumCodeLength       = treeDescriptor->stat_desc->max_length;
    int iHeap;              /* heap index */
    int iNode, secondNode;           /* iterate over the tree elements */
    int bits;           /* bit length */
    int extraBitCount;          /* extra bits */
    ush frequency;              /* frequency */
    int overflow = 0;   /* number of elements with bit length too large */

    for (bits = 0; bits <= MAX_BITS; bits++) deflateState->bl_count[bits] = 0;

    /* In a first pass, compute the optimal bit lengths (which may
     * overflow in the case of the bit length tree).
     */
    tree[deflateState->heap[deflateState->heap_max]].Len = 0; /* root of the heap */

    for (iHeap = deflateState->heap_max+1; iHeap < HEAP_SIZE; iHeap++) {
        iNode = deflateState->heap[iHeap];
        bits = tree[tree[iNode].Dad].Len + 1;
        if (bits > maximumCodeLength) bits = maximumCodeLength, overflow++;
        tree[iNode].Len = (ush)bits;
        /* We overwrite tree[iNode].Dad which is no longer needed */

        if (iNode > lastUsedCode) continue; /* not a leaf node */

        deflateState->bl_count[bits]++;
        extraBitCount = 0;
        if (iNode >= extraBitsBase) extraBitCount = extraBitsByCode[iNode-extraBitsBase];
        frequency = tree[iNode].Freq;
        deflateState->opt_len += (ulg)frequency * (unsigned)(bits + extraBitCount);
        if (staticTree) deflateState->static_len += (ulg)frequency * (unsigned)(staticTree[iNode].Len + extraBitCount);
    }
    if (overflow == 0) return;

    Tracev((stderr,"\nbit length overflow\n"));
    /* This happens for example on obj2 and pic of the Calgary corpus */

    /* Find the first bit length which could increase: */
    do {
        bits = maximumCodeLength-1;
        while (deflateState->bl_count[bits] == 0) bits--;
        deflateState->bl_count[bits]--;      /* move one leaf down the tree */
        deflateState->bl_count[bits+1] += 2; /* move one overflow item as its brother */
        deflateState->bl_count[maximumCodeLength]--;
        /* The brother of the overflow item also moves one step up,
         * but this does not affect bl_count[max_length]
         */
        overflow -= 2;
    } while (overflow > 0);

    /* Now recompute all bit lengths, scanning in increasing frequency.
     * h is still equal to HEAP_SIZE. (It is simpler to reconstruct all
     * lengths instead of fixing only the wrong ones. This idea is taken
     * from 'ar' written by Haruhiko Okumura.)
     */
    for (bits = maximumCodeLength; bits != 0; bits--) {
        iNode = deflateState->bl_count[bits];
        while (iNode != 0) {
            secondNode = deflateState->heap[--iHeap];
            if (secondNode > lastUsedCode) continue;
            if ((unsigned) tree[secondNode].Len != (unsigned) bits) {
                Tracev((stderr,"code %d bits %d->%d\n", secondNode, tree[secondNode].Len, bits));
                deflateState->opt_len += ((ulg)bits - tree[secondNode].Len) * tree[secondNode].Freq;
                tree[secondNode].Len = (ush)bits;
            }
            iNode--;
        }
    }
}

/* ===========================================================================
 * Generate the codes for a given tree and bit counts (which need not be
 * optimal).
 * IN assertion: the array bl_count contains the bit length statistics for
 * the given tree and the field bitLength is set for all tree elements.
 * OUT assertion: the field code is set for all tree elements of non
 *     zero code length.
 */
local void gen_codes (tree, lastUsedCode, bl_count)
    ct_data *tree;             /* the tree to decorate */
    int lastUsedCode;              /* largest code with non zero frequency */
    ushf *bl_count;            /* number of codes at each bit length */
{
    ush nextCodeByLength[MAX_BITS+1]; /* next code value for each bit length */
    unsigned code = 0;         /* running code value */
    int bits;                  /* bit index */
    int iNode;                     /* code index */

    /* The distribution counts are first used to generate the code values
     * without bit reversal.
     */
    for (bits = 1; bits <= MAX_BITS; bits++) {
        code = (code + bl_count[bits-1]) << 1;
        nextCodeByLength[bits] = (ush)code;
    }
    /* Check that the bit counts in bl_count are consistent. The last code
     * must be all ones.
     */
    Assert (code + bl_count[MAX_BITS]-1 == (1<<MAX_BITS)-1,
            "inconsistent bit counts");
    Tracev((stderr,"\ngen_codes: max_code %d ", lastUsedCode));

    for (iNode = 0;  iNode <= lastUsedCode; iNode++) {
        int codeLength = tree[iNode].Len;
        if (codeLength == 0) continue;
        /* Now reverse the bits */
        tree[iNode].Code = (ush)bi_reverse(nextCodeByLength[codeLength]++, codeLength);

        Tracecv(tree != static_ltree, (stderr,"\nn %3d %c l %2d c %4x (%x) ",
             iNode, (isgraph(iNode) ? iNode : ' '), codeLength, tree[iNode].Code, nextCodeByLength[codeLength]-1));
    }
}

/* ===========================================================================
 * Construct one Huffman tree and assigns the code bit strings and lengths.
 * Update the total bit length for the current block.
 * IN assertion: the field freq is set for all tree elements.
 * OUT assertions: the fields codeLength and code are set to the optimal bit length
 *     and corresponding code. The length opt_len is updated; static_len is
 *     also updated if staticTree is not null. The field max_code is set.
 */
local void build_tree(deflateState, treeDescriptor)
    deflate_state *deflateState;
    tree_desc *treeDescriptor; /* the tree descriptor */
{
    ct_data *tree         = treeDescriptor->dyn_tree;
    const ct_data *staticTree  = treeDescriptor->stat_desc->static_tree;
    int elems             = treeDescriptor->stat_desc->elems;
    int iNode, secondNode;          /* iterate over heap elements */
    int lastUsedCode = -1; /* largest code with non zero frequency */
    int node;          /* new node being created */

    /* Construct the initial heap, with least frequent element in
     * heap[SMALLEST]. The sons of heap[iNode] are heap[2*iNode] and heap[2*iNode+1].
     * heap[0] is not used.
     */
    deflateState->heap_len = 0, deflateState->heap_max = HEAP_SIZE;

    for (iNode = 0; iNode < elems; iNode++) {
        if (tree[iNode].Freq != 0) {
            deflateState->heap[++(deflateState->heap_len)] = lastUsedCode = iNode;
            deflateState->depth[iNode] = 0;
        } else {
            tree[iNode].Len = 0;
        }
    }

    /* The pkzip format requires that at least one distance code exists,
     * and that at least one bit should be sent even if there is only one
     * possible code. So to avoid special checks later on we force at least
     * two codes of non zero frequency.
     */
    while (deflateState->heap_len < 2) {
        node = deflateState->heap[++(deflateState->heap_len)] = (lastUsedCode < 2 ? ++lastUsedCode : 0);
        tree[node].Freq = 1;
        deflateState->depth[node] = 0;
        deflateState->opt_len--; if (staticTree) deflateState->static_len -= staticTree[node].Len;
        /* node is 0 or 1 so it does not have extra bits */
    }
    treeDescriptor->max_code = lastUsedCode;

    /* The elements heap[heap_len/2+1 .. heap_len] are leaves of the tree,
     * establish sub-heaps of increasing lengths:
     */
    for (iNode = deflateState->heap_len/2; iNode >= 1; iNode--) pqdownheap(deflateState, tree, iNode);

    /* Construct the Huffman tree by repeatedly combining the least two
     * frequent nodes.
     */
    node = elems;              /* next internal node of the tree */
    do {
        pqremove(deflateState, tree, iNode);  /* iNode = node of least frequency */
        secondNode = deflateState->heap[SMALLEST]; /* m = node of next least frequency */

        deflateState->heap[--(deflateState->heap_max)] = iNode; /* keep the nodes sorted by frequency */
        deflateState->heap[--(deflateState->heap_max)] = secondNode;

        /* Create a new node father of iNode and m */
        tree[node].Freq = tree[iNode].Freq + tree[secondNode].Freq;
        deflateState->depth[node] = (uch)((deflateState->depth[iNode] >= deflateState->depth[secondNode] ?
                                deflateState->depth[iNode] : deflateState->depth[secondNode]) + 1);
        tree[iNode].Dad = tree[secondNode].Dad = (ush)node;
#ifdef DUMP_BL_TREE
        if (tree == deflateState->bl_tree) {
            fprintf(stderr,"\nnode %d(%d), sons %d(%d) %d(%d)",
                    node, tree[node].Freq, iNode, tree[iNode].Freq, secondNode, tree[secondNode].Freq);
        }
#endif
        /* and insert the new node in the heap */
        deflateState->heap[SMALLEST] = node++;
        pqdownheap(deflateState, tree, SMALLEST);

    } while (deflateState->heap_len >= 2);

    deflateState->heap[--(deflateState->heap_max)] = deflateState->heap[SMALLEST];

    /* At this point, the fields freq and dad are set. We can now
     * generate the bit lengths.
     */
    gen_bitlen(deflateState, (tree_desc *)treeDescriptor);

    /* The field bitLength is now set, we can generate the bit codes */
    gen_codes ((ct_data *)tree, lastUsedCode, deflateState->bl_count);
}

/* ===========================================================================
 * Scan a literal or distance tree to determine the frequencies of the codes
 * in the bit length tree.
 */
local void scan_tree (deflateState, tree, lastUsedCode)
    deflate_state *deflateState;
    ct_data *tree;   /* the tree to be scanned */
    int lastUsedCode;    /* and its largest code of non zero frequency */
{
    int iNode;                     /* iterates over all tree elements */
    int previousCodeLength = -1;          /* last emitted length */
    int currentCodeLength;                /* length of current code */
    int nextCodeLength = tree[0].Len; /* length of next code */
    int count = 0;             /* repeat count of the current code */
    int maximumRepeatCount = 7;         /* max repeat count */
    int minimumRepeatCount = 4;         /* min repeat count */

    if (nextCodeLength == 0) maximumRepeatCount = 138, minimumRepeatCount = 3;
    tree[lastUsedCode+1].Len = (ush)0xffff; /* guard */

    for (iNode = 0; iNode <= lastUsedCode; iNode++) {
        currentCodeLength = nextCodeLength; nextCodeLength = tree[iNode+1].Len;
        if (++count < maximumRepeatCount && currentCodeLength == nextCodeLength) {
            continue;
        } else if (count < minimumRepeatCount) {
            deflateState->bl_tree[currentCodeLength].Freq += count;
        } else if (currentCodeLength != 0) {
            if (currentCodeLength != previousCodeLength) deflateState->bl_tree[currentCodeLength].Freq++;
            deflateState->bl_tree[REP_3_6].Freq++;
        } else if (count <= 10) {
            deflateState->bl_tree[REPZ_3_10].Freq++;
        } else {
            deflateState->bl_tree[REPZ_11_138].Freq++;
        }
        count = 0; previousCodeLength = currentCodeLength;
        if (nextCodeLength == 0) {
            maximumRepeatCount = 138, minimumRepeatCount = 3;
        } else if (currentCodeLength == nextCodeLength) {
            maximumRepeatCount = 6, minimumRepeatCount = 3;
        } else {
            maximumRepeatCount = 7, minimumRepeatCount = 4;
        }
    }
}

/* ===========================================================================
 * Send a literal or distance tree in compressed form, using the codes in
 * bl_tree.
 */
local void send_tree (deflateState, tree, lastUsedCode)
    deflate_state *deflateState;
    ct_data *tree; /* the tree to be scanned */
    int lastUsedCode;       /* and its largest code of non zero frequency */
{
    int iNode;                     /* iterates over all tree elements */
    int previousCodeLength = -1;          /* last emitted length */
    int currentCodeLength;                /* length of current code */
    int nextCodeLength = tree[0].Len; /* length of next code */
    int count = 0;             /* repeat count of the current code */
    int maximumRepeatCount = 7;         /* max repeat count */
    int minimumRepeatCount = 4;         /* min repeat count */

    /* tree[max_code+1].Len = -1; */  /* guard already set */
    if (nextCodeLength == 0) maximumRepeatCount = 138, minimumRepeatCount = 3;

    for (iNode = 0; iNode <= lastUsedCode; iNode++) {
        currentCodeLength = nextCodeLength; nextCodeLength = tree[iNode+1].Len;
        if (++count < maximumRepeatCount && currentCodeLength == nextCodeLength) {
            continue;
        } else if (count < minimumRepeatCount) {
            do { send_code(deflateState, currentCodeLength, deflateState->bl_tree); } while (--count != 0);

        } else if (currentCodeLength != 0) {
            if (currentCodeLength != previousCodeLength) {
                send_code(deflateState, currentCodeLength, deflateState->bl_tree); count--;
            }
            Assert(count >= 3 && count <= 6, " 3_6?");
            send_code(deflateState, REP_3_6, deflateState->bl_tree); send_bits(deflateState, count-3, 2);

        } else if (count <= 10) {
            send_code(deflateState, REPZ_3_10, deflateState->bl_tree); send_bits(deflateState, count-3, 3);

        } else {
            send_code(deflateState, REPZ_11_138, deflateState->bl_tree); send_bits(deflateState, count-11, 7);
        }
        count = 0; previousCodeLength = currentCodeLength;
        if (nextCodeLength == 0) {
            maximumRepeatCount = 138, minimumRepeatCount = 3;
        } else if (currentCodeLength == nextCodeLength) {
            maximumRepeatCount = 6, minimumRepeatCount = 3;
        } else {
            maximumRepeatCount = 7, minimumRepeatCount = 4;
        }
    }
}

/* ===========================================================================
 * Construct the Huffman tree for the bit lengths and return the index in
 * bl_order of the last bit length code to send.
 */
local int build_bl_tree(deflateState)
    deflate_state *deflateState;
{
    int lastBitLengthIndex;  /* index of last bit length code of non zero freq */

    /* Determine the bit length frequencies for literal and distance trees */
    scan_tree(deflateState, (ct_data *)deflateState->dyn_ltree, deflateState->l_desc.max_code);
    scan_tree(deflateState, (ct_data *)deflateState->dyn_dtree, deflateState->d_desc.max_code);

    /* Build the bit length tree: */
    build_tree(deflateState, (tree_desc *)(&(deflateState->bl_desc)));
    /* opt_len now includes the length of the tree representations, except
     * the lengths of the bit lengths codes and the 5+5+4 bits for the counts.
     */

    /* Determine the number of bit length codes to send. The pkzip format
     * requires that at least 4 bit length codes be sent. (appnote.txt says
     * 3 but the actual value used is 4.)
     */
    for (lastBitLengthIndex = BL_CODES-1; lastBitLengthIndex >= 3; lastBitLengthIndex--) {
        if (deflateState->bl_tree[bl_order[lastBitLengthIndex]].Len != 0) break;
    }
    /* Update opt_len to include the bit length tree and counts */
    deflateState->opt_len += 3*((ulg)lastBitLengthIndex+1) + 5+5+4;
    Tracev((stderr, "\ndyn trees: dyn %ld, stat %ld",
            deflateState->opt_len, deflateState->static_len));

    return lastBitLengthIndex;
}

/* ===========================================================================
 * Send the header for a block using dynamic Huffman trees: the counts, the
 * lengths of the bit length codes, the literal tree and the distance tree.
 * IN assertion: literalCodeCount >= 257, distanceCodeCount >= 1, bitLengthCodeCount >= 4.
 */
local void send_all_trees(deflateState, literalCodeCount, distanceCodeCount, bitLengthCodeCount)
    deflate_state *deflateState;
    int literalCodeCount, distanceCodeCount, bitLengthCodeCount; /* number of codes for each tree */
{
    int rank;                    /* index in bl_order */

    Assert (literalCodeCount >= 257 && distanceCodeCount >= 1 && bitLengthCodeCount >= 4, "not enough codes");
    Assert (literalCodeCount <= L_CODES && distanceCodeCount <= D_CODES && bitLengthCodeCount <= BL_CODES,
            "too many codes");
    Tracev((stderr, "\nbl counts: "));
    send_bits(deflateState, literalCodeCount-257, 5); /* not +255 as stated in appnote.txt */
    send_bits(deflateState, distanceCodeCount-1,   5);
    send_bits(deflateState, bitLengthCodeCount-4,  4); /* not -3 as stated in appnote.txt */
    for (rank = 0; rank < bitLengthCodeCount; rank++) {
        Tracev((stderr, "\nbl code %2d ", bl_order[rank]));
        send_bits(deflateState, deflateState->bl_tree[bl_order[rank]].Len, 3);
    }
    Tracev((stderr, "\nbl tree: sent %ld", deflateState->bits_sent));

    send_tree(deflateState, (ct_data *)deflateState->dyn_ltree, literalCodeCount-1); /* literal tree */
    Tracev((stderr, "\nlit tree: sent %ld", deflateState->bits_sent));

    send_tree(deflateState, (ct_data *)deflateState->dyn_dtree, distanceCodeCount-1); /* distance tree */
    Tracev((stderr, "\ndist tree: sent %ld", deflateState->bits_sent));
}

/* ===========================================================================
 * Send a stored block
 */
void ZLIB_INTERNAL _tr_stored_block(deflateState, buf, storedLength, isLastBlock)
    deflate_state *deflateState;
    charf *buf;       /* input block */
    ulg storedLength;   /* length of input block */
    int isLastBlock;         /* one if this is the last block for a file */
{
    send_bits(deflateState, (STORED_BLOCK<<1)+isLastBlock, 3);    /* send block type */
    bi_windup(deflateState);        /* align on byte boundary */
    put_short(deflateState, (ush)storedLength);
    put_short(deflateState, (ush)~storedLength);
    zmemcpy(deflateState->pending_buf + deflateState->pending, (Bytef *)buf, storedLength);
    deflateState->pending += storedLength;
#ifdef ZLIB_DEBUG
    deflateState->compressed_len = (deflateState->compressed_len + 3 + 7) & (ulg)~7L;
    deflateState->compressed_len += (storedLength + 4) << 3;
    deflateState->bits_sent += 2*16;
    deflateState->bits_sent += storedLength<<3;
#endif
}

/* ===========================================================================
 * Flush the bits in the bit buffer to pending output (leaves at most 7 bits)
 */
void ZLIB_INTERNAL _tr_flush_bits(deflateState)
    deflate_state *deflateState;
{
    bi_flush(deflateState);
}

/* ===========================================================================
 * Send one empty static block to give enough lookahead for inflate.
 * This takes 10 bits, of which 7 may remain in the bit buffer.
 */
void ZLIB_INTERNAL _tr_align(deflateState)
    deflate_state *deflateState;
{
    send_bits(deflateState, STATIC_TREES<<1, 3);
    send_code(deflateState, END_BLOCK, static_ltree);
#ifdef ZLIB_DEBUG
    deflateState->compressed_len += 10L; /* 3 for block type, 7 for EOB */
#endif
    bi_flush(deflateState);
}

/* ===========================================================================
 * Determine the best encoding for the current block: dynamic trees, static
 * trees or store, and write out the encoded block.
 */
void ZLIB_INTERNAL _tr_flush_block(deflateState, buf, storedLength, isLastBlock)
    deflate_state *deflateState;
    charf *buf;       /* input block, or NULL if too old */
    ulg storedLength;   /* length of input block */
    int isLastBlock;         /* one if this is the last block for a file */
{
    ulg optimalByteLength, staticByteLength; /* opt_len and static_len in bytes */
    int lastBitLengthIndex = 0;  /* index of last bit length code of non zero freq */

    /* Build the Huffman trees unless a stored block is forced */
    if (deflateState->level > 0) {

        /* Check if the file is binary or text */
        if (deflateState->strm->data_type == Z_UNKNOWN)
            deflateState->strm->data_type = detect_data_type(deflateState);

        /* Construct the literal and distance trees */
        build_tree(deflateState, (tree_desc *)(&(deflateState->l_desc)));
        Tracev((stderr, "\nlit data: dyn %ld, stat %ld", deflateState->opt_len,
                deflateState->static_len));

        build_tree(deflateState, (tree_desc *)(&(deflateState->d_desc)));
        Tracev((stderr, "\ndist data: dyn %ld, stat %ld", deflateState->opt_len,
                deflateState->static_len));
        /* At this point, opt_len and static_len are the total bit lengths of
         * the compressed block data, excluding the tree representations.
         */

        /* Build the bit length tree for the above two trees, and get the index
         * in bl_order of the last bit length code to send.
         */
        lastBitLengthIndex = build_bl_tree(deflateState);

        /* Determine the best encoding. Compute the block lengths in bytes. */
        optimalByteLength = (deflateState->opt_len+3+7)>>3;
        staticByteLength = (deflateState->static_len+3+7)>>3;

        Tracev((stderr, "\nopt %lu(%lu) stat %lu(%lu) stored %lu lit %u ",
                optimalByteLength, deflateState->opt_len, staticByteLength, deflateState->static_len, storedLength,
                deflateState->last_lit));

        if (staticByteLength <= optimalByteLength) optimalByteLength = staticByteLength;

    } else {
        Assert(buf != (char*)0, "lost buf");
        optimalByteLength = staticByteLength = storedLength + 5; /* force a stored block */
    }

#ifdef FORCE_STORED
    if (buf != (char*)0) { /* force stored block */
#else
    if (storedLength+4 <= optimalByteLength && buf != (char*)0) {
                       /* 4: two words for the lengths */
#endif
        /* The test buf != NULL is only necessary if LIT_BUFSIZE > WSIZE.
         * Otherwise we can't have processed more than WSIZE input bytes since
         * the last block flush, because compression would have been
         * successful. If LIT_BUFSIZE <= WSIZE, it is never too late to
         * transform a block into a stored block.
         */
        _tr_stored_block(deflateState, buf, storedLength, isLastBlock);

#ifdef FORCE_STATIC
    } else if (staticByteLength >= 0) { /* force static trees */
#else
    } else if (deflateState->strategy == Z_FIXED || staticByteLength == optimalByteLength) {
#endif
        send_bits(deflateState, (STATIC_TREES<<1)+isLastBlock, 3);
        compress_block(deflateState, (const ct_data *)static_ltree,
                       (const ct_data *)static_dtree);
#ifdef ZLIB_DEBUG
        deflateState->compressed_len += 3 + deflateState->static_len;
#endif
    } else {
        send_bits(deflateState, (DYN_TREES<<1)+isLastBlock, 3);
        send_all_trees(deflateState, deflateState->l_desc.max_code+1, deflateState->d_desc.max_code+1,
                       lastBitLengthIndex+1);
        compress_block(deflateState, (const ct_data *)deflateState->dyn_ltree,
                       (const ct_data *)deflateState->dyn_dtree);
#ifdef ZLIB_DEBUG
        deflateState->compressed_len += 3 + deflateState->opt_len;
#endif
    }
    Assert (deflateState->compressed_len == deflateState->bits_sent, "bad compressed size");
    /* The above check is made mod 2^32, for files larger than 512 MB
     * and uLong implemented on 32 bits.
     */
    init_block(deflateState);

    if (isLastBlock) {
        bi_windup(deflateState);
#ifdef ZLIB_DEBUG
        deflateState->compressed_len += 7;  /* align on byte boundary */
#endif
    }
    Tracev((stderr,"\ncomprlen %lu(%lu) ", deflateState->compressed_len>>3,
           deflateState->compressed_len-7*isLastBlock));
}

/* ===========================================================================
 * Save the match info and tally the frequency counts. Return true if
 * the current block must be flushed.
 */
int ZLIB_INTERNAL _tr_tally (deflateState, matchDistance, literalOrLength)
    deflate_state *deflateState;
    unsigned matchDistance;  /* distance of matched string */
    unsigned literalOrLength;    /* match length-MIN_MATCH or unmatched char (if dist==0) */
{
    deflateState->d_buf[deflateState->last_lit] = (ush)matchDistance;
    deflateState->l_buf[deflateState->last_lit++] = (uch)literalOrLength;
    if (matchDistance == 0) {
        /* literalOrLength is the unmatched char */
        deflateState->dyn_ltree[literalOrLength].Freq++;
    } else {
        deflateState->matches++;
        /* Here, literalOrLength is the match length - MIN_MATCH */
        matchDistance--;             /* dist = match distance - 1 */
        Assert((ush)matchDistance < (ush)MAX_DIST(deflateState) &&
               (ush)literalOrLength <= (ush)(MAX_MATCH-MIN_MATCH) &&
               (ush)d_code(matchDistance) < (ush)D_CODES,  "_tr_tally: bad match");

        deflateState->dyn_ltree[_length_code[literalOrLength]+LITERALS+1].Freq++;
        deflateState->dyn_dtree[d_code(matchDistance)].Freq++;
    }

#ifdef TRUNCATE_BLOCK
    /* Try to guess if it is profitable to stop the current block here */
    if ((deflateState->last_lit & 0x1fff) == 0 && deflateState->level > 2) {
        /* Compute an upper bound for the compressed length */
        ulg estimatedOutputBits = (ulg)deflateState->last_lit*8L;
        ulg inputLength = (ulg)((long)deflateState->strstart - deflateState->block_start);
        int distanceCode;
        for (distanceCode = 0; distanceCode < D_CODES; distanceCode++) {
            estimatedOutputBits += (ulg)deflateState->dyn_dtree[distanceCode].Freq *
                (5L+extra_dbits[distanceCode]);
        }
        estimatedOutputBits >>= 3;
        Tracev((stderr,"\nlast_lit %u, in %ld, out ~%ld(%ld%%) ",
               deflateState->last_lit, inputLength, estimatedOutputBits,
               100L - estimatedOutputBits*100L/inputLength));
        if (deflateState->matches < deflateState->last_lit/2 && estimatedOutputBits < inputLength/2) return 1;
    }
#endif
    return (deflateState->last_lit == deflateState->lit_bufsize-1);
    /* We avoid equality with lit_bufsize because of wraparound at 64K
     * on 16 bit machines and because stored blocks are restricted to
     * 64K-1 bytes.
     */
}

/* ===========================================================================
 * Send the block data compressed using the given Huffman trees
 */
local void compress_block(deflateState, literalTree, distanceTree)
    deflate_state *deflateState;
    const ct_data *literalTree; /* literal tree */
    const ct_data *distanceTree; /* distance tree */
{
    unsigned matchDistance;      /* distance of matched string */
    int literalOrLength;             /* match length or unmatched char (if dist == 0) */
    unsigned iLiteral = 0;    /* running index in l_buf */
    unsigned code;      /* the code to send */
    int extraBitCount;          /* number of extra bits to send */

    if (deflateState->last_lit != 0) do {
        matchDistance = deflateState->d_buf[iLiteral];
        literalOrLength = deflateState->l_buf[iLiteral++];
        if (matchDistance == 0) {
            send_code(deflateState, literalOrLength, literalTree); /* send a literal byte */
            Tracecv(isgraph(literalOrLength), (stderr," '%c' ", literalOrLength));
        } else {
            /* Here, literalOrLength is the match length - MIN_MATCH */
            code = _length_code[literalOrLength];
            send_code(deflateState, code+LITERALS+1, literalTree); /* send the length code */
            extraBitCount = extra_lbits[code];
            if (extraBitCount != 0) {
                literalOrLength -= base_length[code];
                send_bits(deflateState, literalOrLength, extraBitCount);       /* send the extra length bits */
            }
            matchDistance--; /* dist is now the match distance - 1 */
            code = d_code(matchDistance);
            Assert (code < D_CODES, "bad d_code");

            send_code(deflateState, code, distanceTree);       /* send the distance code */
            extraBitCount = extra_dbits[code];
            if (extraBitCount != 0) {
                matchDistance -= (unsigned)base_dist[code];
                send_bits(deflateState, matchDistance, extraBitCount);   /* send the extra distance bits */
            }
        } /* literal or match pair ? */

        /* Check that the overlay between pending_buf and d_buf+l_buf is ok: */
        Assert((uInt)(deflateState->pending) < deflateState->lit_bufsize + 2*iLiteral,
               "pendingBuf overflow");

    } while (iLiteral < deflateState->last_lit);

    send_code(deflateState, END_BLOCK, literalTree);
}

/* ===========================================================================
 * Check if the data type is TEXT or BINARY, using the following algorithm:
 * - TEXT if the two conditions below are satisfied:
 *    a) There are no non-portable control characters belonging to the
 *       "black list" (0..6, 14..25, 28..31).
 *    b) There is at least one printable character belonging to the
 *       "white list" (9 {TAB}, 10 {LF}, 13 {CR}, 32..255).
 * - BINARY otherwise.
 * - The following partially-portable control characters form a
 *   "gray list" that is ignored in this detection algorithm:
 *   (7 {BEL}, 8 {BS}, 11 {VT}, 12 {FF}, 26 {SUB}, 27 {ESC}).
 * IN assertion: the fields Freq of dyn_ltree are set.
 */
local int detect_data_type(deflateState)
    deflate_state *deflateState;
{
    /* binaryControlMask is the bit mask of black-listed bytes
     * set bits 0..6, 14..25, and 28..31
     * 0xf3ffc07f = binary 11110011111111111100000001111111
     */
    unsigned long binaryControlMask = 0xf3ffc07fUL;
    int iNode;

    /* Check for non-textual ("black-listed") bytes. */
    for (iNode = 0; iNode <= 31; iNode++, binaryControlMask >>= 1)
        if ((binaryControlMask & 1) && (deflateState->dyn_ltree[iNode].Freq != 0))
            return Z_BINARY;

    /* Check for textual ("white-listed") bytes. */
    if (deflateState->dyn_ltree[9].Freq != 0 || deflateState->dyn_ltree[10].Freq != 0
            || deflateState->dyn_ltree[13].Freq != 0)
        return Z_TEXT;
    for (iNode = 32; iNode < LITERALS; iNode++)
        if (deflateState->dyn_ltree[iNode].Freq != 0)
            return Z_TEXT;

    /* There are no "black-listed" or "white-listed" bytes:
     * this stream either is empty or has tolerated ("gray-listed") bytes only.
     */
    return Z_BINARY;
}

/* ===========================================================================
 * Reverse the first bitLength bits of a code, using straightforward code (a faster
 * method would use a table)
 * IN assertion: 1 <= bitLength <= 15
 */
local unsigned bi_reverse(code, bitLength)
    unsigned code; /* the value to invert */
    int bitLength;       /* its bit length */
{
    register unsigned reversedCode = 0;
    do {
        reversedCode |= code & 1;
        code >>= 1, reversedCode <<= 1;
    } while (--bitLength > 0);
    return reversedCode >> 1;
}

/* ===========================================================================
 * Flush the bit buffer, keeping at most 7 bits in it.
 */
local void bi_flush(deflateState)
    deflate_state *deflateState;
{
    if (deflateState->bi_valid == 16) {
        put_short(deflateState, deflateState->bi_buf);
        deflateState->bi_buf = 0;
        deflateState->bi_valid = 0;
    } else if (deflateState->bi_valid >= 8) {
        put_byte(deflateState, (Byte)deflateState->bi_buf);
        deflateState->bi_buf >>= 8;
        deflateState->bi_valid -= 8;
    }
}

/* ===========================================================================
 * Flush the bit buffer and align the output on a byte boundary
 */
local void bi_windup(deflateState)
    deflate_state *deflateState;
{
    if (deflateState->bi_valid > 8) {
        put_short(deflateState, deflateState->bi_buf);
    } else if (deflateState->bi_valid > 0) {
        put_byte(deflateState, (Byte)deflateState->bi_buf);
    }
    deflateState->bi_buf = 0;
    deflateState->bi_valid = 0;
#ifdef ZLIB_DEBUG
    deflateState->bits_sent = (deflateState->bits_sent+7) & ~7;
#endif
}
