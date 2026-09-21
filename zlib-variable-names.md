# Deflate/inflate variable naming

Descriptive camelCase local variables and parameters in eight C files. Names are chosen by function context. Exported function names, public headers, structure members, layouts, types, expressions, and control flow are preserved. Related comments are updated.

Scope: deflate/inflate setup, execution, reset, copy and teardown; match search; Huffman encoding and decoding; Adler-32 and CRC-32; memory helpers. The separate inflateBack API and gzip file wrappers are outside this rename.

Validation uses Visual Studio 2026 x64 (MSVC 14.51): release, ZLIB_DEBUG, an alternate build with FASTEST/NO_DIVIDE/DYNAMIC_CRC_TABLE/BUILDFIXED/INFLATE_STRICT, and a fallback build with NO_MEMCPY/NOBYFOUR/UNALIGNED_OK.

The existing test/example.c and test/infcover.c programs pass against both original and renamed builds in all four configurations. The DLL .text sections are byte-for-byte identical in all three optimized configurations. Legacy platform compilers and optional assembly implementations were not tested.

## Rename reference

### deflate.c

| Original | Renamed |
|---|---|
| `avail` | `savedInputAvailable` |
| `b` | `shortValue` |
| `beg` | `checksumStart` |
| `best_len` | `bestMatchLength` |
| `bflush` | `shouldFlushBlock` |
| `bstate` | `blockResult` |
| `buf` | `destination` |
| `chain_length` | `chainSearchRemaining` |
| `complen` | `compressedSizeBound` |
| `copy` | `extraBytesToCopy` |
| `cur_match` | `currentMatchPosition` |
| `curr` | `dataEndPosition` |
| `dest` | `destinationStream` |
| `dictLength` | `dictionaryLength` |
| `ds` | `destinationState` |
| `err` | `resultCode` |
| `flush` | `flushMode` |
| `func` | `compressionFunction` |
| `good_length` | `goodMatchLength` |
| `hash_head` | `hashChainHead` |
| `have` | `bufferByteCount` |
| `head` | `gzipHeader` |
| `init` | `bytesToInitialize` |
| `last` | `isLastBlock` |
| `left` | `extraBytesRemaining` |
| `left` | `windowBytesRemaining` |
| `len` | `bytesToFlush` |
| `len` | `bytesToRead` |
| `len` | `dictionaryBytes` |
| `len` | `matchLength` |
| `len` | `storedBlockLength` |
| `length` | `matchLength` |
| `level_flags` | `compressionLevelFlags` |
| `m` | `matchPosition` |
| `match` | `matchNext` |
| `match` | `matchPosition` |
| `max_chain` | `maximumChainLength` |
| `max_insert` | `lastInsertPosition` |
| `max_lazy` | `maximumLazyMatch` |
| `memLevel` | `memoryLevel` |
| `min_block` | `minimumBlockSize` |
| `more` | `windowSpaceAvailable` |
| `n` | `bytesRead` |
| `n` | `entriesRemaining` |
| `n` | `positionsRemaining` |
| `next` | `savedInputNext` |
| `nice_length` | `sufficientMatchLength` |
| `nice_match` | `sufficientMatchLength` |
| `old_flush` | `previousFlush` |
| `overlay` | `bufferOverlay` |
| `p` | `hashEntry` |
| `prev` | `previousPositions` |
| `prev` | `repeatedByte` |
| `put` | `bitsToInsert` |
| `ret` | `resultCode` |
| `s` | `deflateState` |
| `scan` | `scanNext` |
| `scan_end` | `scanEndValue` |
| `scan_end1` | `scanPreviousEndByte` |
| `scan_start` | `scanStartWord` |
| `size` | `destinationCapacity` |
| `source` | `sourceStream` |
| `ss` | `sourceState` |
| `start` | `sourcePosition` |
| `str` | `headerText` |
| `str` | `insertPosition` |
| `stream_size` | `streamSize` |
| `strend` | `scanEnd` |
| `strm` | `stream` |
| `used` | `inputBytesCopied` |
| `val` | `headerByte` |
| `wmask` | `windowMask` |
| `wrap` | `wrapperMode` |
| `wraplen` | `wrapperSize` |
| `wsize` | `windowSize` |
### inflate.c

| Original | Renamed |
|---|---|
| `bits` | `bitCount` |
| `buf` | `bufferedBytes` |
| `buf` | `inputBytes` |
| `copy` | `copyLength` |
| `copy` | `destinationState` |
| `dest` | `destinationStream` |
| `dictLength` | `dictionaryLength` |
| `dictid` | `dictionaryChecksum` |
| `dist` | `windowTailBytes` |
| `flush` | `flushMode` |
| `from` | `matchSource` |
| `got` | `matchedBytes` |
| `have` | `inputBytesAvailable` |
| `have` | `matchedByteCount` |
| `hbuf` | `headerChecksumBytes` |
| `head` | `gzipHeader` |
| `here` | `currentEntry` |
| `hold` | `bitBuffer` |
| `in` | `inputByteCount` |
| `in` | `savedTotalInput` |
| `last` | `parentEntry` |
| `left` | `outputBytesAvailable` |
| `len` | `bytesSearched` |
| `len` | `decodeLength` |
| `len` | `inputLength` |
| `next` | `iInputByte` |
| `next` | `inputNext` |
| `next` | `nextTableEntry` |
| `out` | `outputByteCount` |
| `out` | `savedTotalOutput` |
| `put` | `outputNext` |
| `ret` | `resultCode` |
| `source` | `sourceStream` |
| `state` | `inflateState` |
| `strm` | `stream` |
| `sym` | `iSymbol` |
| `virgin` | `fixedTablesUninitialized` |
| `wrap` | `wrapperMode` |
| `wsize` | `windowSize` |
### inffast.c

| Original | Renamed |
|---|---|
| `beg` | `outputStart` |
| `bits` | `bitCount` |
| `dcode` | `distanceTable` |
| `dist` | `matchDistance` |
| `dmask` | `distanceTableMask` |
| `dmax` | `maximumDistance` |
| `end` | `outputFastLimit` |
| `from` | `matchSource` |
| `here` | `currentEntry` |
| `hold` | `bitBuffer` |
| `in` | `inputNext` |
| `last` | `inputFastLimit` |
| `lcode` | `lengthTable` |
| `len` | `matchLength` |
| `lmask` | `lengthTableMask` |
| `op` | `decodeValue` |
| `out` | `outputNext` |
| `start` | `initialOutputAvailable` |
| `state` | `inflateState` |
| `strm` | `stream` |
| `whave` | `windowBytesAvailable` |
| `wnext` | `windowWriteIndex` |
| `wsize` | `windowSize` |
### inftrees.c

| Original | Renamed |
|---|---|
| `base` | `baseValues` |
| `bits` | `rootBitCount` |
| `codes` | `symbolCount` |
| `count` | `codeLengthCounts` |
| `curr` | `currentTableBits` |
| `dbase` | `distanceBaseValues` |
| `dext` | `distanceExtraBits` |
| `drop` | `bitsToDrop` |
| `extra` | `extraBitValues` |
| `fill` | `iReplicatedEntry` |
| `here` | `currentEntry` |
| `huff` | `huffmanCode` |
| `incr` | `codeIncrement` |
| `lbase` | `lengthBaseValues` |
| `left` | `availablePrefixCodes` |
| `len` | `codeLength` |
| `lens` | `codeLengths` |
| `lext` | `lengthExtraBits` |
| `low` | `rootEntryBits` |
| `mask` | `rootTableMask` |
| `match` | `firstMatchSymbol` |
| `max` | `maximumCodeLength` |
| `min` | `minimumCodeLength` |
| `next` | `nextTable` |
| `offs` | `codeLengthOffsets` |
| `root` | `rootTableBits` |
| `sym` | `iSymbol` |
| `used` | `tableEntriesUsed` |
| `work` | `sortedSymbols` |
### trees.c

| Original | Renamed |
|---|---|
| `base` | `extraBitsBase` |
| `black_mask` | `binaryControlMask` |
| `blcodes` | `bitLengthCodeCount` |
| `curlen` | `currentCodeLength` |
| `dcode` | `distanceCode` |
| `dcodes` | `distanceCodeCount` |
| `desc` | `treeDescriptor` |
| `dist` | `matchDistance` |
| `dtree` | `distanceTree` |
| `extra` | `extraBitCount` |
| `extra` | `extraBitsByCode` |
| `f` | `frequency` |
| `h` | `iHeap` |
| `in_length` | `inputLength` |
| `j` | `iHeapChild` |
| `k` | `iHeapParent` |
| `last` | `isLastBlock` |
| `lc` | `literalOrLength` |
| `lcodes` | `literalCodeCount` |
| `len` | `bitLength` |
| `len` | `codeLength` |
| `ltree` | `literalTree` |
| `lx` | `iLiteral` |
| `m` | `secondNode` |
| `max_blindex` | `lastBitLengthIndex` |
| `max_code` | `lastUsedCode` |
| `max_count` | `maximumRepeatCount` |
| `max_length` | `maximumCodeLength` |
| `min_count` | `minimumRepeatCount` |
| `n` | `iNode` |
| `next_code` | `nextCodeByLength` |
| `nextlen` | `nextCodeLength` |
| `opt_lenb` | `optimalByteLength` |
| `out_length` | `estimatedOutputBits` |
| `prevlen` | `previousCodeLength` |
| `res` | `reversedCode` |
| `s` | `deflateState` |
| `static_lenb` | `staticByteLength` |
| `stored_len` | `storedLength` |
| `stree` | `staticTree` |
| `v` | `heapNode` |
| `val` | `bitValue` |
| `xbits` | `extraBitCount` |
### adler32.c

| Original | Renamed |
|---|---|
| `adler1` | `firstChecksum` |
| `adler2` | `secondChecksum` |
| `buf` | `inputBytes` |
| `i` | `iByte` |
| `len` | `inputLength` |
| `len2` | `secondLength` |
| `n` | `blocksRemaining` |
| `rem` | `lengthRemainder` |
| `sum1` | `byteSum` |
| `sum2` | `weightedByteSum` |
| `tmp` | `upperBits` |
### crc32.c

| Original | Renamed |
|---|---|
| `buf` | `inputBytes` |
| `buf4` | `inputWords` |
| `c` | `crcValue` |
| `crc1` | `firstChecksum` |
| `crc2` | `secondChecksum` |
| `endian` | `byteOrderProbe` |
| `first` | `tableUninitialized` |
| `k` | `iBit` |
| `len` | `inputLength` |
| `len2` | `secondLength` |
| `mat` | `matrix` |
| `n` | `iRow` |
| `n` | `iTableEntry` |
| `p` | `polynomialPowers` |
| `poly` | `polynomial` |
| `sum` | `matrixProduct` |
| `vec` | `vector` |
### zutil.c

| Original | Renamed |
|---|---|
| `dest` | `destination` |
| `err` | `errorCode` |
| `items` | `itemCount` |
| `j` | `iByte` |
| `len` | `byteCount` |
| `opaque` | `allocatorContext` |
| `ptr` | `allocation` |
| `s1` | `firstBytes` |
| `s2` | `secondBytes` |