/*
 * WriteBuffer.h
 * Copyright(C) 2009 Data Robotics. All rights reserved
 */

#ifndef __WRITEBUFFER_H
#define __WRITEBUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

//#define ISCSI_TGT_WRITE_BUFFER_DEBUG 1
#undef ISCSI_TGT_WRITE_BUFFER_DEBUG

#define WRITE_BUFFER_MARK_FREED_BUFFERS 0

#if WRITE_BUFFER_MARK_FREED_BUFFERS
#define WRITE_JOURNAL_FREED_BUFFER_BEGIN_MAGIC_MARKER 0xBEEF0000
#define WRITE_JOURNAL_FREED_BUFFER_END_MAGIC_MARKER 0xDEAD0000
#define WRITE_JOURNAL_ALLOCATED_BUFFER_BEGIN_MAGIC_MARKER 0xCAFE0000
#define WRITE_JOURNAL_ALLOCATED_BUFFER_END_MAGIC_MARKER 0xFACE0000
#endif

#define SHARED_PARTITION_BLOCK_SIZE 512

#define SHARED_PARTITION_COALESCE_HIGH_THRESHOLD_IN_BLOCKS ((((64 * 1024)/ SHARED_PARTITION_BLOCK_SIZE) + 1) * 3)

#define WRITE_JOURNAL_WAIT_TIME_1X_INDEX 0
#define WRITE_JOURNAL_WAIT_TIME_2X_INDEX 1
#define WRITE_JOURNAL_WAIT_TIMES_NUM     2

typedef struct SharedPartitionHeader
{
  uint32_t alloc_size;
  uint32_t num_blocks_in_list;
  struct SharedPartitionHeader *next;
  uint32_t allocated;
  uint32_t intercoreTag; // allows us to correlate the buffer to the cmd, purely for diagnostic purposes
  uint8_t space[SHARED_PARTITION_BLOCK_SIZE - (5 * sizeof(uint32_t))];
} SharedPartitionHeader;

typedef struct SharedPartitionInfoStruct
{
  /* Core 0 private data */
  uint8_t core0private [100];

  /* Fields common to all cores */
  SharedPartitionHeader *SharedPartitionDataStart;
  SharedPartitionHeader *SharedPartitionFreeList;
  uint32_t SharedPartitionSize;
  uint32_t SharedPartitionDataRegionSize;
  uint32_t SharedPartitionDataRegionNumBlocks;
  uint32_t SharedPartitionBlocksUsed;
  uint32_t allocFailedCount;
  uint32_t allocMultipleFailureCount;
  uint32_t highestFailureCount;

  uint8_t core1private [412 - 36];        /* Unused core1 private space */

} SharedPartitionInfoStruct;

#ifdef __cplusplus
}
#endif

#endif

