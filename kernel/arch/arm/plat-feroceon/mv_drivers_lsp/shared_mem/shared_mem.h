#ifndef _SHARED_MEM_H_
#define _SHARED_MEM_H_

#define SHARED_MEM_MAJOR 223
#define SHARED_MEM_NAME "shared_mem"

#define SHARED_MEMORY_HEADER_PHYS_ADDR			((512 * 1024 * 1024) - SHARED_MEM_HEADER_PARTITION_SIZE)
#define SHARED_MEMORY_BASE_PHYS_ADDR			((512 * 1024 * 1024) - SHARED_MEM_TOTAL_SHARED_MEM)
#define SHARED_MEMORY_HEADER_OFFSET_FROM_BASE	(SHARED_MEMORY_HEADER_PHYS_ADDR - SHARED_MEMORY_BASE_PHYS_ADDR)

typedef struct _shared_mem_region_os_info
{
	char tag[4];
	unsigned int offset;
	unsigned int physAddr;
	unsigned int virtKernelAddr;
	unsigned int virtUserAddr;
	unsigned int regionSize;
	unsigned int reserved;
	unsigned int num_mmaps;
} shared_mem_region_os_info;


typedef struct _shared_mem_os_info
{
	unsigned int virt_kernel_header_base;
	unsigned int phys_header_base;
	unsigned int num_regions;
	shared_mem_region_os_info region_info[SHARED_MEM_MAX_PARTITIONS];
} shared_mem_os_info;

typedef struct _shared_mem_dev
{
	void *sharedMemBaseAddr;
	SharedMemPartitionDescHeader *sharedMemHeader;
	int driver_opened_once;
	unsigned int open_count;
	struct semaphore shm_open_mutex;
	shm_queue_os_info queue_os_info;
	shared_mem_os_info  *os_info;	/* This needs to become a list protected by something to allow multiple opens */
	struct cdev cdev;
} shared_mem_dev;

#endif
