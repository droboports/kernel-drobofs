#ifndef _SHARED_MEM_COMMON_H_
#define _SHARED_MEM_COMMON_H_

#define INCLUDE_J2_IN_SHARED_MEMORY_PARTITIONS	1

#define SHARED_MEM_CPU1_READY_TO_WAIT_MARKER	0xDEADBEEF
#define SHARED_MEM_CPU0_CLEARED_TO_WAIT_MARKER	0xBEADCAFE
#define SHARED_MEM_CPU0_INIT_IN_PROGRESS_MARKER	0xFACEBEAD
#define SHARED_MEM_CPU0_INIT_DONE_MARKER		0xCAFEDEAD

#define SHARED_MEM_MAX_PARTITIONS				32
#define SHARED_MEM_MAX_PARTITION_DESC_SIZE		64
#define SHARED_MEM_HEADER_MAGIC					0x53484D50
#define SHARED_MEM_HEADER_VERSION_1				1
#define SHARED_MEM_HEADER_PARTITION_SIZE		(4 * 1024)
#define SHARED_MEM_TOTAL_SHARED_MEM				(192 * 1024 * 1024)
#define SHARED_MEM_HEADER_CUR_VERSION			SHARED_MEM_HEADER_VERSION_1

#define PARTITION_TAG_SHARED_MEM_QUEUES_POOL	"SHMQ"
#define PARTITION_DESC_SHARED_MEM_QUEUES_POOL	"Memory Pool for Allocating Queue Resources"
#define PARTITION_SIZE_SHARED_MEM_QUEUES_POOL	(2 * 1024 * 1024)
#define PARTITION_ALIGN_SHARED_MEM_QUEUES_POOL	(4 * 1024)
#define PARTITION_TAG_J1						"SHJ1"
#define PARTITION_DESC_J1						"J1 for Write IO Buffers"
#define PARTITION_SIZE_J1						(64 * 1024 * 1024)
#define PARTITION_ALIGN_J1						(4 * 1024)
#define PARTITION_TAG_HLBAT_CACHE				"HLRC"
#define PARTITION_DESC_HLBAT_CACHE				"HLBAT Read Cache"
#define PARTITION_SIZE_HLBAT_CACHE				(96 * 1024 * 1024)
#define PARTITION_ALIGN_HLBAT_CACHE				(4 * 1024)
#define PARTITION_TAG_LX_CORE_DYN_SHARED_MEM	"SHLX"
#define PARTITION_DESC_LX_CORE_DYN_SHARED_MEM	"Linux Core Dynamically Allocated Shared Memory Pool"
#define PARTITION_SIZE_LX_CORE_DYN_SHARED_MEM	(2 * 1024 * 1024)
#define PARTITION_ALIGN_LX_CORE_DYN_SHARED_MEM	(4 * 1024)
#define PARTITION_TAG_VX_CORE_DYN_SHARED_MEM	"SHVX"
#define PARTITION_DESC_VX_CORE_DYN_SHARED_MEM	"VxWorks Core Dynamically Allocated Shared Memory Pool"
#define PARTITION_SIZE_VX_CORE_DYN_SHARED_MEM	(2 * 1024 * 1024)
#define PARTITION_ALIGN_VX_CORE_DYN_SHARED_MEM	(4 * 1024)
#define PARTITION_TAG_SGL_DESC_POOL				"SGLD"
#define PARTITION_DESC_SGL_DESC_POOL			"Memory Pool for Allocating Inter-core IO SGLs"
#define PARTITION_SIZE_SGL_DESC_POOL			(1 * 1024 * 1024)
#define PARTITION_ALIGN_SGL_DESC_POOL			(4 * 1024)
#if INCLUDE_J2_IN_SHARED_MEMORY_PARTITIONS
#define PARTITION_TAG_J2						"SHJ2"
#define PARTITION_DESC_J2						"J2 in Shared Memory Partition for fixed Phys. addr."
#define PARTITION_SIZE_J2                       (4 * 1024 * 1024)
#define PARTITION_ALIGN_J2                      (4 * 1024)
#endif

/* Each partitions description */
typedef struct _SharedMemPartitionInfo
{
	char name_tag[4];		/* Name tag for the partition will be 4 characters long */
	unsigned int offset;	/* Offset from header_base_phys */
	unsigned int alignment;/* Alignment for the starting physical address (This does not necessarily align virtual address */
	unsigned int size;		/* Size in bytes of the partition */
	unsigned int reserved;	/* Reserved for future use probably flags or something */
	char partition_desc[SHARED_MEM_MAX_PARTITION_DESC_SIZE];/* A C string describing the partition */ 
} SharedMemPartitionInfo;

/* Header Description of the partitioning of the shared memory */
typedef struct _SharedMemPartitionDescHeader
{
	unsigned int cpu1_wait_status;
	unsigned int cpu0_init_status;
	unsigned int header_magic;
	unsigned int header_version;	
	unsigned int header_base_phys;
	unsigned int total_shared_mem;
	unsigned int num_partitions;
	SharedMemPartitionInfo partitions[SHARED_MEM_MAX_PARTITIONS];
} SharedMemPartitionDescHeader;

#endif
