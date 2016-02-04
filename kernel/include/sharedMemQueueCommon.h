#ifndef _SHARED_MEM_QUEUE_COMMON_H_
#define _SHARED_MEM_QUEUE_COMMON_H_


/* SOME NOTES:
 *  - The initial synchronization method between the two cores will follow similar 
 *    mechanism as the partitioning except there will be no timeout involved.
 *	- Queues can only be created as a Queue pair. Meaning every time you create a queue, 
 *	  a VxWorks to Linux Send and VxWorks to Linux Recv is created.
 *  - Queues can only be created by the VxWorks core side.
 *  - Linux Core can only attach to existing queues.
 *  - The above 2 restrictions are so I can only have one core write to the queue
 *    infrastructure data structures. This allows me to get away without inter-core
 *    locking since VxWorks core reads/writes and Linux core only reads the queue
 *    insfrastructure data structures
 *  - Currently queue infrastructure is assumed invalide on every boot. Meaning it 
 *    will be initialized every boot. So no replay of commands in the queue.
 *  - Each Queue slot in each direction will have a header which is 32-bits long.
 *    This header will tell who owns the slot.
 *  - The above point about header also means that each queue slot will allocate 4
 *    extra bytes per slot.
 *  - Since Queues are only created on the VxWorks core, currently the allocator
 *    code for the slot buffers only exists on the VxWorks side.
 */

#define SHM_QUEUE_SPECIAL_PANIC_DB_BITSHIFT     31

#define SHM_QUEUE_CPU1_READY_TO_WAIT_MARKER		0xDEADBEEF
#define SHM_QUEUE_CPU0_CLEARED_TO_WAIT_MARKER	0xBEADCAFE
#define SHM_QUEUE_CPU0_INIT_IN_PROGRESS_MARKER	0xFACEBEAD
#define SHM_QUEUE_CPU0_INIT_DONE_MARKER			0xCAFEDEAD

#define SHM_MAX_QUEUE_PAIRS							15
#define SHM_QUEUE_PAIR_NAME_MAX_SIZE				16
#define SHM_QUEUE_HEADER_MAGIC						0x53514844
#define SHM_QUEUE_HEADER_MAX_SIZE					(4 * 1024)

#define SHM_QUEUE_PAIR_INVALID_ID					-1

#define SHM_QUEUE_DESCRIPTION_MAX_SIZE				32
#define SHM_QUEUE_INVALID_DB_BIT_SHIFT				-1
#define SHM_QUEUE_MAX_DOORBELL_BIT_SHIFT			31

#define SHM_QUEUE_SLOT_OWNER_LINUX					0x33
#define SHM_QUEUE_SLOT_OWNER_VXWORKS				0x44
#define SHM_QUEUE_SLOT_MAGIC						0xCC

/* The header should be written as a single 32 bit value when
 * ownership is changed
 */
typedef struct _SHMQueueSlotHeader
{
	unsigned char magic;			/* magic to identify the header */
	unsigned char owner;			/* Either SHM_QUEUE_SLOT_OWNER_LINUX or SHM_QUEUE_SLOT_OWNER_VXWORKS */
	unsigned short size;			/* Actual Size of the message in the slot */
} SHMQueueSlotHeader;


#define SHM_QUEUE_SET_SLOT_HEADER(headerPtr, magic, owner, size)	(*((volatile unsigned int *) (headerPtr)) = ((magic) | ((owner) << 8) | ((size) << 16)))
#define SHM_QUEUE_GET_SLOT_HEADER_MAGIC(headerPtr)	(((volatile SHMQueueSlotHeader *) (headerPtr))->magic)
#define SHM_QUEUE_GET_SLOT_HEADER_OWNER(headerPtr)	(((volatile SHMQueueSlotHeader *) (headerPtr))->owner)
#define SHM_QUEUE_GET_SLOT_HEADER_MSG_SIZE(headerPtr)	(((volatile SHMQueueSlotHeader *) (headerPtr))->size)

typedef struct _SHMQueueInfo
{
	int doorbellBitShift;						/* Bit Shift for the doorbell to use for this queue */
	unsigned int numSlots;						/* Number of slots for this queue */
	unsigned int sizeSlots;						/* Size of each slot for this queue */
	unsigned int nextSlotToProcess;				/* Next slot to take out from the queue */
	unsigned int nextSlotToInsert;				/* Next slot to insert into the queue */
	unsigned int slotBufferSize;				/* Size of the slot buffers */
	unsigned int slotBufferOffset;				/* Offset from base SHMQ to buffer for the slots */
	char queueDescription[SHM_QUEUE_DESCRIPTION_MAX_SIZE];		/* Queue Description */
} SHMQueueInfo;

typedef struct _SHMQueuePairInfoStatus
{
	int qPairId;											/* ID of the queue pair. -1 means unallocated */
	char queuePairName[SHM_QUEUE_PAIR_NAME_MAX_SIZE];		/* Unique name ID for the queue pair */
	SHMQueueInfo qVxToLx;									/* VxWorks to Linux direction queue info */
	SHMQueueInfo qLxToVx;									/* Linux to VxWorks direction queue info */
} SHMQueuePairInfoStatus;


typedef struct _SHMQueueHeader
{
	unsigned int cpu1_queue_wait_status;
	unsigned int cpu0_queue_init_status;
	unsigned int queueHeaderMagic;
	unsigned int nextChId;
	unsigned int slotBufferPoolSize;
	unsigned int slotBufferPoolOffset;
	SHMQueuePairInfoStatus queuePairInfo[SHM_MAX_QUEUE_PAIRS];
} SHMQueueHeader;

/************************* SPECIFIC QUEUE INFO ****************************/
/* This is mostly the identfication info for the queues that we will create
 * between the two cores
 */
#define ISCSI_SCSI_QUEUE_PAIR_NAME				"ScsiQPair"
#define ISCSI_SCSI_CMD_QUEUE_DESC				"ScsiCmdQueue"
#define ISCSI_SCSI_RESPONSE_QUEUE_DESC			"ScsiRespQueue"

#define ISCSI_RESOURCE_QUEUE_PAIR_NAME			"ResMsgQPair"
#define ISCSI_VX_RESOURCE_QUEUE_DESC			"VxResMsgQueue"
#define ISCSI_LX_RESOURCE_QUEUE_DESC			"LxResMsgQueue"

#define CORE_MSG_QUEUE_PAIR_NAME				"CoreMsgQPair"
#define CORE_LX_TO_VX_QUEUE_DESC				"LxToVxMsgQueue"
#define CORE_VX_TO_LX_QUEUE_DESC				"VxToLxMsgQueue"
/************************* SPECIFIC QUEUE INFO ****************************/
#endif
