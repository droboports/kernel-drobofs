#ifndef _SHARED_MEM_QUEUE_H_
#define _SHARED_MEM_QUEUE_H_

#include "ctrlEnv/mvCtrlEnvSpec.h"

#define SHM_DB_CAUSE_REG(cpu)        (CPU_IF_BASE(cpu) + 0x400)
#define SHM_DB_MASK_REG(cpu)         (CPU_IF_BASE(cpu) + 0x404) 

#define SHM_DOORBELL_SET(remote_cpu, doorbell)           \
            MV_REG_WRITE( SHM_DB_CAUSE_REG(remote_cpu), (1 << doorbell) )

#define SHM_DOORBELL_CLEAR(local_cpu, doorbell)          \
            MV_REG_WRITE( SHM_DB_CAUSE_REG(local_cpu), ~(1 << doorbell) )

#define SHM_DOORBELL_DISABLE(local_cpu, doorbell)        \
            MV_REG_BIT_RESET(SHM_DB_MASK_REG(local_cpu), (1 << doorbell) )

#define SHM_DOORBELL_ENABLE(local_cpu, doorbell)         \
            MV_REG_BIT_SET(SHM_DB_MASK_REG(local_cpu), (1 << doorbell) )

#define SHM_DOORBELL_VXWORKS_CPU_ID		0x0
#define SHM_DOORBELL_LINUX_CPU_ID		0x1

typedef struct _shm_queue_pair_os_info
{
	SHMQueuePairInfoStatus *shmQPairPtr;
	unsigned char *slot_buf_ptr_lx_to_vx;
	unsigned char *slot_buf_ptr_vx_to_lx;
	wait_queue_head_t recv_vx_to_lx_wait_queue_head;
	wait_queue_head_t send_lx_to_vx_wait_queue_head;
	struct semaphore q_lx_to_vx_mutex;
	struct semaphore q_vx_to_lx_mutex;
} shm_queue_pair_os_info;

typedef enum _shm_queue_bit_shift_type
{
	BIT_SHIFT_NONE,
	BIT_SHIFT_LX_TO_VX,
	BIT_SHIFT_VX_TO_LX
} shm_queue_bit_shift_type;

typedef struct _shm_q_bit_shift_to_queue_index
{
	shm_queue_bit_shift_type type;
	int index;
	wait_queue_head_t *wq_to_signal;
} shm_q_bit_shift_to_queue_index;

typedef struct _shm_queue_os_info
{
	int cores_synced_for_queues;
	SHMQueueHeader *queueHeader;
	unsigned char *baseSlotBufferPtr;
	struct semaphore os_info_sem;
	shm_queue_pair_os_info	q_pair_os_info[SHM_MAX_QUEUE_PAIRS];
	shm_q_bit_shift_to_queue_index db_to_q_index[SHM_QUEUE_MAX_DOORBELL_BIT_SHIFT+1];
} shm_queue_os_info;


#endif
