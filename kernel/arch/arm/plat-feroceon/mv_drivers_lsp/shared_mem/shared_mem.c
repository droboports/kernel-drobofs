#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/mman.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/mman.h>
#include <linux/slab.h>
#include <linux/ioctl.h>
#include "linux/swap.h"
#include "sharedMemCommon.h"
#include "sharedMemQueueCommon.h"
#include "shared_mem_queue.h"
#include "shared_mem.h"
#include "shared_mem_interface.h"
#include "ctrlEnv/mvCtrlEnvLib.h"

/*______________________________________________________________________________________________*/
/*______________________________________________________________________________________________*/
/*______________________________________________________________________________________________*/
/* QUEUE CODE BEGIN																				*/
/*______________________________________________________________________________________________*/
/*______________________________________________________________________________________________*/
/*______________________________________________________________________________________________*/

int find_region_from_tag(shared_mem_os_info *os_info, SharedMemGetRegionOffset *offset);

static irqreturn_t shm_doorbell_isr( int irq , void *dev_id )
{
    unsigned int cause;
#if 0	
	int i;
#else
	int db;	
#endif
	shm_queue_os_info *queue_os_info = (shm_queue_os_info *) dev_id;

    cause = MV_REG_READ(SHM_DB_CAUSE_REG(SHM_DOORBELL_LINUX_CPU_ID)) & 
			MV_REG_READ(SHM_DB_MASK_REG(SHM_DOORBELL_LINUX_CPU_ID));

    //printk("shmDoorbellISR: cause=0x%lx\n", (unsigned long int) cause);

    if (cause) 
    {
        /* Clear cause */
        MV_REG_WRITE( SHM_DB_CAUSE_REG(SHM_DOORBELL_LINUX_CPU_ID), ~cause);
#if 0		
		for(i = 0; (i < SHM_MAX_QUEUE_PAIRS) && cause; i++)
		{
			if(queue_os_info->q_pair_os_info[i].shmQPairPtr)
			{
				if(queue_os_info->q_pair_os_info[i].shmQPairPtr->qPairId != SHM_QUEUE_PAIR_INVALID_ID)
				{
					if (cause & (1 << queue_os_info->q_pair_os_info[i].shmQPairPtr->qLxToVx.doorbellBitShift))
					{
						cause &= ~(1 << queue_os_info->q_pair_os_info[i].shmQPairPtr->qLxToVx.doorbellBitShift);
						wake_up_interruptible(&(queue_os_info->q_pair_os_info[i].send_lx_to_vx_wait_queue_head));
					}
					if (cause & (1 << queue_os_info->q_pair_os_info[i].shmQPairPtr->qVxToLx.doorbellBitShift))
					{
						wake_up_interruptible(&(queue_os_info->q_pair_os_info[i].recv_vx_to_lx_wait_queue_head));
						cause &= ~(1 << queue_os_info->q_pair_os_info[i].shmQPairPtr->qVxToLx.doorbellBitShift);
					}				
				}
			}
		}
#else
		db = 0;
		while(cause)
		{
			if(cause & (1 << db))
			{
				if( (queue_os_info->db_to_q_index[db].type == BIT_SHIFT_VX_TO_LX) ||
				    (queue_os_info->db_to_q_index[db].type == BIT_SHIFT_LX_TO_VX) )
				{
					if(queue_os_info->db_to_q_index[db].wq_to_signal)
					{
						wake_up_interruptible(queue_os_info->db_to_q_index[db].wq_to_signal);
					}					
				}
				cause &= ~(1 << db);
			}
			db++;
		}
#endif
    }

	return IRQ_HANDLED;
}

int shm_doorbell_init(shm_queue_os_info *queue_os_info)
{
	int i;
	
	/* Mask all interrupts to start off */
	for(i = 0; i <= SHM_QUEUE_MAX_DOORBELL_BIT_SHIFT; i++)
	{
		SHM_DOORBELL_DISABLE(SHM_DOORBELL_VXWORKS_CPU_ID, i);
		SHM_DOORBELL_CLEAR(SHM_DOORBELL_VXWORKS_CPU_ID, i);
	}

	/* Hook up the interrupts and turn them on */
    if( request_irq(DOORBELL_IN_IRQ, 
					shm_doorbell_isr, 
					(IRQF_DISABLED | IRQF_SAMPLE_RANDOM) ,  
                    SHARED_MEM_NAME, 
					(void *) queue_os_info) ) 
    {
		printk("SHARED_MEM: shm_doorbell_init: Unable to get an irq for doorbell in interrupt: 0x%lx\n", (unsigned int long) DOORBELL_IN_IRQ);
        return -1;
    }
	return 0;
}

void shm_queue_init(shm_queue_os_info *queue_os_info)
{
	unsigned int i;
	queue_os_info->cores_synced_for_queues = 0;
	queue_os_info->queueHeader = NULL;
	queue_os_info->baseSlotBufferPtr = NULL;	
	init_MUTEX(&(queue_os_info->os_info_sem));
	memset(queue_os_info->q_pair_os_info, 0x0, sizeof(queue_os_info->q_pair_os_info));
	shm_doorbell_init(queue_os_info);
	for(i = 0; i <= SHM_QUEUE_MAX_DOORBELL_BIT_SHIFT; i++)
	{
		queue_os_info->db_to_q_index[i].type = BIT_SHIFT_NONE;
		queue_os_info->db_to_q_index[i].index = SHM_QUEUE_PAIR_INVALID_ID;
		queue_os_info->db_to_q_index[i].wq_to_signal = NULL;
	}
}

/* IMPORTANT NOTE: THIS CAN BLOCK FOREVER IF THERE ARE ANY ISSUES ON THE OTHER CORE */
unsigned int shm_queue_sync_cores(shared_mem_dev *shm_dev)
{
	int retVal;
	SharedMemGetRegionOffset region_offset;
	shm_queue_os_info *queue_os_info = &(shm_dev->queue_os_info);

	if(queue_os_info->cores_synced_for_queues)
	{
		/* If we are already synced just return good */
		return SHMQSTATUS_OK;
	}

	/* First thing is to map the shmq partitions into the kernel memory */
	memcpy(region_offset.tag, PARTITION_TAG_SHARED_MEM_QUEUES_POOL, 4);
	retVal = find_region_from_tag(shm_dev->os_info, &region_offset);

	if(retVal)
	{
		printk("SHARED_MEM: shm_queue_sync_cores: Unable to find shared memory queue partition\n");
		return SHMQSTATUS_GENERIC_ERROR;
	}

	/* Now map the queue partition pages into the kernel space. We don't need it in user space */
	queue_os_info->queueHeader = ioremap_nocache(shm_dev->os_info->phys_header_base - region_offset.offset, PARTITION_SIZE_SHARED_MEM_QUEUES_POOL);

	if(!queue_os_info->queueHeader)
	{
		printk("SHARED_MEM: shm_queue_sync_cores: Unable to map SHMQ partition at phys addr: 0x%lx in kernel mem for size 0x%lx\n", 
				(long unsigned int) (shm_dev->os_info->phys_header_base - region_offset.offset),
				(long unsigned int) PARTITION_SIZE_SHARED_MEM_QUEUES_POOL);
		return SHMQSTATUS_GENERIC_ERROR;
	}

	printk("SHARED_MEM: shm_queue_sync_cores: Mapped SHMQ partition at phys addr: 0x%lx in kernel mem for size 0x%lx\n", 
		(long unsigned int) (shm_dev->os_info->phys_header_base - region_offset.offset),
		(long unsigned int) PARTITION_SIZE_SHARED_MEM_QUEUES_POOL);

	/* First we identify to the other core that we are ready to wait for its init */
	queue_os_info->queueHeader->cpu1_queue_wait_status = SHM_QUEUE_CPU1_READY_TO_WAIT_MARKER;

	printk("SHARED_MEM: CPU1: shm_queue_sync_cores: Ready to wait for init of SHMQ\n");

	/* Wait for the other core to tell us that it is ready to initialize */
	while(queue_os_info->queueHeader->cpu1_queue_wait_status != SHM_QUEUE_CPU0_CLEARED_TO_WAIT_MARKER)
	{
		msleep_interruptible(100);
	}

	printk("SHARED_MEM: CPU1: shm_queue_sync_cores: Queue Init Started CPU0. Waiting on Init Done\n");

	/* Now we wait for the initialization of the shared memory to be done */
	while(queue_os_info->queueHeader->cpu0_queue_init_status != SHM_QUEUE_CPU0_INIT_DONE_MARKER)
	{
		msleep_interruptible(100);
	}

	printk("SHARED_MEM: CPU1: shm_queue_sync_cores: Init of SHMQ done\n");

	/* At this point we can assume the queue partition header has been initialized */

	/* Init our own Data structures. It is important to remember that we do not write
	 * to the header of the shared memory structure ever. Only read it. We do write to
	 * individual queues.
	 */
	queue_os_info->baseSlotBufferPtr = ((unsigned char *) queue_os_info->queueHeader) + queue_os_info->queueHeader->slotBufferPoolOffset;

	/* NOTE: This should be done last since this marks that we are ready to accept attaches and queue specific
	 * commands. So everything else needs to be already inited.
	 */
	queue_os_info->cores_synced_for_queues = 1;

	return SHMQSTATUS_OK;
}

int shm_queue_attach(shared_mem_dev *shm_dev, SharedMemQueueAttach *attach_args)
{
	int retVal;
	int i;
	unsigned int ret;

	shm_queue_os_info *queue_os_info = &(shm_dev->queue_os_info);

	if(!queue_os_info || !attach_args)
	{
		return -EINVAL;
	}

	if(down_interruptible(&queue_os_info->os_info_sem))
	{
		printk("SHARED_MEM: shm_queue_attach: Waiting for os info mutex failed. Most likely signal interruption.\n");
		return -EINTR;
	}	

	attach_args->handle = SHARED_MEM_QUEUE_INVALID_HANDLE;

	if(!queue_os_info->cores_synced_for_queues)
	{
		/* If we are not synced we try to sync on the first attach */
		ret = shm_queue_sync_cores(shm_dev);
		if(ret != SHMQSTATUS_OK)
		{
			printk("SHARED_MEM: shm_queue_attach: Unable to sync for SHMQ with other core\n");
			retVal = -EAGAIN;
			goto shm_queue_attach_error;
		}
	}

	/* At this point we should always be synced between the cores in queue header inited in shm */
	
	/* First check if we already attached */
	for(i = 0; i < SHM_MAX_QUEUE_PAIRS; i++)
	{
		if(queue_os_info->q_pair_os_info[i].shmQPairPtr)
		{
			if (!strncmp(attach_args->qPairName, queue_os_info->q_pair_os_info[i].shmQPairPtr->queuePairName, SHM_QUEUE_PAIR_NAME_MAX_SIZE))
			{
				/* We found an existing qPair which we attached to in the past */
				/* return the index as the handle */
				attach_args->handle = (SharedMemQHandle) i;
				retVal = 0;
				goto shm_queue_attach_error;
			}
		}
	}

	/* We give up the mutex at this point because we may sleep in the following loop */
	up(&queue_os_info->os_info_sem);

	/* Now search for the named queue in a poll wait */
	while(true)
	{
		/* This is inefficient but hopefully at attach points we don't care about efficiency since
		 * it is during init. I can imagine slightly more efficient ways but simple is easy right now.
		 */
		for(i = 0; i < SHM_MAX_QUEUE_PAIRS; i++)
		{
			if(queue_os_info->queueHeader->queuePairInfo[i].qPairId != SHM_QUEUE_PAIR_INVALID_ID)
			{
				if(!strncmp(queue_os_info->queueHeader->queuePairInfo[i].queuePairName, attach_args->qPairName, SHM_QUEUE_PAIR_NAME_MAX_SIZE))
				{
					/* We found the qPair */
					/* Retake the semaphore */
					if(down_interruptible(&queue_os_info->os_info_sem))
					{
						printk("SHARED_MEM: shm_queue_attach: Waiting for os info mutex failed in the queue create wait loop. Most likely signal interruption.\n");
						return -EINTR;
					}
					
					/* Init our data structures */
					queue_os_info->q_pair_os_info[i].shmQPairPtr = &(queue_os_info->queueHeader->queuePairInfo[i]);
					queue_os_info->q_pair_os_info[i].slot_buf_ptr_lx_to_vx = 
						((unsigned char *) queue_os_info->queueHeader) + queue_os_info->queueHeader->queuePairInfo[i].qLxToVx.slotBufferOffset;
					queue_os_info->q_pair_os_info[i].slot_buf_ptr_vx_to_lx = 
						((unsigned char *) queue_os_info->queueHeader) + queue_os_info->queueHeader->queuePairInfo[i].qVxToLx.slotBufferOffset;
					init_MUTEX(&(queue_os_info->q_pair_os_info[i].q_lx_to_vx_mutex));
					init_MUTEX(&(queue_os_info->q_pair_os_info[i].q_vx_to_lx_mutex));
					init_waitqueue_head(&(queue_os_info->q_pair_os_info[i].recv_vx_to_lx_wait_queue_head));
					init_waitqueue_head(&(queue_os_info->q_pair_os_info[i].send_lx_to_vx_wait_queue_head));

					queue_os_info->db_to_q_index[queue_os_info->q_pair_os_info[i].shmQPairPtr->qVxToLx.doorbellBitShift].type = BIT_SHIFT_VX_TO_LX;
					queue_os_info->db_to_q_index[queue_os_info->q_pair_os_info[i].shmQPairPtr->qVxToLx.doorbellBitShift].index = i;
					queue_os_info->db_to_q_index[queue_os_info->q_pair_os_info[i].shmQPairPtr->qVxToLx.doorbellBitShift].wq_to_signal = &(queue_os_info->q_pair_os_info[i].recv_vx_to_lx_wait_queue_head);
					queue_os_info->db_to_q_index[queue_os_info->q_pair_os_info[i].shmQPairPtr->qLxToVx.doorbellBitShift].type = BIT_SHIFT_LX_TO_VX;
					queue_os_info->db_to_q_index[queue_os_info->q_pair_os_info[i].shmQPairPtr->qLxToVx.doorbellBitShift].index = i;
					queue_os_info->db_to_q_index[queue_os_info->q_pair_os_info[i].shmQPairPtr->qLxToVx.doorbellBitShift].wq_to_signal = &(queue_os_info->q_pair_os_info[i].send_lx_to_vx_wait_queue_head);


					SHM_DOORBELL_ENABLE(SHM_DOORBELL_LINUX_CPU_ID, queue_os_info->queueHeader->queuePairInfo[i].qVxToLx.doorbellBitShift);
					SHM_DOORBELL_ENABLE(SHM_DOORBELL_LINUX_CPU_ID, queue_os_info->queueHeader->queuePairInfo[i].qLxToVx.doorbellBitShift);
			
					attach_args->handle = (SharedMemQHandle) i;
					retVal = 0;
					goto shm_queue_attach_error;
				}
			}
		}

		/* we didn't find any qPairs with the given name */
		if(attach_args->flags & SHMQ_ATTACH_FLAG_NO_WAIT)
		{
			/* We just return NULL for the handle but complete the request corrrectly */
			retVal = 0;
			break;
		}

		if(msleep_interruptible(1000))
		{
			retVal = -EINTR;
			printk("SHARED_MEM: shm_queue_attach: Waiting on poll sleep interrupted early. Most likely signal interruption.\n");
			break;
		}
	}

shm_queue_attach_error:
	up(&queue_os_info->os_info_sem);

	return retVal;
}
EXPORT_SYMBOL(shm_queue_attach);

int shm_queue_kern_send_msg(shared_mem_dev *shm_dev, SharedMemQueueSendMsg *send_args)
{
	return shm_queue_send_msg_base(shm_dev, send_args, 0); /* From kern */
}
EXPORT_SYMBOL(shm_queue_kern_send_msg);

int shm_queue_send_msg(shared_mem_dev *shm_dev, SharedMemQueueSendMsg *send_args)
{
	return shm_queue_send_msg_base(shm_dev, send_args, 1); /* From User */
}

int shm_queue_send_msg_base(shared_mem_dev *shm_dev, SharedMemQueueSendMsg *send_args, int from_user)
{
	int retVal;
	volatile SHMQueueSlotHeader *slotHeader = NULL;
	DEFINE_WAIT(wait);

	shm_queue_os_info *queue_os_info = &(shm_dev->queue_os_info);
	shm_queue_pair_os_info *queue_pair_os_info = NULL;

	if(!queue_os_info || !send_args)
	{
		return -EINVAL;
	}

	send_args->status = SHMQSTATUS_GENERIC_ERROR;

	if(!queue_os_info->cores_synced_for_queues)
	{
		printk("SHARED_MEM: shm_queue_send_msg: Cannot send message before we are done syncing between cores\n");
		return -EINVAL;
	}

	if( (send_args->handle <= SHARED_MEM_QUEUE_INVALID_HANDLE) ||
		(send_args->handle >= SHM_MAX_QUEUE_PAIRS) )
	{
		printk("SHARED_MEM: shm_queue_send_msg: %d queue handle is invalid\n", (int) send_args->handle);
		return -EINVAL;
	}

	if(!queue_os_info->q_pair_os_info[send_args->handle].shmQPairPtr)
	{
		/* This queue hasn't been attached to yet */
		printk("SHARED_MEM: shm_queue_send_msg: %d queue handle has not been attached to yet. Please attach first\n", (int) send_args->handle);
		return -EINVAL;
	}
	
	queue_pair_os_info = &(queue_os_info->q_pair_os_info[send_args->handle]);

	if(send_args->msgSize > queue_pair_os_info->shmQPairPtr->qLxToVx.sizeSlots)
	{
		printk("SHARED_MEM: shm_queue_send_msg: %d queue: msg too large: %d. max allowed msg size: %d\n", 
				(int) send_args->handle, (int) send_args->msgSize, (int) queue_pair_os_info->shmQPairPtr->qLxToVx.sizeSlots);
		return -EINVAL;
	}

	/* Take the queue specific mutex */
	if(down_interruptible(&queue_pair_os_info->q_lx_to_vx_mutex))
	{
		printk("SHARED_MEM: shm_queue_send_msg: Waiting for  %d queue pair mutex failed. Most likely signal interruption.\n", (int) send_args->handle);
		return -EINTR;
	}
	
	/* Check if the next slot to insert is owned by us */
	slotHeader = (SHMQueueSlotHeader *) (queue_pair_os_info->slot_buf_ptr_lx_to_vx + 
										( queue_pair_os_info->shmQPairPtr->qLxToVx.nextSlotToInsert * 
										  (queue_pair_os_info->shmQPairPtr->qLxToVx.sizeSlots + sizeof(SHMQueueSlotHeader))));
	if(SHM_QUEUE_GET_SLOT_HEADER_MAGIC(slotHeader) != SHM_QUEUE_SLOT_MAGIC)
	{
		printk("SHARED_MEM: shm_queue_send_msg: Magic at slot %d in queue %s invalid. SlotPtr: 0x%lx Got Value 0x%lx expected: 0x%lx\n", 
				(int) queue_pair_os_info->shmQPairPtr->qLxToVx.nextSlotToInsert, queue_pair_os_info->shmQPairPtr->queuePairName, (unsigned long int) slotHeader,
				(unsigned long int) SHM_QUEUE_GET_SLOT_HEADER_MAGIC(slotHeader), (unsigned long int) SHM_QUEUE_SLOT_MAGIC);
		retVal = 0;
		goto shm_queue_send_msg_error;
	}

	while(SHM_QUEUE_GET_SLOT_HEADER_OWNER(slotHeader) != SHM_QUEUE_SLOT_OWNER_LINUX)
	{
#if 0
		printk("SHARED_MEM: shm_queue_send_msg: Slot %d in queue %s not owned by us. Probably full. Next Out: %d\n", 
				(int) queue_pair_os_info->shmQPairPtr->qLxToVx.nextSlotToInsert, queue_pair_os_info->shmQPairPtr->queuePairName, 
				(int) queue_pair_os_info->shmQPairPtr->qLxToVx.nextSlotToProcess);
#endif

		if (send_args->flags & SHMQ_SEND_FLAG_NO_WAIT)
		{
			send_args->status = SHMQSTATUS_Q_FULL;
			retVal = 0;
			goto shm_queue_send_msg_error;
		}

		/* Currently we only support interrupt based wait */
		prepare_to_wait(&queue_pair_os_info->send_lx_to_vx_wait_queue_head, &wait, TASK_INTERRUPTIBLE);
		if(SHM_QUEUE_GET_SLOT_HEADER_OWNER(slotHeader) != SHM_QUEUE_SLOT_OWNER_LINUX)
		{
			schedule();
		}
		finish_wait(&queue_pair_os_info->send_lx_to_vx_wait_queue_head, &wait);
		if(signal_pending(current))
		{
			printk("SHARED_MEM: shm_queue_send_msg: Returning because Q is full and we got a signal while waiting for an entry in the queue for q index %d\n", (int) send_args->handle);
			send_args->status = SHMQSTATUS_Q_FULL;
			retVal = -EINTR;
			goto shm_queue_send_msg_error;
		}

#if 0
		/* sleep poll based full checking. No timeout implemented right now */
		if(msleep_interruptible(100))
		{
			retVal = -EINTR;
			printk("SHARED_MEM: shm_queue_send_msg: Waiting on poll sleep interrupted early. Most likely signal interruption.\n");
			goto shm_queue_send_msg_error;
		}
#endif
	}
	
	if (from_user)
		copy_from_user((void *) (slotHeader + 1), (void __user *)send_args->msg, send_args->msgSize);
	else
		memcpy((void *)(slotHeader + 1), (void *)send_args->msg, send_args->msgSize);

	SHM_QUEUE_SET_SLOT_HEADER(slotHeader, SHM_QUEUE_SLOT_MAGIC, SHM_QUEUE_SLOT_OWNER_VXWORKS, send_args->msgSize);
	queue_pair_os_info->shmQPairPtr->qLxToVx.nextSlotToInsert = (queue_pair_os_info->shmQPairPtr->qLxToVx.nextSlotToInsert + 1) % queue_pair_os_info->shmQPairPtr->qLxToVx.numSlots;	

	/* Now set the doorbell for the other Core */
	SHM_DOORBELL_SET(SHM_DOORBELL_VXWORKS_CPU_ID, queue_pair_os_info->shmQPairPtr->qLxToVx.doorbellBitShift);

	send_args->status = SHMQSTATUS_OK;
	retVal = 0;

shm_queue_send_msg_error:

	up(&queue_pair_os_info->q_lx_to_vx_mutex);

	return retVal;
}

int shm_queue_kern_recv_msg(shared_mem_dev *shm_dev, SharedMemQueueRecvMsg *recv_args)
{
	return shm_queue_recv_msg(shm_dev, recv_args);
}
EXPORT_SYMBOL(shm_queue_kern_recv_msg);

int shm_queue_recv_msg(shared_mem_dev *shm_dev, SharedMemQueueRecvMsg *recv_args)
{
	return shm_queue_recv_msg_base(shm_dev, recv_args, 1);
}

int shm_queue_recv_msg_base(shared_mem_dev *shm_dev, SharedMemQueueRecvMsg *recv_args, int from_user)
{
	int retVal;
	unsigned int size;
	volatile SHMQueueSlotHeader *slotHeader = NULL;

	shm_queue_os_info *queue_os_info = &(shm_dev->queue_os_info);
	shm_queue_pair_os_info *queue_pair_os_info = NULL;
	DEFINE_WAIT(wait);

	if(!queue_os_info || !recv_args)
	{
		return -EINVAL;
	}

	recv_args->status = SHMQSTATUS_GENERIC_ERROR;
	recv_args->msgRecvSize = 0x0;

	if(!queue_os_info->cores_synced_for_queues)
	{
		printk("SHARED_MEM: shm_queue_recv_msg: Cannot send message before we are done syncing between cores\n");
		return -EINVAL;
	}

	if( (recv_args->handle <= SHARED_MEM_QUEUE_INVALID_HANDLE) ||
		(recv_args->handle >= SHM_MAX_QUEUE_PAIRS) )
	{
		printk("SHARED_MEM: shm_queue_recv_msg: %d queue handle is invalid\n", (int) recv_args->handle);
		return -EINVAL;
	}

	if(!queue_os_info->q_pair_os_info[recv_args->handle].shmQPairPtr)
	{
		/* This queue hasn't been attached to yet */
		printk("SHARED_MEM: shm_queue_recv_msg: %d queue handle has not been attached to yet. Please attach first\n", (int) recv_args->handle);
		return -EINVAL;
	}
	
	queue_pair_os_info = &(queue_os_info->q_pair_os_info[recv_args->handle]);

	/* Take the queue specific mutex */
	if(down_interruptible(&queue_pair_os_info->q_vx_to_lx_mutex))
	{
		printk("SHARED_MEM: shm_queue_recv_msg: Waiting for %d queue pair mutex failed. Most likely signal interruption.\n", (int) recv_args->handle);
		return -EINTR;
	}

	slotHeader = (SHMQueueSlotHeader *) (queue_pair_os_info->slot_buf_ptr_vx_to_lx + 
										( queue_pair_os_info->shmQPairPtr->qVxToLx.nextSlotToProcess * 
										  (queue_pair_os_info->shmQPairPtr->qVxToLx.sizeSlots + sizeof(SHMQueueSlotHeader))));

	while(SHM_QUEUE_GET_SLOT_HEADER_OWNER(slotHeader) != SHM_QUEUE_SLOT_OWNER_LINUX)
	{
#if 0
		printk("SHARED_MEM: shm_queue_recv_msg: Slot %d in queue %s not owned by us. Probably empty. Next In: %d\n", 
				(int) queue_pair_os_info->shmQPairPtr->qVxToLx.nextSlotToProcess, queue_pair_os_info->shmQPairPtr->queuePairName, 
				(int) queue_pair_os_info->shmQPairPtr->qVxToLx.nextSlotToInsert);
#endif

		if (recv_args->flags & SHMQ_RECV_FLAG_NO_WAIT)
		{
			printk("SHARED_MEM: shm_queue_recv_msg: Returning because Q is empty and no wait flag is set in request\n");
			recv_args->status = SHMQSTATUS_Q_EMPTY;
			retVal = 0;
			goto shm_queue_recv_msg_error;
		}

		/* Currently we only support interrupt based wait */
		prepare_to_wait(&queue_pair_os_info->recv_vx_to_lx_wait_queue_head, &wait, TASK_INTERRUPTIBLE);
		if(SHM_QUEUE_GET_SLOT_HEADER_OWNER(slotHeader) != SHM_QUEUE_SLOT_OWNER_LINUX)
		{
			schedule();
		}
		finish_wait(&queue_pair_os_info->recv_vx_to_lx_wait_queue_head, &wait);
		if(signal_pending(current))
		{
			printk("SHARED_MEM: shm_queue_recv_msg: Returning because Q is empty we got a signal while waiting for an entry in the queue for q index %d\n", (int) recv_args->handle);
			recv_args->status = SHMQSTATUS_Q_EMPTY;
			retVal = -EINTR;
			goto shm_queue_recv_msg_error;
		}
	}

	size = SHM_QUEUE_GET_SLOT_HEADER_MSG_SIZE(slotHeader);

	if(size > recv_args->bufSize)
	{
		recv_args->msgRecvSize = recv_args->bufSize;
	}
	else
	{
		recv_args->msgRecvSize = size;
	}

	if (from_user)
		copy_to_user((void __user *) recv_args->buf, (void *) (slotHeader + 1), recv_args->msgRecvSize);
	else
		memcpy((void *)recv_args->buf, (void *) (slotHeader + 1), recv_args->msgRecvSize);

	SHM_QUEUE_SET_SLOT_HEADER(slotHeader, SHM_QUEUE_SLOT_MAGIC, SHM_QUEUE_SLOT_OWNER_VXWORKS, 0x0);
	queue_pair_os_info->shmQPairPtr->qVxToLx.nextSlotToProcess = (queue_pair_os_info->shmQPairPtr->qVxToLx.nextSlotToProcess + 1) % queue_pair_os_info->shmQPairPtr->qVxToLx.numSlots;

	/* Now set the doorbell for the other Core in case it is waiting and the queue is full */
	SHM_DOORBELL_SET(SHM_DOORBELL_VXWORKS_CPU_ID, queue_pair_os_info->shmQPairPtr->qVxToLx.doorbellBitShift);

	recv_args->status = SHMQSTATUS_OK;
	retVal = 0;

shm_queue_recv_msg_error:

	up(&queue_pair_os_info->q_vx_to_lx_mutex);

	return retVal;
}

/*______________________________________________________________________________________________*/
/*______________________________________________________________________________________________*/
/*______________________________________________________________________________________________*/
/* QUEUE CODE END																				*/
/*______________________________________________________________________________________________*/
/*______________________________________________________________________________________________*/
/*______________________________________________________________________________________________*/

/* MAJOR device number. We use a global to allow us to switch to hard coded number in the future if needed */
static int shared_mem_major = 0;

/*
 * This means that we can only have one shared mem area to one other core. So be it.
 */
static int shared_mem_init_done = 0;
static shared_mem_dev *g_shm_dev = NULL;
DEFINE_MUTEX(init_shared_mem_mutex);

/*
 * This returns the shared mem pointer.
 *
 * To ensure that the caller can get this correctly, we might need some locking ... testing for != NULL might 
 * not be enough
 */
void *dri_shm_get_dev(void)
{
	return (void *)g_shm_dev;
}

EXPORT_SYMBOL(dri_shm_get_dev);

void wait_for_init_shared_mem(void *dev)
{
	shared_mem_dev *shm_dev = (shared_mem_dev *)dev;
	int ret = 0;

	printk(KERN_INFO "%s: Entering Mutex for shared memory init\n", __func__);

	while ((ret = mutex_lock_interruptible(&init_shared_mem_mutex)) == -EINTR);

	if (ret) {
		printk(KERN_ERR "%s: Could not get mutex. BAD! Err: %d\n", __func__, ret);
		return;
	}

	if (!shared_mem_init_done)
	{
		/* First we identify to the other core that we are ready to wait for its init */
		shm_dev->sharedMemHeader->cpu1_wait_status = SHARED_MEM_CPU1_READY_TO_WAIT_MARKER;

		printk("SHARED_MEM: CPU1: READY TO WAIT FOR INITIALIZATION OF SHARED MEMORY\n");

		/* Wait for the other core to tell us that it is ready to initialize */
		while(shm_dev->sharedMemHeader->cpu1_wait_status != SHARED_MEM_CPU0_CLEARED_TO_WAIT_MARKER)
		{
			msleep_interruptible(100);
		}

		printk("SHARED_MEM: CPU1: INITIALIZATION STARTED BY CPU0. WAITING ON INIT DONE\n");

		/* Now we wait for the initialization of the shared memory to be done */
		while(shm_dev->sharedMemHeader->cpu0_init_status != SHARED_MEM_CPU0_INIT_DONE_MARKER)
		{
			msleep_interruptible(100);
		}

		printk("SHARED_MEM: CPU1: INITIALIZATION OF SHARED MEMORY BY CPU0 DONE DETECTED\n");

		/* At this point we can assume all the partitions have been initialized */
		/* Now we can fill out our data structures with the init values of the 
	 	* various partitions
	 	*/

		shared_mem_init_done = 1;
	}

	mutex_unlock(&init_shared_mem_mutex);
	printk(KERN_INFO "%s: Exiting Mutex for shared memory init\n", __func__);
}

EXPORT_SYMBOL(wait_for_init_shared_mem);

void parse_shared_mem_header(shared_mem_dev *shm_dev, shared_mem_os_info *info)
{
  int i;
  shared_mem_region_os_info *region_info;
  SharedMemPartitionInfo *partitionInfo;

  memset(info, 0x0, sizeof(*info));

  if(!shm_dev->sharedMemHeader || (shm_dev->sharedMemHeader->header_magic != SHARED_MEM_HEADER_MAGIC))
  {
	  return;
  }

  info->virt_kernel_header_base = (unsigned int) shm_dev->sharedMemHeader;
  info->phys_header_base = SHARED_MEMORY_HEADER_PHYS_ADDR;
  info->num_regions = 0;

  for(i = 0; i < shm_dev->sharedMemHeader->num_partitions; i++)
  {
	  region_info = &(info->region_info[info->num_regions]);
	  partitionInfo = &(shm_dev->sharedMemHeader->partitions[i]);
	  memcpy(region_info->tag, partitionInfo->name_tag, 4);
	  region_info->offset = partitionInfo->offset;
	  region_info->physAddr = info->phys_header_base - partitionInfo->offset;
	  /*
	   * Remap them all for now.
	   */
	  region_info->virtKernelAddr = ioremap_nocache(region_info->physAddr, partitionInfo->size);
          region_info->virtUserAddr = 0x0;
	  region_info->regionSize = partitionInfo->size;
	  region_info->reserved = 0x0;
	  region_info->num_mmaps = 0x0;
	  info->num_regions++;
  }
}

int shared_mem_open_main(void *dev)
{
  shared_mem_dev *shm_dev = (shared_mem_dev *)dev;
  shared_mem_os_info *os_info;

  if(!shm_dev || !shm_dev->sharedMemHeader)
  {
	  printk("SHARED_MEM: shared_mem_open: About to take semaphore. shm_dev: 0x%lx shm_dev->sharedMemHeader: 0x%lx\n", (unsigned long int) shm_dev, (unsigned long int) shm_dev->sharedMemHeader);
	  return -EAGAIN;
  }

  //printk("SHARED_MEM: shared_mem_open: About to take semaphore.\n");
  /* Make sure the shared memory has been inited by the other core already */
  if(down_interruptible(&shm_dev->shm_open_mutex))
  {
	  printk("SHARED_MEM: shared_mem_open: Waiting for open mutex failed. Most likely signal interruption.\n");
	  return -EINTR;
  }

  //printk("SHARED_MEM: shared_mem_open: Got Semaphore.\n");

  if (!shm_dev->driver_opened_once)
  {
     wait_for_init_shared_mem((void *)shm_dev);
     shm_dev->driver_opened_once = 1;
  }  

  /* For now we only support two opens at a time :-) */
  if(shm_dev->open_count >= 2)
  {
    printk("SHARED_MEM: shared_mem_open: Driver already open. We only support two opens at a time currently\n");
	up(&shm_dev->shm_open_mutex);  
	return -EAGAIN;
  }

  /* Now init the open for this particular openi, buut only if needed */
  if (!shm_dev->os_info)
  {
  	os_info = (shared_mem_os_info *) kmalloc(sizeof(shared_mem_os_info), GFP_KERNEL);

  	if(!os_info)
  	{
	  printk("SHARED_MEM: shared_mem_open: Unable to allocate memory for driver structure\n");
	  return -ENOMEM;
  	}

  	parse_shared_mem_header(shm_dev, os_info);

  	shm_dev->os_info = os_info;
  }

  shm_dev->open_count++;
  //printk("SHARED_MEM: shared_mem_open: Incrememnted open count: 0x%lx\n", (unsigned long int) shm_dev->open_count);

  up(&shm_dev->shm_open_mutex);

  return 0;
}
EXPORT_SYMBOL(shared_mem_open_main);

static int shared_mem_open(struct inode *inode, struct file *file)
{
  shared_mem_os_info *os_info;
  shared_mem_dev *shm_dev;

  printk("SHARED_MEM: shared_mem_open BEGIN.\n");

  shm_dev = container_of(inode->i_cdev, shared_mem_dev, cdev);

  shared_mem_open_main(shm_dev);

  file->private_data = (void *) shm_dev;

  printk("SHARED_MEM: shared_mem_open END.\n");
  return 0;
}

static int shared_mem_release(struct inode *inode, struct file *file)
{
  shared_mem_dev *shm_dev = (shared_mem_dev *) file->private_data;

  printk("SHARED_MEM: shared_mem_release: BEGIN\n");

  if(!shm_dev)
  {
	  return -EINVAL;
  }

  if(shm_dev->open_count == 0)
  {
	  return -EINVAL;
  }

  /* Make sure the shared memory has been inited by the other core already */
  if(down_interruptible(&shm_dev->shm_open_mutex))
  {
	  printk("SHARED_MEM: shared_mem_release: Waiting for open mutex failed. Most likely signal interruption.\n");
	  return -EINTR;
  }

  //printk("SHARED_MEM: shared_mem_release: Got open mutex for release\n");

  if(shm_dev->os_info)
  {
    kfree(shm_dev->os_info);
	shm_dev->os_info = NULL;
  }
  
  if(shm_dev->open_count > 0)
  {
	shm_dev->open_count--;
	printk("SHARED_MEM: shared_mem_release: Decremented openCount 0x%lx\n", (unsigned long int) shm_dev->open_count);
  }

  //printk("SHARED_MEM: shared_mem_release: About to release open mutex\n");

  up(&shm_dev->shm_open_mutex); 

  printk("SHARED_MEM: shared_mem_release: END\n");
  return 0;
}

int find_region_kern_addr_from_tag(void *dev, char *tag, void **phys, void **virt, int **reg_size)
{
	int i;
	shared_mem_dev *shm_dev = (shared_mem_dev *)dev;

	if (!shm_dev || !shm_dev->os_info || !virt || !reg_size)
	{
		return -EINVAL;
	}

	for (i = 0; i < shm_dev->os_info->num_regions; i++)
	{
		if (!memcmp(shm_dev->os_info->region_info[i].tag, tag, 4))
		{
                        *phys = shm_dev->os_info->region_info[i].physAddr;
			*virt = shm_dev->os_info->region_info[i].virtKernelAddr;
			*reg_size = shm_dev->os_info->region_info[i].regionSize;
			return 0;
		}
	}

	return -ENOENT;
}
EXPORT_SYMBOL(find_region_kern_addr_from_tag);

int find_region_from_tag(shared_mem_os_info *os_info, SharedMemGetRegionOffset *offset)
{
	int i;

	if(!os_info || !offset)
	{
		return -EINVAL;
	}

	for(i = 0; i < os_info->num_regions; i++)
	{
		if(!memcmp(offset->tag, os_info->region_info[i].tag, 4))
		{
			offset->offset = os_info->region_info[i].offset;
			return 0;
		}
	}

	printk("SHARED_MEM: find_region_from_tag: offset for tag: ");
	for(i = 0; i < 4; i++)
	{
		printk("%c", offset->tag[i]);
	}
	printk(" not found\n");

	return -EINVAL;
}

int find_region_from_offset(shared_mem_os_info *os_info, unsigned int offset, unsigned int size, shared_mem_region_os_info **region_info)
{
	int i;

	if(!os_info)
	{
		return -EINVAL;
	}

	for(i = 0; i < os_info->num_regions; i++)
	{
		if(os_info->region_info[i].offset == offset)
		{
			if(size <= os_info->region_info[i].regionSize)
			{
				*region_info = &(os_info->region_info[i]);
				return 0;
			}
			else
			{
				printk("SHARED_MEM: find_region_from_offset: region found for offset 0x%lx but size too big: 0x%lx\n", 
						(unsigned long int) offset, (unsigned long int) size);
				return -EINVAL;
			}
		}
	}

	printk("SHARED_MEM: find_region_from_offset: region not found for offset 0x%lx\n", (unsigned long int) offset);

	return -EINVAL;  
}

int find_region_from_user_addr(shared_mem_os_info *os_info, loff_t virtAddr, unsigned long *physAddr)
{
        int i;

        if(!os_info)
        {
                return -EINVAL;
        }

        for(i = 0; i < os_info->num_regions; i++)
        {
                if (   (virtAddr >= os_info->region_info[i].virtUserAddr)
                    && (virtAddr < (os_info->region_info[i].virtUserAddr + os_info->region_info[i].regionSize))) 
                {
                  *physAddr = os_info->region_info[i].physAddr + (virtAddr - os_info->region_info[i].virtUserAddr);
                  return 0;
                }
        }

//        printk("SHARED_MEM: find_region_from_offset: region not found for virt_addr 0x%lx\n", (unsigned long int) virtAddr);

        return -EINVAL;
}


void printRegionInfo(shared_mem_os_info *os_info)
{
	int i,j;

	printk("-----------------------------------------------------------------\n");
	printk("SHARED_MEM: REGION INFO:\n");
	printk("-----------------------------------------------------------------\n");
	printk("\nHEADER ADDR: PHYSICAL: 0x%08lx\tVIRTUAL KERNEL: 0x%08lx\n", (unsigned long int) os_info->phys_header_base, (unsigned long int) os_info->virt_kernel_header_base);
	printk("NUMBER OF REGIONS: %d\n\n", os_info->num_regions);
	printk("-----------------------------------------------------------------\n");
	for(i = 0; i < os_info->num_regions; i++)
	{
		printk("%d) ", i);
		for(j = 0; j < 4; j++)
		{
			printk("%c", os_info->region_info[i].tag[j]);
		}
		printk("\n");
		printk("\tOFFSET FROM HEADER: 0x%08lx\n", (unsigned long int) os_info->region_info[i].offset);
		printk("\tPHYSICAL ADDRESS: 0x%08lx\n", (unsigned long int) os_info->region_info[i].physAddr);
		printk("\tREGION SIZE: 0x%08lx\n", (unsigned long int) os_info->region_info[i].regionSize);
		printk("\tKERNEL VIRTUAL ADDRESS: ");
		if(os_info->region_info[i].virtKernelAddr)
		{
			printk("0x%08lx\n", (unsigned long int)os_info->region_info[i].virtKernelAddr);
		}
		else
		{
			printk("Unmapped in Kernel\n");
		}
		printk("\tUSER VIRTUAL ADDRESS: ");
		if(os_info->region_info[i].virtUserAddr)
		{
			printk("0x%08lx\n", (unsigned long int)os_info->region_info[i].virtUserAddr);
		}
		else
		{
			printk("Unmapped in this process\n");
		}
		printk("\tNUMBER OF MMAPS ON REGION IN THIS PROCESS: %d\n", os_info->region_info[i].num_mmaps);
		printk("\n\n");
	}
	printk("-----------------------------------------------------------------\n");
}

static int shared_mem_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
  shared_mem_dev *shm_dev = (shared_mem_dev *) file->private_data;
  int retval = 0;

  //printk("SHARED_MEM: shared_mem_ioctl. BEGIN\n");
  
  if(!shm_dev)
  {
	return -EINVAL;
  }

  if(shm_dev->open_count == 0 || !shm_dev->os_info)
  {
	  return -EINVAL;
  }

  if(_IOC_TYPE(cmd) != SHARED_MEM_IOC_MAGIC)
  {
	return -ENOTTY;
  }

  if(_IOC_NR(cmd) > SHARED_MEM_IOC_MAXNR)
  {
	  return -ENOTTY;
  }

  /* Possibly do access_ok based checks here if any IOCTL don't use copy_from/to_user */

  switch(cmd)
  {
    case SHARED_MEM_IOC_GET_REGION_OFFSET:
	{
	  SharedMemGetRegionOffset region_offset;
	  if(copy_from_user((void *) &region_offset, (const void __user *) arg, sizeof(region_offset)))
	  {
		retval = -EINVAL;
		break;
	  }
	  retval = find_region_from_tag(shm_dev->os_info, &region_offset);

	  if(copy_to_user((void __user *) arg, (void *) &region_offset, sizeof(region_offset)))
	  {
		retval = -EINVAL;
	  }
	  break;
	}

	case SHARED_MEM_IOC_DUMP_REGION_INFO:
	{
	  printRegionInfo(shm_dev->os_info);
	  retval = 0;
	  break;
	}

	case SHARED_MEM_IOC_QUEUE_ATTACH:
	{
	  SharedMemQueueAttach attach_args;
	  if(copy_from_user((void *) &attach_args, (const void __user *) arg, sizeof(attach_args)))
	  {
		  retval = -EINVAL;
		  break;
	  }
	  retval = shm_queue_attach(shm_dev, &attach_args);

	  if(copy_to_user((void __user *) arg, (void *) &attach_args, sizeof(attach_args)))
	  {
		  retval = -EINVAL;
	  }
	  break;		
	}

	case SHARED_MEM_IOC_QUEUE_SEND_MSG:
	{
	  SharedMemQueueSendMsg send_args;
	  if(copy_from_user((void *) &send_args, (const void __user *) arg, sizeof(send_args)))
	  {
		  retval = -EINVAL;
		  break;
	  }
	  retval = shm_queue_send_msg(shm_dev, &send_args);

	  if(copy_to_user((void __user *) arg, (void *) &send_args, sizeof(send_args)))
	  {
		  retval = -EINVAL;
	  }
	  break;		
	}

	case SHARED_MEM_IOC_QUEUE_RECV_MSG:
	{
	  SharedMemQueueRecvMsg recv_args;
	  if(copy_from_user((void *) &recv_args, (const void __user *) arg, sizeof(recv_args)))
	  {
		  retval = -EINVAL;
		  break;
	  }
	  retval = shm_queue_recv_msg(shm_dev, &recv_args);

	  if(copy_to_user((void __user *) arg, (void *) &recv_args, sizeof(recv_args)))
	  {
		  retval = -EINVAL;
	  }
	  break;		
	}

    default:
	  return -ENOTTY;
  }

  //printk("SHARED_MEM: shared_mem_ioctl. END\n");

  return retval;
}

static int shared_mem_mmap(struct file *file, struct vm_area_struct *vma)
{
  int ret;
  shared_mem_region_os_info *region_info = NULL;

  shared_mem_dev *shm_dev = (shared_mem_dev *) file->private_data;

  if(!shm_dev || !(shm_dev->sharedMemHeader))
  {
    return -EAGAIN;
  }

  ret = find_region_from_offset(shm_dev->os_info, vma->vm_pgoff << PAGE_SHIFT, vma->vm_end - vma->vm_start, &region_info);

  if(ret)
  {
	  printk("SHARED_MEM: shared_mem_mmap: Error finding region at the given offset. page offset given: 0x%lx\n", (unsigned long int) (vma->vm_pgoff << PAGE_SHIFT));
	  return ret;
  }

  vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

  ret = remap_pfn_range(vma,
			vma->vm_start, 
			(SHARED_MEMORY_HEADER_PHYS_ADDR - (vma->vm_pgoff << PAGE_SHIFT)) >> PAGE_SHIFT, 
			vma->vm_end - vma->vm_start,
			vma->vm_page_prot);

  // Store the user virtual address that was assigned for this physical range

  region_info->virtUserAddr = vma->vm_start;

  if(ret != 0)
  {
	printk("SHARED_MEM: shared_mem_mmap: remap_pfn_range: ERROR: %d mapping for page offset: 0x%lx Addr start: 0x%lx, end: 0x%lx prot: 0x%lx\n", 
			ret, vma->vm_pgoff, vma->vm_start, vma->vm_end, vma->vm_page_prot);
  }
  else
  {
	if (region_info)
	{
	  region_info->num_mmaps++;
	}
	printk("SHARED_MEM: shared_mem_mmap: remap_pfn_range: mapped for page offset: 0x%lx Addr start: 0x%lx, end: 0x%lx prot: 0x%lx\n", 
			vma->vm_pgoff, vma->vm_start, vma->vm_end, vma->vm_page_prot);
#if defined (CONFIG_MV_USE_XOR_FOR_COPY_USER_BUFFERS) || (CONFIG_MV_IDMA_COPYUSER)
	register_dri_virt_to_phys_range(region_info->physAddr, region_info->virtUserAddr, vma->vm_end - vma->vm_start);
#endif
  }

  return ret;
}

#if 0
static ssize_t shared_mem_read(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
  printk("SHARED_MEM: shared_mem_read. No operation\n");
  return 0;
}

static ssize_t shared_mem_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
  printk("SHARED_MEM: shared_mem_write. No operation\n");
  return 0;
}
#endif

// do_shared_mem_file_read
//
// Implement sendfile operation for our shared memory buffers.
// 
// This routine will check the start address of the data to be transmitted
// is within one of our buffers. It assumes that the whole buffer to be sent
// is in the buffer
//
// We assume that the virtual range to be copied is stored in physically
// contiguous pages
//
// If the address is not found, we return -EINVAL to the calling code

static void do_shared_mem_file_read(struct file *filp, loff_t *ppos, read_descriptor_t *desc, read_actor_t actor)
{
#if 0
        struct inode *inode = filp->f_path.dentry->d_inode;
        struct address_space *mapping = inode->i_mapping;
#endif
        unsigned long index, offset;
        struct page *page = NULL;
        unsigned long physAddr;
        unsigned int pfn;
        int ret;

        shared_mem_dev *shm_dev = (shared_mem_dev *) filp->private_data;

        ret = find_region_from_user_addr(shm_dev->os_info, *ppos, &physAddr);

        index = *ppos >> PAGE_CACHE_SHIFT;
        offset = *ppos & ~PAGE_CACHE_MASK;

        if (ret) 
        {
          desc->error = ret;
//          if (desc->error == -EINVAL)
//            desc->error = 0;
          goto out;
        }

        pfn = physAddr >> PAGE_SHIFT;
        for (;;) {
                unsigned long nr, ret;

                // Get the page for the current virtual address

                page = pfn_to_page(pfn);

                // Calculate number of bytes to send in this page
                nr = PAGE_CACHE_SIZE;
                nr -= offset;

                if (page) {
                        /*
                         * If users can be writing to this page using arbitrary
                         * virtual addresses, take care about potential aliasing
                         * before reading the page on the kernel side.
                         */

                        // Possible optimization - turn this off as our pages are in
                        // uncached memory. 
#if 0
                        if (mapping_writably_mapped(mapping))
                                flush_dcache_page(page);
#endif
                        /*
                         * Mark the page accessed if we read the beginning.
                         */
                        if (!offset)
                                mark_page_accessed(page);
                } else {
                        page = ZERO_PAGE(0);
                        page_cache_get(page);
                }

                /*
                 * Ok, we have the page, and it's up-to-date, so
                 * now we can copy it to the network stack
                 *
                 * The actor routine returns how many bytes were actually used..
                 * NOTE! This may not be the same as how much of a user buffer
                 * we filled up (we may be padding etc), so we can only update
                 * "pos" here (the actor routine has to update the user buffer
                 * pointers and the remaining count).
                 */
                ret = actor(desc, page, offset, nr);
                offset += ret;
                index += offset >> PAGE_CACHE_SHIFT;
                offset &= ~PAGE_CACHE_MASK;

                // Move to the next page - we're assuming physical continuity here
                ++pfn;

//                page_cache_release(page);
                if (ret != nr || !desc->count)
                        break;

                cond_resched();
        }

out:
        *ppos = ((loff_t) index << PAGE_CACHE_SHIFT) + offset;
        file_accessed(filp);
}

ssize_t shared_mem_sendfile(struct file *in_file, loff_t *ppos,
                         size_t count, read_actor_t actor, void *target)
{
        read_descriptor_t desc;

        if (!count)
                return 0;

        desc.written = 0;
        desc.count = count;
        desc.arg.data = target;
        desc.error = 0;

        do_shared_mem_file_read(in_file, ppos, &desc, actor);
        if (desc.written)
                return desc.written;
        return desc.error;
}

static int shared_mem_panic_event(struct notifier_block *th, unsigned long event, void *ptr)
{
    /* We had a panic. interrupt the other core to allow for a safe reboot */
    printk("shared_mem_panic_event: Kernel panic event handler called. Notifying VxWorks Core of Kernel panic.\n");
    SHM_DOORBELL_SET(SHM_DOORBELL_VXWORKS_CPU_ID, SHM_QUEUE_SPECIAL_PANIC_DB_BITSHIFT);
    return NOTIFY_DONE;
}

static struct notifier_block shared_mem_panic_block = {
    .notifier_call = shared_mem_panic_event,
    .priority       = INT_MAX,
};

struct file_operations shared_mem_fops =
{
	.owner	=	THIS_MODULE,
	.llseek	=	NULL,
	.read	=	NULL,
	.write	=	NULL,
	.readdir=	NULL,
	.poll	=	NULL,
	.ioctl	=	shared_mem_ioctl,
	.mmap	=	shared_mem_mmap,
	.open	=	shared_mem_open,
	.flush	=	NULL,
	.release=	shared_mem_release,
	.fsync	=	NULL,
	.fasync	=	NULL,
	.lock	=	NULL,
    .sendfile =     shared_mem_sendfile,
};


static int __init shared_mem_module_init(void)
{
  int ret;
  dev_t devno;

#if 0
  volatile unsigned int *mem;
#endif    

  shared_mem_dev *shm_dev = (shared_mem_dev *) kmalloc(sizeof(shared_mem_dev), GFP_KERNEL);

  if(!shm_dev)
  {
	  return -ENOMEM;
  }

  atomic_notifier_chain_register(&panic_notifier_list, &shared_mem_panic_block);

  memset(shm_dev, 0x0, sizeof(*shm_dev));

  if(shared_mem_major)
  {
	  devno = MKDEV(shared_mem_major, 0);
	  ret = register_chrdev_region(devno, 1, SHARED_MEM_NAME);
  }
  else
  {
	ret = alloc_chrdev_region(&devno, 0, 1, SHARED_MEM_NAME);
	shared_mem_major = MAJOR(devno);
  }

  if(ret != 0)
  {
     printk("SHARED_MEM: alloc_chrdev_region failed for driver with return: %d\n", ret);
     return ret;
  }

  cdev_init(&(shm_dev->cdev), &shared_mem_fops);
  shm_dev->cdev.owner = THIS_MODULE;
  shm_dev->cdev.ops = &shared_mem_fops;

  init_MUTEX(&shm_dev->shm_open_mutex);
  
#if 0
  shm_dev->sharedMemBaseAddr = (void *) ioremap_nocache(SHARED_MEMORY_BASE_PHYS_ADDR, SHARED_MEM_TOTAL_SHARED_MEM);

  if(!shm_dev->sharedMemBaseAddr)
  {
    printk("SHARED_MEM: error mapping shared memory into kernel memory\n");
    return -EIO;
  }

  shm_dev->sharedMemHeader = (SharedMemPartitionDescHeader *) (((unsigned char *) shm_dev->sharedMemBaseAddr) + SHARED_MEMORY_HEADER_OFFSET_FROM_BASE);
#else
  shm_dev->sharedMemBaseAddr = 0x0;
  shm_dev->sharedMemHeader = (SharedMemPartitionDescHeader *) ioremap_nocache(SHARED_MEMORY_HEADER_PHYS_ADDR, SHARED_MEM_HEADER_PARTITION_SIZE);

  if(!shm_dev->sharedMemHeader)
  {
    printk("SHARED_MEM: error mapping shared memory Header into kernel memory\n");
    return -EIO;
  }
#endif

  printk("SHARED_MEM: INIT: Shared Memory mapped into Kernel space at Addr: 0x%lx for size 0x%lx\n", (long unsigned int) shm_dev->sharedMemBaseAddr, (long unsigned int) SHARED_MEM_TOTAL_SHARED_MEM);
  printk("SHARED_MEM: INIT: Shared Memory Header in Kernel space at Addr: 0x%lx\n", (long unsigned int) shm_dev->sharedMemHeader);


#if 0
  mem = (volatile unsigned int *) shm_dev->sharedMemHeader;
  *mem = 0x12345678;
  
  printk("SHARED_MEM: Testing: Addr: 0x%lx, Value: 0x%lx\n", (long unsigned int) mem, (long unsigned int) *mem);
#endif

	/* Init the queue structure. Although for the most part not much is done here but in the first attach ioctl call */
	shm_queue_init(&(shm_dev->queue_os_info));

#if 0
  ret = register_chrdev(SHARED_MEM_MAJOR, SHARED_MEM_NAME, &shared_mem_fops);
#else
  ret = cdev_add(&shm_dev->cdev, devno, 1);
#endif
  if(ret)
  {
	  printk(KERN_NOTICE "Error initializing shared memory driver");
  }

  /*
   * Save this as well
 _ */
  printk(KERN_INFO "%s: Location of shm_dev: %p\n", __func__, (void *)shm_dev);
  g_shm_dev = shm_dev;

  return ret;
}

#if 0
static void shared_mem_module_exit(void)
{
#if 0
  unregister_chrdev(SHARED_MEM_MAJOR,SHARED_MEM_NAME);
#else
  unregister_chrdev_region(shared_mem_major, 1);
#endif
  printk("Exiting Shared Mem module\n");
  return;
}
#endif

/*module_init(shared_mem_module_init);
module_exit(shared_mem_module_exit);*/
__initcall(shared_mem_module_init);
MODULE_AUTHOR("www.drobo.com");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DroboPro Shared memory driver");

