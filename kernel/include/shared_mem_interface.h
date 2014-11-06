#ifndef _SHARED_MEM_INTERFACE_H_
#define _SHARED_MEM_INTERFACE_H_

#define SHARED_MEM_QUEUE_INVALID_HANDLE			-1

// Queue Send Recv Flags
#define SHMQ_ATTACH_FLAG_NO_WAIT				0x00000001
//#define SHMQ_ATTACH_FLAG_WAIT_FOREVER_INT		0x00000002
#define SHMQ_ATTACH_FLAG_WAIT_FOREVER_POLL		0x00000004
//#define SHMQ_ATTACH_FLAG_WAIT_WITH_TIMEOUT_INT	0x00000008
//#define SHMQ_ATTACH_FLAG_WAIT_WITH_TIMEOUT_POLL	0x00000010
#define SHMQ_SEND_FLAG_NO_WAIT					0x00000020
#define SHMQ_SEND_FLAG_WAIT_FOREVER				0x00000040
//#define SHMQ_SEND_FLAG_WAIT_WITH_TIMEOUT		0x00000080
#define SHMQ_RECV_FLAG_NO_WAIT					0x00000100
#define SHMQ_RECV_FLAG_WAIT_FOREVER_INT			0x00000200
//#define SHMQ_RECV_FLAG_WAIT_WITH_TIMEOUT		0x00000400

typedef int SharedMemQHandle;

#define	SHMQSTATUS_OK							0x0
#define SHMQSTATUS_GENERIC_ERROR				0x1
#define SHMQSTATUS_INVALID_HANDLE				0x2
#define SHMQSTATUS_Q_FULL_TIMEOUT				0x3
#define SHMQSTATUS_Q_EMPTY						0x4
#define SHMQSTATUS_Q_FULL						0x5
#define SHMQSTATUS_Q_MSG_TOO_LARGE				0x6

typedef struct _SharedMemGetRegionOffset
{
	char tag[4];											/* IN */
	unsigned int offset;									/* OUT */
} SharedMemGetRegionOffset;

typedef struct _SharedMemQueueAttach
{
	char qPairName[SHM_QUEUE_PAIR_NAME_MAX_SIZE];			/* IN */
	unsigned int flags;										/* IN */
	SharedMemQHandle handle;								/* OUT */
} SharedMemQueueAttach;

typedef struct _SharedMemQueueSendMsg
{
	SharedMemQHandle handle;								/* IN */
	unsigned char *msg;										/* IN */
	unsigned int msgSize;									/* IN */
	unsigned int flags;										/* IN */
	unsigned int timeoutInMS;								/* IN */
	unsigned int status;									/* OUT */
} SharedMemQueueSendMsg;

typedef struct _SharedMemQueueRecvMsg
{
	SharedMemQHandle handle;								/* IN */
	unsigned char *buf;										/* IN */
	unsigned int bufSize;									/* IN */
	unsigned int flags;										/* IN */
	unsigned int timeoutInMS;								/* IN */
	unsigned int msgRecvSize;								/* OUT */
	unsigned int status;									/* OUT */
} SharedMemQueueRecvMsg;

#define SHARED_MEM_IOC_MAGIC				0xDB		/* Our Driver's base code */

#define SHARED_MEM_IOC_GET_REGION_OFFSET	_IOWR(SHARED_MEM_IOC_MAGIC, 1, SharedMemGetRegionOffset *)
#define SHARED_MEM_IOC_DUMP_REGION_INFO		_IO(SHARED_MEM_IOC_MAGIC, 2)
#define SHARED_MEM_IOC_QUEUE_ATTACH			_IOWR(SHARED_MEM_IOC_MAGIC, 3, SharedMemQueueAttach *)
#define SHARED_MEM_IOC_QUEUE_SEND_MSG		_IOWR(SHARED_MEM_IOC_MAGIC, 4, SharedMemQueueSendMsg *)
#define SHARED_MEM_IOC_QUEUE_RECV_MSG		_IOWR(SHARED_MEM_IOC_MAGIC, 5, SharedMemQueueRecvMsg *)


#define SHARED_MEM_IOC_MAXNR				10


#endif
