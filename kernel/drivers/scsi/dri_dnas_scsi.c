/*
 * Copyright (C) Data Robotics, Inc, 2009
 *
 * A SCSI LLD for the Drobo Disk Pack in the DNAS. 
 *
 * We make access to the SCSI device on the other side of the shared
 * memory segment possible. 
 *
 * Author(s): Richard Sharpe, ...
 *
 */

#include <linux/module.h>

#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/genhd.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/vmalloc.h>
#include <linux/moduleparam.h>
#include <linux/scatterlist.h>
#include <linux/blkdev.h>
#include <linux/completion.h>
#include <linux/stat.h>
#include <linux/mutex.h>

#include <scsi/scsi.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_device.h>
#include <scsi/scsi_host.h>
#include <scsi/scsi_tcq.h>
#include <scsi/scsicam.h>
#include <scsi/scsi_eh.h>

#include "sharedMemQueueCommon.h"
#include "shared_mem_interface.h"
#include "WriteBuffer.h"

/*
 * Define some things that didn't turn up until later versions of the 
 * kernel.
 */
#if 1 /* (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 23)) */
#define scsi_bufflen(cmd) ((cmd)->request_bufflen)
#define scsi_set_resid(cmd, len) ((cmd)->resid = (len))
#endif

#define MIN(_a_, _b_) ((_a_) < (_b_) ? (_a_) : (_b_))
#define MAX(_a_, _b_) ((_a_) > (_b_) ? (_a_) : (_b_))

/*
 * Define some driver parameters that might change
 */
#ifndef DRI_MAX_SECTORS
#define DRI_MAX_SECTORS 1024
#endif

#ifndef DRI_USE_CLUSTERING
#define DRI_USE_CLUSTERING DISABLE_CLUSTERING
#endif

static int dnas_debug_level = 0;
/*
 * Debugging macro ...
 */
#define DBG(_lvl_, _fmt_, args...) \
  { \
    if (unlikely((_lvl_) <= dnas_debug_level)) { \
      printk((_fmt_), ## args); \
    } \
  }

/*
 * Intercore structs ... move to an include file!
 */
#define INTER_CORE_SGL_MAX_ELEMENTS                   520  /* Max size returable from RTP */

typedef struct core_ctl_lx_user_recv_ {
	int no_response;  /* 0 = send appropriate repsonse from userland 1 = don't send response since driver had to do early response */
	void *cmd_buf;  /* Buffer allocated by user space to recv  */
	unsigned int cmd_size;  /* Size of the buffer allocated by*/
	unsigned int cmd_recv_size;    /* Size of the InterCoreCtrlVxToLx 
					struct actually received by the LLD 
					over intercore core ctl queue */
	void *data_buf;  /* Data buffer allocated in user space to receive 
			    data related to InterCoreCtrlVxToLx struct that 
			    is based on offsets/pointers in the 
			    InterCoreCtrlVxToLx struct */
	unsigned int data_size; /* Size of the data buffer allocated in user 
				space */
	unsigned int data_recv_size;  /* Size of the data related to 
					InterCoreCtrlVxToLx struct actually 
					received by the LLD */
} core_ctl_lx_user_recv;

typedef struct core_ctl_lx_user_send_ {
	void *cmd_buf;  /* Buffer allocated by user space to send the 
			   InterCoreCtrlVxToLx struct */
	unsigned int cmd_size;  /* Size of the buffer allocated by user space 
				to send the InterCoreCtrlVxToLx struct */
	unsigned int cmd_sent_size; /* Size of the InterCoreCtrlVxToLx struct 
				       actually sent by the LLD over intercore 
				       core ctl queue */
	void *data_buf; /* Data buffer allocated in user space to send data 
			related to InterCoreCtrlVxToLx struct that is based on 
			offsets/pointers in the InterCoreCtrlVxToLx struct */
	unsigned int data_size; /* Size of the data buffer allocated in user 
				space */
	unsigned int data_sent_size;  /* Size of the data related to 
					InterCoreCtrlVxToLx struct actually 
					sent by the LLD */
} core_ctl_lx_user_send;


#define DRI_NAS_GET_MSG		_IOR(0xF1,0xF1, core_ctl_lx_user_recv)
#define DRI_NAS_SEND_MSG	_IOW(0xF1, 0xF2, core_ctl_lx_user_send)

typedef unsigned int InterCoreCmdTag;

typedef struct InterCoreSGLElement
{
  unsigned int bufOffset;
  unsigned int bufLength;
  unsigned int bufSrc;    /* Only valid if the bufSrc in InterCoreSGL is invalid */
} InterCoreSGLElement;

/* NOTE: If bufSrc is Invalid that means each element defines its own bufSrc.
 *  * Otherwise all elements come from the bufSrc in InterCoreSGL structure
 *   */
typedef struct InterCoreSGL
{
  unsigned int numElements;
  unsigned int bufSrc;
  unsigned int totalLength;
  InterCoreSGLElement elements[INTER_CORE_SGL_MAX_ELEMENTS];
} InterCoreSGL;

typedef struct InterCoreBufferId
{
  unsigned int bufOffset;     /* This offset is from the bufSrc base address */
  unsigned int length;
  unsigned int bufSrc;
} InterCoreBufferId;

/* Send from Vx To Lx Core */
typedef struct InterCoreSCSIResp {
	unsigned int version;
	InterCoreCmdTag tag;
	unsigned int backEndTag;      /* Resource tag set by back-end stack */
	unsigned int status;
	InterCoreBufferId bufId1;
	InterCoreBufferId bufId2;
} InterCoreSCSIResp;

/* Sent from Lx To Vx core */
typedef struct InterCoreSCSICmd
{
	unsigned int version;
	InterCoreCmdTag tag;
	unsigned char lun[8];
	unsigned char cdb[16];
	InterCoreBufferId bufId;
} InterCoreSCSICmd;

/* Lx Resource are cmd are sent from Vx To Lx when Vx Core is done
 *  * using Lx Core resource and Lx core can free its resources
 *   */
typedef struct InterCoreLxResource {
 	unsigned int version;
	InterCoreCmdTag tag;
	InterCoreBufferId bufId1;
	InterCoreBufferId bufId2;
} InterCoreLxResource;

/* Vx Resource are cmd are sent from Lx To Vx for when Lx Core is done
 *  * using Vx Core resource and Vx Core can free its resources
 *   */
typedef struct InterCoreVxResource {
	unsigned int version;
	InterCoreCmdTag tag;
	unsigned int backEndTag;      /* Resource tag set by back-end stack */
	InterCoreBufferId bufId1;
	InterCoreBufferId bufId2;
} InterCoreVxResource;

/* Message for Vx to Lx core control queue usage. */
typedef struct InterCoreCtrlVxToLx
{
	unsigned int version;
	InterCoreCmdTag tag;
	unsigned int flags;
	time_t secondsSinceEpoch;    /* Seconds since Unix epoch, see flags */
/*  InterCoreSCSIResp scsiResp; */
	union {
		struct {
			unsigned int fileNum;
			unsigned int cmd;
			InterCoreBufferId bufId;
		} fileReq; // request
		struct {
			unsigned int version;
			unsigned int cmd;
			InterCoreBufferId bufId;
		} netReq; // request
		struct {
            		uint64_t diskPackId;
                        unsigned int FSMigrationFlag;
                        unsigned int reserved[2];
                } migReq; // request

	} u;
} InterCoreCtrlVxToLx;

/* Message for Lx to Vx core control queue usage. */
typedef struct InterCoreCtrlLxToVx {
	unsigned int version;
	InterCoreCmdTag tag;
	unsigned int flags;
/*   InterCoreSCSICmd scsiCmd; */
	union {
    		struct {
			unsigned int status;
			unsigned int len;
			unsigned int eof;
		} fileReq; // response
		struct {
			unsigned int version;
			unsigned int status;
		} netReq; //response
	} u;
} InterCoreCtrlLxToVx;

typedef struct VxToLxCommonInfo
{
       unsigned char serialNumber[32];
} VX_TO_LX_COMMON_INFO;

#define SCSI_VALID    0x00000001  /* SCSI cmd/resp is valid */
#define ISCSI_UP      0x00000002  /* iSCSI stack has come up (lx->vx) */
#define ISCSI_DOWN    0x00000004  /* iSCSI stack has gone down (lx->vx) */
#define ISCSI_ENABLE  0x00000008  /* iSCSI stack ENABLE request (vx->lx) */
#define ISCSI_DISABLE 0x00000010  /* iSCSI stack DISABLE request (vx->lx) */
#define TIME_VALID    0x00000020  /* secondsSinceEpoch is valid (vx->lx) */
#define REQ_LOGS      0x00000040  /* Request Lx logs (vx->lx + resp) */
#define RTN_RESOURCE  0x00000080  /* Return resource (lx->vx) */
#define ISCSI_CRASH   0x00000100  /* iSCSI stack has crashed (lx->vx) */
#define NET_INFO           0x00000200  /* Get/Set network Info */
#define UNEXPECTED_REBOOT  0x00000400  /* The last reboot was unexpectd; copy last log to crash file (vx->lx) */
#define FLUSH_DISKS   0x00000800  /* Request disks are flushed (lx->vx) */
#define LX_SHUTDOWN   0x00001000  /* Request that Linux shut down       */
#define LX_DOWN       0x00002000  /* Lx->Vx we are going down           */
#define LX_CLEARDISK  0x00004000  /* Request from Vx to do ClearDisk stuff */
#define LX_CLEARFLASH 0x00008000  /* Request from Vx to clear flash only */
#define FS_MIGRATION  0x00010000  /* Message informing us that this is a migration */

#define REQ_LOG_OPEN  0x00000001  /* Open a file */
#define REQ_LOG_NEXT  0x00000002  /* Get the next chunk of the open file */
#define REQ_LOG_CLOSE 0x00000003  /* Close the file */

#define REQ_LOG_FILE_DMESG          0x00000001 /* Logs from the dmesg log */
#define REQ_LOG_FILE_ISCSITGT       0x00000002 /* Logs from the iscsitgt output */
#define REQ_LOG_FILE_LAST_CRASH     0x00000003 /* Logs from the last crash */
#define REQ_LOG_FILE_LAST_BOOT      0x00000004 /* Logs from the last boot */

#define REQ_LOG_STAT_OK             0x00000000
#define REQ_LOG_STAT_CANT_OPEN_FILE 0x00000001

#define SET_NET_INFO       0x00000001  /* Set Network Info to Lx Core */
#define GET_NET_INFO       0x00000002  /* Get Network Info from Lx Core */
#define SET_LX_COMMON_INFO 0x00000003  /* Set Other info, currently only Serial Number  */
#define SET_IMMED_NET_INFO 0x00000004  /* Set the Network Info to Lx Core and take effect immediately */
#define SET_ESX_CERT       0x00000005  /* Set ESX certification mode on Lx core */

#define NET_INFO_OK        0x00000001
#define NET_INFO_ERROR     0x00000002

#define NETWORK_INFO_CUR_VERSION (2)

#define INTER_CORE_CMD_PARTITION_ID_INVALID           0x01
#define INTER_CORE_CMD_PARTITION_ID_J1                0x10
#define INTER_CORE_CMD_PARTITION_ID_HLBAT_CACHE       0x11
#define INTER_CORE_CMD_PARTITION_ID_LINUX_DYN_MEM     0x12
#define INTER_CORE_CMD_PARTITION_ID_VXWORKS_DYN_MEM   0x13
#define INTER_CORE_CMD_PARTITION_ID_SGL_MEM           0x14

#define INTER_CORE_CMD_PROTOCOL_VERSION_1             0x2
#define INTER_CORE_CMD_PROTOCOL_CUR_VERSION           INTER_CORE_CMD_PROTOCOL_VERSION_1

/*
 * Allow for 16 LUNs by default.
 */

#define DEF_MAX_LUNS 16

/*
 * 64 SG segments allows for 256kB all of 4kB pages 
 * The queue size on the VxCore for SCSI requests is 64
 */
#define DRI_DNAS_MAX_SG_SEGMENTS 64
#define DRI_DNAS_MAX_QUEUE 64

extern void wait_for_init_shared_mem(void *dev);
extern int shared_mem_open_main(void *dev);
extern void *dri_shm_get_dev(void);
extern int find_region_kern_addr_from_tag(void *dev, char *tag, void **phys,
                                          void **virt,
					  int *reg_size);
extern int shm_queue_attach(void *shm_dev, SharedMemQueueAttach *attach_args);
extern int shm_queue_kern_send_msg(void *shm_dev, 
			SharedMemQueueSendMsg *send_args);
extern int shm_queue_kern_recv_msg(void *shm_dev, 
			SharedMemQueueRecvMsg *recv_args);

static void dri_dnas_fake_0_release(struct device *dev);
static struct bus_type dri_dnas_fake_lld_bus;
static struct device dri_dnas_fake_primary;
static struct device_driver dri_dnas_driverfs_driver;

/*
 * Debug support ...
 */
unsigned int wb_history_enabled = 1;
unsigned int driver_debug_enabled = 0;

/*
 * Whether to drop requests or not and
 * how often. This number is how frequently to drop them.
 */
unsigned int drop_requests = 0;

/*
 * Whether to delay requests and how long to delay them for
 */
unsigned int delay_requests = 0;
unsigned int delay_skip = 0;   /* How many IOs to skip delays after a delay */

/*
 * Whether or not to bypass acceleration in read-path and write-path
 */
unsigned int no_read_acceleration = 0;
unsigned int no_write_acceleration = 0;

static int dri_dnas_max_luns = DEF_MAX_LUNS;

#define DRI_DNAS_VERSION "0.1.0"
static const char *dri_dnas_version_date = "20090720";

/*
 * There might one day be more than one Drobo device connected, so collect
 * them together in a list here. All the fields needed to manage a SCSI
 * host should be in the struct dri_dnas_device.
 */

static LIST_HEAD(dri_dnas_device_list);
static DEFINE_SPINLOCK(dri_dnas_device_list_lock);

static char dri_dnas_proc_name[] = "dri_dnas_scsi";

static unsigned char dri_dnas_serial_number[33] = { 0 };

static LIST_HEAD(dnas_sysfs_userland_queue);
static DECLARE_COMPLETION(dnas_userland_ctrl_completion);
static int dnas_waiting_for_userland_queue = 0;
static DEFINE_MUTEX(dnas_sysfs_queue_mutex);

/*
 * This structure ties together requests (struct Scsi_Cmnd *) coming down
 * from the SCSI Mid Layer, Write Buffers if a write, tags, the done 
 * routine and so forth.
 *
 * We pass the address of this structure across the intercore as the tag
 * on a request. When we get a response we can look up this structure and
 * deal with the various pieces.
 *
 * This tag will stick around for write requests until the WriteBuffer is
 * freed, so we will never have duplicate tags. For read requests, we can
 * free this structure after we have copied the data and freed the HLBAT 
 * resources.
 *
 * WRITE_IO requests point into the WRITE_BUFFER.
 * WRITE requests point into the dyn area
 * READ_IO requests point into the HLBAT???
 * READ requests point into the Dyn Area?
 * 
 */

/*
 * A list of core control commands not handled in the driver is kept
 * and they are queued to the userland process, and retrieved via an
 * ioctl.
 */
struct dnas_userland_queue_elt {
	struct list_head core_ctrl_elt;
	int sent_to_userland;
	int status;                   /* Signals an error */
	InterCoreCtrlVxToLx msg;
	int no_response;              /* Tells userland if they should send */
				      /* a response or not */
};

/* 
 * Not sure if all these are needed. WRITE_IO and READ_IO are for READ_XX and
 * WRITE_XX ... the others are for non IO commands. These might not be needed
 * as we might be able to tell from the BUF SRC IDs what type we are dealing 
 * with.
 */
#define DNAS_WRITE    0
#define DNAS_READ     1
#define DNAS_WRITE_IO 2
#define DNAS_READ_IO  3
#define DNAS_NOIO     4

struct dnas_tag_struct {
	int request_type;
	struct scsi_cmnd *req_p;          /* The request */
	void (*done)(struct scsi_cmnd *);
        struct mutex tag_mutex;           /* Lock the structure when needed */
	struct list_head req_queue_ent;   /* We queue requests to a thread */
};

struct buff_elt_struct;

struct dri_dnas_device {
	struct list_head device_list;
	struct Scsi_Host *shost;
	struct device dev;
	int registered;
        int first_time_valid;
	char init_name[32];               /* Name of device        */
	int scsi_enabled;		  /* SCSI Enabled          */
	int driver_init_complete;         /* All threads and driver inited */
	int dev_path_present;             /* See if the path is ready for  */
					  /* userland                      */
	struct task_struct *comm_th;      /* Comms thread          */
	struct task_struct *res_th;       /* Resource thread       */
	struct task_struct *scsi_resp_th; /* scsi response thread  */
	struct task_struct *scsi_req_th;  /* Write thread          */
	int waiting_for_core_ctrl;
	struct kmem_cache *core_ctrl_cache;
	struct mutex core_ctrl_queue_mutex;
        struct mutex scsi_completion_mutex;
	struct completion core_ctrl_completion;
	struct list_head core_ctrl_queue;
	struct list_head req_queue;       /* Queue of scsi requests */
	int req_thread_waiting;           /* When we are waiting   */
	struct completion req_completion;
	spinlock_t req_spinlock;          /* Protect access to the queue */
	void *shm_dev;                    /* Shared mem device ptr */
	int scsi_queue_handle;            /* Handle for SCSI command Q */
	int resource_queue_handle;
	int core_ctrl_queue_handle;
	struct completion buffer_completion;
	int threads_waiting_for_buffers;
	struct mutex wb_mutex;
	struct buff_elt_struct *wb_head;
	void *write_buffer_start;         /* Virtual address */
        void *write_buffer_phys;
	int write_buffer_size;
	void *hlbat_buffer_start;
        void *hlbat_buffer_phys;
	int hlbat_buffer_size;
	void *sgl_buffer_start;
        void *sgl_buffer_phys;
	int sgl_buffer_size;
	struct mutex lx_db_mutex;
	struct buff_elt_struct *lx_db_head;
	void *lx_dyn_buffer_start;
        void *lx_dyn_buffer_phys;
	int lx_dyn_buffer_size;
	void *vx_dyn_buffer_start;
        void *vx_dyn_buffer_phys;
	int vx_dyn_buffer_size;
	struct kmem_cache *dnas_tag_cache;
        /* Some stats */
        unsigned int scsi_requests;
        unsigned int scsi_complete;
};

#define to_dri_dnas_device(d) \
	container_of(d, struct dri_dnas_device, dev)

int send_scsi_request(struct dri_dnas_device *dnas_dev, 
			uint8_t *cdb, uint32_t lun,
			struct dnas_tag_struct *tag, uint32_t buf, 
			uint32_t buf_len, uint32_t src);

static int dri_dnas_queuecommand(struct scsi_cmnd *scmd,
				void (*done)(struct scsi_cmnd *));
static int dri_dnas_ioctl(struct scsi_device *dev, int cmd, void __user *arg);
int dri_dnas_abort(struct scsi_cmnd *cmd);
int dri_dnas_device_reset(struct scsi_cmnd *cmd);

void *alloc_write_buffer(struct dri_dnas_device *dnas_dev, unsigned int size);
int dump_write_buffer(struct dri_dnas_device *dnas_dev, char *buf);
int dump_lx_buffer(struct dri_dnas_device *dnas_dev, char *buf);
int dump_write_history(struct dri_dnas_device *dnas_dev, char *buf);
int dump_driver_stats(struct dri_dnas_device *dnas_dev, char *buf);
int xfer_from_request_buffs(void *buf, struct scsi_cmnd *scmd,
                            void *base_virt, void *base_phys);

static int num_luns = 0;
static int num_cmds = 0;

/*
 * Set and show the value of the debug variable
 */
static ssize_t dnas_debug_show(struct device_driver *ddp, char *buf)
{
	int len = 0;

	len = snprintf(buf, PAGE_SIZE, "debug_level = %d\n", dnas_debug_level);

	return len;
}

static ssize_t dnas_debug_set(struct device_driver *ddp, const char *buf,
				size_t count)
{
	int lcl_debug_level = 0;

	if (sscanf(buf, "%d", &lcl_debug_level) != 1)
		return -EINVAL;

	dnas_debug_level = lcl_debug_level;

	return count;
}

/*
 * Set and show the value of the drop_requests variable
 */
static ssize_t dnas_drop_req_show(struct device_driver *ddp, char *buf)
{
	int len = 0;

	len = snprintf(buf, PAGE_SIZE, "drop_requests = %d "
                        "(0 means disabled)\n", drop_requests);

	return len;
}

static ssize_t dnas_drop_req_set(struct device_driver *ddp, const char *buf,
				size_t count)
{
	int lcl_drop_requests = 0;

	if (sscanf(buf, "%d", &lcl_drop_requests) != 1)
		return -EINVAL;

	drop_requests = lcl_drop_requests;

	return count;
}

/*
 * Set the value of the delay requests variable ...
 */
static ssize_t dnas_delay_req_show(struct device_driver *ddp, char *buf)
{
	int len = 0;

	len = snprintf(buf, PAGE_SIZE, "delay_requests = %d "
                        "(0 means disabled)\n", delay_requests);

	return len;
}

static ssize_t dnas_delay_req_set(struct device_driver *ddp, const char *buf,
			  	  size_t count)
{
	int lcl_delay_requests = 0;

	if (sscanf(buf, "%d", &lcl_delay_requests) != 1)
		return -EINVAL;

	delay_requests = lcl_delay_requests;

	return count;
}

/*
 * Set the value of the delay skip variable ...
 */
static ssize_t dnas_delay_skip_show(struct device_driver *ddp, char *buf)
{
	int len = 0;

	len = snprintf(buf, PAGE_SIZE, "delay_skip = %d "
                        "(0 means skip none)\n", delay_requests);

	return len;
}

static ssize_t dnas_delay_skip_set(struct device_driver *ddp, const char *buf,
			  	  size_t count)
{
	int lcl_delay_skip = 0;

	if (sscanf(buf, "%d", &lcl_delay_skip) != 1)
		return -EINVAL;

	delay_skip = lcl_delay_skip;

	return count;
}

/*
 * Grub through various structures and print out some info
 */
static ssize_t dnas_wb_hist_show(struct device_driver *ddp, char *buf)
{
	int len = 0;
	struct dri_dnas_device *dnas_dev = NULL, *tmp = NULL;

	/*
	 * First, the free list
	 */
	list_for_each_entry_safe(dnas_dev, tmp, &dri_dnas_device_list, 
		device_list) {
		len += dump_write_history(dnas_dev, buf);
	}

	return len;
}

/*
 * Grub through various structures and print out some info
 */
static ssize_t dnas_wb_show(struct device_driver *ddp, char *buf)
{
	int len = 0;
	struct dri_dnas_device *dnas_dev = NULL, *tmp = NULL;

	/*
	 * First, the free list
	 */
	list_for_each_entry_safe(dnas_dev, tmp, &dri_dnas_device_list, 
		device_list) {
		len += dump_write_buffer(dnas_dev, buf);
	}

	return len;
}

static ssize_t dnas_read_hist_show(struct device_driver *ddp, char *buf)
{
	int len = 0;
	struct dri_dnas_device *dnas_dev = NULL, *tmp = NULL;

	list_for_each_entry_safe(dnas_dev, tmp, &dri_dnas_device_list,
		device_list) {

	}

	return len;
}

static ssize_t dnas_lx_show(struct device_driver *ddp, char *buf)
{
	int len = 0;
	struct dri_dnas_device *dnas_dev = NULL, *tmp = NULL;

	/*
	 * First, the free list
	 */
	list_for_each_entry_safe(dnas_dev, tmp, &dri_dnas_device_list, 
		device_list) {
		len += dump_lx_buffer(dnas_dev, buf);
	}

	return len;
}

static ssize_t dnas_stats_show(struct device_driver *ddp, char *buf)
{
	int len = 0;
	struct dri_dnas_device *dnas_dev = NULL, *tmp = NULL;

	list_for_each_entry_safe(dnas_dev, tmp, &dri_dnas_device_list,
		device_list) {
		len += dump_driver_stats(dnas_dev, buf);
	}

	return len;
}

static ssize_t dnas_read_serial_number(struct device_driver *ddp, char *buf)
{
        int len = 0;
        len += snprintf(&buf[len], PAGE_SIZE, "%s", dri_dnas_serial_number);

        return len;
}

/*
 * Get a message from the InterCore queue and pass it back as a series
 * of bytes. Limited functionality. Will hang until a message available
 * or we get an intr. We DO NOT copy any intercore buffers! Only the 
 * intercore message!
 */
static ssize_t dnas_icctl_get(struct device_driver *ddp, char *buf)
{
        int len = 0;
        struct dnas_userland_queue_elt *ul_elt = NULL;

        /*
         * Get the first element on the queue and copy it to the buffer.
         *
         * Lock the userland queue mutex first ...
         */
        printk(KERN_INFO "about to lock the queue\n");
        mutex_lock(&dnas_sysfs_queue_mutex);

        printk(KERN_INFO "about to look for elements\n");
        /* Find an element */
        while (list_empty(&dnas_sysfs_userland_queue)) {
                int rc = 0;
                if (!dnas_waiting_for_userland_queue) {
                        init_completion(&dnas_userland_ctrl_completion);
                        dnas_waiting_for_userland_queue = 1;
                }
                dnas_waiting_for_userland_queue = 1;
                printk(KERN_INFO "about to unlock the queue\n");
                mutex_unlock(&dnas_sysfs_queue_mutex);
                printk(KERN_INFO "about to wait for completion\n");
                rc = wait_for_completion_interruptible(
                                &dnas_userland_ctrl_completion);
                mutex_lock(&dnas_sysfs_queue_mutex);

                /*
                 * Check the error ...
                 */
                if (rc) {
                        printk(KERN_INFO "%s: interrupted, returning\n",
                               __func__);
                        mutex_unlock(&dnas_sysfs_queue_mutex);
                        dnas_waiting_for_userland_queue = 0;
                        return rc;
                }
        }

        /* We have the lock and an element is available */
        ul_elt = list_entry(dnas_sysfs_userland_queue.next, 
                                struct dnas_userland_queue_elt, core_ctrl_elt);
        list_del(&ul_elt->core_ctrl_elt);

        /* We have what we want, unlock the mutex and then copy the data */

        mutex_unlock(&dnas_sysfs_queue_mutex);

        len = sizeof(InterCoreCtrlVxToLx);

        printk(KERN_INFO "about to copy the message\n");
        memcpy(buf, (void *)&ul_elt->msg, len);

        return len;
}

static ssize_t dnas_icctl_set(struct device_driver *ddp, const char *buf,
				size_t count)
{
        int len = 0;

        return len;
}

static ssize_t dnas_no_read_acc_show(struct device_driver *ddp, char *buf)
{
	int len = 0;

        len += snprintf(&buf[len], PAGE_SIZE, "%u\n", no_read_acceleration);

	return len;
}

static ssize_t dnas_no_read_acc_set(struct device_driver *ddp, const char *buf,
				size_t count)
{
	int lcl_no_read_acc = 0;

	if (sscanf(buf, "%d", &lcl_no_read_acc) != 1)
		return -EINVAL;

        no_read_acceleration = lcl_no_read_acc;

        return count;
}

static ssize_t dnas_no_write_acc_show(struct device_driver *ddp, char *buf)
{
	int len = 0;

        len += snprintf(&buf[len], PAGE_SIZE, "%u\n", no_write_acceleration);

	return len;
}

static ssize_t dnas_no_write_acc_set(struct device_driver *ddp, const char *buf,
				size_t count)
{
	int lcl_no_write_acc = 0;

	if (sscanf(buf, "%d", &lcl_no_write_acc) != 1)
		return -EINVAL;

        no_write_acceleration = lcl_no_write_acc;

        return count;
}

static DRIVER_ATTR(debug, S_IRUGO | S_IWUSR, dnas_debug_show, dnas_debug_set);
static DRIVER_ATTR(write_buffer, S_IRUGO, dnas_wb_show, NULL);
static DRIVER_ATTR(lx_buffer, S_IRUGO, dnas_lx_show, NULL);
static DRIVER_ATTR(wb_history, S_IRUGO, dnas_wb_hist_show, NULL);
static DRIVER_ATTR(read_history, S_IRUGO, dnas_read_hist_show, NULL);
static DRIVER_ATTR(stats, S_IRUGO, dnas_stats_show, NULL);
static DRIVER_ATTR(serial, S_IRUGO, dnas_read_serial_number, NULL);
static DRIVER_ATTR(icore_control, S_IRUGO | S_IWUSR, dnas_icctl_get, 
                                                     dnas_icctl_set);
static DRIVER_ATTR(drop_requests, S_IRUGO | S_IWUSR, dnas_drop_req_show,
                                                     dnas_drop_req_set);
static DRIVER_ATTR(delay_requests, S_IRUGO | S_IWUSR, dnas_delay_req_show,
                                                      dnas_delay_req_set);
static DRIVER_ATTR(delay_skip, S_IRUGO | S_IWUSR, dnas_delay_skip_show,
                                                  dnas_delay_skip_set);
static DRIVER_ATTR(no_write_acc, S_IRUGO | S_IWUSR, dnas_no_write_acc_show,
                                                    dnas_no_write_acc_set);
static DRIVER_ATTR(no_read_acc, S_IRUGO | S_IWUSR, dnas_no_read_acc_show,
                                                   dnas_no_read_acc_set);

void dnas_create_driverfs_files(void)
{
	int ret;

	ret = driver_create_file(&dri_dnas_driverfs_driver, 
		&driver_attr_write_buffer);
	if (ret) {
		printk(KERN_INFO "%s: Error registering driver file "
			"write_buffer: %u\n", __func__, ret);
	}
	ret = driver_create_file(&dri_dnas_driverfs_driver, 
		&driver_attr_lx_buffer);
	if (ret) {
		printk(KERN_INFO "%s: Error registering driver file "
			"lx_buffer: %u\n", __func__, ret);
	}
	ret = driver_create_file(&dri_dnas_driverfs_driver, 
		&driver_attr_wb_history);
	if (ret) {
		printk(KERN_INFO "%s: Error registering driver file "
			"wb_history: %u\n", __func__, ret);
	}
	ret = driver_create_file(&dri_dnas_driverfs_driver, 
		&driver_attr_debug);
	if (ret) {
		printk(KERN_INFO "%s: Error registering driver file "
			"debug: %u\n", __func__, ret);
	}
	ret = driver_create_file(&dri_dnas_driverfs_driver, 
		&driver_attr_read_history);
	if (ret) {
		printk(KERN_INFO "%s: Error registering driver file "
			"read_history: %u\n", __func__, ret);
	}
	ret = driver_create_file(&dri_dnas_driverfs_driver, 
		&driver_attr_stats);
	if (ret) {
		printk(KERN_INFO "%s: Error registering driver file "
			"read_history: %u\n", __func__, ret);
	}
        ret = driver_create_file(&dri_dnas_driverfs_driver,
                &driver_attr_serial);
        if (ret) {
                printk(KERN_INFO "%s: Error registering driver file "
                        "serial: %u\n", __func__, ret);
        }
        ret = driver_create_file(&dri_dnas_driverfs_driver,
                &driver_attr_icore_control);
        if (ret) {
                printk(KERN_INFO "%s: Error registering driver file "
                        "icore_control: %u\n", __func__, ret);
        }
        ret = driver_create_file(&dri_dnas_driverfs_driver,
                &driver_attr_drop_requests);
        if (ret) {
                printk(KERN_INFO "%s: Error registering driver file "
                        "drop_requests: %u\n", __func__, ret);
        }
        ret = driver_create_file(&dri_dnas_driverfs_driver,
                &driver_attr_delay_requests);
        if (ret) {
                printk(KERN_INFO "%s: Error registering driver file "
                        "delay_requests: %u\n", __func__, ret);
        }
        ret = driver_create_file(&dri_dnas_driverfs_driver,
                &driver_attr_delay_skip);
        if (ret) {
                printk(KERN_INFO "%s: Error registering driver file "
                        "delay_skip: %u\n", __func__, ret);
        }
        ret = driver_create_file(&dri_dnas_driverfs_driver,
                &driver_attr_no_write_acc);
        if (ret) {
                printk(KERN_INFO "%s: Error registering driver file "
                        "no_write_acc: %u\n", __func__, ret);
        }
        ret = driver_create_file(&dri_dnas_driverfs_driver,
                &driver_attr_no_read_acc);
        if (ret) {
                printk(KERN_INFO "%s: Error registering driver file "
                        "no_read_acc: %u\n", __func__, ret);
        }
}

static int dri_dnas_proc_info(struct Scsi_Host *host, char *buffer,
				char **start, off_t offset, int length,
				int inout)
{
	int len, pos, begin;

	if (inout == 1)
		return -EACCES; /* No one can write just yet */

	begin = 0;
	pos = len = sprintf(buffer, "dri_dnas adapter driver, version "
			"%s [%s]\nnum_luns=%d, num_cmds=%d\n",
			DRI_DNAS_VERSION, dri_dnas_version_date, num_luns, 
			num_cmds);

	if (pos < offset) {
		len = 0;
		begin = pos;
	}
	if (start)
		*start = buffer + (offset - begin);
	len -= (offset - begin);
	if (len > length)
		len = length;

	return len;
}

static char dri_dnas_info_buf[256];

static const char *dri_dnas_info(struct Scsi_Host *shp)
{
	sprintf(dri_dnas_info_buf, "dri_dnas, version %s [%s], "
		"Commands: %d, Aborts: 0, Device Resets: 0",
		DRI_DNAS_VERSION, dri_dnas_version_date,
		num_cmds);

	return dri_dnas_info_buf;
}

/*
 * The host template we will register
 */
static struct scsi_host_template dri_dnas_host_template = {
	.proc_info			= dri_dnas_proc_info,
	.proc_name			= dri_dnas_proc_name,
	.name				= "dri_dnas_drobo",
	.info				= dri_dnas_info,
	.queuecommand			= dri_dnas_queuecommand,
	.ioctl				= dri_dnas_ioctl,
   	.eh_abort_handler		= dri_dnas_abort,
   	.eh_device_reset_handler	= dri_dnas_device_reset,
	.sg_tablesize			= DRI_DNAS_MAX_SG_SEGMENTS,
	.can_queue			= DRI_DNAS_MAX_QUEUE,
	.this_id			= 15,
	.cmd_per_lun			= 32,
	.max_sectors			= DRI_MAX_SECTORS,
	.use_clustering			= DISABLE_CLUSTERING,
	.skip_settle_delay		= 1,
	.module				= THIS_MODULE,
/*
 * Some stuff for later
 * 	.eh_target_reset_handler	= dri_dnas_target_reset,
 */
};

int dri_dnas_abort(struct scsi_cmnd *cmd)
{
	struct dri_dnas_device *dnas_dev = NULL;
        struct dnas_tag_struct *tag = NULL;

	dnas_dev = to_dri_dnas_device(scsi_get_device(cmd->device->host));
	tag = (struct dnas_tag_struct *)cmd->host_scribble;

	mutex_lock(&dnas_dev->scsi_completion_mutex);
        printk(KERN_INFO "%s: called to abort cmnd: %p, %p\n", __func__, cmd,
                tag);
        /*
         * If the tag is null, we were on the edge of a timeout and the 
         * SCSI Midlayer scheduled the abort around the same time the
         * command completed.
         */
        if (tag) {
                /*
                 * We have the tag now, so lock it because we are changing
                 * things. Note, that the protocol here is that we lock the
                 * tag after locking the scsi_completion_mutex ... and
                 * request processing locks the tag while working on it. The
                 * completion thread only locks the scsi_completion_mutex
                 * since a tag can only be being handled by one of our two
                 * threads at a time.
                 */
                mutex_lock(&tag->tag_mutex);
                tag->req_p = NULL;   /* Command aborted! */
                mutex_unlock(&tag->tag_mutex);
        }
	mutex_unlock(&dnas_dev->scsi_completion_mutex);
        return SUCCESS;
}

int dri_dnas_device_reset(struct scsi_cmnd *cmd)
{
        printk(KERN_INFO "%s: called to reset device, cmnd: %p\n", __func__,
                cmd);
        return FAILED;
}

/*
 * The ioctl function. We interface with the core control module etc.
 *
 * The core control tread queues any commands from the VxCore that it does
 * not handle to the userland_queue, and we peel one off for each _GET_MSG
 * request. If not are available, we wait.
 *
 * When we get a _SEND_MSG, we rummage through our list of requests sent to 
 * userland, match the tag and use that to copy any data to and then send
 * the response. The response is sent from process context.
 *
 * We use a list of requests waiting to go to userland and a list of items sent
 * to userland and a mutex for each. 
 */

/*
 * Find the element with tag tag
 */
struct dnas_userland_queue_elt *find_core_ctrl(unsigned int tag,
			struct dri_dnas_device *dnas_dev)
{
	struct dnas_userland_queue_elt *ul_elt = NULL, *tmp = NULL;

	list_for_each_entry_safe(ul_elt, tmp, &dnas_dev->core_ctrl_queue,
				core_ctrl_elt) {
		if (ul_elt->msg.tag == tag)
			return ul_elt;
	}
	return NULL;
}

/*
 * Find the first element that has not been sent to userland
 */
struct dnas_userland_queue_elt *find_unsent_core_ctrl(
			struct dri_dnas_device *dnas_dev)
{
	struct dnas_userland_queue_elt *ul_elt = NULL, *tmp = NULL;

	list_for_each_entry_safe(ul_elt, tmp, &dnas_dev->core_ctrl_queue,
				core_ctrl_elt) {
		if (!ul_elt->sent_to_userland)
			return ul_elt;
	}
	return NULL;
}

static int process_get_msg(struct dri_dnas_device *dnas_dev, void __user *arg)
{
	core_ctl_lx_user_recv recv_msg;
	unsigned long size = 0, len = 0;
	struct dnas_userland_queue_elt *ul_elt;
	void *buf = NULL;

	/*
	 * We need to write to this area later
	 */
	if (!access_ok(VERIFY_WRITE, arg, size)) {
		DBG(5, KERN_INFO "%s: Unable to access area of size %lu" 
			" pointed to!\n", __func__, size);
		return -EFAULT;
	}

	len = copy_from_user((void *)&recv_msg, (void __user *)arg,
				sizeof(core_ctl_lx_user_recv));
	if (len) {
		DBG(5, KERN_INFO "%s: Unable to copy area of size %lu (%lu) "
			"pointed to!\n", __func__, size, len);
		return -EFAULT;
	}

	DBG(5, KERN_INFO "cmd_size: %u, data_size: %u, s: %lu\n",
		recv_msg.cmd_size, recv_msg.data_size, size);

	/*
	 * Sanity tests before we wait for anything
	 */
	if (recv_msg.cmd_size < sizeof(InterCoreCtrlVxToLx)) {
		DBG(5, KERN_INFO "%s: buffer to receive intercore ctrl msg "
			"too small: %u should be: %u\n", __func__,
			recv_msg.cmd_size, sizeof(InterCoreCtrlVxToLx));
		return -EINVAL;
	}

	if (recv_msg.cmd_buf == NULL || recv_msg.data_buf == NULL) {
		DBG(5, KERN_INFO "%s: invalid cmd buf or data buf pointer -"
			" cmdp: %p, datap: %p\n", __func__, recv_msg.cmd_buf,
			recv_msg.data_buf);
		return -EINVAL;
	}

	/*
	 * Now, grab the mutex, check the list and so forth
	 */
	mutex_lock(&dnas_dev->core_ctrl_queue_mutex);

	while (!(ul_elt = find_unsent_core_ctrl(dnas_dev))) {
                int rc = 0;
		if (!dnas_dev->waiting_for_core_ctrl) {
			init_completion( &dnas_dev->core_ctrl_completion);
			dnas_dev->waiting_for_core_ctrl = 1;
		}
		mutex_unlock(&dnas_dev->core_ctrl_queue_mutex);
		rc = wait_for_completion_interruptible(
		        	&dnas_dev->core_ctrl_completion);
		mutex_lock(&dnas_dev->core_ctrl_queue_mutex);
                /*
                 * It is not clear to me whether we will race with the
                 * other thread that wants the above lock in the interrupt
                 * case, but it should not be a problem to place the cleanup
                 * after reaquiring the lock.
                 */
                if (rc) { /* We were interrupted, so cleanup and exit */
                        printk(KERN_INFO "%s: cleaning up after interrupt\n",
                                __func__);
                        dnas_dev->waiting_for_core_ctrl = 0;
		        mutex_unlock(&dnas_dev->core_ctrl_queue_mutex);
                        return rc;
                }
	}

	/*
	 * If the device is being yanked we get a bad status
	 * here ... return it to userland
	 */
	if (ul_elt->status) {
                printk(KERN_INFO "%s:Err trying to free elt: %p, cache: %p\n",
                        __func__, ul_elt, dnas_dev->core_ctrl_cache);
		list_del(&ul_elt->core_ctrl_elt);
		mutex_unlock(&dnas_dev->core_ctrl_queue_mutex);
		kmem_cache_free(dnas_dev->core_ctrl_cache, ul_elt);
		return ul_elt->status;
	}

	/*
	 * Now copy the data pointed to in the msg to userspace
	 * We should only do this for messages that require
	 * this, but that means we have to parse the damn
	 * requests here. They should be in VX Dyn memory
	 *
	 * However, only copy if the buffer is valid.
	 *
	 * Then we copy the message itself. 
	 */
	size = 0;  /* Incase we do not copy below */
	if (ul_elt->msg.u.fileReq.bufId.bufSrc != 
		INTER_CORE_CMD_PARTITION_ID_INVALID) {
		size = ul_elt->msg.u.fileReq.bufId.length;

		/* XXX: Fixme, what if wrong region! */
		buf = dnas_dev->vx_dyn_buffer_start + 
			ul_elt->msg.u.fileReq.bufId.bufOffset;

		if (size > recv_msg.data_size) {
			DBG(5, KERN_INFO "%s: User buffer too short! "
				"user_size: %u, size: %lu\n:",
				__func__, recv_msg.data_size, size);
                        size = recv_msg.data_size; 
			/* mutex_unlock(&dnas_dev->core_ctrl_queue_mutex);
			return -EINVAL; */
		}

		/*
		 * Copy the buffer
		 */
		if (!access_ok(VERIFY_WRITE, recv_msg.data_buf, 
				recv_msg.data_size)) {
			mutex_unlock(&dnas_dev->core_ctrl_queue_mutex);
			DBG(5, KERN_INFO "%s: Unable to get access to "
				"requested user space mem of size %u\n",
				__func__, recv_msg.data_size);
			return -EFAULT;
		}

		len = copy_to_user((void __user *)recv_msg.data_buf,
					(void *)buf, size);
		if (len) {
			DBG(5, KERN_INFO "%s: Unable to copy %lu (%lu) "
				", buf: %p, userp: %p\n", __func__, size, len, 
				buf, recv_msg.data_buf);
			mutex_unlock(&dnas_dev->core_ctrl_queue_mutex);
			return -EFAULT; /* Msg lost now! */
		}
	}

	/* XXX: Fixme, use structures here! */
	__put_user(size, (unsigned int *)(arg + 24));

	/*
	 * Copy the message now as well
	 */
	size = MIN(sizeof(ul_elt->msg), recv_msg.cmd_size);
	if (!access_ok(VERIFY_WRITE, recv_msg.cmd_buf, size)) {
		DBG(5, KERN_INFO "%s: Unable to get access to requested "
			"user space msg of size %lu\n",__func__, size);
		mutex_unlock(&dnas_dev->core_ctrl_queue_mutex);
		return -EFAULT;
	}

	len = copy_to_user((void __user *)recv_msg.cmd_buf,
				(void *)&ul_elt->msg, size);
	if (len) {
		DBG(5, KERN_INFO "%s: User cmd buffer too short! cmd size: "
			"%u, a: %lu, l: %lu\n", __func__, recv_msg.cmd_size, 
			size, len); 
		return -EFAULT;
	}

	/*
	 * Copy the no_response as well
	 */

	__put_user(ul_elt->no_response, 
			(unsigned int *)&(recv_msg.no_response));

	ul_elt->sent_to_userland = 1;

	/*
	 * update the actual data amounts copied to userland
	 * This calculation should be done on structures
	 */
	__put_user(size, (unsigned int *)(arg + 12));

        /*
         * If response not needed, free the elt ... no leaks
         */
        if (ul_elt->no_response) {
                printk(KERN_INFO "%s:NR trying to free elt: %p, cache: %p\n",
                        __func__, ul_elt, dnas_dev->core_ctrl_cache);
		list_del(&ul_elt->core_ctrl_elt);
		kmem_cache_free(dnas_dev->core_ctrl_cache, ul_elt);
                DBG(5, KERN_INFO "%s: Free'd a no_response elt\n", __func__);
        }

	mutex_unlock(&dnas_dev->core_ctrl_queue_mutex);

	return 0;
}

static int dri_dnas_ioctl(struct scsi_device *dev, int cmd, void __user *arg)
{
	struct dri_dnas_device *dnas_dev = NULL;
	int ret = 0, size = 0, len = 0;
	struct dnas_userland_queue_elt *ul_elt;
	void *buf = NULL;

	dnas_dev = to_dri_dnas_device(scsi_get_device(dev->host));

	DBG(5, KERN_INFO "%s: ioctl cmd: %u, device: %p\n", 
		__func__, cmd, dnas_dev);

	if (!arg) {
		return -EINVAL;
	}

	size = _IOC_SIZE(cmd);

	switch (cmd) {
        case BLKFLSBUF:
                /* 
                 * We will send a SYNC CACHE request soon, but for now ignore
                 * this to quieten the error message.
                 */

                break;

	case DRI_NAS_GET_MSG:
		ret = process_get_msg(dnas_dev, arg);
		break;

	case DRI_NAS_SEND_MSG:
		{
			core_ctl_lx_user_send send_msg;
			InterCoreCtrlLxToVx reply;
			SharedMemQueueSendMsg send_args;

			/*
			 * We need to write to this area later
			 */
			if (!access_ok(VERIFY_WRITE, arg, size)) {
				return -EFAULT;
			}

			len = copy_from_user((void *)&send_msg, 
					(void __user *)arg,
					sizeof(core_ctl_lx_user_send));
			if (len) {
				return -EFAULT;
			}

			if (send_msg.cmd_size < sizeof(InterCoreCtrlLxToVx)){
				DBG(5, KERN_INFO "%s: buffer to send"
					"intercore ctrl reply too small: %u "
					" should be: %u\n", __func__,
					send_msg.cmd_size,
					sizeof(InterCoreCtrlLxToVx));
				return -EINVAL;
			}

			DBG(5, KERN_INFO "%s: Got the send_msg ...\n", 
				__func__);

			/*
			 * Copy the reply from userspace
			 */
			size = MIN(send_msg.cmd_size, 
				sizeof(InterCoreCtrlLxToVx));
			len = copy_from_user((void *)&reply, 
					(void *)send_msg.cmd_buf,
					size);

			/*
			 * We need to find the original message
			 * Then we copy the data to its buffer as specified
			 * in send_msg or the original command
			 * Then send it directly from here and delete the
			 * original message.
			 */

			mutex_lock(&dnas_dev->core_ctrl_queue_mutex);

			if (!(ul_elt = find_core_ctrl(reply.tag, 
						dnas_dev))) {
				mutex_unlock(&dnas_dev->core_ctrl_queue_mutex);
				DBG(5, KERN_INFO "%s: Unable to find command "
					"message relates to: %0X\n", __func__,
					reply.tag);
				return -EINVAL;
			}

			list_del(&ul_elt->core_ctrl_elt);

			mutex_unlock(&dnas_dev->core_ctrl_queue_mutex);

			DBG(5, KERN_INFO "%s: Got the core ctrl queue elt\n",
				__func__);

			buf = dnas_dev->vx_dyn_buffer_start + 
				ul_elt->msg.u.fileReq.bufId.bufOffset;

			/*
			 * Now, copy the data from userland.
			 */
			if (send_msg.data_buf) {
				size = MIN(ul_elt->msg.u.fileReq.bufId.length,
					send_msg.data_size);
				len = copy_from_user(
					(void __user *)send_msg.data_buf,
					(void *)buf, size);
				if (len) {
					return -EFAULT;
				}
			}

			/*
			 * Now send the response and free the structure 
			 */
			send_args.handle = dnas_dev->core_ctrl_queue_handle;
			send_args.msg         = (void *)&reply;
			send_args.msgSize     = sizeof(send_msg);
			send_args.flags       = SHMQ_SEND_FLAG_WAIT_FOREVER;
			send_args.timeoutInMS = 0;
			ret = shm_queue_kern_send_msg(dnas_dev->shm_dev,
						&send_args);
			if (ret) {
				DBG(5, KERN_INFO "%s: failed to send response "
					"to core ctrl message: %d, ignored!\n",
					__func__, ret);
			}

			kmem_cache_free(dnas_dev->core_ctrl_cache, ul_elt);

		}
		break;

	default:
		DBG(3, KERN_INFO "%s: Invalid IOCTL request: %0X",__func__,
			cmd);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*
 * Fill this in ... later
 */
static void copy_sense(struct scsi_cmnd *cmd, uint8_t *sense_buf, 
			unsigned int sense_len)
{
	unsigned int actual_len = MIN(sense_len, SCSI_SENSE_BUFFERSIZE);
	memcpy(cmd->sense_buffer, sense_buf, actual_len);
}

/*
 * Utility function to perform done processing. Can also do error injection
 * here if needed.
 *
 * However, if the cmd has been cancelled, don't do anything.
 */
static int dri_dnas_send_resp(struct scsi_cmnd *cmd,
				uint8_t *sense_buf,
				unsigned int sense_len,
				void (*done)(struct scsi_cmnd *),
				int scsi_result)
{
	int ret = 0;

        /* Clean up this so no one gets confused */
        cmd->host_scribble = NULL;

	/*
	 * Do we need to simulate autosense?
	 */
	if (cmd && sense_buf) {
		if (SAM_STAT_CHECK_CONDITION == (scsi_result & 0xFF))
			DBG(5, KERN_INFO "%s: Copying sense data: len %u\n",
                                __func__, sense_len);
			copy_sense(cmd, sense_buf, sense_len);
	}

	if (cmd)
		cmd->result = scsi_result;

	if (done)
		done(cmd);

	return ret;
}

/*
 * The Queue Command function called by the mid layer ...
 *
 * We allocate a tag structure to tie this command together with other stuff
 * we need, do some initial processing and then pass it the Vx core. The rest
 * of the processing is handled in the scsi thread.
 */
static int dri_dnas_queuecommand(struct scsi_cmnd *scmd,
				void (*done)(struct scsi_cmnd *))
{
	int ret = 0;
	struct dri_dnas_device *dnas_dev = NULL;
	struct dnas_tag_struct *tag = NULL;
	unsigned char cmd = scmd->cmnd[0];

	dnas_dev = to_dri_dnas_device(scsi_get_device(scmd->device->host));

	DBG(5, KERN_INFO "%s: Rqst: %p SCSI cmd: %u, device: %p, req_len: %u\n",
		__func__, scmd, cmd, dnas_dev, scsi_bufflen(scmd));

	if (scmd->device->id == scmd->device->host->hostt->this_id) {
		printk(KERN_ERR "%s: initiator's id used as target\n",
			__func__);
		return dri_dnas_send_resp(scmd, NULL, 0, done, 
					DID_NO_CONNECT << 16);
	}

        dnas_dev->scsi_requests++;

        /* Should we drop the request? */
        if (drop_requests) {
                /* Drop every drop_requests request */
                if ((dnas_dev->scsi_requests % drop_requests) == 0) {
                        DBG(5, "%s: Rqst: %p SCSI cmd: %u, device: %p "
                        "dropped!\n", __func__, scmd, cmd, dnas_dev);
                        return ret;
                }
        }

	scsi_set_resid(scmd, 0);

	/*
	 * We need ATOMIC here, we ARE in ATOMIC
	 * context here because our caller has done spin_lock_irqsave.
	 *
	 * We could unlock and allow others to preempt us, but we do so
	 * little work here that it does not seem worth it.
	 */
	tag = kmem_cache_alloc(dnas_dev->dnas_tag_cache, GFP_ATOMIC);
	if (!tag) {
		printk(KERN_ERR "%s: out of memory at line %d\n",
			__func__, __LINE__);
                dump_stack();
		return -ENOMEM;
	}

        mutex_init(&tag->tag_mutex);
	tag->req_p = scmd;
	tag->done  = done;

        /*
         * We link back to the tag so that we can find the tag when we are
         * asked to abort the command
         */
        scmd->host_scribble = (unsigned char *)tag;

	/*
	 * Now queue the request to the request thread. Our caller is holding
	 * a spin_lock_irqsave which protects us. No one can preempt or 
	 * interrupt us.
	 */

	list_add_tail(&tag->req_queue_ent, &dnas_dev->req_queue);
	if (dnas_dev->req_thread_waiting) {
		/*
		 * Make sure we only come through here once per sleep event
		 * for the other thread
		 */
		dnas_dev->req_thread_waiting = 0;
		complete(&dnas_dev->req_completion);
	}

	return ret;
}

/*
 * Transfer the data from the scsi request to the buffer provided.
 *
 * We assume that the buffer has the correct size.
 *
 * We return 0 for success, a SCSI error otherwise
 */
static int write_block_count = 0;

extern void dri_memcpy(void *to, void *from, __kernel_size_t n, 
                       void *base_virt, void *base_phys);

int xfer_from_request_buffs(void *buf, struct scsi_cmnd *scmd, 
                            void *base_virt, void *base_phys)
{
	int ret = 0;
	unsigned int size = scsi_bufflen(scmd);
	unsigned int tot_size = 0;

	if (!scmd->use_sg) {  /* XXX: Turn this into a macro */

		DBG(5, ("Non SG write-like request\n"));

                if (no_write_acceleration)
                        asm_memmove(buf, scmd->request_buffer, size);
                else
		        dri_memcpy(buf, scmd->request_buffer, size, base_virt,
                                base_phys);

		ret = size;

	} else {
		struct scatterlist *sgp = 
			(struct scatterlist *)scmd->request_buffer;
		int k;
		void *kaddr, *kaddr_off;

		for (k = 0; k < scmd->use_sg; k++, sgp++) {

			/*
			 * Map the page. Is there more than one page? If we 
			 * enable clustering we have to be prepared for 
			 * multiple pages in every sgp!
			 *
			 * FIXME: This works because all physical memory
			 * fits between PAGE_OFFSET and highmem ... and
			 * PAGE_OFFSET is 0x8000000 for us.
			 */
			kaddr = page_address(sgp->page);
			if (!kaddr) {
				printk(KERN_INFO "%s: failed to kmap\n",
					__func__); 
				return (DID_ERROR << 16);
			}
			kaddr_off = (unsigned char *)kaddr + sgp->offset;
			flush_dcache_page(sgp->page);

                        /* FIXME: This only works for one page NOW! */

                        if (no_write_acceleration)
                                asm_memmove(buf + tot_size, kaddr_off, 
                                        sgp->length);
                        else
			        dri_memcpy(buf + tot_size, kaddr_off, 
                                        sgp->length, base_virt, base_phys);
			tot_size += sgp->length;

			write_block_count++;

			/* kunmap_atomic(sgp->page, KM_USER0); */
		}
	}

	if (tot_size != size) {
		printk(KERN_INFO "%s: Unable to transfer all requested data!"
                        "size = %u, tot_size = %u\n", __func__, size, 
                        tot_size);
		return (DID_ERROR << 16);
	}

	return ret;
}

int dump_driver_stats(struct dri_dnas_device *dnas_dev, char *buf)
{
	unsigned len = 0;

	len += snprintf(&buf[len], PAGE_SIZE - len, 
			"Pages copied to J1: %u\n", write_block_count);
	return len;
}

/*
 * Transfer data to the buffers specified in the request. We have to figure
 * out where the data is based on the buffer id and then what to do, depending
 * on whether or not we have an SGL we are pointing to or a buffer.
 *
 * This has to deal with scatter gather lists from above us and scatter gather
 * lists from below us.
 *
 * The VxCore might give us a Dyn Buffer or an SGL. Convert Dyn Buffers to
 * a single element SGL to make the coding easier.
 *
 * We return 0 for success, SCSI error otherwise.
 */
#define get_buffer(_sgle_, _bufSrc_, _dnas_dev_, _buf_, _len_, _virt_, _phys_) \
  if ((_bufSrc_) != INTER_CORE_CMD_PARTITION_ID_INVALID) { \
    if ((_bufSrc_) == INTER_CORE_CMD_PARTITION_ID_VXWORKS_DYN_MEM) { \
      (_buf_) = (uint8_t *)((_dnas_dev_)->vx_dyn_buffer_start + \
              (_sgle_)->bufOffset); \
      (_virt_) = (void *)((_dnas_dev_)->vx_dyn_buffer_start); \
      (_phys_) = (void *)((_dnas_dev_)->vx_dyn_buffer_phys); \
    } else if ((_bufSrc_) == INTER_CORE_CMD_PARTITION_ID_J1) { \
      (_buf_) = (uint8_t *)((_dnas_dev_)->write_buffer_start + \
              (_sgle_)->bufOffset); \
      (_virt_) = (void *)((_dnas_dev_)->write_buffer_start); \
      (_phys_) = (void *)((_dnas_dev_)->write_buffer_phys); \
    } else if ((_bufSrc_) == INTER_CORE_CMD_PARTITION_ID_HLBAT_CACHE) { \
      (_buf_) = (uint8_t *)((_dnas_dev_)->hlbat_buffer_start + \
              (_sgle_)->bufOffset); \
      (_virt_) = (void *)((_dnas_dev_)->hlbat_buffer_start); \
      (_phys_) = (void *)((_dnas_dev_)->hlbat_buffer_phys); \
    } else { \
      printk(KERN_INFO "%s: Unknown BUF ID in SGL Element\n", __func__); \
    } \
  } else { \
    if ((_sgle_)->bufSrc == INTER_CORE_CMD_PARTITION_ID_VXWORKS_DYN_MEM) { \
      (_buf_) = (uint8_t *)((_dnas_dev_)->vx_dyn_buffer_start + \
              (_sgle_)->bufOffset); \
      (_virt_) = (void *)((_dnas_dev_)->vx_dyn_buffer_start); \
      (_phys_) = (void *)((_dnas_dev_)->vx_dyn_buffer_phys); \
    } else if ((_sgle_)->bufSrc == INTER_CORE_CMD_PARTITION_ID_J1) { \
      (_buf_) = (uint8_t *)((_dnas_dev_)->write_buffer_start + \
              (_sgle_)->bufOffset); \
      (_virt_) = (void *)((_dnas_dev_)->write_buffer_start); \
      (_phys_) = (void *)((_dnas_dev_)->write_buffer_phys); \
    } else if ((_sgle_)->bufSrc == INTER_CORE_CMD_PARTITION_ID_HLBAT_CACHE) { \
      (_buf_) = (uint8_t *)((_dnas_dev_)->hlbat_buffer_start + \
              (_sgle_)->bufOffset); \
      (_virt_) = (void *)((_dnas_dev_)->hlbat_buffer_start); \
      (_phys_) = (void *)((_dnas_dev_)->hlbat_buffer_phys); \
    } else { \
      printk(KERN_INFO "%s: Unknown BUF ID in SGL Element\n", __func__); \
    } \
  } \
  (_len_) = (_sgle_)->bufLength; \
  (_sgle_)++;

extern void use_mm(struct mm_struct *mm);
extern void unuse_mm(struct mm_struct *mm);

int xfer_to_request_buffs(struct dri_dnas_device *dnas_dev, 
		struct dnas_tag_struct *dnas_tag, InterCoreBufferId *id)
{
	struct scsi_cmnd *scmd = dnas_tag->req_p;
	uint8_t *vx_buffer = NULL;
	unsigned int vx_buffer_len = 0, tot_len = 0;
	void *kaddr, *kaddr_off;
	int k;
	struct scatterlist *sgp;
	unsigned int returned_len = id->length;
	unsigned int bufSrc = id->bufSrc;
	InterCoreSGL *sgl = NULL;
	InterCoreSGLElement *sgle = (InterCoreSGLElement *)id;
	unsigned int sgl_count = 1;
        void *base_virt = NULL, *base_phys = NULL;

	/*if (id->bufSrc == INTER_CORE_CMD_PARTITION_ID_VXWORKS_DYN_MEM) {
		returned_len = id->length;
	} else if (id->bufSrc == INTER_CORE_CMD_PARTITION_ID_HLBAT_CACHE) {
		vx_buffer = (uint8_t *)(dnas_dev->hlbat_buffer_start +
				id->bufOffset);
	} else*/

        if (id->bufSrc == INTER_CORE_CMD_PARTITION_ID_SGL_MEM) {
		/* An SGL, it has been vetted in some way, so process */
		sgl = (InterCoreSGL *)(dnas_dev->sgl_buffer_start + 
			id->bufOffset);
		bufSrc = sgl->bufSrc;
		sgle = (InterCoreSGLElement *)&sgl->elements;
		sgl_count = sgl->numElements;
		returned_len = sgl->totalLength;
	}

	/*
	 * If we were not given an SGL by the Mid Layer, copy from the 
	 * VxCore buffers, which are an SGL, even if with only one element.
	 */	
	if (!scmd->use_sg) {
		unsigned int data_copied = 0;
		unsigned int tot_len = scsi_bufflen(scmd);
		unsigned int elt_len;

		/*
		 * This code is wrong. We need to flush each page!
		 */
		DBG(5, KERN_INFO "We got a non-sgl request ...\n");
		while (sgl_count && (data_copied < tot_len)) {
			get_buffer(sgle, bufSrc, dnas_dev, vx_buffer, 
				vx_buffer_len, base_virt, base_phys);
			elt_len = MIN(vx_buffer_len, (tot_len - data_copied));
                        if (no_read_acceleration)
                                asm_memmove(scmd->request_buffer + data_copied,
                                        vx_buffer, elt_len);
                        else
			        dri_memcpy(scmd->request_buffer + data_copied,
				        vx_buffer, elt_len, base_virt, 
                                        base_phys);
			data_copied += elt_len;
			flush_dcache_page(virt_to_page(scmd->request_buffer));
		}
		scsi_set_resid(scmd, tot_len - returned_len);

		return 0;
	} else {
		unsigned int lxsgle_len, remain_vxsgle = 0;
		unsigned int start_vxsgle = 0;

		sgp = (struct scatterlist *)scmd->request_buffer;

		for (k = 0; k < scmd->use_sg; k++, sgp++) {
			unsigned int start_lxsgle = 0;
			unsigned int cur_len = 0;
			unsigned int flush_offs = 0;

			/*
			 * Map the page. Is there more than one page?
			 */
			/* kaddr = (unsigned char *)kmap_atomic(sgp->page, 
						KM_USER0); */
			kaddr = page_address(sgp->page);
			if (!kaddr) {
				printk(KERN_INFO "Failed to kmap\n");
				return (DID_ERROR << 16);
			}
			kaddr_off = (unsigned char *)kaddr + sgp->offset;

			/*
			 * Get the length of the current Linux SG element
			 */
			lxsgle_len = sgp->length;

			/*printk("lxsgle_len: 0x%X\n", lxsgle_len);*/
			/*
			 * If anything remains from the last Vx SGLe, copy it
			 */
			if (remain_vxsgle) {
				start_vxsgle = vx_buffer_len - remain_vxsgle;
				cur_len = MIN(remain_vxsgle, lxsgle_len);
                                if (! ((returned_len >= 0x10000) && (dnas_debug_level == 2)))
                                {
                                if (no_read_acceleration)
                                        asm_memmove(kaddr_off, 
                                                vx_buffer + start_vxsgle,
                                                cur_len);
                                else
				        dri_memcpy(kaddr_off, 
					        vx_buffer + start_vxsgle, 
                                                cur_len, base_virt, base_phys);
                                }
				remain_vxsgle = remain_vxsgle - cur_len;
				tot_len += cur_len;
				start_vxsgle += cur_len;
				start_lxsgle = cur_len;
				lxsgle_len -= cur_len;
				if (remain_vxsgle)
					goto cleanup; /* This Lx SGLe is done */
			}

			/*
			 * Now copy as many Vx SGLes as will fit. remain_vxsgle
			 * tells us how much of the last one will fit.
			 */
			while (lxsgle_len && (tot_len < returned_len)) {
				if (!remain_vxsgle) {
					get_buffer(sgle, bufSrc, dnas_dev, 
						vx_buffer, vx_buffer_len, 
                                                base_virt, base_phys);
					remain_vxsgle = vx_buffer_len;
					start_vxsgle = 0;
				}
				cur_len = MIN(lxsgle_len, remain_vxsgle);
                                if (! ((returned_len >= 0x10000) && (dnas_debug_level == 2)))
                                {
                                if (no_read_acceleration)
                                        asm_memmove(kaddr_off + start_lxsgle,
                                                vx_buffer + start_vxsgle,
                                                cur_len);
                                else
				        dri_memcpy(kaddr_off + start_lxsgle, 
					        vx_buffer + start_vxsgle, 
                                                cur_len, base_virt, base_phys);
                                }
				remain_vxsgle -= cur_len;
				start_vxsgle += cur_len;
				start_lxsgle += cur_len;
				lxsgle_len -= cur_len;
				tot_len += cur_len;
			}

cleanup:
			/*
			 * Make sure that the page is flushed in the cache. If
			 * the SGL crosses a page boundary, flush each page.
			 *
			 * NOTE! We are re-using kaddr_off and incrementing
			 * it here!
			 */
			lxsgle_len = sgp->length;
			flush_offs = sgp->offset;
			while (lxsgle_len) {
				flush_dcache_page(virt_to_page(kaddr_off));
				flush_icache_range((unsigned long)kaddr_off,
					(unsigned long)kaddr_off + sgp->length); 
				/*
				 * Figure out how far to the next page
				 */
				if (flush_offs + lxsgle_len > PAGE_SIZE) {
					DBG(5, KERN_INFO "Flushing more than "
                                                "one page!\n");
					lxsgle_len -= (PAGE_SIZE - flush_offs);
					kaddr_off += (PAGE_SIZE - flush_offs);
					flush_offs = 0;
				} else {
					lxsgle_len = 0;
				}
			}
		}

		/* Set the resid correctly, I think */
		scsi_set_resid(scmd, scsi_bufflen(scmd) - returned_len);
	}

	return 0;
}

/*
 * Elements in the write buffer free list are doubly linked and cover the
 * element that is free. On free, we coalesce what we can.
 */

struct buff_elt_struct {
	unsigned int size;            /* Size of this item, including header */
	struct buff_elt_struct *next;  /* pointer to next item! */
	struct buff_elt_struct *prev;
};

#define get_write_buff_addr(_dnas_dev_, _off_) \
	(((void *)(_dnas_dev_)->write_buffer_start) + (_off_))
#define get_lx_dyn_buff_addr(_dnas_dev_, _off_) \
	(((void *)(_dnas_dev_)->lx_dyn_buffer_start) + (_off_))

/*
 * Set up the initial write buffer pool. Assumes that no one else is running
 * and thus trying to access the write buffer area. This is copied from 
 * Write_Buffer.cpp
 */
void init_write_buffer(struct dri_dnas_device *dnas_dev)
{
	SharedPartitionInfoStruct *wb_area = NULL;

	wb_area = (SharedPartitionInfoStruct *)dnas_dev->write_buffer_start;

	wb_area->SharedPartitionSize = dnas_dev->write_buffer_size;
	wb_area->SharedPartitionDataStart = 
		(void *)wb_area + sizeof(SharedPartitionInfoStruct);
	wb_area->SharedPartitionDataRegionSize = dnas_dev->write_buffer_size -
		SHARED_PARTITION_BLOCK_SIZE;

	wb_area->SharedPartitionFreeList = 
		wb_area->SharedPartitionDataStart;
	wb_area->SharedPartitionDataRegionNumBlocks = 
		wb_area->SharedPartitionDataRegionSize /
			SHARED_PARTITION_BLOCK_SIZE;
	wb_area->SharedPartitionBlocksUsed = 0;
	wb_area->SharedPartitionFreeList->alloc_size = 0;
	wb_area->SharedPartitionFreeList->num_blocks_in_list =
		wb_area->SharedPartitionDataRegionNumBlocks;
	wb_area->SharedPartitionFreeList->next = NULL;
	wb_area->SharedPartitionFreeList->allocated = 0;
	wb_area->allocFailedCount = 0;
	wb_area->allocMultipleFailureCount = 0;
	wb_area->highestFailureCount = 0;

	printk(KERN_INFO "%s: dev: %p, FreeList: %p, FreeBlocks: %u\n",
		__func__, dnas_dev, wb_area->SharedPartitionFreeList,
		wb_area->SharedPartitionDataRegionNumBlocks);
#if 0
	/*
	 * Mark all the sectors as being free, except the header
	 */
	TBD
#endif

	/*
	 * We are done here 
	 */
}

/*
 * Debug info
 */
struct write_buffer_debug {
	unsigned int blocks;
	void *offset;
};
static struct write_buffer_debug wb_alloc_history[128];
static unsigned int wb_alloc_hist_next = 0;

int dump_write_buffer(struct dri_dnas_device *dnas_dev, char *buf)
{
	SharedPartitionInfoStruct *wb_area = 
		(SharedPartitionInfoStruct *)dnas_dev->write_buffer_start;
	SharedPartitionHeader *tmp = NULL;
	int len = 0;

	/*
	 * Grab the mutex, since we are messing with the free list
	 */
	mutex_lock(&dnas_dev->wb_mutex);

	tmp = wb_area->SharedPartitionFreeList;

	while ((void *)tmp < (void *)(dnas_dev->write_buffer_start + 
					dnas_dev->write_buffer_size)) {
		if (len <= PAGE_SIZE - 80)
			len += snprintf(&buf[len], PAGE_SIZE - len, 
				"Free ent: %u, addr: 0x%p, blocks: %u\n",
				!tmp->allocated, tmp, tmp->num_blocks_in_list);

		tmp = tmp + tmp->num_blocks_in_list;
	}

	mutex_unlock(&dnas_dev->wb_mutex);

	return len;
}

int dump_write_history(struct dri_dnas_device *dnas_dev, char *buf)
{
	int len = 0;

	/*
	 * Grab the mutex, since we are messing with the free list
	 */
	mutex_lock(&dnas_dev->wb_mutex);

	/*
	 * Now dump the history buffer
	 */
	if (wb_history_enabled) {
		int count = 0, index = wb_alloc_hist_next;
		len += snprintf(&buf[len], PAGE_SIZE - len,
			"\nWrite Buffer Allocation History (50)\n"
			"===================================\n");
		while (count < 50) {
			index = index - 1;
			if (index < 0)
				index = 127;
			len += snprintf(&buf[len], PAGE_SIZE - len,
			"Size: %8u, buff addr: %p\n",
			wb_alloc_history[index].blocks,
			wb_alloc_history[index].offset);
			count++;
		}
	}

	mutex_unlock(&dnas_dev->wb_mutex);

	return len;
}

/*
 * Free a write buffer
 */
int free_write_buffer_noid(struct dri_dnas_device *dnas_dev, 
			unsigned int offset, unsigned int size, int direct);
int free_write_buffer(struct dri_dnas_device *dnas_dev, InterCoreBufferId *id)
{
	/*
	 * Sanity checking
	 */
	if (!id || !dnas_dev || id->bufSrc != INTER_CORE_CMD_PARTITION_ID_J1) {
		printk(KERN_WARNING "%s: Trying to free non J1 buffer: %u, "
			"0x%X or bad pointer. Buffer leaked!\n", __func__, 
			(id ? id->bufOffset : 0), 
			(id ? id->bufSrc : 0));
		return -EINVAL;
	}

	return free_write_buffer_noid(dnas_dev, id->bufOffset, id->length, 0);
}

/*
 * Dump the header ...
 */
static void dump_header(char *hdr)
{
        unsigned int i = 0;

        for (i = 0; i < 512; i += 16) {
                printk(KERN_INFO "0x%02x%02x%02x%02x 0x%02x%02x%02x%02x "
                       "0x%02x%02x%02x%02x 0x%02x%02x%02x%02x\n",
                        hdr[i + 0], hdr[i + 1], hdr[i + 2], hdr[i + 3],
                        hdr[i + 4], hdr[i + 5], hdr[i + 6], hdr[i + 7],
                        hdr[i + 8], hdr[i + 9], hdr[i + 10], hdr[i + 11],
                        hdr[i + 12], hdr[i + 13], hdr[i + 14], hdr[i + 15]);
        }
}

/*
 * Free a write buffer given just its offset.
 */
int free_write_buffer_noid(struct dri_dnas_device *dnas_dev, 
			unsigned int offset, unsigned int size, int direct)
{
	int ret = 0, waiting = 0;
	SharedPartitionInfoStruct *wb_area = 
		(SharedPartitionInfoStruct *)dnas_dev->write_buffer_start;
	SharedPartitionHeader *hdr, *tmp = NULL, *prev = NULL;

        DBG(3, KERN_INFO "%s: freeing write buffer: offset: %u, size: %u\n",
                __func__, offset, size);
	/*
	 * Sanity checking
	 */
	if (!dnas_dev || offset > dnas_dev->write_buffer_size) {
		printk(KERN_WARNING "%s: Trying to free invalid buffer offset"
			" (%u) or using incorrect device (%p)!", __func__,
			offset, dnas_dev); 
		return -EINVAL;
	}

	hdr = (SharedPartitionHeader *)get_write_buff_addr(dnas_dev, offset);
	hdr--;    /* Back up to the header before the buffer */

	/*
	 * More sanity checking. If the size stamped in the hdr is not the
	 * same as the size in the request, then issue a warning!
	 * We will actually free the buffer indicated by the header, however.
	 */
	if (hdr->alloc_size != size) {
		printk(KERN_INFO "%s: Error freeing buffer, size in buffer "
			"header: %u != size claimed: %u! Direct: %d, "
                        "Possibly buffer already free!\n",
			__func__, hdr->alloc_size, size, direct);
                printk(KERN_INFO "%s: Buffer location: %p \n", __func__,
                        hdr + 1);
                dump_header((char *)hdr);
                dump_stack();
	}
        if (!hdr->allocated) { /* We should be allocated! */
                printk(KERN_INFO "%s: Error freeing buffer, buffer not "
                        "allocated! Direct: %d!\n",
                        __func__, direct);
                return -EINVAL;
        }

        /*
         * Finished sanity tests, mark buffer freed ...
         */
        hdr->space[64] = 1;
        hdr->space[68] = direct + 1;

	/*
	 * Grab the mutex, since we are messing with the free list
	 */
	mutex_lock(&dnas_dev->wb_mutex);

	tmp = wb_area->SharedPartitionFreeList;

	while (tmp) {
		if ((void *)tmp->next < dnas_dev->write_buffer_start && 
			tmp->next != NULL) {
			printk(KERN_INFO "%s: bad pointer in free list: "
				"next = %p\n", __func__, tmp->next);
		}
		/* Does this block but up against the one being freed? */
		if ((tmp + tmp->num_blocks_in_list) == hdr) {
			/* 
			 * Do we have three in a row? It is safe because
			 * nothing will be equal to NULL.
			 *
			 * Also, we are merging a currently free item already
			 * on the list with an item being freed.
			 */
			if ((hdr + hdr->num_blocks_in_list) == tmp->next) {
				if (!tmp->next) {
				printk(KERN_INFO "%s: bad pointer1: %p\n",
					__func__, tmp);
				}
				tmp->num_blocks_in_list += 
					hdr->num_blocks_in_list +
					tmp->next->num_blocks_in_list;
				tmp->next = tmp->next->next;
				break; /* We are done here */
			}
			else {
				tmp->num_blocks_in_list +=
					hdr->num_blocks_in_list;
				break;
			}
		}
		/* Does the block being freed but up against this one */
		else if ((hdr + hdr->num_blocks_in_list) == tmp) {
			/*
			 * Combine the freed block with the one following
			 */
			hdr->allocated = 0;
			hdr->alloc_size = 0;
			hdr->num_blocks_in_list += tmp->num_blocks_in_list;
			hdr->next = tmp->next;
			/*
			 * If the block we are freeing is before the 
			 * first element in the free list, we have to be
			 * carefull.
			 */
			if (prev)
				prev->next = hdr;
			else
				wb_area->SharedPartitionFreeList = hdr;
			tmp = hdr; /* Make things below work correctly */
			break;
		}
		/* Does this block belong before the current one? */
		else if (hdr < tmp) {
			hdr->allocated = 0;
			hdr->alloc_size = 0;
			if (prev == NULL) {
				if (!tmp || ! hdr) {
					printk(KERN_INFO "%s: bad pointer: "
						"wb_area: %p, hdr: %p\n",
						__func__, wb_area, hdr);
				}
				wb_area->SharedPartitionFreeList = hdr;
				hdr->next = tmp;
				break;
			} else {
				prev->next = hdr;
				hdr->next = tmp;
				break;
			}
		}

		prev = tmp;
		tmp = tmp->next;
	}

	/*
	 * Was the list empty or did we get to the end of the list?
	 */
	if (tmp == NULL) {
		hdr->allocated = 0;
		hdr->alloc_size = 0;
		if (prev == NULL) {  /* List is empty */
			wb_area->SharedPartitionFreeList = hdr;
			hdr->next = NULL;
		} else {
			hdr->next = NULL;
			prev->next = hdr;
		}
	}

	/*
	 * update stats
	 */
	if (wb_area->SharedPartitionBlocksUsed < hdr->num_blocks_in_list)
		wb_area->SharedPartitionBlocksUsed = 0;
	else
		wb_area->SharedPartitionBlocksUsed -= hdr->num_blocks_in_list;

	/*
	 * Find out if there are any threads waiting ...
	 */
	waiting = dnas_dev->threads_waiting_for_buffers;
	if (waiting) {
		dnas_dev->threads_waiting_for_buffers = 0;
		complete_all(&dnas_dev->buffer_completion);
	}

	mutex_unlock(&dnas_dev->wb_mutex);

	return ret;
}

/*
 * Alloc a write buffer
 */
void *alloc_write_buffer(struct dri_dnas_device *dnas_dev, unsigned int size)
{
	SharedPartitionInfoStruct *wb_area = 
		(SharedPartitionInfoStruct *)dnas_dev->write_buffer_start;
	SharedPartitionHeader *tmp = NULL, *prev = NULL, *alloc = NULL;

	unsigned int blocks = (size + SHARED_PARTITION_BLOCK_SIZE - 1) /
				SHARED_PARTITION_BLOCK_SIZE;
	unsigned int largest_free = 0;
	unsigned int num_tries = 0;
        unsigned int i;
	void *buffer = NULL;

	blocks++;  /* Account for the header */

	DBG(3, KERN_INFO "%s:allocating write buffer: size: %u, blocks: %u\n",
		__func__, size, blocks);

retry:
	prev = NULL;
	tmp = wb_area->SharedPartitionFreeList;

	largest_free = 0;

	/*
	 * Grab the mutex, since we are messing with the free list
	 */
	mutex_lock(&dnas_dev->wb_mutex);

	/*
	 * First fit
	 */
	while (tmp) {

		if (tmp->num_blocks_in_list == blocks) {

			if (prev == NULL)
				wb_area->SharedPartitionFreeList = tmp->next;
			else
				prev->next = tmp->next;

			tmp->allocated = 1;
			tmp->alloc_size = size;
			tmp->num_blocks_in_list = blocks;
			wb_area->SharedPartitionBlocksUsed += blocks;

			buffer = (void *)(tmp + 1);
			alloc = tmp;
			break;
		} else if (tmp->num_blocks_in_list > blocks) {
			alloc = tmp + tmp->num_blocks_in_list - blocks;
			alloc->alloc_size = size;
			alloc->num_blocks_in_list = blocks;
			alloc->next = NULL;
			alloc->allocated = 1;
			tmp->num_blocks_in_list -= blocks;
			wb_area->SharedPartitionBlocksUsed += blocks;

			buffer = (void *)(alloc + 1);

			break;
		}

		if (largest_free < tmp->num_blocks_in_list) {
			largest_free = tmp->num_blocks_in_list;
		}

		prev = tmp;
		tmp = tmp->next;
	}

	if (!buffer) {
		InterCoreCtrlLxToVx ic_send;
		SharedMemQueueSendMsg send_args;
		int sret = 0;

		if (!dnas_dev->threads_waiting_for_buffers)
			init_completion(&dnas_dev->buffer_completion);
		dnas_dev->threads_waiting_for_buffers = 1;
		num_tries++;

		mutex_unlock(&dnas_dev->wb_mutex);

		/*
		 * First, send a message to flush disks and then wait
		 */
		ic_send.version = INTER_CORE_CMD_PROTOCOL_CUR_VERSION;
		ic_send.tag     = 0;
		ic_send.flags   = FLUSH_DISKS;

		send_args.handle      = dnas_dev->core_ctrl_queue_handle;
		send_args.msg         = (void *)&ic_send;
		send_args.msgSize     = sizeof(ic_send);
		send_args.flags       = SHMQ_SEND_FLAG_WAIT_FOREVER;
		send_args.timeoutInMS = 0;
		sret = shm_queue_kern_send_msg(dnas_dev->shm_dev, &send_args);
		if (sret) {
			printk(KERN_INFO "%s: failure sending FLUSH DISKS msg:"
				" %d, Ignoring and hoping!\n", __func__, sret);
		}

		if (num_tries > 5) {
			msleep_interruptible(10000);
		}
		wait_for_completion_interruptible(&dnas_dev->buffer_completion);
		goto retry;
	}

	/*
	 * If we get here we have the blocks we want
	 */
	wb_area->SharedPartitionBlocksUsed += alloc->num_blocks_in_list;

	/*
	 * Update the history
	 */
	if (unlikely(wb_history_enabled)) {
		wb_alloc_history[wb_alloc_hist_next].blocks = blocks - 1;
		wb_alloc_history[wb_alloc_hist_next].offset = buffer;
		wb_alloc_hist_next++;
		if (wb_alloc_hist_next > 127) /* XXX: Fixme */
			wb_alloc_hist_next = 0;
	}

	mutex_unlock(&dnas_dev->wb_mutex);

        /*
         * Stamp some stuff in the header for debugging
         */
        for (i = 0; i < 16; i++) {
                ((unsigned int *)alloc->space)[i] = 0xefbeadde;
        }

        alloc->space[64] = 2;

	return buffer;
}

/*
 * Set up the initial lx dyn buffer pool. Assumes that no one else is running
 * and thus trying to access the lx buffer area. We keep pointers in the
 * chain and convert to offsets when adding to a buffer descriptor.
 *
 * NOTE! Min elt size must be a multiple of word size, but lets make it
 * 512. The TransportManager sends 1 byte of data in some requests (when none
 * is needed) for hysterical reasons. This be bad for the memory managemen
 * strategy used below. 
 */
void init_lx_dyn_buffer(struct dri_dnas_device *dnas_dev)
{
	struct buff_elt_struct *lx_elt;

	lx_elt = (struct buff_elt_struct *)get_lx_dyn_buff_addr(dnas_dev, 0);

	lx_elt->size = dnas_dev->lx_dyn_buffer_size;
	lx_elt->next = NULL;
	lx_elt->prev = NULL;

	dnas_dev->lx_db_head = lx_elt;
}

/* Dump info about the Lx pool */
int dump_lx_buffer(struct dri_dnas_device *dnas_dev, char *buf)
{
	struct buff_elt_struct *head = dnas_dev->lx_db_head;
	int len = 0;

	/*
	 * Grab the mutex, since we are messing with the free list
	 */
	mutex_lock(&dnas_dev->lx_db_mutex);

	while (head != NULL) {
	        len += snprintf(&buf[len], PAGE_SIZE - len, 
				"Free Lx ent addr: 0x%p, size: %u bytes\n",
				head, head->size);

		head = head->next;
	}

	mutex_unlock(&dnas_dev->lx_db_mutex);

	return len;
}

/*
 * Walk the free list and insert this element ...
 */
int free_lx_dyn_buffer_noid(struct dri_dnas_device *dnas_dev, unsigned int off,
			unsigned int size);
int free_lx_dyn_buffer(struct dri_dnas_device *dnas_dev, InterCoreBufferId *id)
{
	return free_lx_dyn_buffer_noid(dnas_dev, id->bufOffset, id->length);
}

/*
 * Walk the free list and instert this element based on buff offset
 */
int free_lx_dyn_buffer_noid(struct dri_dnas_device *dnas_dev, unsigned int off,
			unsigned int size)
{
	int ret = 0;
	int waiting = 0;
	struct buff_elt_struct *head = dnas_dev->lx_db_head;
	struct buff_elt_struct *tmp = head;
	struct buff_elt_struct *free_elt;

	free_elt = (struct buff_elt_struct *)get_lx_dyn_buff_addr(dnas_dev, 
				off);

        DBG(3, KERN_INFO "%s: freeing Lx Dyn elt: addr: %p, off: %u, size: %d"
                "\n", __func__, free_elt, off, size);

	free_elt->size = size;
	free_elt->next = NULL;
	free_elt->prev = NULL;

	/*
	 * Grab the mutex, since we are messing with the free list
	 */
	mutex_lock(&dnas_dev->lx_db_mutex);

	/*
	 * Walk down the list until we find where we should be, or 
	 * hit the last element. This prevents us walking off the end
	 * of the list.
	 */
	while (tmp && tmp < free_elt && tmp->next)
		tmp = tmp->next;

	/*
	 * tmp now points to the element after which we will insert
	 * the free element. Check to see if we can merge as well.
	 */

        DBG(4, KERN_INFO "%s: tmp: %p, size: %u, free: %p\n", __func__, 
                tmp, tmp->size, free_elt);

	/* 
	 * Do we have an empty list?
	 */
	if (!tmp) {
		dnas_dev->lx_db_head = free_elt;
	
	/*
	 * Are we butting up against tmp but after it?
	 */
	} else if ((struct buff_elt_struct *)((void *)tmp + tmp->size) == 
                        free_elt) {

                /* We always absorb the one abutting ... */
		tmp->size += free_elt->size;

		/*
		 * Now check to see if the free element just absorbed
		 * buts up against the one after it. We don't need to
		 * worry about NULL in tmp->next below.
		 */
		if ((struct buff_elt_struct *)((void *)tmp + tmp->size + size) 
                        == tmp->next) {
			tmp->size += tmp->next->size + size;
			tmp->next = tmp->next->next;
                }
                DBG(6, KERN_INFO "%s: tmpbutting: %p, size: %u\n", __func__, 
                       tmp, tmp->size);
	/*
	 * Or are we before tmp and butting up with it
	 */
	} else if ((struct buff_elt_struct *)((void *)free_elt + free_elt->size)
                        == tmp) {
		free_elt->size += tmp->size;
		free_elt->next = tmp->next;
		if (tmp->prev == NULL)
			dnas_dev->lx_db_head = free_elt;
		else
			tmp->prev->next = free_elt;
                
                DBG(6, KERN_INFO "%s: before tmp: %p, %p\n", __func__, 
                        tmp, free_elt);
	} else {
		/*
		 * Just insert the element.
		 */
		free_elt->next = tmp->next;
		if (tmp == head)
			free_elt->prev = NULL;
		else
			free_elt->prev = tmp;

		tmp->next = free_elt;
                DBG(6, KERN_INFO "%s: after tmp, no more: %p, %p\n", __func__, 
                        tmp, free_elt);
	}

        DBG(4, KERN_INFO "%s: tmp: %p, size: %u\n", __func__, tmp, tmp->size);
	/*
	 * Find out if there are any threads waiting ...
	 */
	waiting = dnas_dev->threads_waiting_for_buffers;

	if (waiting)
		complete_all(&dnas_dev->buffer_completion);

	mutex_unlock(&dnas_dev->lx_db_mutex);

	return ret;
}

void * alloc_lx_dyn_buffer(struct dri_dnas_device *dnas_dev, unsigned int size)
{
	struct buff_elt_struct *buff = NULL;
	struct buff_elt_struct *head = dnas_dev->lx_db_head;
	struct buff_elt_struct *tmp = head;

	/*
	 * Grab the mutex, since we are messing with the free list
	 */
retry:
	mutex_lock(&dnas_dev->lx_db_mutex);

	/*
	 * First fit ...
	 */
	while (tmp && tmp->size < size) {
		tmp = tmp->next;
	}

	/*
	 * If we did not find one, we have to stall and wait for space to
	 * become available ...
	 */

	if (tmp == NULL) {
		dnas_dev->threads_waiting_for_buffers++;
		mutex_unlock(&dnas_dev->lx_db_mutex);
		wait_for_completion_interruptible(&dnas_dev->buffer_completion);
		goto retry;
	}

	buff = tmp;
	if (tmp->size == size) {
		/*
		 * Found exactly what we wanted, remove it and return it
		 */
		if (tmp->prev == NULL) {
			head = tmp->next;
			if (head)
				head->next->prev = NULL;
				dnas_dev->lx_db_head = head;
			} else {
				tmp->prev->next = tmp->next;
				if (tmp->next)
					tmp->next->prev = tmp->prev;
			}
		} else {
			/*
			 * Larger than we need, carve out what we need. Take off the
			 * end so we do not need to adjust pointers, just size
			 */
			/*
			 * The casting here is to avoid problems with pointer 
			 * arithmetic.
			 */
			buff = (struct buff_elt_struct *)((void *)buff + 
					(buff->size - size));
			tmp->size -= size;
                        DBG(4, KERN_INFO "%s: remaining buf: %p, size: %u\n",
                                __func__, tmp, tmp->size);
		}	

		mutex_unlock(&dnas_dev->lx_db_mutex);

		return (void *)buff;
	}

/*
 * Free a vx dyn buffer by writing to the resource queue pair
 */
int free_vx_dyn_buffer(struct dri_dnas_device *dnas_dev, unsigned int tag,
			unsigned int be_tag,
			InterCoreBufferId *id1, InterCoreBufferId *id2)
{
	int ret = 0;
	InterCoreVxResource ic_vxr;
	SharedMemQueueSendMsg send_args;

	memset(&ic_vxr, 0, sizeof(ic_vxr));

	ic_vxr.version    = INTER_CORE_CMD_PROTOCOL_CUR_VERSION;
	ic_vxr.tag        = tag;
	ic_vxr.backEndTag = be_tag;

	if (id1) {
		ic_vxr.bufId1.bufOffset = id1->bufOffset;
		ic_vxr.bufId1.length    = id1->length;
		ic_vxr.bufId1.bufSrc    = id1->bufSrc;
	} else {
		ic_vxr.bufId1.bufSrc	= INTER_CORE_CMD_PARTITION_ID_INVALID;
	}

	if (id2) {
		ic_vxr.bufId2.bufOffset = id2->bufOffset;
		ic_vxr.bufId2.length    = id2->length;
		ic_vxr.bufId2.bufSrc    = id2->bufSrc;
	} else {
		ic_vxr.bufId2.bufSrc	= INTER_CORE_CMD_PARTITION_ID_INVALID;
	}

	send_args.handle      = dnas_dev->resource_queue_handle;
	send_args.msg         = (void *)&ic_vxr;
	send_args.msgSize     = sizeof(ic_vxr);
	send_args.flags       = SHMQ_SEND_FLAG_WAIT_FOREVER;
	send_args.timeoutInMS = 0;

	ret = shm_queue_kern_send_msg(dnas_dev->shm_dev, &send_args);

	if (ret) {
		printk(KERN_WARNING "%s:problems sending message: %d\n",
			__func__, ret);
	}

	return ret;
}

/*
 * Send a SCSI Command to the Vx core ...
 */
int send_scsi_request(struct dri_dnas_device *dnas_dev, 
			uint8_t *cdb, uint32_t lun,
			struct dnas_tag_struct *tag, uint32_t buf, 
			uint32_t buf_len, uint32_t src)
{
	int ret = 0;
	InterCoreSCSICmd ic_scsi_cmd;
	SharedMemQueueSendMsg send_args;

	send_args.flags       = SHMQ_SEND_FLAG_WAIT_FOREVER;
	send_args.timeoutInMS = 0;
	send_args.handle      = dnas_dev->scsi_queue_handle;
	send_args.msg         = (void *)&ic_scsi_cmd;
	send_args.msgSize     = sizeof(ic_scsi_cmd);

	ic_scsi_cmd.tag = (int)tag;
	memcpy(&ic_scsi_cmd.cdb, cdb, 16);  /* XXX: May need to be fixed */
	ic_scsi_cmd.lun[2] = ic_scsi_cmd.lun[3] = 0;
	ic_scsi_cmd.lun[0] = lun >> 16;
	ic_scsi_cmd.lun[1] = lun & 0xFFFF;

	DBG(5, KERN_INFO "REQ: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x"
                         ":%02x\n",
			cdb[0], cdb[1], cdb[2], cdb[3], cdb[4], cdb[5], cdb[6],
                        cdb[7], cdb[8], cdb[9]);

	ic_scsi_cmd.bufId.bufOffset = buf;
	ic_scsi_cmd.bufId.length = buf_len;
	ic_scsi_cmd.bufId.bufSrc = src;

	ret = shm_queue_kern_send_msg(dnas_dev->shm_dev, &send_args);
	if (ret) {
		printk(KERN_INFO "%s: Error sending scsi cmd\n",
		__func__);
	}

	return ret;
}

/*
 * Create the nospace file ... 
 */
static char *nospace_path = "/var/nasd.nospace";

void create_nospace_file(void)
{
        struct file *fd;
        int res = 0;

        fd = filp_open(nospace_path, O_CREAT | O_SYNC, 0666);
        if (IS_ERR(fd)) {
                res = PTR_ERR(fd);
                DBG(0, KERN_INFO "%s: unable to create file %s, %d\n",
                        __func__, nospace_path, res);
                return;
        }

        filp_close(fd, NULL);
}

/*
 * The SCSI Queue handler ... waits for messages from that queue
 * and handles them
 *
 * Note, commands could be cancelled by the SCSI layer before we get called
 * here. If a request takes too long, for example. In that case, we detect it
 * because the req_p in the tag structure is NULL. For IOs that have been 
 * cancelled, do not transfer any data and do not call the done routine!
 */
int dri_dnas_shared_mem_scsi_thread(void *data)
{
	int ret = 0, created_nospace_file = 0;
	struct dri_dnas_device *dnas_dev = (struct dri_dnas_device *)data;
	SharedMemQueueRecvMsg recv_args;
	InterCoreSCSIResp ic_rsp;

	DBG(5, KERN_INFO "%s: Started\n", __func__);

	/*
	 * Now, process events on the SCSI Command queue.
	 */
	memset(&recv_args, 0, sizeof(recv_args));
	recv_args.flags       = SHMQ_RECV_FLAG_WAIT_FOREVER_INT;
	recv_args.timeoutInMS = 0;
	recv_args.handle      = dnas_dev->scsi_queue_handle;
	recv_args.buf         = (void *)&ic_rsp;
	recv_args.bufSize     = sizeof(ic_rsp);

	while (!kthread_should_stop()) {
	        struct dnas_tag_struct *dnas_tag = NULL;
                int io_abort = 0;
                char *sens_buff = NULL;
                unsigned int sens_len = 0;

		DBG(10, "%s: Waiting for next SCSI Response!\n", __func__);
		ret = shm_queue_kern_recv_msg(dnas_dev->shm_dev, &recv_args);
		if (ret) {
			printk(KERN_ERR "%s: Unable to receive message from "
				"other core: %d\n", __func__, ret);
                        dump_stack();
			return ret;
		}

		/*
		 * What we have in the tag is a pointer to our dnas_tag_struct
		 * Get that and then deal with the IO ... however, we need
		 * to figure out if the IO was aborted ... by the cmd being
		 * set to NULL.
		 */
		dnas_tag = (struct dnas_tag_struct *)ic_rsp.tag;

		DBG(10, "%s: Processing the next SCSI Response!\n", __func__);
		if (ic_rsp.bufId2.length > 0 && ic_rsp.bufId2.bufSrc !=
			INTER_CORE_CMD_PARTITION_ID_INVALID) {

			DBG(5, KERN_INFO "%s: We got a sense buffer, status: "
                            "%0X!\n", __func__, ic_rsp.status);
			/*
                         * We set up sens_buff and sens_len for the actual code
                         * below to use ... It could be possible for the Vx
                         * core to return a partially complete read with a
                         * sense buffer ... I imagine.
                         */

                        /* The sense buff is pointed to by bufId2 */
			sens_buff = dnas_dev->vx_dyn_buffer_start +
					ic_rsp.bufId2.bufOffset;

                        sens_len = ic_rsp.bufId2.length;

                        /*
                         * If we have an out-of-memory response from VxWorks
                         * then create a file /var/nasd.nospace ... to
                         * signal to NASd that on the next boot we cannot 
                         * write.
                         */
                        if (sens_buff[0] == 0x70 && sens_buff[2] == 3 &&
                            sens_buff[12] == 0x0C && sens_buff[13] == 2) {
                                DBG(0, KERN_INFO "%s: NoSpace sense buffer "
                                        "from VxWorks. Creating special file\n",
                                        __func__);
                                if (!created_nospace_file) {
                                        create_nospace_file();
                                        created_nospace_file = 1;
                                }
                        }

		}

		/*
		 * XXX: Todo, if we have both data and sense buffer ...
		 */

		/*
		 * If we got an SGL response, then verify that it looks 
		 * reasonable. If not, we have to abort the IO ...???
		 */
		if (ic_rsp.bufId1.bufSrc == 
			INTER_CORE_CMD_PARTITION_ID_SGL_MEM) {
			void *sgl = NULL;

			if (ic_rsp.bufId1.length < sizeof(InterCoreSGL)) {
				printk(KERN_WARNING "%s: Invalid SGL len: %d\n",
					__func__, ic_rsp.bufId1.length);
                                dump_stack();
				/* Abort the IO ... how?*/

			}

			sgl = dnas_dev->sgl_buffer_start + 
			ic_rsp.bufId1.bufOffset;
			if (ic_rsp.bufId1.bufOffset > 
				dnas_dev->sgl_buffer_size ||
				(ic_rsp.bufId1.bufOffset + 
					sizeof(InterCoreSGL)) > 
					dnas_dev->sgl_buffer_size) {
				printk(KERN_WARNING "%s: SGL or part of the "
					"SGL falls outside the shared region:"
					"tag: %0X, offset: %0X\n", __func__,
					ic_rsp.tag, ic_rsp.bufId1.bufOffset);
                                dump_stack();
				/* Abort the IO ... how?*/
			}

		}

		/*
		 * Now we figure out what type of command it was. If 
		 * a READ or READ_IO we have to move the data from the
		 * intercore buffer (HLBAT or LX Dyn area) to the SGL or
		 * request we were given. Then we can call the IO done 
		 * routine and free the resources (HLBAT frees involve 
		 * telling the Vx core), and then free the tag.
		 *
		 * For writes, if it is a WRITE_IO, we cannot free the
		 * resource until the buffer is freed. However, we can
		 * complete the IO and free up the tag and dyn buffer in the
		 * case of a WRITE.
		 */

                /*
                 * Lock out any command abort processing until we are done
                 * This simplifies the handling of aborts and prevents 
                 * races ... it ensures that resources we expect to be there,
                 * eg the pages pointed to by the SG list in the request
                 * stick around until we have copied any data into them.
                 *
                 * The only contention for this lock will be from command
                 * abort requests, which should not be that frequent.
                 */
	        mutex_lock(&dnas_dev->scsi_completion_mutex);

                io_abort = dnas_tag->req_p == NULL;
		switch (dnas_tag->request_type) {
		case DNAS_WRITE:    /* Data in Lx Dyn buffers */
			DBG(5, "%s: Handled SCSI Write Response!\n", __func__);
                        if (!io_abort)
			        (void)dri_dnas_send_resp(dnas_tag->req_p, 
                                                sens_buff, sens_len,
						dnas_tag->done,
						(ret) ? ret : ic_rsp.status);

			if (ic_rsp.bufId1.bufSrc == 
				INTER_CORE_CMD_PARTITION_ID_LINUX_DYN_MEM) {
				/*
				 * Free up the LX Dyn buffer ... J1 buffers
				 * are freed in the resource thread
				 */
				free_lx_dyn_buffer(dnas_dev, &ic_rsp.bufId1);
			}
                        /* Free this, or we leak ... */
			kmem_cache_free(dnas_dev->dnas_tag_cache, dnas_tag);

			break;

		case DNAS_WRITE_IO:  /* Data in Write Buffers, AKA J1 */

			break;

		case DNAS_READ:      /* Data in Vx Dyn or HLBAT buffers */
			DBG(5, "%s: Handled SCSI Read Response!\n", __func__);

                        if (!io_abort)
			        ret = xfer_to_request_buffs(dnas_dev, dnas_tag, 
				        &ic_rsp.bufId1);
			free_vx_dyn_buffer(dnas_dev, ic_rsp.tag, 
					ic_rsp.backEndTag, &ic_rsp.bufId1, 
					NULL);
                        if (!io_abort)
			        (void)dri_dnas_send_resp(dnas_tag->req_p, 
                                                sens_buff, sens_len, 
						dnas_tag->done, 
						(ret) ? ret : ic_rsp.status);
			
			kmem_cache_free(dnas_dev->dnas_tag_cache, dnas_tag);
			break;

		case DNAS_READ_IO:

			break;

		case DNAS_NOIO:      /* No data at all    */
                        if (!io_abort)
			        (void)dri_dnas_send_resp(dnas_tag->req_p, 
                                                NULL, 0, 
						dnas_tag->done, ic_rsp.status);
			kmem_cache_free(dnas_dev->dnas_tag_cache, dnas_tag);
			break;

		default:

			break;
		}

                /*
                 * If there is a sense buffer, we have to free it.
                 */
                if (sens_buff) {
                        DBG(5, KERN_INFO "%s: freeing sense buffer\n",
                                __func__);
	                free_vx_dyn_buffer(dnas_dev, ic_rsp.tag, 
					ic_rsp.backEndTag, &ic_rsp.bufId2, 
					NULL);
                }

	        mutex_unlock(&dnas_dev->scsi_completion_mutex);

	}

	return ret;
}

/*
 * Process a scsi request ... We could drop the tag lock while allocating a J1
 * buffer, because this could take a while. However, we will do that later.
 */
int process_scsi_request(struct dnas_tag_struct *tag, 
			struct dri_dnas_device *dnas_dev)
{
	int ret = 0;
	struct scsi_cmnd *scmd = tag->req_p;
	unsigned char cmd = scmd->cmnd[0];

	DBG(4, "%s: Processing SCSI Request: %02X, LUN: %u, len: %u\n", 
		__func__, cmd, scmd->device->lun, scsi_bufflen(scmd));

	/*
	 * Figure out direction and type and dispatch the command
	 */
	switch (scmd->sc_data_direction) {

	/*
	 * This is a WRITE or a WRITE_IO command, so deal with it correctly.
	 *
	 * We queue all write requests to a separate write thread that takes
	 * requests off the write queue. This is because we might be called in
	 * a soft_irq context, but we have to allocate space for the writes.
	 */
	case DMA_TO_DEVICE:
		tag->request_type = DNAS_WRITE;

		/*
		 * Figure out the type of write command ... so we can
		 * allocate the correct type of buffer.
		 */
		switch (cmd) {
			unsigned int buf_off = 0;
			void *buf = NULL;
                        unsigned int buf_len = 0;
		case WRITE_6:
		case WRITE_10:
		case WRITE_16:
			if ((buf = alloc_write_buffer(dnas_dev,
				scsi_bufflen(scmd))) == NULL) {
				printk(KERN_INFO "%s: Unable to allocate J1 "
					"buffer of size: %u!\n",
					__func__, scsi_bufflen(scmd));
				return -ENOMEM; /* XXX: Fixme */
			}

			/*
			 * Now, transfer the data to the buffer and then
			 * send the request
			 */	
			buf_off = (unsigned int)(buf - 
					dnas_dev->write_buffer_start);

			ret = xfer_from_request_buffs(buf, scmd, 
                                        dnas_dev->write_buffer_start,
                                        dnas_dev->write_buffer_phys);
			if (ret) {
				free_write_buffer_noid(dnas_dev, buf_off,
					scsi_bufflen(scmd), 1);
				return ret; /* XXX: Fixme, what to do here */
			}

			ret = send_scsi_request(dnas_dev, scmd->cmnd,
				scmd->device->lun, tag, buf_off,
				scsi_bufflen(scmd),
				INTER_CORE_CMD_PARTITION_ID_J1);
			break;

		default:
                        /*
                         * Note, we must round up lx size to 512-byte multiple
                         * See comments in allocator.
                         */
                        buf_len = (scsi_bufflen(scmd) + 511) & ~511;
			if ((buf = alloc_lx_dyn_buffer(dnas_dev, 
				buf_len)) == NULL) {
				printk(KERN_INFO "%s: Unable to allocate LX "
					"dyn buffer of size: %u\n",
					__func__, scsi_bufflen(scmd));
				return -ENOMEM; /* XXX: Fixme */
			}

                        /*
                         * Calculate this early so we have it for the error
                         * path below.
                         */
			buf_off = (unsigned int)(buf -
					dnas_dev->lx_dyn_buffer_start);

			/*
			 * Now, transfer the data to the buffer and then
			 * send the request.
			 */

			ret = xfer_from_request_buffs(buf, scmd, 
                                        dnas_dev->lx_dyn_buffer_start,
                                        dnas_dev->lx_dyn_buffer_phys);
			if (ret) {
				free_lx_dyn_buffer_noid(dnas_dev, buf_off, 
                                                        buf_len);
				return ret;
			}

			ret = send_scsi_request(dnas_dev, scmd->cmnd,
				scmd->device->lun, tag, buf_off, buf_len,
				INTER_CORE_CMD_PARTITION_ID_LINUX_DYN_MEM);

			break;
		}

		break;

	/*
	 * This is a READ or READ_IO command, so not much to do, as we only
	 * need the distinction in the scsi thread to tell us which buffer
	 * types we got, and we know there.
	 */
	case DMA_FROM_DEVICE:
		tag->request_type = DNAS_READ;

                if (cmd == 0xEAi && scmd->cmnd[3] == 4) { /* Vendor specific */
                        printk(KERN_INFO "vendor specific upload diags command\n");
                }

		ret = send_scsi_request(dnas_dev, scmd->cmnd, 
				scmd->device->lun, tag, 0,
				scsi_bufflen(scmd), 
				INTER_CORE_CMD_PARTITION_ID_INVALID);
		break;

	/*
	 * We do not handle BIDI commands, I believe, so abort.
	 */
	case DMA_BIDIRECTIONAL:
		printk(KERN_ERR "%s: BIDI command rejected: %0X\n",
			__func__, cmd);
		return dri_dnas_send_resp(scmd, NULL, 0, tag->done, 
					DID_NO_CONNECT << 16);
		break;
	/*
	 * No data with this, just send the command.
	 */
	case DMA_NONE:
	default:
		tag->request_type = DNAS_NOIO;

		ret = send_scsi_request(dnas_dev, scmd->cmnd, 
				scmd->device->lun, tag, 0,
				0, INTER_CORE_CMD_PARTITION_ID_INVALID);
		break;
	}

	return ret;
}

/*
 * The SCSI Request thread ... we handle all SCSI requests here so that
 * queuerequest does not get into problems of calling schedule while waiting
 * for write buffers or dyn buffers or waiting on trying to insert something
 * on the queue pairs to the VxCore, because queuecommand can be called from
 * the SCSI SoftIRQ context!
 */
int dri_dnas_scsi_request_thread(void *data)
{
	struct dri_dnas_device *dnas_dev = (struct dri_dnas_device *)data;  
	struct dnas_tag_struct *tag = NULL;
	unsigned long flags;
	int ret = 0, d_skip = delay_skip;

	DBG(5, KERN_INFO "%s: Started\n", __func__);

	while (!kthread_should_stop()) {

		/*
		 * We wait on the write completion if there is no work to do.
		 * When woken up, grab the spinlock and work and do it.
		 */

		spin_lock_irqsave(&dnas_dev->req_spinlock, flags);
		if (list_empty(&dnas_dev->req_queue)) {
			dnas_dev->req_thread_waiting = 1;
			spin_unlock_irqrestore(&dnas_dev->req_spinlock, flags);
			wait_for_completion_interruptible(
				&dnas_dev->req_completion);
			spin_lock_irqsave(&dnas_dev->req_spinlock, flags);
		}

		/*
		 * At this point there should be items in the request list
		 * Take the first and process it. We also have the spinlock at
		 * this point, so drop it as soon as we have grabbed an item
		 */

                if (delay_requests) {
                        /*
                         * Delay the request by the number of seconds specified
                         */
                        if (d_skip > 0)
                                d_skip--;
                        else {
                                ssleep(delay_requests);
                                d_skip = delay_skip;
                                DBG(0, KERN_INFO "%s: delayed IO by %us, next"
                                        "in %d IOs\n", __func__, 
                                        delay_requests, d_skip);
                        }
                }

		tag = list_entry(dnas_dev->req_queue.next, 
			struct dnas_tag_struct, req_queue_ent);


		list_del(&tag->req_queue_ent);
		spin_unlock_irqrestore(&dnas_dev->req_spinlock, flags);

                /*
                 * Lock the tag whie we are working on it ... we do this after
                 * the spinlock to prevent deadlocks where the lock is held by
                 * the SCSI EH thread and we have IRQs locked out. Removing the
                 * element from the queue while we do not hold the lock should 
                 * not be an issue as the lock is to protect us from having the
                 * IO aborted out from under us before we get to processing it
                 * or while we are processing it.
                 */
                mutex_lock(&tag->tag_mutex);

		/*
		 * Now process the request ... no one else has a ref to this
		 * item at this point. However, if this request has been 
		 * cancelled, then there is not much to do. It can only have
		 * been cancelled while on the queue at this point since we 
		 * now have the lock. We only have to free the tag in the case
		 * that the IO has been cancelled.
		 */
                if (tag->req_p == NULL) {
                        mutex_unlock(&tag->tag_mutex);
			kmem_cache_free(dnas_dev->dnas_tag_cache, tag);
                        continue;
                }

		ret = process_scsi_request(tag, dnas_dev);

		/*
		 * Error handling ... if the IO failed, tell the SCSI mid layer
		 * and free the tag struct.
		 */
		if (ret) {
			dri_dnas_send_resp(tag->req_p, NULL, 0, tag->done, ret);
			kmem_cache_free(dnas_dev->dnas_tag_cache, tag);
		}

                mutex_unlock(&tag->tag_mutex);
	}

	return ret;
}

/*
 * The resource Queue handler ... waits for messages from that queue
 * and handles them
 */
int dri_dnas_shared_mem_resource_thread(void *data)
{
	int ret = 0;
	struct dri_dnas_device *dnas_dev = (struct dri_dnas_device *)data;  
	SharedMemQueueRecvMsg recv_args;
	InterCoreLxResource ic_lxr;

	DBG(5, KERN_INFO "%s: Started\n", __func__);

	/*
	 * Now, process events on the resource Command queue.
	 */
	memset(&recv_args, 0, sizeof(recv_args));
	recv_args.flags       = SHMQ_RECV_FLAG_WAIT_FOREVER_INT;
	recv_args.timeoutInMS = 0;
	recv_args.handle      = dnas_dev->resource_queue_handle;
	recv_args.buf         = (void *)&ic_lxr;
	recv_args.bufSize     = sizeof(ic_lxr);

	while (!kthread_should_stop()) {

		ret = shm_queue_kern_recv_msg(dnas_dev->shm_dev, &recv_args);
		if (ret) {
			printk(KERN_ERR "%s: Unable to receive message from "
				"other core: %d\n", __func__, ret);
                        dump_stack();
			return ret;
		}

		/*
		 * Now, figure out the typf of buffer. If the buffer points to
		 * the J1 area we free that. If it points to the Dyn 
		 * area, free those buffers as well. Any other type is an 
		 * error, but we should keep on trucking.
		 */
		if (ic_lxr.bufId1.bufSrc == INTER_CORE_CMD_PARTITION_ID_J1) {
                        if (ic_lxr.bufId1.length == 0) {
                                printk(KERN_INFO "Got write buffer len 0\n");
                        }
			free_write_buffer(dnas_dev, &ic_lxr.bufId1);
		} else if (ic_lxr.bufId1.bufSrc ==
				INTER_CORE_CMD_PARTITION_ID_LINUX_DYN_MEM) {
			free_lx_dyn_buffer(dnas_dev, &ic_lxr.bufId1);
		} else {
			printk(KERN_INFO "%s: Unknown resource type in msg: "
				"%0X, no action taken\n", __func__,
				ic_lxr.bufId1.bufSrc);
		}

	}

	return ret;
}

/*
 * Queue an intercore message to sysfs. Only a few such messages go in this
 * direction. Specifically, the first ISCSI_ENABLE and LX_CLEARDISK
 *
 * We use the same userland queue elements ... as queue_to_userland
 */
int queue_to_sysfs(InterCoreCtrlVxToLx *msg, 
                   struct dri_dnas_device *dnas_dev)
{
        int ret = 0;
        struct dnas_userland_queue_elt *ul_elt = NULL;


        ul_elt = kmem_cache_alloc(dnas_dev->core_ctrl_cache, GFP_KERNEL);
        if (!ul_elt) {
                return -ENOMEM;
        }

        ul_elt->sent_to_userland = 0;
        ul_elt->status = 0;

        if (msg) {
                memcpy(&ul_elt->msg, msg, sizeof(InterCoreCtrlVxToLx));
                DBG(5, KERN_INFO "%s: buffSize: %u", __func__,
                        msg->u.fileReq.bufId.length);
        }

        ul_elt->no_response = 1; /* No response for now */

        /*
         * We use the same mutex for both queues, since only a small number
         * of messages will go this way and there will be limited contention
         * for them. BUG: FIXME when we have more than one intercore 
         * device.
         */
        mutex_lock(&dnas_sysfs_queue_mutex);

        list_add_tail(&ul_elt->core_ctrl_elt, &dnas_sysfs_userland_queue);

        if (dnas_waiting_for_userland_queue) {
                dnas_waiting_for_userland_queue = 0;
                complete_all(&dnas_userland_ctrl_completion);
        }

        mutex_unlock(&dnas_sysfs_queue_mutex);

        return ret;
}

/*
 * queue an intercore message to userland
 */
int queue_to_userland(int status, InterCoreCtrlVxToLx *msg,
		struct dri_dnas_device *dnas_dev, int no_response)
{
	int ret = 0;
	struct dnas_userland_queue_elt *ul_elt = NULL;

	ul_elt = kmem_cache_alloc(dnas_dev->core_ctrl_cache, GFP_KERNEL);
	if (!ul_elt) {
		return -ENOMEM;
	}

	ul_elt->sent_to_userland = 0;
	ul_elt->status = status;

	if (msg) {
		memcpy(&ul_elt->msg, msg, sizeof(InterCoreCtrlVxToLx));
                DBG(5, KERN_INFO "%s: buffSize: %u", __func__, 
                        msg->u.fileReq.bufId.length);
	}

	ul_elt->no_response = 1;  /* No response today */

	mutex_lock(&dnas_dev->core_ctrl_queue_mutex);

	list_add_tail(&ul_elt->core_ctrl_elt, &dnas_dev->core_ctrl_queue);

	if (dnas_dev->waiting_for_core_ctrl) {
		dnas_dev->waiting_for_core_ctrl = 0;
		complete_all(&dnas_dev->core_ctrl_completion);
	}

	mutex_unlock(&dnas_dev->core_ctrl_queue_mutex);

	return ret;
}

/*
 * Handle a net info request ...
 */
int handle_net_info(InterCoreCtrlVxToLx *msg, struct dri_dnas_device *dnas_dev)
{
	int ret = 0;
	int send_ret;
	uint32_t *req = NULL;
	InterCoreCtrlLxToVx reply;
	SharedMemQueueSendMsg send_args;

	req = (uint32_t *)(dnas_dev->vx_dyn_buffer_start + 
				msg->u.netReq.bufId.bufOffset);

	if (msg->u.netReq.bufId.bufSrc !=
		INTER_CORE_CMD_PARTITION_ID_VXWORKS_DYN_MEM) {
		printk(KERN_ERR "%s: Invalid InterCore partition type: %0X\n",
			__func__, msg->u.netReq.bufId.bufSrc);
		ret = -EINVAL;
                dump_stack();
		goto reply;
	}

	if (msg->u.netReq.cmd == GET_NET_INFO) {
		if (msg->u.netReq.bufId.length < (sizeof(uint32_t) * 9)) {
			printk(KERN_INFO "%s: Invalid buffer len: %d\n",
				__func__, msg->u.netReq.bufId.length);
			/*ret = -EINVAL;
			goto reply; */
		}

		/*
		 * Dummy up a response here
		 */
		*(req + 2) = 1;
		*(req + 3) = 0xA9FE0100;
		*(req + 4) = 0xFFFF0000;
		*(req + 5) = 1500;
		*(req + 6) = 0;
		*(req + 7) = 0;
		*(req + 8) = 0;

		/*
		 * Queue to userland, but we have handled it!
		 */
		(void)queue_to_userland(0, msg, dnas_dev, 1);
	} else if (msg->u.netReq.cmd == SET_NET_INFO ||
			msg->u.netReq.cmd == SET_IMMED_NET_INFO) {
		DBG(5, KERN_INFO "%s: Got GET_SET_INFO or SET_IMMED request\n",
			__func__);
		/*
		 * Hand off to userland
		 */
		return(queue_to_userland(0, msg, dnas_dev, 0));
	} else if (msg->u.netReq.cmd == SET_LX_COMMON_INFO) {
		DBG(5, KERN_INFO "%s: Got SET_LX_COMMON_INFO request\n",
			__func__);

                /* 
                 * Store the serial number in the global variable for 
                 * proc entry
                 */
                if(msg->u.netReq.bufId.length == 
                        (sizeof(VX_TO_LX_COMMON_INFO) + 
                         (2 * sizeof(unsigned int)))) {
                        memset(dri_dnas_serial_number, 0x0, 
                                sizeof(dri_dnas_serial_number));
                        strncpy(dri_dnas_serial_number, (char *)(req + 2), 
                                sizeof(dri_dnas_serial_number) - 1);
                }


                /*
		 * XXX: Fixme.
		 * Eventually we have to pass this to userland and let
		 * userland process it. For the moment say we dealt with it!
		 */
		(void)queue_to_userland(0, msg, dnas_dev, 1);
	} else {
		printk(KERN_ERR "%s: Invalid intercore control message: %d\n",
			__func__, msg->u.netReq.cmd);
                dump_stack();
		ret = -EINVAL;
	}

reply:
	reply.version = INTER_CORE_CMD_PROTOCOL_CUR_VERSION;
	reply.tag     = msg->tag;
	reply.flags   = NET_INFO | RTN_RESOURCE;

	reply.u.netReq.version = NETWORK_INFO_CUR_VERSION;
	reply.u.netReq.status  = (ret) ? NET_INFO_ERROR : NET_INFO_OK;

	if (req && (msg->u.netReq.bufId.length >= (sizeof(unsigned int) * 2))) {
		*req       = reply.u.netReq.version;
		*(req + 1) = reply.u.netReq.status;
	}

	send_args.handle      = dnas_dev->core_ctrl_queue_handle;
	send_args.msg         = (void *)&reply;
	send_args.msgSize     = sizeof(reply);
	send_args.flags       = SHMQ_SEND_FLAG_WAIT_FOREVER;
	send_args.timeoutInMS = 0;

	send_ret = shm_queue_kern_send_msg(dnas_dev->shm_dev, &send_args);
	if (send_ret || send_args.status != SHMQSTATUS_OK) {
		printk(KERN_ERR "%s: Failed to send message: %d, %d\n",
			__func__, send_ret, send_args.status);
                dump_stack();
		ret = (send_ret) ? send_ret : -EINVAL;
	}

	return ret;
}

int handle_intercore_msg(InterCoreCtrlVxToLx *msg, 
			struct dri_dnas_device *dnas_dev)
{
	int ret = 0;

	if (msg->flags & ISCSI_ENABLE) {
		struct task_struct *th;
		InterCoreCtrlLxToVx send_msg;
		SharedMemQueueSendMsg send_args;

		DBG(0, KERN_INFO "%s: Got ISCSI_ENABLE message\n",
			__func__);

		if (!dnas_dev->driver_init_complete) {

			/* 
		 	* Start the other threads ... when all done will tell 
		 	* other core from one of the other threads. Start the 
		 	* resource thread before the scsi thread to ensure 
		 	* that we never send a command before we can handle 
		 	* resource messages.
		 	*/
			th = kthread_run(dri_dnas_shared_mem_resource_thread, 
			dnas_dev, "dri_shared_res_%d", 1);
			if (IS_ERR(th)) {
				printk(KERN_ERR "%s: unable to start thread to "
				"handle resource responses: %ld\n", __func__, 
				PTR_ERR(th));
				ret = PTR_ERR(th);
				return ret;
			}

			dnas_dev->res_th = th;

			th = kthread_run(dri_dnas_shared_mem_scsi_thread, 
				dnas_dev, "dri_scsi_resp_%d", 2);
			if (IS_ERR(th)) {
				printk(KERN_ERR "%s: unable to start thread to "
					"handle SCSI responses: %ld\n", 
					__func__, PTR_ERR(th));
				ret = PTR_ERR(th);
				return ret;
			}

			dnas_dev->scsi_resp_th = th;

			/*
	 		 * Initialize the buffer spaces. We need them before 
	 		 * any SCSI requests can be handled.
	 		 */ 
			init_write_buffer(dnas_dev);
			init_lx_dyn_buffer(dnas_dev);

			/* 
			 * Now, start the start the SCSI Request handling queue
			 */
			th = kthread_run(dri_dnas_scsi_request_thread, 
				dnas_dev, "dri_scsi_req_%d", 3);
			if (IS_ERR(th)) {
				printk(KERN_ERR "%s: unable to start thread to "					"handle SCSI requests: %ld\n", 
					__func__, PTR_ERR(th));
				ret = PTR_ERR(th);
				return ret;
			}

			dnas_dev->scsi_req_th = th;

			dnas_dev->driver_init_complete = 1;

                        /*
                         * Queue to sysfs
                         */
                        printk(KERN_INFO "%s: QUEUING ISCSI_ENABLE to sysfs\n",
                                __func__);
                        if (queue_to_sysfs(msg, dnas_dev)) {
                                DBG(0, KERN_INFO "%s: Failed to queue "
                                        "ISCSI_ENABLE to sysfs!\n", 
                                        __func__);
                        }
		}

                /*
                 * If we have already done the rest, get out of here because
                 * in some cases we can get an Enabled message twice ...
                 */
                if (dnas_dev->scsi_enabled == 0x01) {
                        DBG(5, KERN_INFO "%s: ISCSI_ENABLE called when already"
                                "enabled\n", __func__);
                        return ret;
                }
		/*
		 * Also send the interface up message from here, 
		 * since no SCSI requests will flow until the device is
		 * registered this is fine and makes life easier for other 
		 * threads. We will also register the device which creates the 
		 * SCSI host as well and scans the devices too.
		 */
		send_msg.version = INTER_CORE_CMD_PROTOCOL_CUR_VERSION;
		send_msg.tag = 0;
		send_msg.flags = ISCSI_UP;

		send_args.handle  = dnas_dev->core_ctrl_queue_handle;
		send_args.msg     = (void *)&send_msg;
		send_args.msgSize = sizeof(send_msg);
		send_args.flags   = SHMQ_SEND_FLAG_WAIT_FOREVER;
		send_args.timeoutInMS = 0;

		ret = shm_queue_kern_send_msg(dnas_dev->shm_dev, 
				&send_args);
		if (ret) {
			printk(KERN_ERR "%s: Could not send SCSI UP "
			"message to other core: %d\n", __func__, ret);
			return ret;
		}

		/*
		 * Now, register the device. This causes the probe routine
		 * to run, which creates a SCSI host and scans it, which 
		 * causes all hell to break loose!
		 */
		dnas_dev->dev.bus     = &dri_dnas_fake_lld_bus;
		dnas_dev->dev.parent  = &dri_dnas_fake_primary;
		dnas_dev->dev.release = dri_dnas_fake_0_release; 
		sprintf(dnas_dev->dev.bus_id, "dnas_adp_%d", 1);

		ret = device_register(&dnas_dev->dev);
		if (ret) {
			printk(KERN_ERR "%s: error registering device: %d. "
				"The kernel cannot see a SCSI device\n",
				__func__, ret);
			/* Keep on trucking so we can handle requests */
		} else {
			dnas_dev->scsi_enabled = 0x1;
		}
	}

	if (msg->flags & ISCSI_DISABLE) {
		DBG(0, KERN_INFO "%s: Got ISCSI_DISABLE message\n",
			__func__);

		/*
		 * This is going to remove the SCSI host!
		 */
		device_unregister(&dnas_dev->dev);
		dnas_dev->scsi_enabled = 0x0;
		dnas_dev->dev_path_present = 0x0;

		/*
		 * If someone is waiting on core control, send them a bad
		 * status event and complete them so they wake up and 
		 * return.
		 */
		if (dnas_dev->waiting_for_core_ctrl) {
			ret = queue_to_userland(-ENODEV, NULL, dnas_dev, 1);
			if (ret) {
				DBG(0, KERN_INFO "%s: Failed to queue error "
					"to userland: %d\n", __func__, ret);
			}
		}
	}

        if (msg->flags & LX_CLEARDISK) {
                DBG(0, KERN_INFO "%s: Got a LX_CLEARDISK request\n",
                        __func__);
                return queue_to_sysfs(msg, dnas_dev);
        }

	if (msg->flags & NET_INFO) {
		printk(KERN_INFO "%s: Got a NET_INFO request\n",
			__func__);
		ret = handle_net_info(msg, dnas_dev);
		if (ret) {
			DBG(5, KERN_ERR "%s: Error handling NET_INFO: "
				"%d\n", __func__, ret);
			return ret;
		}
	}
	if (msg->flags & FS_MIGRATION) {
                printk(KERN_INFO "%s: Got an FS_MIGRATION request\n",
                        __func__);
                ret = queue_to_sysfs(msg, dnas_dev);
                if (ret) {
                        DBG(5, KERN_ERR "%s: Error handling FS_MIGRATION: "
                                "%d\n", __func__, ret);
                        return ret;
                }
        }

        /*
         * We queue the first TIME_VALID message to the sysfs interface
         * Subsequent ones go via the IOCTL method.
         */
	if (msg->flags & TIME_VALID) {
		DBG(0, KERN_INFO "%s: Got Time Valid message\n",
			__func__);
                if (!dnas_dev->first_time_valid) {
                        dnas_dev->first_time_valid = 1;
                        return queue_to_sysfs(msg, dnas_dev);
                } else {
		        return queue_to_userland(0, msg, dnas_dev, 1); 
                }
	}

        if (msg->flags & LX_CLEARFLASH) {
                DBG(0, KERN_INFO "%s: Got LX_CLEARFLASH message\n", __func__);
                return queue_to_userland(0, msg, dnas_dev, 1);
        }

	if (msg->flags & UNEXPECTED_REBOOT) {
		DBG(0, KERN_INFO "%s: Got UNEXEPECTED_REBOOT "
			"message\n", __func__);
		return queue_to_userland(0, msg, dnas_dev, 1); 
	}

        if (msg->flags & LX_SHUTDOWN) {
                DBG(0, KERN_INFO "%s: Got LX_SHUTDOWN message\n",
                        __func__);
                return queue_to_userland(0, msg, dnas_dev, 0);
        }

	return ret;
}


/*
 * The kernel thread that waits for shared mem init and then handles the
 * comms queues.
 */

int dri_dnas_shared_mem_comms_thread(void *data)
{
	int ret = 0;
	struct dri_dnas_device *dnas_dev = (struct dri_dnas_device *)data;  
	SharedMemQueueAttach attach_args;
	SharedMemQueueRecvMsg recv_args;
	InterCoreCtrlVxToLx msg;

	dnas_dev->shm_dev = dri_shm_get_dev();

	if (!dnas_dev->shm_dev) {
		printk(KERN_ERR "%s: Could not get shm area pointer!\n",
			__func__);
		return -ENOENT;
	}

	/*
	 * Now, wait for the intercore to come up and then open the 
	 * shared mem stuff, get pointers to the shared areas and then
	 * attach to the queues.
	 */
	wait_for_init_shared_mem(dnas_dev->shm_dev);

	ret = shared_mem_open_main(dnas_dev->shm_dev);

	if (ret) {
		printk(KERN_ERR "%s: unable to open shared mem areas: %d\n",
			__func__, ret);
		return ret;
	}

	/*
         * Get the sizes of each of the regions we are interested in.
         * These defs should come from a file!
         */
	ret = find_region_kern_addr_from_tag(dnas_dev->shm_dev, "SHJ1",
                                             &dnas_dev->write_buffer_phys,
					     &dnas_dev->write_buffer_start,
					     &dnas_dev->write_buffer_size);
	if (ret) {
		printk(KERN_ERR "%s: Unable to get SHJ1 info: %d\n",
			__func__, ret);
		return ret;
	}

	ret = find_region_kern_addr_from_tag(dnas_dev->shm_dev, "HLRC",
                                             &dnas_dev->hlbat_buffer_phys,
					     &dnas_dev->hlbat_buffer_start,
					     &dnas_dev->hlbat_buffer_size);
	if (ret) {
		printk(KERN_ERR "%s: Unable to get HLRC info: %d\n",
			__func__, ret);
		return ret;
	}

	ret = find_region_kern_addr_from_tag(dnas_dev->shm_dev, "SGLD",
                                             &dnas_dev->sgl_buffer_phys,
					     &dnas_dev->sgl_buffer_start,
					     &dnas_dev->sgl_buffer_size);
	if (ret) {
		printk(KERN_ERR "%s: Unable to get SGLD info: %d\n",
			__func__, ret);
		return ret;
	}

	ret = find_region_kern_addr_from_tag(dnas_dev->shm_dev, "SHLX",
                                             &dnas_dev->lx_dyn_buffer_phys,
					     &dnas_dev->lx_dyn_buffer_start,
					     &dnas_dev->lx_dyn_buffer_size);
	if (ret) {
		printk(KERN_ERR "%s: Unable to get SHLX info: %d\n",
			__func__, ret);
		return ret;
	}

	ret = find_region_kern_addr_from_tag(dnas_dev->shm_dev, "SHVX",
                                             &dnas_dev->vx_dyn_buffer_phys,
					     &dnas_dev->vx_dyn_buffer_start,
					     &dnas_dev->vx_dyn_buffer_size);
	if (ret) {
		printk(KERN_ERR "%s: Unable to get SHVX info: %d\n",
			__func__, ret);
		return ret;
	}

	printk("Shared Memory Partition Map Addresses:\n");
	printk("-----------------------------------------------------------------------------\n");
	printk("SHJ1: Start:\t 0x%x, End:\t 0x%x, Phys:\t 0x%x\n", (unsigned int) dnas_dev->write_buffer_start, ((unsigned int) dnas_dev->write_buffer_start) + dnas_dev->write_buffer_size, (unsigned int)dnas_dev->write_buffer_phys);
	printk("HLRC: Start:\t 0x%x, End:\t 0x%x, Phys:\t 0x%x\n", (unsigned int) dnas_dev->hlbat_buffer_start, ((unsigned int) dnas_dev->hlbat_buffer_start) + dnas_dev->hlbat_buffer_size, (unsigned int) dnas_dev->hlbat_buffer_phys);
	printk("SGLD: Start:\t 0x%x, End:\t 0x%x, Phys:\t 0x%x\n", (unsigned int) dnas_dev->sgl_buffer_start, ((unsigned int) dnas_dev->sgl_buffer_start) + dnas_dev->sgl_buffer_size, (unsigned int) dnas_dev->sgl_buffer_phys);
	printk("SHLX: Start:\t 0x%x, End:\t 0x%x, Phys:\t 0x%x\n", (unsigned int) dnas_dev->lx_dyn_buffer_start, ((unsigned int) dnas_dev->lx_dyn_buffer_start) + dnas_dev->lx_dyn_buffer_size, (unsigned int) dnas_dev->lx_dyn_buffer_phys);
	printk("SHVX: Start:\t 0x%x, End:\t 0x%x, Phys:\t 0x%x\n", (unsigned int) dnas_dev->vx_dyn_buffer_start, ((unsigned int) dnas_dev->vx_dyn_buffer_start) + dnas_dev->vx_dyn_buffer_size, (unsigned int)dnas_dev->vx_dyn_buffer_phys);
	printk("-----------------------------------------------------------------------------\n");

	/*
	 * Now attach to the queues ...
	 */ 
	memset(&attach_args, 0, sizeof(SharedMemQueueAttach));
	strcpy(attach_args.qPairName, "CoreMsgQPair");
	attach_args.flags = SHMQ_ATTACH_FLAG_WAIT_FOREVER_POLL;

	ret = shm_queue_attach(dnas_dev->shm_dev, &attach_args);

	if (ret) {
		printk(KERN_ERR "%s: Unable to get CoreMsgQPair handle: %d\n",
			__func__, ret);
		return ret;
	}

	dnas_dev->core_ctrl_queue_handle = attach_args.handle;

	memset(&attach_args, 0, sizeof(SharedMemQueueAttach));
	strcpy(attach_args.qPairName, "ScsiQPair");
	attach_args.flags = SHMQ_ATTACH_FLAG_WAIT_FOREVER_POLL;

	ret = shm_queue_attach(dnas_dev->shm_dev, &attach_args);

	if (ret) {
		printk(KERN_ERR "%s: Unable to get ScsiQPair handle: %d\n",
			__func__, ret);
		return ret;
	}

	dnas_dev->scsi_queue_handle = attach_args.handle;

	memset(&attach_args, 0, sizeof(SharedMemQueueAttach));
	strcpy(attach_args.qPairName, "ResMsgQPair");
	attach_args.flags = SHMQ_ATTACH_FLAG_WAIT_FOREVER_POLL;

	ret = shm_queue_attach(dnas_dev->shm_dev, &attach_args);

	if (ret) {
		printk(KERN_ERR "%s: Unable to get ResMsgQPair handle: %d\n",
			__func__, ret);
		return ret;
	}

	dnas_dev->resource_queue_handle = attach_args.handle;

	/*
	 * Allocate a cahce for our tags first ...
	 */
	dnas_dev->dnas_tag_cache = kmem_cache_create("dnas_tag_cache",
				sizeof(struct dnas_tag_struct),
				0, 0, NULL, NULL);
	if (!dnas_dev->dnas_tag_cache) {
		printk(KERN_WARNING "%s: We're fucked! No memory for our "
			"tag cache.", __func__);
		return -ENOMEM;
	}

	/*
	 * Allocate a cache for core ctrl messages to userspace
	 */
	dnas_dev->core_ctrl_cache = kmem_cache_create("dnas_core_ctrl_cache",
				sizeof(struct dnas_userland_queue_elt),
				0, 0, NULL, NULL);
	if (!dnas_dev->core_ctrl_cache) {
		printk(KERN_WARNING "%s: Could not allocate memory for our "
			"core control cache.", __func__);
		return -ENOMEM;
	}

	/*
	 * Now, process events on the Core Control Command queue.
	 */
	memset(&recv_args, 0, sizeof(recv_args));
	recv_args.flags       = SHMQ_RECV_FLAG_WAIT_FOREVER_INT;
	recv_args.timeoutInMS = 0;
	recv_args.handle      = dnas_dev->core_ctrl_queue_handle;
	recv_args.buf         = (void *)&msg;
	recv_args.bufSize     = sizeof(msg);

	while (!kthread_should_stop()) {

		ret = shm_queue_kern_recv_msg(dnas_dev->shm_dev, &recv_args);
		if (ret) {
			printk(KERN_ERR "%s: Unable to receive message from "
				"other core: %d\n", __func__, ret);
			return ret;
		}

		ret = handle_intercore_msg(&msg, dnas_dev);
		if (ret) {
			/* We are done here, I think ... */
			return ret;
		}
	}
	
	return 0;
}

/*
 * Init and exit routines
 */

/*
 * We register the primary device, register the (fake) bus, register the 
 * driver, and start the thread that waits for inter-core comms to come up.
 */
static struct device_driver dri_dnas_driverfs_driver = {
	.name = dri_dnas_proc_name,
	.bus  = &dri_dnas_fake_lld_bus,
};

static int dri_dnas_fake_match(struct device *dev,
				struct device_driver *dev_driver)
{
	/*struct dri_dnas_device *dnas_dev = to_dri_dnas_device(dev);*/

	/* Pretend all is OK, because it is */
	return 1;
}

/*
 * We are called when one of our devices is added. We create a SCSI host
 * add it and scan the bus.
 */
static int dri_dnas_fake_probe(struct device *dev)
{
	int ret = 0;
	struct dri_dnas_device *dnas_dev = NULL;
	struct Scsi_Host *host;

	dnas_dev = to_dri_dnas_device(dev);

	/*
	 * I don't think I need any space in the host struct, but add
	 * enough for one unit32_t.
	 */
	host = scsi_host_alloc(&dri_dnas_host_template, 
			sizeof(uint32_t));
	if (NULL == host) {
		printk(KERN_ERR "%s: unable to allocate memory for SCSI host\n",
			__func__);
		return -ENODEV;
	}

	host->max_cmd_len = 16; /* Make sure we accept 16-byte CDBs, Jane! */

	*((struct dri_dnas_device **)host->hostdata) = dnas_dev;
	host->max_id = 1;  /* Only one target out there */
	host->max_lun = dri_dnas_max_luns - 1;

	/* Going live now! */
	ret = scsi_add_host(host, dev);
	if (ret) {
		printk(KERN_ERR "%s: failed to add SCSI host: %d\n",
			__func__, ret);
		scsi_host_put(host);
		ret = -ENODEV;
	} else {
		scsi_scan_host(host);
        /* GP: I am assuming the /dev path is present at this point or very close to it */
        dnas_dev->dev_path_present = 0x1;
	}

	dnas_dev->shost = host;

	return ret;
}

static int dri_dnas_fake_remove(struct device *dev)
{
	int ret = 0;
	struct dri_dnas_device *dnas_dev = NULL;

	dnas_dev = to_dri_dnas_device(dev);

	printk(KERN_INFO "%s: removing device: %p\n", __func__, dnas_dev);

	if (!dnas_dev) {
		printk(KERN_ERR "%s: Unable to locate host info for dev: %p!\n",
			__func__, dev);
		return -ENODEV;
	}

        dnas_dev->dev_path_present = 0x0;

	scsi_remove_host(dnas_dev->shost);

	scsi_host_put(dnas_dev->shost);

	return ret;
}

static struct bus_type dri_dnas_fake_lld_bus = {
	.name   = "dri_dnas_fake_bus",
	.match  = dri_dnas_fake_match,
	.probe  = dri_dnas_fake_probe,
	.remove = dri_dnas_fake_remove,
};

static void dri_dnas_fake_0_release(struct device *dev)
{
}

static struct device dri_dnas_fake_primary = {
	.bus_id	 = "dri_dnas_primary",
	.release = dri_dnas_fake_0_release,
};

static __init int dri_dnas_scsi_init(void)
{
	int ret = 0;
	struct task_struct *th = NULL;
	struct dri_dnas_device *dev = NULL;

	dev = kzalloc(sizeof(struct dri_dnas_device), GFP_KERNEL);
	if (NULL == dev) {
		printk(KERN_ERR "%s: out of memory allocating"
			" dri_dnas_dev at line %d\n", __func__, __LINE__);
		return -ENOMEM;
	}

	ret = device_register(&dri_dnas_fake_primary);
	if (ret < 0) {
		printk(KERN_WARNING "%s: device register error: %s, %d\n",
			__func__, dri_dnas_fake_primary.bus_id, ret);
		goto done;
	}
	ret = bus_register(&dri_dnas_fake_lld_bus);
	if (ret < 0) {
		printk(KERN_WARNING "%s: unable to register bus %s: %d\n",
			__func__, dri_dnas_fake_lld_bus.name, ret);
		goto unregister_primary;
	}
	ret = driver_register(&dri_dnas_driverfs_driver);
	if (ret < 0) {
		printk(KERN_WARNING "%s: unable to register driverfs %s: %d\n",
			__func__, dri_dnas_driverfs_driver.name, ret);
		goto unregister_bus;
	}

	/*
         * Init some field in our dev structure ...
         */
	strncpy(dev->init_name, "dri_dnas_host_0", sizeof(dev->init_name));
	spin_lock(&dri_dnas_device_list_lock);
	list_add_tail(&dev->device_list, &dri_dnas_device_list);
	spin_unlock(&dri_dnas_device_list_lock);

	/*
	 * Init the mutexes here etc ... might want to move this to the thread
	 */
	mutex_init(&dev->core_ctrl_queue_mutex);
	mutex_init(&dev->scsi_completion_mutex);
	mutex_init(&dev->wb_mutex);
	mutex_init(&dev->lx_db_mutex);
	dev->req_spinlock = SPIN_LOCK_UNLOCKED;
	init_completion(&dev->req_completion);
	init_completion(&dev->buffer_completion);
	dev->req_thread_waiting = 0;
	dev->threads_waiting_for_buffers = 0;

	/*
	 * Init the various queues since one is needed early
	 */
	INIT_LIST_HEAD(&dev->req_queue);
	INIT_LIST_HEAD(&dev->core_ctrl_queue);

	/*
	 * Create the driver files we need ...
	 */
	dnas_create_driverfs_files();

	/*
	 * Now start up that kernel thread that does stuff in the background
	 */
	th = kthread_run(dri_dnas_shared_mem_comms_thread, dev, 
		"dri_dnas_thread_%d", 0);
	if (IS_ERR(th)) {
		printk(KERN_ERR "%s: unable to start thread to init shared mem"
			"comms: %ld\n", __func__, PTR_ERR(th));
		ret = PTR_ERR(th);
		goto unregister_driver;
	}


done:
	return ret;

unregister_driver:
	driver_unregister(&dri_dnas_driverfs_driver);
unregister_bus:
	bus_unregister(&dri_dnas_fake_lld_bus);
unregister_primary:
	device_unregister(&dri_dnas_fake_primary);
	kfree(dev);

	goto done;
}


static __exit void dri_dnas_scsi_exit(void)
{

}

device_initcall(dri_dnas_scsi_init);
module_exit(dri_dnas_scsi_exit);

MODULE_AUTHOR("www.drobo.com");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DNAS SCSI Low Level Driver");
