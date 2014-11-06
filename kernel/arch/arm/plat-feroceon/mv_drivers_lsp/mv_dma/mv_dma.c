/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysdev.h>
#include <asm/mach/time.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>

#include "idma/mvIdma.h"
#include "ctrlEnv/sys/mvSysIdma.h"
#include "ctrlEnv/mvCtrlEnvLib.h"
#include "../shared_mem/sharedMemCommon.h"

#undef DEBUG
//#define DEBUG

#ifdef DEBUG
	#define DPRINTK(s, args...)  printk("MV_DMA: " s, ## args)
#else
	#define DPRINTK(s, args...)
#endif


#undef DEBUG
//#define DEBUG

#ifdef DEBUG
	#define DPRINTK(s, args...)  printk("MV_DMA: " s, ## args)
#else
	#define DPRINTK(s, args...)
#endif

#undef CPY_USE_DESC
#undef RT_DEBUG
#define RT_DEBUG
#define CONFIG_ENABLE_IDMA_INTERRUPTS

#define CPY_IDMA_CTRL_LOW_VALUE      ICCLR_DST_BURST_LIM_128BYTE   \
                                    | ICCLR_SRC_BURST_LIM_128BYTE   \
                                    | ICCLR_INT_MODE_MASK           \
                                    | ICCLR_BLOCK_MODE              \
                                    | ICCLR_CHAN_ENABLE             \
                                    | ICCLR_DESC_MODE_16M

#define DRI_COPY    3
#define USER_COPY   2
#define MEM_COPY    1
#define IDLE        0

#define USERTO_DMA_CHAN    1
#define USERFROM_DMA_CHAN  1
#define MEMZERO_DMA_CHAN   1
#define MEMCOPY_DMA_CHAN   0  // atomic usage
#define DRI_MEMCPY_CHAN    1  // non-atomic usage

#define CPY_DMA_TIMEOUT	   0x100000
#define EVENT_DMA_TIMEOUT	     100000  // wait about a seconf for an iDMA to complete

//#define NEXT_CHANNEL(channel)	(((channel) + 1) % MV_IDMA_MAX_CHAN)
//#define PREV_CHANNEL(channel)	((channel) ? (channel) - 1 : (MV_IDMA_MAX_CHAN - 1))

struct idma_channel_t
{
  MV_DMA_DESC *pDescriptor;
  dma_addr_t   descPhyAddr;
  
  wait_queue_head_t waitq;
  struct semaphore  sema;
  
#ifdef CONFIG_ENABLE_IDMA_INTERRUPTS
  int   atomic;
  int   irq_num;
  const char *name;
#endif

  int chan_alloc;
  int chan_int;
  int bad_int;
  int chan_num;
  int chan_active;
};

struct idma_channel_t  idma_channel[MV_IDMA_MAX_CHAN];


static MV_U32 current_dma_channel = 0;
static int idma_init = 0;
static int idma_busy = 0;

#ifdef RT_DEBUG
static int dma_wait_loops = 0;
#endif

#ifdef CONFIG_MV_IDMA_MEMZERO
#define	DMA_MEMZERO_CHUNK    0x80 /*128*/    /* this is based on the size of the DST and SRC burst limits */
static MV_U8	dmaMemInitBuff[DMA_MEMZERO_CHUNK] __attribute__(( aligned(128) ));
#endif

#define IDMA_MIN_COPY_CHUNK CONFIG_MV_IDMA_COPYUSER_THRESHOLD

static int dma_to_user      = 0;
static int dma_from_user    = 0;
static int dma_dricpy_cnt   = 0;
static int dma_memcpy_cnt   = 0;
static int dma_memzero_cnt  = 0;
static int dma_chan_miss    = 0;
static int dma_dri_miss    = 0;
static int dma_memzero_miss = 0;
static int dma_user_miss    = 0;
static int dma_chan_timeout = 0;
static long dma_user_size   = 0;
static long dma_user_cnt    = 0;
static long dma_user_index  = 0;
static int dma_interrupts   = 0;

#ifdef RT_DEBUG
static int dma_activations = 0;
#endif


/*======================================================================================*/
/* Code to do alternate lookup for physical addresses */
/*======================================================================================*/
#ifdef CONFIG_MV_IDMA_COPYUSER
#define DRI_VIRT_TO_PHYS_MAX_ENTRIES            128
#define DRI_PHYS_UNCACHED_ADDR_START_LIMIT      ((512 * 1024 * 1024) - SHARED_MEM_TOTAL_SHARED_MEM)

struct dri_virt_to_phys_entry
{
  u32 valid;
  struct mm_struct *mm;         /* Which space was this entry mapped in */
  u32 virt_base;
  u32 phys_base;
  u32 map_size;
};

struct dri_virt_to_phy_info
{
  u32 num_entries;
  struct dri_virt_to_phys_entry entries[DRI_VIRT_TO_PHYS_MAX_ENTRIES];
};

rwlock_t dir_translate_lock = RW_LOCK_UNLOCKED;
struct dri_virt_to_phy_info dri_translate_info = { 0 };

/* Currently we don't have any unregister routines */
int register_dri_virt_to_phys_range(u32 phys_base, u32 virt_base, u32 map_size)
{
  int i;
  struct mm_struct * mm = (virt_base >= TASK_SIZE)? &init_mm : current->mm;

  if (map_size == 0)
  {
    return 0;
  }

  /* We make sure there are no overlaps */
  write_lock(&dir_translate_lock);

  for (i = 0; i < dri_translate_info.num_entries; i++)
  {
    if (dri_translate_info.entries[i].valid && (mm == dri_translate_info.entries[i].mm))
    {
      if (((dri_translate_info.entries[i].virt_base <= virt_base) &&
          ((dri_translate_info.entries[i].virt_base + dri_translate_info.entries[i].map_size) > virt_base)) ||
          ((dri_translate_info.entries[i].virt_base < (virt_base + map_size)) &&
          ((dri_translate_info.entries[i].virt_base + dri_translate_info.entries[i].map_size) >= (virt_base + map_size))))
      {
        write_unlock(&dir_translate_lock);
        printk("register_dri_virt_to_phys_range: ERROR: In memory space: 0x%x, Overlap in new range registered\n", (u32) mm);
        printk("\tNew Range: Virt addr: 0x%x, Phys addr: 0x%x, Size: 0x%x\n", virt_base, phys_base, map_size);
        printk("\tOverlaps with: Virt addr: 0x%x, Phys addr: 0x%x, Size: 0x%x\n",
          dri_translate_info.entries[i].virt_base, dri_translate_info.entries[i].phys_base, dri_translate_info.entries[i].map_size);
        return -1;
      }
    }
  }

  dri_translate_info.entries[dri_translate_info.num_entries].mm = mm;
  dri_translate_info.entries[dri_translate_info.num_entries].virt_base = virt_base;
  dri_translate_info.entries[dri_translate_info.num_entries].phys_base = phys_base;
  dri_translate_info.entries[dri_translate_info.num_entries].map_size = map_size;
  dri_translate_info.entries[dri_translate_info.num_entries].valid = 0x1;

  dri_translate_info.num_entries++;

  write_unlock(&dir_translate_lock);
  printk("register_dri_virt_to_phys_range: Registered New Range for translation: Virt addr: 0x%x, Phys addr: 0x%x, Size: 0x%x\n",
    virt_base, phys_base, map_size);
  return 0;
}

EXPORT_SYMBOL(register_dri_virt_to_phys_range);

/* translate the address if possible */
static int dri_translate_virt_to_phys(struct mm_struct * mm, u32 virt_addr, u32 *phys_addr)
{
  int i;

  read_lock(&dir_translate_lock);

  for (i = 0; i < dri_translate_info.num_entries; i++)
  {
    if (dri_translate_info.entries[i].valid && (mm == dri_translate_info.entries[i].mm))
    {
      if ((virt_addr >= dri_translate_info.entries[i].virt_base) &&
        (virt_addr < (dri_translate_info.entries[i].virt_base + dri_translate_info.entries[i].map_size)))
      {
        *phys_addr = dri_translate_info.entries[i].phys_base + (virt_addr - dri_translate_info.entries[i].virt_base);
        read_unlock(&dir_translate_lock);
        return 0;
      }
    }
  }

  read_unlock(&dir_translate_lock);
  return -1;
}

rwlock_t dir_ktranslate_lock = RW_LOCK_UNLOCKED;
struct dri_virt_to_phy_info dri_ktranslate_info = { 0 };

/* Currently we don't have any unregister routines */
int register_dri_kernel_virt_to_phys_range(u32 phys_base, u32 virt_base, u32 map_size)
{
  int i;

  if (map_size == 0)
  {
    return 0;
  }

  /* We make sure there are no overlaps */
  write_lock(&dir_ktranslate_lock);

  for (i = 0; i < dri_ktranslate_info.num_entries; i++)
  {
    if (dri_ktranslate_info.entries[i].valid)
    {
      if (((dri_ktranslate_info.entries[i].virt_base <= virt_base) &&
          ((dri_ktranslate_info.entries[i].virt_base + dri_ktranslate_info.entries[i].map_size) > virt_base)) ||
          ((dri_ktranslate_info.entries[i].virt_base < (virt_base + map_size)) &&
          ((dri_ktranslate_info.entries[i].virt_base + dri_ktranslate_info.entries[i].map_size) >= (virt_base + map_size))))
      {
        write_unlock(&dir_ktranslate_lock);
        printk("register_dri_kernel_virt_to_phys_range: ERROR: Overlap in new range registered\n");
        printk("\tNew Range: Virt addr: 0x%x, Phys addr: 0x%x, Size: 0x%x\n", virt_base, phys_base, map_size);
        printk("\tOverlaps with: Virt addr: 0x%x, Phys addr: 0x%x, Size: 0x%x\n",
          dri_ktranslate_info.entries[i].virt_base, dri_ktranslate_info.entries[i].phys_base, dri_ktranslate_info.entries[i].map_size);
        return -1;
      }
    }
  }

  dri_ktranslate_info.entries[dri_ktranslate_info.num_entries].mm = 0;
  dri_ktranslate_info.entries[dri_ktranslate_info.num_entries].virt_base = virt_base;
  dri_ktranslate_info.entries[dri_ktranslate_info.num_entries].phys_base = phys_base;
  dri_ktranslate_info.entries[dri_ktranslate_info.num_entries].map_size = map_size;
  dri_ktranslate_info.entries[dri_ktranslate_info.num_entries].valid = 0x1;

  dri_ktranslate_info.num_entries++;

  write_unlock(&dir_ktranslate_lock);
  printk("register_dri_kernel_virt_to_phys_range: Registered New Range for translation: Virt addr: 0x%x, Phys addr: 0x%x, Size: 0x%x\n",
    virt_base, phys_base, map_size);
  return 0;
}

EXPORT_SYMBOL(register_dri_kernel_virt_to_phys_range);

/* translate the address if possible */
static int dri_translate_kernel_virt_to_phys(u32 virt_addr, u32 *phys_addr)
{
  int i;

  read_lock(&dir_ktranslate_lock);

  for (i = 0; i < dri_ktranslate_info.num_entries; i++)
  {
    if (dri_ktranslate_info.entries[i].valid)
    {
      if ((virt_addr >= dri_ktranslate_info.entries[i].virt_base) &&
        (virt_addr < (dri_ktranslate_info.entries[i].virt_base + dri_ktranslate_info.entries[i].map_size)))
      {
        *phys_addr = dri_ktranslate_info.entries[i].phys_base + (virt_addr - dri_ktranslate_info.entries[i].virt_base);
        read_unlock(&dir_ktranslate_lock);
        return 0;
      }
    }
  }

  read_unlock(&dir_ktranslate_lock);
  return -1;
}

#endif
/*======================================================================================*/

static inline u32 page_remainder(u32 virt)
{
  return PAGE_SIZE - (virt & ~PAGE_MASK);
}

/*
 * map a kernel virtual address or kernel logical address to a phys address
 */
static inline u32 physical_address(u32 virt, int write)
{
  struct page *page;
  struct mm_struct * mm = (virt >= TASK_SIZE)? &init_mm : current->mm;

  /* kernel static-mapped address */
  DPRINTK(" get physical address: virt %x , write %d\n", virt, write);

#ifdef CONFIG_MV_IDMA_COPYUSER
  {
    /* First our own mapping */
    u32 phys_addr = 0;
    if (!dri_translate_virt_to_phys(mm, virt, &phys_addr))
    {
      return phys_addr;
    }
  }
#endif

  if (virt_addr_valid(virt))
  {
    return __pa((u32) virt);
  }
  
  if (virt >= high_memory)
    return 0;

  if (virt >= TASK_SIZE)
  {
    page = follow_page(find_extend_vma(&init_mm, virt), (u32) virt, write);
  }
  else
  {
    page = follow_page(find_extend_vma(current->mm, virt), (u32) virt, write);
  }

  if (pfn_valid(page_to_pfn(page)))
  {
    return ((page_to_pfn(page) << PAGE_SHIFT) |
      ((u32) virt & (PAGE_SIZE - 1)));
  }
  else
  {
    return 0;
  }
}

/*
 * map a kernel virtual address or kernel logical address to a phys address
 */
static inline u32 kphysical_address(u32 virt, int write)
{
  u32 phys_addr = 0;

  /* kernel static-mapped address */
  DPRINTK(" get physical address: virt %x , write %d\n", virt, write);

  /* First our own mapping */
  if (!dri_translate_kernel_virt_to_phys(virt, &phys_addr))
  {
    return phys_addr;
  }
  else
  {
    return __pa((u32) virt);
  }
}

static int allocate_a_channel(int chan, int mode)
{
  down_interruptible(&idma_channel[chan].sema);
    
  if (mvDmaStateGet(chan) != MV_IDLE)
  {
    printk("ERR: %s iDMA chan %d is not idle", __FUNCTION__, chan);
  }
    
  // update the channel state
  idma_channel[chan].chan_active = mode;
  idma_channel[chan].chan_alloc++;
  
  return 1;
}

static int allocate_a_channel_nowait(int chan, int mode)
{
  if (down_trylock(&idma_channel[chan].sema))
  {
      DPRINTK("iDMA engine %d is busy\n", chan);
      return -1;
  }
    
  if (mvDmaStateGet(chan) != MV_IDLE)
  {
    printk("ERR: %s iDMA chan %d is not idle", __FUNCTION__, chan);
  }
    
  // update the channel state
  idma_channel[chan].chan_active = mode;
  idma_channel[chan].chan_alloc++;
  
  return 0;
}

static struct semaphore alloc_sema;

static int allocate_pool_channel(int mode)
{
  int chan;

  /*
   * Take the semaphore to make sure we don't screw up
   */
  if (down_interruptible(&alloc_sema)) {
    /* Should we retry? Dunno ... punt for now */
    return -1;
  }
 
  for (chan = DRI_MEMCPY_CHAN; chan < MV_IDMA_MAX_CHAN; chan++)
  {
    if (idma_channel[chan].chan_active != IDLE)
    {
      // channel is being used
      continue;
    }
    
    if (mvDmaStateGet(chan) != MV_IDLE)
    {
      printk("ERR: %s iDMA chan %d is not idle", __FUNCTION__, chan);
      continue;
    }
    
    // update the channel state
    idma_channel[chan].chan_active = mode;
    idma_channel[chan].chan_alloc++;
 
    up(&alloc_sema);
 
    return chan;
  }
 
  up(&alloc_sema);
 
  DPRINTK("iDMA engines are busy, return\n");
  return -1;
}

static void inline free_pool_channel(int chan)
{
  MV_U32 reg;

  /*
   * Take the semaphore to make sure we don't screw up
   */
  if (down_interruptible(&alloc_sema)) {
    /* Should we retry? Dunno but this is bad*/
    return;
  }

  if ((reg = mvDmaStateGet(chan)) != MV_IDLE)
  {
    printk(KERN_INFO "ERR: %s: iDMA channel %d is not idle [state 0x%08x)\n",
        __FUNCTION__, chan, reg);
  }

  up(&alloc_sema);
}

static void inline free_channel(int chan)
{
  MV_U32 reg;
  
  if ((reg = mvDmaStateGet(chan)) != MV_IDLE)
  {
    printk("ERR: %s iDMA chan %d is not idle [state 0x%08x)\n", __FUNCTION__, chan, reg);
//    BUG();
  }
  
  up(&idma_channel[chan].sema);
}

unsigned int wait_for_idma(int chan)
{
  u32 timeout = 0;
  DEFINE_WAIT(wait);

//#ifdef CONFIG_ENABLE_IDMA_INTERRUPTS
#if 1
    // wait for channel event
//    timeout = wait_event_timeout(idma_channel[chan].waitq,
//                                 (mvDmaStateGet(chan) != MV_IDLE),
//                                 EVENT_DMA_TIMEOUT);
//    if (timeout <= 0)
//    {
      // event wait timed out
//      dma_chan_timeout++;
//
    // Only wait on event if this is not an atomic call
    if (idma_channel[chan].atomic == 0)
    {
      /* Currently we only support interrupt based wait */
      prepare_to_wait(&idma_channel[chan].waitq, &wait, TASK_INTERRUPTIBLE);
      if (idma_channel[chan].chan_active != IDLE)
      {
         schedule();
      }
      finish_wait(&idma_channel[chan].waitq, &wait);
    }

    while (mvDmaStateGet(chan) != MV_IDLE)
    {
      DPRINTK(" ctrl low is %x \n", MV_REG_READ(IDMA_CTRL_LOW_REG(chan)));

      udelay(10);

#ifdef RT_DEBUG
      dma_wait_loops++;
#endif

      if (timeout++ > CPY_DMA_TIMEOUT)
      {
        printk("dma_copy: IDMA %d timed out , ctrl low is %x \n",
               chan, MV_REG_READ(IDMA_CTRL_LOW_REG(chan)));
        return 1;
      }
    }

    // should mask back iDMA interrupts here??
#else
    /* wait for completion */
    while (mvDmaStateGet(chan) != MV_IDLE)
    {
      DPRINTK(" ctrl low is %x \n", MV_REG_READ(IDMA_CTRL_LOW_REG(chan)));

      udelay(10);

#ifdef RT_DEBUG
      dma_wait_loops++;
#endif

      if (timeout++ > CPY_DMA_TIMEOUT)
      {
        printk("dma_copy: IDMA %d timed out , ctrl low is %x \n",
               chan, MV_REG_READ(IDMA_CTRL_LOW_REG(chan)));
        return 1;
      }
    }

    DPRINTK("IDMA complete in %x \n", timeout);
#endif

  return 0;
}

#if 0
unsigned int wait_for_all_idma(void)
{
  u32 timeout = 0;
  int chan;

  // scan all iDMA channels and wait for each to become idle
  for (chan = 0; chan < MV_IDMA_MAX_CHAN; ++chan)
  {
    // check the channel status
    if ( (idma_channel[chan].chan_active == USER_COPY)
        && (mvDmaStateGet(chan) != MV_IDLE))
    {
#ifdef CONFIG_ENABLE_IDMA_INTERRUPTS
      // wait for channel event
      timeout = wait_event_timeout(idma_channel[chan].waitq,
                                   (mvDmaStateGet(chan) != MV_IDLE),
                                   EVENT_DMA_TIMEOUT);
      if (timeout <= 0)
      {
        // event wait timed out
        dma_chan_timeout++;

        if (mvDmaStateGet(chan) != MV_IDLE)
        {
          printk("dma_copy: IDMA %d timed out , ctrl low is %x \n",
                  chan, MV_REG_READ(IDMA_CTRL_LOW_REG(chan)));
        }
      }

      // should mask back iDMA interrupts here??
#else
      /* wait for completion */
      while (mvDmaStateGet(chan) != MV_IDLE)
      {
        DPRINTK(" ctrl low is %x \n", MV_REG_READ(IDMA_CTRL_LOW_REG(chan)));

        udelay(1);

#ifdef RT_DEBUG
        dma_wait_loops++;
#endif

        if (timeout++ > CPY_DMA_TIMEOUT)
        {
          printk("dma_copy: IDMA %d timed out , ctrl low is %x \n",
                  chan, MV_REG_READ(IDMA_CTRL_LOW_REG(chan)));
          break;
        }
      }

      DPRINTK("IDMA complete in %x \n", timeout);
#endif
    }
  }

  return 0;
}
#endif

static struct proc_dir_entry *dma_proc_entry;

static int dma_read_proc(char *, char **, off_t, int, int *, void *);

static int dma_read_proc(char *buf, char **start, off_t offset, int len,
						 int *eof, void *data)
{
  MV_U32 reg, cause, mask, idmaCause, idmaMask, i, j;
  len = 0;
  
#ifdef CONFIG_ENABLE_IDMA_INTERRUPTS
  len += sprintf(buf + len, "iDMA using interrupts on %d channels\n", MV_IDMA_MAX_CHAN);
#endif

#ifdef CONFIG_MV_IDMA_COPYUSER
  len += sprintf(buf + len, "iDMA min buffer size for copy to/from user %d\n", COPYUSER_MIN_SIZE);
#endif

#ifdef CONFIG_MV_IDMA_MEMCOPY
  len += sprintf(buf + len, "iDMA min buffer size for memcpy and memmove %d\n", CONFIG_MV_IDMA_MEMCOPY_THRESHOLD);
#endif

#ifdef CONFIG_MV_IDMA_MEMZERO
  len += sprintf(buf + len, "iDMA min buffer size for memzero %d\n", CONFIG_MV_IDMA_MEMZERO_THRESHOLD);
#endif

  len += sprintf(buf + len, "Number of iDMA copy to user %d copy from user %d \n", dma_to_user, dma_from_user);
  len += sprintf(buf + len, "Number of iDMA memzero %d \n", dma_memzero_cnt);
  len += sprintf(buf + len, "Number of iDMA memcpy %d \n", dma_memcpy_cnt);
  len += sprintf(buf + len, "Number of iDMA memzero allocation misses %d \n", dma_memzero_miss);
  len += sprintf(buf + len, "Number of iDMA memcpy allocation misses %d \n", dma_chan_miss);
  len += sprintf(buf + len, "Number of iDMA usercpy allocation misses %d \n", dma_user_miss);
  len += sprintf(buf + len, "Number of iDMA wait timeouts %d \n", dma_chan_timeout);
  len += sprintf(buf + len, "Average iDMA user copy size %ld \n", (dma_user_size / dma_user_cnt));
  len += sprintf(buf + len, "Average iDMA user copy chunks %ld \n", (dma_user_index / dma_user_cnt));
  
#ifdef RT_DEBUG
  len += sprintf(buf + len, "Number of iDMA activations %d\n", dma_activations);
  len += sprintf(buf + len, "Number of iDMA interrupts %d\n", dma_interrupts);
  len += sprintf(buf + len, "Number of wait for iDMA loops %d\n", dma_wait_loops);
  
  len += sprintf(buf + len, "Channel 0: Allocations %d, Interrupts %d, Busy %d, mode %d, state 0x%x\n",
                 idma_channel[0].chan_alloc, idma_channel[0].chan_int, idma_channel[0].bad_int,
                 idma_channel[0].chan_active, mvDmaStateGet(0));
  len += sprintf(buf + len, "Channel 1: Allocations %d, Interrupts %d, Busy %d, mode %d, state 0x%x\n",
                 idma_channel[1].chan_alloc, idma_channel[1].chan_int, idma_channel[1].bad_int,
                 idma_channel[1].chan_active, mvDmaStateGet(1));
  len += sprintf(buf + len, "Channel 2: Allocations %d, Interrupts %d, Busy %d, mode %d, state 0x%x\n",
                 idma_channel[2].chan_alloc, idma_channel[2].chan_int, idma_channel[2].bad_int,
                 idma_channel[2].chan_active, mvDmaStateGet(2));
  len += sprintf(buf + len, "Channel 3: Allocations %d, Interrupts %d, Busy %d, mode %d, state 0x%x\n",
                 idma_channel[3].chan_alloc, idma_channel[3].chan_int, idma_channel[3].bad_int,
                 idma_channel[3].chan_active, mvDmaStateGet(3));
 
  i     = MV_REG_READ(CPU_INT_LOW_REG(0));
  j      = MV_REG_READ(CPU_INT_MASK_LOW_REG(0));
  cause     = MV_REG_READ(CPU_INT_LOW_REG(1));
  mask      = MV_REG_READ(CPU_INT_MASK_LOW_REG(1));
  idmaCause = MV_REG_READ(IDMA_CAUSE_REG);
  idmaMask  = MV_REG_READ(IDMA_MASK_REG);
  reg = mvDmaStateGet(USERTO_DMA_CHAN);

  // We got an interrupt, but the iDMA channel is not IDLE
  len += sprintf(buf + len, "iDMA channel %d settings, state 0x%08x, cause 0x%08x [0x%08x], mask 0x%08x [0x%08x], idmaCause 0x%08x, idmaMask 0x%08x\n",
                 USERTO_DMA_CHAN, reg, cause, i, mask, j, idmaCause, idmaMask);
         
#endif

  return len;
}

#ifdef CONFIG_MV_IDMA_COPYUSER
#warning "no protection from speculative I fetch"
//probably not needed since operation is closed in spin_lock_irq 
/*=======================================================================*/
/*  Procedure:  dma_copy()                                               */
/*                                                                       */
/*  Description:    DMA-based copy_to_user.                              */
/*                                                                       */
/*  Parameters:  to: destination address                                 */
/*               from: source address                                    */
/*               n: number of bytes to transfer (n must be greater       */
/*                   equal than 64)                                     */
/*		 to_user: (1) Copy TO user (0) Copy FROM user	 	 */
/*                                                                       */
/*  Returns:     unsigned long: number of bytes NOT copied               */
/*                                                                       */
/*=======================================================================*/
static unsigned long dma_copy(void *to, const void *from, unsigned long n, unsigned int to_user)
{
  u32 chunk, i;
  u32 k_chunk = 0;
  u32 u_chunk = 0;
  u32 phys_from, phys_to;

  unsigned long flags;
  u32 unaligned_to;
  u32 index = 0;
  u32 temp;
  int chan;

  unsigned long uaddr, kaddr;
  unsigned char kaddr_kernel_static = 0;
  DPRINTK("dma_copy: entering\n");

  /*
   * The unaligned is taken care seperatly since the dst might be part of a cache line that is changed
   * by other process -> we must not invalidate this cache lines and we can't also flush it, since other
   * process (or the exception handler) might fetch the cache line before we copied it.
   */

  dma_user_cnt++;
  dma_user_size += n;
  
  /*
   * Ok, start addr is not cache line-aligned, so we need to make it so.
   */
  unaligned_to = (u32)to & 31;
  if (unaligned_to)
  {
    DPRINTK("Fixing up starting address %d bytes\n", 32 - unaligned_to);

    if (to_user)
      __arch_copy_to_user(to, from, 32 - unaligned_to);
    else
      __arch_copy_from_user(to, from, 32 - unaligned_to);

    temp = (u32)to + (32 - unaligned_to);
    to = (void *)temp;
    temp = (u32)from + (32 - unaligned_to);
    from = (void *)temp;

    /*it's ok, n supposed to be greater than 32 bytes at this point*/
    n -= (32 - unaligned_to);
  }

  /*
   * Ok, we're aligned at the top, now let's check the end
   * of the buffer and align that. After this we should have
   * a block that is a multiple of cache line size.
   */
  unaligned_to = ((u32)to + n) & 31;
  if (unaligned_to)
  {
    u32 tmp_to = (u32)to + (n - unaligned_to);
    u32 tmp_from = (u32)from + (n - unaligned_to);
    DPRINTK("Fixing ending alignment %d bytes\n", unaligned_to);

    if (to_user)
      __arch_copy_to_user((void *)tmp_to, (void *)tmp_from, unaligned_to);
    else
      __arch_copy_from_user((void *)tmp_to, (void *)tmp_from, unaligned_to);

    /*it's ok, n supposed to be greater than 32 bytes at this point*/
    n -= unaligned_to;
  }

  if (to_user)
  {
    uaddr = (unsigned long)to;
    kaddr = (unsigned long)from;
  }
  else
  {
    uaddr = (unsigned long)from;
    kaddr = (unsigned long)to;
  }
  
  if (virt_addr_valid(kaddr))
  {
    kaddr_kernel_static = 1;
    k_chunk = n;
  }
  else
  {
    DPRINTK("kernel address is not linear, fall back\n");
    goto exit_dma;
  }

  i = 0;
  while (n > 0)
  {
    // take the page table lock
    spin_lock_irqsave(&current->mm->page_table_lock, flags);
  
    if (k_chunk == 0)
    {
      /* virtual address */
      k_chunk = page_remainder((u32)kaddr);
      DPRINTK("kaddr reminder %d \n", k_chunk);
    }

    if (u_chunk == 0)
    {
      u_chunk = page_remainder((u32)uaddr);
      DPRINTK("uaddr reminder %d \n", u_chunk);
    }

    chunk = ((u_chunk < k_chunk) ? u_chunk : k_chunk);
    if (n < chunk)
    {
      chunk = n;
    }

    if (chunk == 0)
    {
      break;
    }
    
    phys_from = physical_address((u32)from, 0);
    phys_to = physical_address((u32)to, 1);
    DPRINTK("choose chunk %d \n", chunk);
    
    /*
     *  Prepare the IDMA.
     */
//    if (chunk < IDMA_MIN_COPY_CHUNK)
    if (1)
    {
      DPRINTK(" chunk %d too small , use memcpy \n", chunk);
      /* the "to" address might cross cache line boundary, so part of the line*/
      /* may be subject to DMA, so we need to wait to last DMA engine to finish */
      if (to_user)
        __arch_copy_to_user((void *)to, (void *)from, chunk);
      else
        __arch_copy_from_user((void *)to, (void *)from, chunk);
    }
    else if ((!phys_from) || (!phys_to))
    {
      /* The requested page isn't available, fall back to */
      printk(" no physical address, fall back: from %p , to %p \n", from, to);
      
      goto exit_dma;
//      goto wait_for_idmas;
    }
    else
    {
      /*
       * Ensure that the cache is clean:
       *      - from range must be cleaned
       *      - to range must be invalidated
       */
      if (phys_from < DRI_PHYS_UNCACHED_ADDR_START_LIMIT)
        dmac_flush_range(from, from + chunk);
        
      if (phys_to < DRI_PHYS_UNCACHED_ADDR_START_LIMIT)
        dmac_inv_range(to, to + chunk);
     
      // release the page table lock        
      spin_unlock_irqrestore(&current->mm->page_table_lock, flags);

      /* Start DMA */
      // find an iDMA channel to run this on
      // allocate an iDMA channel if we already haven't
      if (index == 0)
      {
        if (to_user)
        {
          chan = USERTO_DMA_CHAN;
        }
        else
        {
          chan = USERFROM_DMA_CHAN;
        }
        
        allocate_a_channel(chan, USER_COPY);      
    
        idma_channel[chan].atomic = 1;
      }

      index++;
      idma_busy++;
      
      DPRINTK(" activate DMA: channel %d from %x to %x len %x\n",
              chan, phys_from, phys_to, chunk);
        
      mvDmaTransfer(chan, phys_from, phys_to, chunk, 0);
      
      // enable the iDMA interrupt
//      MV_REG_BIT_SET(IDMA_MASK_REG, ICICR_CAUSE_OFFS(idma_channel[chan].chan_num));
      temp = MV_REG_READ(IDMA_MASK_REG);
      temp |= 0x01 << ICICR_CAUSE_OFFS(chan); 
      MV_REG_WRITE(IDMA_MASK_REG, temp);

#ifdef RT_DEBUG
      dma_activations++;
#endif
      dma_user_index++;
      
      // wait for the dma to complete
      if (wait_for_idma(chan))
      {
        BUG();
      }
    }

    /* go to next chunk */
    from += chunk;
    to += chunk;
    kaddr += chunk;
    uaddr += chunk;
    n -= chunk;
    u_chunk -= chunk;
    k_chunk -= chunk;
  }

//wait_for_idmas:
  if (index > 0)
  {
    // release the channel
    free_channel(chan);
  }

exit_dma:

  DPRINTK("dma_copy(0x%x, 0x%x, %lu): exiting\n", (u32) to,
    (u32) from, n);

  if (n != 0)
  {
    if (to_user)
      return __arch_copy_to_user((void *)to, (void *)from, n);
    else
      return __arch_copy_from_user((void *)to, (void *)from, n);
  }
  return 0;
}

/*=======================================================================*/
/*  Procedure:  dma_copy_to_user()                                       */
/*                                                                       */
/*  Description:    DMA-based copy_to_user.                              */
/*                                                                       */
/*  Parameters:  to: destination address                                 */
/*               from: source address                                    */
/*               n: number of bytes to transfer                          */
/*                                                                       */
/*  Returns:     unsigned long: number of bytes NOT copied               */
/*                                                                       */
/*  Notes/Assumptions:                                                   */
/*              Assumes that kernel physical memory is contiguous, i.e., */
/*              the physical addresses of contiguous virtual addresses   */
/*              are also contiguous.                                     */
/*              Assumes that kernel memory doesn't get paged.            */
/*              Assumes that to/from memory regions cannot overlap       */
/*                                                                       */
/*=======================================================================*/
unsigned long dma_copy_to_user(void *to, const void *from, unsigned long n)
{
//  if (!idma_init)
    return __arch_copy_to_user((void *)to, (void *)from, n);

  dma_to_user++;
  
  DPRINTK(KERN_CRIT "dma_copy_to_user(%#10x, 0x%#10x, %lu): entering\n", (u32) to, (u32) from, n);

  return dma_copy(to, from, n, 1);
}

/*=======================================================================*/
/*  Procedure:  dma_copy_from_user()                                     */
/*                                                                       */
/*  Description:    DMA-based copy_from_user.                            */
/*                                                                       */
/*  Parameters:  to: destination address                                 */
/*               from: source address                                    */
/*               n: number of bytes to transfer                          */
/*                                                                       */
/*  Returns:     unsigned long: number of bytes NOT copied               */
/*                                                                       */
/*  Notes/Assumptions:                                                   */
/*              Assumes that kernel virtual memory is contiguous, i.e.,  */
/*              the physical addresses of contiguous virtual addresses   */
/*              are also contiguous.                                     */
/*              Assumes that kernel memory doesn't get paged.            */
/*              Assumes that to/from memory regions cannot overlap       */
/*              XXX this one doesn't quite work right yet                */
/*                                                                       */
/*=======================================================================*/
unsigned long dma_copy_from_user(void *to, const void *from, unsigned long n)
{
//  if (!idma_init)
    return __arch_copy_from_user((void *)to, (void *)from, n);

  dma_from_user++;
  
  DPRINTK(KERN_CRIT "dma_copy_from_user(0x%x, 0x%x, %lu): entering\n", (u32) to, (u32) from, n);
  
  return dma_copy(to, from, n, 0);
}

#endif /* CONFIG_MV_IDMA_COPYUSER */

#ifdef CONFIG_MV_IDMA_MEMZERO
/*=======================================================================*/
/*  Procedure:  dma_memzero()                                             */
/*                                                                       */
/*  Description:    DMA-based in-kernel memzero.                          */
/*                                                                       */
/*  Parameters:  to: destination address                                 */
/*               n: number of bytes to transfer                          */
/*                                                                       */
/*  Notes/Assumptions:                                                   */
/*              Assumes that kernel physical memory is contiguous, i.e., */
/*              the physical addresses of contiguous virtual addresses   */
/*              are also contiguous.                                     */
/*              Assumes that kernel memory doesn't get paged.            */
/*              The DMA is polling                                       */
/*                                                                       */
/*=======================================================================*/
void dma_memzero(void *to, __kernel_size_t n)
{
  u32 phys_from, phys_to;
  u32 unaligned_to;
  unsigned long flags;
  int chan;
  u32 temp;

  DPRINTK("dma_memcopy: entering\n");

  /* This is used in the very early stages */
//  if (!idma_init)
    return asm_memzero(to, n);

  /* Fallback for the case that one or both buffers are not physically contiguous  */
  if (!virt_addr_valid(to))
  {
    DPRINTK("Failing back to asm_memzero because of limitations\n");
    return asm_memzero(to, n);
  }

  /*
   * If buffer start addr is not cache line-aligned, so we need to make it so.
   */
  unaligned_to = (u32)to & 31;
  if (unaligned_to)
  {
    DPRINTK("Fixing up starting address %d bytes\n", 32 - unaligned_to);

    asm_memzero(to, 32 - unaligned_to);

    to = (void*)((u32)to + (32 - unaligned_to));

    /*it's ok, n supposed to be greater than 32 bytes at this point*/
    n -= (32 - unaligned_to);
  }

  /*
   * If buffer end addr is not cache line-aligned, so we need to make it so.
   */
  unaligned_to = ((u32)to + n) & 31;
  if (unaligned_to)
  {
    u32 tmp_to = (u32)to + (n - unaligned_to);
    DPRINTK("Fixing ending alignment %d bytes\n", unaligned_to);

    asm_memzero((void *)tmp_to, unaligned_to);

    /*it's ok, n supposed to be greater than 32 bytes at this point*/
    n -= unaligned_to;
  }

  phys_from = kphysical_address((u32)dmaMemInitBuff, 0);
  phys_to   = kphysical_address((u32)to, 1);

  /*
   *  Prepare the IDMA.
   */
  if ((!phys_from) || (!phys_to))
//  if (1)
  {
    /* The requested page isn't available, fall back to */
    DPRINTK(" no physical address, fall back: to %p \n", to);
    return asm_memzero(to, n);
  }

  chan = MEMZERO_DMA_CHAN;
  if (allocate_a_channel_nowait(chan, MEM_COPY) )
  {
    DPRINTK("iDMA engines are busy, return\n");
    dma_memzero_miss++;
    return asm_memzero(to, n);
  }
    
  idma_channel[chan].atomic = 1;
      
  ++dma_memzero_cnt;

  // take the page table lock
  spin_lock_irqsave(&current->mm->page_table_lock, flags);
  
  /* Ensure that the destination revion is invalidated */
  mvOsCacheInvalidate(NULL, (void *)to, n);

  // release the page table lock
  spin_unlock_irqrestore(&current->mm->page_table_lock, flags);

  /* Start DMA */
  idma_busy++;

  DPRINTK(" activate DMA: channel %d from %x with source hold to %x len %x\n", chan, phys_from, phys_to, n);
  
  mvDmaMemInit(chan, phys_from, phys_to, n);

  // enable the iDMA interrupt
//  MV_REG_BIT_SET(IDMA_MASK_REG, ICICR_CAUSE_OFFS(idma_channel[chan].chan_num));
  temp = MV_REG_READ(IDMA_MASK_REG);
  temp |= 0x01 << ICICR_CAUSE_OFFS(chan); 
  MV_REG_WRITE(IDMA_MASK_REG, temp);

#ifdef RT_DEBUG
  dma_activations++;
#endif

  // wait for the dma to complete
  if (wait_for_idma(chan))
  {
    BUG();
  }

  // release the channel
  free_channel(chan);

  dma_unmap_single(NULL, virt_to_phys(to), n, DMA_FROM_DEVICE);

  DPRINTK("dma_memzero(0x%x, %lu): exiting\n", (u32) to, n);
}
#endif  /* CONFIG_MV_IDMA_MEMZERO */

#ifdef CONFIG_MV_IDMA_MEMCOPY
//*=======================================================================*/
/*  Procedure:  dma_memcpy()                                             */
/*                                                                       */
/*  Description:    DMA-based in-kernel memcpy.                          */
/*                                                                       */
/*  Parameters:  to: destination address                                 */
/*               from: source address                                    */
/*               n: number of bytes to transfer                          */
/*                                                                       */
/*  Returns:     void*: to                                               */
/*                                                                       */
/*  Notes/Assumptions:                                                   */
/*              Assumes that kernel physical memory is contiguous, i.e., */
/*              the physical addresses of contiguous virtual addresses   */
/*              are also contiguous.                                     */
/*              Assumes that kernel memory doesn't get paged.            */
/*              The DMA is polling                                       */
/*		source and destination buffers can overlap(like memmove) */
/*                                                                       */
/*=======================================================================*/
void *dma_memcpy(void *to, const void *from, __kernel_size_t n)
{
  u32 phys_from, phys_to;
  u32 unaligned_to;
  unsigned long flags;
  int chan;
  u32 temp;

  DPRINTK("dma_memcopy: entering\n");

  /* This is used in the very early stages */
//  if (!idma_init)
    return asm_memmove(to, from, n);

#if 1
  /* Fallback for the case that one or both buffers are not physically contiguous  */
  if (!virt_addr_valid(to) || !virt_addr_valid(from))
  {
//    printk("Failing back to asm_memmove because of limitations\n");
    return asm_memmove(to, from, n);
  }
#endif

  /* Check for Overlap */
  if (((to + n > from) && (to < from)) ||((from < to) && (from + n > to)))
  {
//    printk("overlapping copy region (0x%x, 0x%x, %lu), falling back\n",
//            to, from, (unsigned long)n);
    return asm_memmove(to, from, n);
  }

  /*
   * Ok, start addr is not cache line-aligned, so we need to make it so.
   */
  unaligned_to = (u32)to & 31;
  if (unaligned_to)
  {
    DPRINTK("Fixing up starting address %d bytes\n", 32 - unaligned_to);

    asm_memmove(to, from, 32 - unaligned_to);

    to = (void*)((u32)to + (32 - unaligned_to));
    from = (void*)((u32)from + (32 - unaligned_to));

    /*it's ok, n supposed to be greater than 32 bytes at this point*/
    n -= (32 - unaligned_to);
  }

  // take the page table lock
  spin_lock_irqsave(&current->mm->page_table_lock, flags);

  phys_from = kphysical_address((u32)from, 0);
  phys_to   = kphysical_address((u32)to, 1);

  /*
   *  Prepare the IDMA.
   */
  if ((!phys_from) || (!phys_to))
//  if (1)
  {
    /* The requested page isn't available, fall back to */
    printk(" no physical address, fall back: from %p , to %p \n", from, to);
    
    spin_unlock_irqrestore(&current->mm->page_table_lock, flags);
    
    return asm_memmove(to, from, n);
  }
  else
  {
    chan = MEMCOPY_DMA_CHAN;
    if (allocate_a_channel_nowait(chan, MEM_COPY) )
    {
      DPRINTK("iDMA engines are busy, return\n");
      dma_chan_miss++;
      return asm_memmove(to, from, n);
    }
  
//    if (dma_memcpy_cnt >= 800)
//      idma_channel[chan].atomic = 0;
//    else
      idma_channel[chan].atomic = 1;
    
    ++dma_memcpy_cnt;

    /*
     * Ensure that the cache is clean:
     *      - from range must be cleaned
     *      - to range must be invalidated
     */
    dmac_flush_range(from, from + n);
    dmac_inv_range(to, to + n);

    // release the page table lock
    spin_unlock_irqrestore(&current->mm->page_table_lock, flags);

    /* Start DMA */
    idma_busy++;

    DPRINTK(" activate DMA: channel %d from %x to %x len %x\n", chan, phys_from, phys_to, n);
    
    mvDmaTransfer(chan, phys_from, phys_to, n, 0);
    
#ifdef RT_DEBUG
    dma_activations++;
#endif
  }

  // wait for the dma to complete
  if (wait_for_idma(chan))
  {
    BUG();
  }

  // release the channel
  free_channel(chan);

  // Since we did not map anything, and we don't want anyone doing 
  // virt to phys and phys to virt and stuff, let's not do this.
  //dma_unmap_single(NULL, virt_to_phys(to), n, DMA_FROM_DEVICE);

  DPRINTK("dma_memcopy(0x%x, 0x%x, %lu): exiting\n", (u32) to, (u32) from, n);

  return 0;
}

#endif  /* CONFIG_MV_IDMA_MEMCOPY */

//*=======================================================================*/
/*  Procedure:  dri_memcpy()                                             */
/*                                                                       */
/*  Description:    DMA-based in-kernel memcpy. One of the addresses is  */
/*                  shared.                                              */
/*                                                                       */
/*  Parameters:  to: destination address                                 */
/*               from: source address                                    */
/*               n: number of bytes to transfer                          */
/*               base_virt: base virtual addr of region above highmem    */
/*               base_phys: base physical addr of region above highmem   */
/*                                                                       */
/*  Returns:     void*: to                                               */
/*                                                                       */
/*  Notes/Assumptions:                                                   */
/*              Assumes that kernel physical memory is contiguous, i.e., */
/*              the physical addresses of contiguous virtual addresses   */
/*              are also contiguous.                                     */
/*              Assumes that kernel memory doesn't get paged.            */
/*              The DMA is polling                                       */
/*		source and destination buffers can overlap(like memmove) */
/*                                                                       */
/*              One of the buffers is in an ioremap area and is thus     */
/*              above highmem. The base virt and phys is passed in so    */
/*              we can create valid physical addresses for it.           */
/*                                                                       */
/*=======================================================================*/
#define my_virt_to_phys(_virt_, _virt_base_, _phys_base_) \
  (unsigned long)((unsigned long)(_phys_base_) + \
   ((unsigned long)(_virt_) - (unsigned long)(_virt_base_)))

void *dri_memcpy(void *to, const void *from, __kernel_size_t n, 
                 void *base_virt, void *base_phys)
{
  u32 phys_from, phys_to;
  u32 unaligned_to;
  unsigned long flags;
  int chan;
  u32 temp;

  DPRINTK("dma_memcopy: entering\n");

  /* This is used in the very early stages and for copies less than 192 bytes */
  if (!idma_init || n <= 512)
    return asm_memmove(to, from, n);

#if 1
#define my_virt_addr_valid(_addr_) \
  (((unsigned long)(_addr_) >= PAGE_OFFSET) & \
   ((unsigned long)(_addr_) < 0xc0000000)) 
  /* Fallback for the case that one or both buffers are not physically contiguous  */
  //if (!virt_addr_valid(to) || !virt_addr_valid(from))
  //if (!my_virt_addr_valid(to) || !(my_virt_addr_valid(from)))
  //{
  //  printk("Failing back to asm_memmove because of virt addr problems: "
  //         "From: %p, to: %p, high_mem: 0x%0x\n", from, to, high_memory);
  //  return asm_memmove(to, from, n);
  //}
#endif

  /* We know there is no overlap */

  /*
   * Ok, start addr is not cache line-aligned, so we need to make it so.
   */
  unaligned_to = (u32)to & 31;
  if (unaligned_to)
  {
    //DPRINTK("Fixing up starting address %d bytes\n", 32 - unaligned_to);

    asm_memmove(to, from, 32 - unaligned_to);

    to = (void*)((u32)to + (32 - unaligned_to));
    from = (void*)((u32)from + (32 - unaligned_to));

    /*it's ok, n supposed to be greater than 32 bytes at this point*/
    n -= (32 - unaligned_to);
  }

  // Figure out which address is in the shared mem area by checking if
  // it is greater than the base_virt we were given ...
  if (virt_addr_valid(to))
  {
    phys_from = my_virt_to_phys((u32)from, base_virt, base_phys);
    phys_to   = virt_to_phys((u32)to);
    //printk("Starting IDMA: From phys: %p, To phys: %p, len: %d\n", phys_from, 
    //     phys_to, n);
    //printk("Virtual addresses: From: %p, To: %p, base_virt: %p, base_phys: %p\n",
    //     from, to, base_virt, base_phys);
    //printk("Intermediate: 0x%0x\n", (unsigned long)((unsigned long)from -
    //    (unsigned long)base_virt));
  }
  else
  {
    phys_from = virt_to_phys((u32)from);
    phys_to   = my_virt_to_phys((u32)to, base_virt, base_phys);
    //printk("Starting IDMA: From phys: %p, To phys: %p, len: %d\n", phys_from, 
    //     phys_to, n);
    //printk("Virtual addresses: From: %p, To: %p, base_virt: %p, base_phys: %p\n",
    //     from, to, base_virt, base_phys);
    //printk("Intermediate: 0x%0x\n", (unsigned long)((unsigned long)to -
    //    (unsigned long)base_virt));
  }

  /*
   *  Prepare the IDMA.
   */
  if ((!phys_from) || (!phys_to))
  {
    /* The requested page isn't available, fall back to */
    printk(" no physical address, fall back: from %p , to %p \n", from, to);
    
    //spin_unlock_irqrestore(&current->mm->page_table_lock, flags);
    
    return asm_memmove(to, from, n);
  }
  else
  {
    chan = allocate_pool_channel(DRI_COPY);
    if (chan < 0)
    {
      DPRINTK("iDMA engines are busy, return\n");
      dma_dri_miss++;
      return asm_memmove(to, from, n);
    }
  
    idma_channel[chan].atomic = 0;
    
    ++dma_dricpy_cnt;

    /*
     * Ensure that the cache is clean:
     *      - from range must be cleaned if in virt memory ... not shared
     *      - to range must be invalidated if in virt memory ... not shared
     */
    if (virt_addr_valid(from))
      dmac_flush_range(from, from + n);
    /* Make sure that the cache is told that there should be no entries */
    if (virt_addr_valid(to))
      dmac_inv_range(to, to + n); 

    /* Start DMA */
    atomic_inc(&idma_busy);

    mvDmaTransfer(chan, phys_from, phys_to, n, 0);
    
    // enable the iDMA interrupt
    // MV_REG_BIT_SET(IDMA_MASK_REG, ICICR_CAUSE_OFFS(idma_channel[chan].chan_num));
    temp = MV_REG_READ(IDMA_MASK_REG);
    temp |= 0x01 << ICICR_CAUSE_OFFS(chan); 
    MV_REG_WRITE(IDMA_MASK_REG, temp);

#ifdef RT_DEBUG
    dma_activations++;
#endif
  }

  // wait for the dma to complete
  if (wait_for_idma(chan))
  {
    BUG();
  }

  // release the channel
  free_pool_channel(chan);

  // We need to properly handle this page if it is cached, and virt_addr_valid
  // is correct here
  if (virt_addr_valid(to))
    dma_unmap_single(NULL, virt_to_phys(to), n, DMA_FROM_DEVICE);

  //DPRINTK("dri_memcopy(0x%x, 0x%x, %lu): exiting\n", (u32) to, (u32) from, n);

  return 0;
}

EXPORT_SYMBOL(dri_memcpy);

//*=======================================================================*/
/*  Procedure:  dri_memcpy_virt()                                        */
/*                                                                       */
/*  Description:    DMA-based in-kernel memcpy with all virt.            */
/*                                                                       */
/*  Parameters:  to: destination address                                 */
/*               from: source address                                    */
/*               n: number of bytes to transfer                          */
/*                                                                       */
/*  Returns:     void*: to                                               */
/*                                                                       */
/*  Notes/Assumptions:                                                   */
/*              Assumes that kernel physical memory is contiguous, i.e., */
/*              the physical addresses of contiguous virtual addresses   */
/*              are also contiguous.                                     */
/*              Assumes that kernel memory doesn't get paged.            */
/*              The DMA is polling                                       */
/*		source and destination buffers can overlap(like memmove) */
/*                                                                       */
/*              One of the buffers is in an ioremap area and is thus     */
/*              above highmem. The base virt and phys is passed in so    */
/*              we can create valid physical addresses for it.           */
/*                                                                       */
/*=======================================================================*/
void *dri_memcpy_virt(void *to, const void *from, __kernel_size_t n)
{
  u32 phys_from, phys_to;
  u32 unaligned_to;
  unsigned long flags;
  int chan;
  u32 temp;

  DPRINTK("dma_memcopy_virt: entering\n");

  printk(KERN_INFO "%s: to: %p, from: %p, len: %u\n", __func__, to, from, n);

  /* This is used in the very early stages and for copies less than 192 bytes */
  if (!idma_init || n <= 192)
    return asm_memmove(to, from, n);

#if 1
#define my_virt_addr_valid(_addr_) \
  (((unsigned long)(_addr_) >= PAGE_OFFSET) & \
   ((unsigned long)(_addr_) < 0xc0000000)) 
  /* Fallback for the case that one or both buffers are not physically contiguous  */
  //if (!virt_addr_valid(to) || !virt_addr_valid(from))
  //if (!my_virt_addr_valid(to) || !(my_virt_addr_valid(from)))
  //{
  //  printk("Failing back to asm_memmove because of virt addr problems: "
  //         "From: %p, to: %p, high_mem: 0x%0x\n", from, to, high_memory);
  //  return asm_memmove(to, from, n);
  //}
#endif

  /* Check for Overlap */
  if (((to + n > from) && (to < from)) ||((from < to) && (from + n > to)))
  {
    printk("overlapping copy region (0x%x, 0x%x, %lu), falling back\n",
            to, from, (unsigned long)n);
    return asm_memmove(to, from, n);
  }

  /*
   * Ok, start addr is not cache line-aligned, so we need to make it so.
   */
  unaligned_to = (u32)to & 31;
  if (unaligned_to)
  {
    //DPRINTK("Fixing up starting address %d bytes\n", 32 - unaligned_to);

    asm_memmove(to, from, 32 - unaligned_to);

    to = (void*)((u32)to + (32 - unaligned_to));
    from = (void*)((u32)from + (32 - unaligned_to));

    /*it's ok, n supposed to be greater than 32 bytes at this point*/
    n -= (32 - unaligned_to);
  }

  // take the page table lock, we are coming from a user process!
  spin_lock_irqsave(&current->mm->page_table_lock, flags);

  phys_from = virt_to_phys((u32)from);
  phys_to   = virt_to_phys((u32)to);

  /*
   *  Prepare the IDMA.
   */
  if ((!phys_from) || (!phys_to))
//  if (1)
  {
    /* The requested page isn't available, fall back to */
    printk(" no physical address, fall back: from %p , to %p \n", from, to);
    
    //spin_unlock_irqrestore(&current->mm->page_table_lock, flags);
    
    return asm_memmove(to, from, n);
  }
  else
  {
    chan = allocate_pool_channel(DRI_COPY);
    if (chan < 0)
    {
      DPRINTK("iDMA engines are busy, return\n");
      dma_dri_miss++;
      return asm_memmove(to, from, n);
    }
  
    idma_channel[chan].atomic = 0;
    
    ++dma_dricpy_cnt;

    /*
     * Ensure that the cache is clean:
     *      - from range must be cleaned if in virt memory ... not shared
     *      - to range must be invalidated if in virt memory ... not shared
     */
    dmac_flush_range(from, from + n);
    /* Make sure that the cache is told that there should be no entries */
    dmac_inv_range(to, to + n); 

    /* Start DMA */
    idma_busy++;

  //  DPRINTK(" activate DMA: channel %d from %x to %x len %x\n", chan, phys_from, phys_to, n);
    
    mvDmaTransfer(chan, phys_from, phys_to, n, 0);
    
    // enable the iDMA interrupt
//    MV_REG_BIT_SET(IDMA_MASK_REG, ICICR_CAUSE_OFFS(idma_channel[chan].chan_num));
    temp = MV_REG_READ(IDMA_MASK_REG);
    temp |= 0x01 << ICICR_CAUSE_OFFS(chan); 
    MV_REG_WRITE(IDMA_MASK_REG, temp);

#ifdef RT_DEBUG
    dma_activations++;
#endif
  }

  // wait for the dma to complete
  if (wait_for_idma(chan))
  {
    BUG();
  }

  // release the channel, although the ISR actually frees the channel
  free_pool_channel(chan);

  // We need to properly handle this page if it is cached, and virt_addr_valid
  // is correct here
  if (virt_addr_valid(to))
    dma_unmap_single(NULL, virt_to_phys(to), n, DMA_FROM_DEVICE);

  //DPRINTK("dri_memcopy(0x%x, 0x%x, %lu): exiting\n", (u32) to, (u32) from, n);

  return 0;
}

EXPORT_SYMBOL(dri_memcpy_virt);

#ifdef CONFIG_ENABLE_IDMA_INTERRUPTS
static irqreturn_t
mv_idma_isr(int irq, void *dev_id)
{
  MV_U32 reg, cause, mask, idmaCause, idmaMask;
  int    chan = (int) dev_id;

  // check if this channel is being used
  reg = mvDmaStateGet(chan);
  DPRINTK("%s: unit 0: cause 0x%08x, chan %d\n", __func__, reg, (int)dev_id);
  
  if (idma_channel[chan].chan_active != IDLE)
  {
    idma_channel[chan].chan_int++;
  
    // Clear the interrupt cause registers
    // i     = MV_REG_READ(CPU_INT_LOW_REG(0));
    // j      = MV_REG_READ(CPU_INT_MASK_LOW_REG(0));
    cause     = MV_REG_READ(CPU_INT_LOW_REG(1));
    mask      = MV_REG_READ(CPU_INT_MASK_LOW_REG(1));
    idmaCause = MV_REG_READ(IDMA_CAUSE_REG);
    idmaMask  = MV_REG_READ(IDMA_MASK_REG);

    if (reg == MV_IDLE)
    {
      // clear the iDMA enable
      MV_REG_BIT_RESET(IDMA_CTRL_LOW_REG(chan), ICCLR_CHAN_ENABLE);
  
      // update the channel state
      idma_channel[chan].chan_active = IDLE;
    
      // signal the waiting thread
      if (idma_channel[chan].atomic == 0)
      {
        // We got an interrupt
        if (cause & 0x00030000)  // An error ...
        printk("%s: iDMA channel %d interrupt, state 0x%08x, cause 0x%08x mask 0x%08x idmaCause 0x%08x, idmaMask 0x%08x\n",
               __func__, chan, reg, cause, mask, idmaCause, idmaMask);
          
        wake_up_interruptible(&idma_channel[chan].waitq);
      }

      idma_busy--;
    }
  }
  else if (idma_channel[chan].atomic != 0)
  {
    // Clear the interrupt cause registers
    cause     = MV_REG_READ(CPU_INT_LOW_REG(1));
    mask      = MV_REG_READ(CPU_INT_MASK_LOW_REG(1));
    idmaCause = MV_REG_READ(IDMA_CAUSE_REG);
    idmaMask  = MV_REG_READ(IDMA_MASK_REG);

    // We got an interrupt, but the iDMA channel is not IDLE
    printk("%s error: iDMA channel %d is IDLE, state 0x%08x, cause 0x%08x, mask 0x%08x, idmaCause 0x%08x, idmaMask 0x%08x\n",
           __func__, chan, reg, cause, mask, idmaCause, idmaMask);
          
    idma_channel[chan].bad_int++;
  }

  // clear the interrupts
  // cause = MV_REG_READ(CPU_INT_LOW_REG(1));
  // cause &= ~(1 << idma_channel[chan].chan_int);
  // MV_REG_BIT_RESET(CPU_INT_LOW_REG(0), idma_channel[chan].irq_num);
  MV_REG_BIT_RESET(CPU_INT_LOW_REG(1), idma_channel[chan].irq_num);
  MV_REG_BIT_SET(CPU_INT_MASK_LOW_REG(1), idma_channel[chan].irq_num);
  
//  idmaCause = MV_REG_READ(IDMA_CAUSE_REG);
  idmaCause = ~(0x1F << ICICR_CAUSE_OFFS(chan));
  MV_REG_WRITE(IDMA_CAUSE_REG, idmaCause);

  dma_interrupts++;
  
  // check that the drive is still in sync
  if (idma_busy < 0)
  {
    printk("%s error: channel busy cound went negative.\n", __func__);
  }
  
  return IRQ_HANDLED;
}
#endif

int mv_dma_init(void)
{
  int chan;
  MV_U32 ctrlWord;
#ifdef CONFIG_ENABLE_IDMA_INTERRUPTS
  int err = 0;
  int temp;
  MV_U32 reg, cause, mask, idmaCause, idmaMask, i, j;
#endif

#ifdef CONFIG_MV78200
  if (MV_FALSE == mvIsUnitMappedToThisCpu(IDMA))
  {
    printk("IDMA is not mapped to this CPU\n");
    return -ENODEV;
  }
#endif
  printk(KERN_INFO "Use IDMA channels for enhancing the following function:\n");
#ifdef CONFIG_MV_IDMA_COPYUSER
  printk(KERN_INFO "  o Copy From/To user space operations.\n");
#endif
#ifdef CONFIG_MV_IDMA_MEMCOPY
  printk(KERN_INFO "  o memcpy() and memmove() operations.\n");
#endif
#ifdef CONFIG_MV_IDMA_MEMZERO
  printk(KERN_INFO "  o memzero() operations.\n");
#endif

#ifdef CONFIG_MV_IDMA_MEMZERO
  DPRINTK(KERN_ERR "ZERO buffer address 0x%08x\n", (u32)dmaMemInitBuff);

  asm_memzero(dmaMemInitBuff, sizeof(dmaMemInitBuff));
  dmac_flush_range(dmaMemInitBuff, dmaMemInitBuff + sizeof(dmaMemInitBuff));
#endif

  sema_init(&alloc_sema, 1);  /* Init the allocation semaphore */

  // setup the iDMA channels
  for (chan = 0; chan < MV_IDMA_MAX_CHAN; chan++)
  {
    // initialize the channel hardware
    ctrlWord = ICCLR_DST_BURST_LIM_32BYTE |
               ICCLR_SRC_INC |
               ICCLR_DST_INC |
               ICCLR_SRC_BURST_LIM_32BYTE |
               ICCLR_BLOCK_MODE |
               ICCLR_CHANNEL_ABORT |
               ICCLR_NON_CHAIN_MODE;

    MV_REG_WRITE(IDMA_CTRL_LOW_REG(chan), ctrlWord);

    MV_REG_WRITE(IDMA_BYTE_COUNT_REG(chan), 0);
    MV_REG_WRITE(IDMA_CURR_DESC_PTR_REG(chan), 0);
    MV_REG_WRITE(IDMA_CTRL_HIGH_REG(chan), ICCHR_ENDIAN_LITTLE
#ifdef MV_CPU_LE
                 | ICCHR_DESC_BYTE_SWAP_EN
#endif
                 );
                
    // initialize the channel management info
    idma_channel[chan].chan_num    = chan;
    idma_channel[chan].pDescriptor = NULL;
    idma_channel[chan].descPhyAddr = NULL;
    idma_channel[chan].chan_active = IDLE;
    idma_channel[chan].irq_num     = IRQ_IDMA(chan);
    idma_channel[chan].chan_alloc  = 0;
    idma_channel[chan].chan_int    = 0;
    idma_channel[chan].bad_int     = 0;

    sema_init(&idma_channel[chan].sema, 1);
//    sema_init(&idma_channel[chan].waitSema, 0);
    init_waitqueue_head(&idma_channel[chan].waitq);

#ifdef CONFIG_ENABLE_IDMA_INTERRUPTS
    MV_REG_WRITE( IDMA_CAUSE_REG, 0);

    switch (chan)
    {
      case 0:
        idma_channel[chan].name = "idma_chan0";
        break;
        
      case 1:
        idma_channel[chan].name = "idma_chan1";
        break;
        
      case 2:
        idma_channel[chan].name = "idma_chan2";
        break;
        
      case 3:
        idma_channel[chan].name = "idma_chan3";
        break;
        
      default:
        printk(KERN_ERR "%s: trying to configure bad idma channel\n", __func__);
        return -ENXIO;
    }
    
    err = request_irq(idma_channel[chan].irq_num, mv_idma_isr, IRQF_DISABLED,
                      idma_channel[chan].name, (void *)chan);
    if (err < 0)
    {
      printk(KERN_ERR "%s: unable to request IRQ %d for "
             "iDMA %d: %d\n", __func__, idma_channel[chan].irq_num, chan, err);
      return -EBUSY;
    }
    else
    {
      printk(KERN_INFO "Enable iDMA %s interrupt IRQ %d for iDMA %d\n", idma_channel[chan].name,
             idma_channel[chan].irq_num, chan);
            
      // unmask the interrupt
//      MV_REG_BIT_RESET(CPU_INT_MASK_LOW_REG(0), idma_channel[chan].irq_num);
//      MV_REG_BIT_SET(CPU_INT_MASK_LOW_REG(1), idma_channel[chan].irq_num);
      
      // enable the channel interrupt
      MV_REG_BIT_SET(IDMA_MASK_REG, ICICR_CAUSE_OFFS(idma_channel[chan].chan_num));
//      temp = MV_REG_READ(IDMA_MASK_REG);
//      temp |= 0x01 << ICICR_CAUSE_OFFS(chan); 
//      MV_REG_WRITE(IDMA_MASK_REG, temp);
      
      // clear the interrupt cause reg
//      MV_REG_BIT_RESET(CPU_INT_LOW_REG(0), idma_channel[chan].irq_num);
//      MV_REG_BIT_RESET(CPU_INT_LOW_REG(1), idma_channel[chan].irq_num);
    }

#endif
  }
  
  // clear the interrupt cause reg
//  MV_REG_WRITE(IDMA_CAUSE_REG, 0x00000000);
  
  i     = MV_REG_READ(CPU_INT_LOW_REG(0));
  j      = MV_REG_READ(CPU_INT_MASK_LOW_REG(0));
  cause     = MV_REG_READ(CPU_INT_LOW_REG(1));
  mask      = MV_REG_READ(CPU_INT_MASK_LOW_REG(1));
  idmaCause = MV_REG_READ(IDMA_CAUSE_REG);
  idmaMask  = MV_REG_READ(IDMA_MASK_REG);
  reg = mvDmaStateGet(USERTO_DMA_CHAN);

  // We got an interrupt, but the iDMA channel is not IDLE
  printk("%s: iDMA channel %d settings, state 0x%08x, cause 0x%08x [0x%08x], mask 0x%08x [0x%08x], idmaCause 0x%08x, idmaMask 0x%08x\n",
         __func__, USERTO_DMA_CHAN, reg, cause, i, mask, j, idmaCause, idmaMask);
        
  // create a proc entry
  dma_proc_entry = create_proc_entry("mv_idma", S_IFREG | S_IRUGO, 0);
  dma_proc_entry->read_proc = dma_read_proc;
  dma_proc_entry->write_proc = NULL;  // dma_write_proc;
  dma_proc_entry->nlink = 1;

  idma_init = 1;

  return 0;
}

void mv_dma_exit(void)
{
}

module_init(mv_dma_init);
module_exit(mv_dma_exit);
MODULE_LICENSE(GPL);

#ifdef CONFIG_MV_IDMA_MEMCOPY
EXPORT_SYMBOL(dma_memcpy);
#endif

#ifdef CONFIG_MV_IDMA_MEMZERO
EXPORT_SYMBOL(dma_memzero);
#endif

#ifdef CONFIG_MV_IDMA_COPYUSER
EXPORT_SYMBOL(dma_copy_to_user);
EXPORT_SYMBOL(dma_copy_from_user);			
#endif
