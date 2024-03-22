/****************************************************************************
 * Copyright (C) 2005 - Benko Attila
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA 02111-1307, USA.
 * 
 */
// RCTRAN /proc/rctran and joystick emulation interface
// Purpose  : use your radio control transmitter as pc joystick
// Author   : Benko" Attila
// Thanks to: Jan Edward Kansky for original rctran code
//            Martin Dressler <dr@musicabona.com> for original rc joystick code
/****************************************************************************/

// 0: no joystick device emulation
// 1: joystick device emulation
#define RCTRAN_JOYDEV 1

#include <linux/module.h>
#include <linux/kernel.h>

#ifdef CONFIG_MODVERSION
# if CONFIG_MODVERSION==1
#  define MODVERSIONS
#  include <linux/modversions.h>
# endif
#endif

#include <linux/input.h>
#include <asm/io.h>
#include <linux/proc_fs.h>
#include <asm/delay.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/version.h>


/****************************************************************************/
#define LPT_BASE iobase         // Use 0x3bc-LPT0, 0x378-LPT1, 0x278-LPT2
#define IRQ_LINE irq            // Parallel port IRQ line.
#define DATA_PORT iobase+0      // Address of data register for bits D0-D7
#define STATUS_PORT iobase+1    // Address of LPT status register
#define CONTROL_PORT iobase+2   // Address of LPT control register
#define MAX_CHANNELS 8
#define SYNC_PULSE 3000


MODULE_DESCRIPTION("RCTRAN driver for Linux");
MODULE_LICENSE("GPL");

static int iobase = 0x378;
module_param(iobase, int, 0);
MODULE_PARM_DESC(iobase, "Parallel port base address (default=0x378, lp0)");

static int irq = 7;
module_param(irq, int, 0);
MODULE_PARM_DESC(irq, "Parallel port IRQ number (default=7, lp0)");

/****************************************************************************/
#if (RCTRAN_JOYDEV != 0)
static struct input_dev rctran;
#endif
static unsigned long long cycles, last = 0, diff;
static unsigned long pulses[8] = {0, 0, 0, 0, 0, 0, 0, 0};
static int current_channel=1, dev_used = 0;
/****************************************************************************/

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
static irqreturn_t handle_transmitter_pulse(int             irq_number,
                                            void*           dev_id,
                                            struct pt_regs* regs)
#else
static void handle_transmitter_pulse(int             irq_number,
                                     void*           dev_id,
                                     struct pt_regs* regs)
#endif
{
    unsigned long i;
    struct timeval tv;
    
    if (irq_number == IRQ_LINE) {
	do_gettimeofday(&tv);
	i = 1e6;
	cycles = i * tv.tv_sec + tv.tv_usec;
	diff = cycles - last;
	last = cycles;
	if (diff > SYNC_PULSE) current_channel=0;
	if ((current_channel > 0) && (current_channel < 9)) {
	    pulses[current_channel - 1] = diff;
#if (RCTRAN_JOYDEV != 0)
	    if (dev_used) input_report_abs(&rctran, current_channel - 1, diff);
#endif
	}
	current_channel++;
    }
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    return IRQ_HANDLED;
#endif
}

/****************************************************************************/

int procfile_read(char *sysbuffer, char **mybuffer, off_t file_pos,
                  int count, int *eof, void *data)
{
  static char the_string[200];
  int len;

  if (file_pos > 0) return 0;
  len = sprintf(the_string, "%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n",
          pulses[0],pulses[1],pulses[2],pulses[3],
	  pulses[4],pulses[5],pulses[6],pulses[7]);
  *mybuffer = the_string;
  return len;
}


#if (RCTRAN_JOYDEV != 0)

/****************************************************************************/

static int rctran_open(struct input_dev *dev)
{
    if (dev_used++) return 0;
    printk(KERN_INFO "RCTRAN jostick emulation module opened.\n");
    return 0;
}

/****************************************************************************/
static void rctran_close(struct input_dev *dev)
{
    if (!--dev_used) printk(KERN_INFO "RCTRAN joystick emulation module closed.\n");
}

#endif

/****************************************************************************/

static int __init rctran_init(void)
{
    int i, res;
    struct proc_dir_entry *entry=NULL;

#if (RCTRAN_JOYDEV != 0)
    rctran.open = rctran_open;
    rctran.close = rctran_close;
    rctran.name="RCTRAN";
    rctran.evbit[0] = BIT(EV_KEY) | BIT(EV_ABS);
    set_bit(BTN_0, rctran.keybit);
    set_bit(BTN_1, rctran.keybit);
    for(i=0;i<MAX_CHANNELS;i++) {
	set_bit(i, rctran.absbit);
	rctran.absmin[i]  = 1000;
	rctran.absmax[i]  = 2000;
	rctran.absfuzz[i] = 15;
	rctran.absflat[i] = 8;
    }
    input_register_device(&rctran);
#endif
  
    printk(KERN_INFO "RCTRAN Module loaded, iobase=0x%x, irq=%d\n", iobase, irq);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
    printk(KERN_INFO "RCTRAN Device number: %d\n",rctran.number);
#endif
    res=request_irq(IRQ_LINE,handle_transmitter_pulse,SA_INTERRUPT,"rctran",NULL);
    if (res != 0)
    {
	dev_used--;
	printk(KERN_ERR "Request_irq for rctran use failed.\n");
	if (res == -EINVAL) printk(KERN_ERR "The IRQ number you requested was either invalid or reserved.\n");
	else if (res == -ENOMEM) printk(KERN_ERR "request_irq could not allocate memory for the new interrupt.\n");
	else if (res == -EBUSY) printk(KERN_ERR "The IRQ is already being handled and can not be shared.\n");
    }
    outb(inb(CONTROL_PORT) | 0x10,CONTROL_PORT);  // Enable Parallel Port IRQ

    if (inb(CONTROL_PORT) & 0x10) printk(KERN_INFO "Parallel port interrupt is enabled.\n");
    else printk(KERN_INFO "Parallel port interrupt is disabled.\n");
  
    // Provide supply for external circuit
    outb(0xFF, DATA_PORT);
    
    entry = create_proc_entry("rctran", S_IFREG|S_IRUGO, &proc_root);
    if (!entry) {
	printk(KERN_ERR "unable to create /proc/rctran entry\n");
	return -EIO;
    } else entry->read_proc = procfile_read;

    return 0;
}

/****************************************************************************/
static void __exit rctran_exit(void)
{
#if (RCTRAN_JOYDEV != 0)
    input_unregister_device(&rctran);
#endif
    free_irq(IRQ_LINE,NULL);
    outb(inb(CONTROL_PORT)& 0xEF,CONTROL_PORT);  // Disable Parallel Port IRQ
    if (inb(CONTROL_PORT) & 0x10) printk(KERN_INFO "Parallel port interrupt is enabled.\n");
    else printk(KERN_INFO "Parallel port interrupt is disabled.\n");
    remove_proc_entry("rctran", &proc_root);
    printk(KERN_INFO "RCTRAN module unloaded!\n");\
}
/****************************************************************************/

module_init(rctran_init);
module_exit(rctran_exit);
