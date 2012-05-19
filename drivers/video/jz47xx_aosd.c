#include <linux/time.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/sched.h>
#include <asm/time.h>
#include <asm/jzsoc.h>
#include "jz47xx_aosd.h"

#define IRQ_AOSD 43
#define COMP_RATIO_TEST 0

extern struct jz47xx_aosd_info *aosd_info;
extern wait_queue_head_t compress_wq;
/*static cycle_t last_cycle;*/
/*static unsigned long cur_cycle;*/
/*static unsigned int freq;*/

extern cycle_t jz_get_cycles(struct clocksource *cs);

void print_aosd_registers(void)	/* debug */
{
	/* LCD Controller Resgisters */
	printk("REG_AOSD_ADDR0:\t0x%08x\n", REG_AOSD_ADDR0);
	printk("REG_AOSD_ADDR1:\t0x%08x\n", REG_AOSD_ADDR1);
	printk("REG_AOSD_ADDR2:\t0x%08x\n", REG_AOSD_ADDR2);
	printk("REG_AOSD_ADDR3:\t0x%08x\n", REG_AOSD_ADDR3);
	printk("REG_AOSD_WADDR:\t0x%08x\n", REG_AOSD_WADDR);
	printk("REG_AOSD_ADDRLEN:\t0x%08x\n", REG_AOSD_ADDRLEN);
	printk("REG_AOSD_ALPHA_VALUE:\t0x%08x\n", REG_AOSD_ALPHA_VALUE);
	printk("REG_AOSD_CTRL:\t0x%08x\n",REG_AOSD_CTRL);
	printk("REG_AOSD_INT:\t0x%08x\n", REG_AOSD_INT);

	printk("REG_COMPRESS_DST_OFFSET:\t0x%08x\n", REG_COMPRESS_DST_OFFSET);
	printk("REG_COMPRESS_FRAME_SIZE:\t0x%08x\n", REG_COMPRESS_FRAME_SIZE);
	printk("REG_COMPRESS_CTRL:\t0x%08x\n", REG_COMPRESS_CTRL);
	printk("REG_COMPRESS_RATIO:\t0x%08x\n", REG_COMPRESS_RATIO);
	printk("REG_COMPRESS_SRC_OFFSET:\t0x%08x\n", REG_COMPRESS_SRC_OFFSET);
	printk("==================================\n");

}

/* compute the compress ratio */
extern unsigned char *buf_comp, *buf_comp1; //buffer for compress output
void calc_comp_ratio(int dy, unsigned int height, unsigned int width, unsigned int frame_size)
{
	unsigned char *tmpptr;
	unsigned int startline[1000];
	unsigned int sum = 0;
	int i;

	if (dy)
		tmpptr = buf_comp;
	else
		tmpptr = buf_comp1;

	for(i=0; i<height; i++) {
		startline[i] = tmpptr[i * frame_size];
//		printk("%x\t", startline[i]);
		sum += (startline[i] & 0xfff000) >> 12;;
//		if (!(i%6)) printk("\n");
	}
//	printk("\n");
	sum = (sum*100)/height;
//	printk("the average words of compressed line: %d\n", sum);
	sum = sum/width;
	printk("compression ratio: %d percent\n", sum);

	return;
}

static irqreturn_t jz47xx_aosd_interrupt_handler(int irq, void *dev_id)
{
	unsigned int state;
//	int cnt = 0;
//	unsigned long irq_flags;

	state = REG_AOSD_INT;

	if (state & AOSD_INT_COMPRESS_END) {
		aosd_info->compress_done = 1;
		REG_AOSD_INT = state & AOSD_INT_COMPRESS_END; //write 1 to clear INT flag
	}

	if (state & AOSD_INT_AOSD_END) {
		printk("state & AOSD_INT_AOSD_END\n");

		REG_AOSD_INT = state & AOSD_INT_AOSD_END;
//		REG_AOSD_ADDR0 = virt_to_phys((void *)addr0); 
/* 		REG_AOSD_ADDR1 = virt_to_phys((void *)addr1);  */
/* 		REG_AOSD_ADDR2 = virt_to_phys((void *)addr2);  */
/* 		REG_AOSD_ADDR3 = virt_to_phys((void *)addr3);  */
//		aosd_info->compress_done = 1;
	}
	wake_up(&compress_wq);

	return IRQ_HANDLED;
}

void jz47xx_start_compress(void)
{
//	print_aosd_registers();
#ifdef CALC_COMP_RATIO
	cur_cycle = (jz_get_cycles(0)-last_cycle);
	freq = 750000/cur_cycle;// how many times during 1s.
	cur_cycle /= 750;	// how many ms since last update
	
	printk("%lums passed, frequency = %ufps\n", cur_cycle, freq);
	last_cycle = jz_get_cycles(0);
#endif
	while(!(REG_COMPRESS_CTRL & COMPRESS_CTRL_COMP_END)) {
		printk("%s %s waiting COMPRESS_CTRL_COMP_END\n", __FILE__, __FUNCTION__);
		msleep(10);
	}
	aosd_info->compress_done = 0;
	__compress_start();
}

void jz47xx_compress_set_mode(struct jz47xx_aosd_info *info)
{
	unsigned int reg_ctrl;
	unsigned int width, height, src_stride, dst_stride;
	unsigned int ALIGNED;
	unsigned int with_alpha;

//	printk("<< aosd: addr0:%x, waddr:%x, width:%d, length:%d, with_alpha:%d >>\n", info->addr0, info->waddr, info->width, info->height, info->with_alpha);
	reg_ctrl = 0;
	width = info->width;
	height= info->height ;
	src_stride= info->src_stride ;
	dst_stride= info->dst_stride ;

	//with_alpha = info->with_alpha;

	/* if bpp=16, width/=2) */
	if (info->bpp == 16) {
		with_alpha = 1;
		width /= 2;
	}
	else if (info->bpp == 24) {
		with_alpha = 0;
	}
	else {						/* 32bpp */
		with_alpha = 1;
	}

/* 	if (info->aligned_64) { */
/* 		ALIGNED = (64*4); /\* 64Words *\/ */
/* 		reg_ctrl |= COMPRESS_CTRL_ALIGNED_64_WORD; */
/* 	} */
/* 	else  */
	{
		ALIGNED = (16*4); /* 16Words */
		reg_ctrl |= COMPRESS_CTRL_ALIGNED_16_WORD;
	}

	if ( dst_stride & (ALIGNED-1) 
		 || src_stride & (ALIGNED-1) 
		) {
		printk("Warning: AOSD Compress stride not %d aligned. src_stride=%d, dst_stride=%d\n",
			   ALIGNED, src_stride, dst_stride);
		dst_stride &= ~(ALIGNED-1);
		src_stride &= ~(ALIGNED-1);
	}

	/*SET SCR AND DES ADDR*/
	REG_AOSD_ADDR0 = info->addr0;
	REG_COMPRESS_DES_ADDR = info->waddr;

	/*SET DST OFFSIZE */
	REG_COMPRESS_DST_OFFSET = dst_stride;
	/*SET SCR OFFSET*/
	REG_COMPRESS_SRC_OFFSET = src_stride;
	/*SET SIZE*/
	// frame's actual size, no need to align
	REG_COMPRESS_FRAME_SIZE = (width & 0xffff) | ((height & 0xffff) << 16);		

	if(with_alpha)
		reg_ctrl |= COMPRESS_CTRL_WITH_ALPHA;
	else
		reg_ctrl |= COMPRESS_CTRL_WITHOUT_ALPHA;
	reg_ctrl |= COMPRESS_CTRL_INT_MASK | COMPRESS_CTRL_COMP_ENABLE;
	/* SET CTRL*/	
	REG_COMPRESS_CTRL = reg_ctrl;
}

int jz47xx_compress_init(void) 
{
	init_waitqueue_head(&compress_wq);

	// enable aosd cpm
//	*((volatile u32*)0xB0000028) &= ~(1 << 10);
	REG_CPM_CLKGR1 &= ~CLKGR1_OSD;
	msleep(10);

	if (request_irq(IRQ_AOSD, jz47xx_aosd_interrupt_handler, IRQF_DISABLED,"oasd_compress", 0)) {
		printk("Faield to request ALPHA OSD IRQ %d.\n", IRQ_AOSD);
		return -EBUSY;
	}

	REG_COMPRESS_CTRL = (COMPRESS_CTRL_INT_MASK | COMPRESS_CTRL_COMP_ENABLE); /* enable compress and enable finished interrupt */

//	cur_cycle = last_cycle = jz_get_cycles(0);
	return 0;
}
