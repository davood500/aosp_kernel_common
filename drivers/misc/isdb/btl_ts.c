/*
 * drivers/misc/isdb/btl_ts.c
 *
 * MPEG2-TS interface driver for the Ingenic JZ4770.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/kthread.h>
#include <linux/freezer.h>

#include <asm/uaccess.h>

#include "btl_interface.h"
#include "btl_ts.h"

#define 	USE_DMA

#define TSSI_IRQ   IRQ_TSSI

#define DMA_ID_TSSI 	2 //5	

#define RING_BUF_NUM  	20

static struct jz_tssi_t jz_tssi_g;
static struct jz_tssi_buf_ring_t jz_tssi_ring_g;

static unsigned int TSSI_Count;
static unsigned int DMA_Count;
static unsigned int DMA_RealCount;
static void BBDC_DMAStart(void);
static void BBDC_TSSIDisable(void);
static void BBDC_TSSIEnable(void);

static void print_reg( void )
{
	printk("REG_TSSI_ENA   %02x \n",    BTL_RREG8( TSSI_ENA ));
	printk("REG_TSSI_CFG   %04x \n",    BTL_RREG16( TSSI_CFG ));
	printk("REG_TSSI_CTRL  %02x \n",    BTL_RREG8( TSSI_CTRL ));
	printk("REG_TSSI_STAT  %02x \n",    BTL_RREG8( TSSI_STAT ));
	printk("REG_TSSI_FIFO  %08x \n",    BTL_RREG32( TSSI_FIFO ));
	printk("REG_TSSI_PEN   %08x \n",    BTL_RREG32( TSSI_PEN ));
	/*
	printk("\nREG_GPIO_PXFUN(1)   %08x \n",    REG_GPIO_PXFUN(1));
	printk("REG_GPIO_PXSEL(1)   %08x \n",    REG_GPIO_PXSEL(1));
	printk("REG_GPIO_PXTRG(1)   %08x \n",    REG_GPIO_PXTRG(1));	
	*/
}

void dump_dma_channel(unsigned int dmanr)
{
	printk("DMA%d Registers:\n", dmanr);
	printk("  DMAC0  = 0x%8x\n", REG_DMAC_DMACR(0));
	printk("  DMAC1  = 0x%8x\n", REG_DMAC_DMACR(1));
	printk("  DSAR   = 0x%8x\n", REG_DMAC_DSAR(dmanr));
	printk("  DTAR   = 0x%8x\n", REG_DMAC_DTAR(dmanr));
	printk("  DTCR   = 0x%8x\n", REG_DMAC_DTCR(dmanr));
	printk("  DRSR   = 0x%8x\n", REG_DMAC_DRSR(dmanr));
	printk("  DCCSR  = 0x%8x\n", REG_DMAC_DCCSR(dmanr));
	printk("  DCMD   = 0x%8x\n", REG_DMAC_DCMD(dmanr));
	printk("  DDA    = 0x%8x\n", REG_DMAC_DDA(dmanr));
	printk("  DMADBR = 0x%8x\n", REG_DMAC_DMADBR(1));
	printk("  DMAC_DMACKE = 0x%8x\n", REG_DMAC_DMACKE(0));
	printk("  DMAC_DMACKES = 0x%8x\n", REG_DMAC_DMACKES(0));
}

void gTSSIRegPrint(void)
{
	struct jz_tssi_t *tssi = &jz_tssi_g;

	print_reg();
	dump_dma_channel(tssi->dma_chan);
}

static int tssi_buf_init( struct jz_tssi_buf_ring_t * ring ) 	
{
	int i;
	struct tssi_buf *bp, *ap, *cp;

	ap = cp = bp = (struct tssi_buf *)kmalloc(sizeof(struct tssi_buf ),
						GFP_KERNEL );  //the first
	if ( !bp ) { 
		printk("Can not malloc buffer! \n");
		return -1;
	}

	for ( i = 0; i < RING_BUF_NUM; i++ ) {
		bp = ap;
		bp->buf = (unsigned int *) kmalloc(TSSI_PACK_SIZE / 4 *
				sizeof(unsigned int), GFP_KERNEL);
		if ( !bp->buf ) {        
			printk("Can not malloc buffer! \n");
			return -1;
		}

		memset(bp->buf, 0 , TSSI_PACK_SIZE / 4 *sizeof(unsigned int));

		bp->index = i;
		bp->pos = 0;
		ap = (struct tssi_buf *)kmalloc( sizeof( struct tssi_buf ),
						GFP_KERNEL );
		if ( !ap ) { 
			printk("Can not malloc buffer!\n");
			return -1;
		}

		bp->next = ap;      //point to next!
	}

	bp->next = cp;                  //point loop to first!
	ring->front = cp;
	ring->rear  = cp;
	ring->fu_num = 0;
	kfree(ap); 			/* free the last ap */
	return 0;
}

static int tssi_buf_clear( struct jz_tssi_buf_ring_t * ring ) 	
{
	int i;

	for ( i = 0; i < RING_BUF_NUM; i++ ) {
		ring->front->pos = 0;
		ring->front = ring->front->next;
	}

	ring->fu_num = 0;

	return 0;
}

static void tssi_free_buf( struct jz_tssi_buf_ring_t * ring )
{
	int i;
	struct tssi_buf * ap;
	for ( i = 0; i < RING_BUF_NUM; i++ ) {
		ap = ring->front;
		ring->front = ring->front->next;
		kfree( ap );
	}
}

/**read data from TSSI FIFO
 */
static void tssi_read_fifo(void *dev_id)
{
	struct 	jz_tssi_t* tssi = ( struct jz_tssi_t* )dev_id;
	struct 	jz_tssi_buf_ring_t * ring = tssi->cur_buf;

	if ( ring->fu_num > RING_BUF_NUM )
	{
		printk("Ring buffer full ! %d\n", ring->fu_num);
		return;
	}

	while(ring->front->pos < TSSI_PACK_SIZE/4)
	{
		if (BTL_RREG8(0xB0073018) == 0x0)
			break;
		ring->front->buf[ring->front->pos++] = REG_TSSI_FIFO;
	}
	if ( ring->front->pos >= TSSI_PACK_SIZE/4 ) {
		mutex_lock(&tssi->buf_lock);
		ring->fu_num++;
		ring->front = ring->front->next;
		ring->front->pos = 0;
		mutex_unlock(&tssi->buf_lock);
	}
}

static void tssi_config_filting( void )
{

	__gpio_as_tssi();
	__tssi_disable_ovrn_irq();   	//TSCTRL use dma ,no need irq
	__tssi_disable_trig_irq();	
	BTL_WREG8(TSSI_ENA , 0x00);
        //Trigger : , not add 0 , word order, Serial, data7 , clk_fast
	BTL_WREG16(TSSI_CFG , 0x068E);	
        //TSENA
	__tssi_filter_disable();
	__tssi_dma_enable();
	__tssi_clear_state();		    //TSSTAT
	BTL_WREG8(TSSI_CTRL , 0x07);

}

static void tssi_dma_start(unsigned int dma_chan, unsigned int dest_addr)
{
	
	REG_DMAC_DSAR(dma_chan) = CPHYSADDR(TSSI_FIFO);	/*DSAn SRC addr */
	REG_DMAC_DTAR(dma_chan) = CPHYSADDR(dest_addr);
	REG_DMAC_DTCR(dma_chan) = TSSI_PACK_SIZE/16; //DTCn transfer count  
	REG_DMAC_DCCSR(dma_chan) = DMAC_DCCSR_EN | DMAC_DCCSR_NDES;  /* Enable DMA */
}

static void BBDC_DMAStart(void)
{
	struct jz_tssi_t *tssi = &jz_tssi_g;
	struct jz_tssi_buf_ring_t *ring = tssi->cur_buf;

	/* overrun masking */
	__tssi_state_clear_overrun();

	tssi_dma_start(tssi->dma_chan,(unsigned int)ring->front->buf);
	//printk("DMAStart() is started - \n");
	printk("BBDC_DMAStart(%x) Start\n" , tssi->dma_chan);
}

static void tssi_dma_reset()
{
	BBDC_TSSIDisable();
	mdelay(10);
	BBDC_DMAStart();
	BBDC_TSSIEnable();
}

static irqreturn_t tssi_dma_irq(int irq, void * dev_id)
{
	struct jz_tssi_t *tssi = (struct jz_tssi_t *)dev_id;
	struct jz_tssi_buf_ring_t *ring = tssi->cur_buf;
	int val = REG_TSSI_NUM;

	if (REG_TSSI_STAT & TSSI_STAT_OVRN)  {
		if (val > 100)
		{
			printk("+++++over run, val = %d\n", val);
			tssi_dma_reset();		//lator try... 110811
		}
		else
			__tssi_state_clear_overrun();
                       //TSSTAT overrun interrupt flag is active 
	}

       	DMA_Count++;
	if (__dmac_channel_transmit_end_detected(tssi->dma_chan)) {
		__dmac_disable_channel(tssi->dma_chan);
		__dmac_channel_clear_transmit_end(tssi->dma_chan);

		DMA_RealCount++;
		if ( ring->fu_num < RING_BUF_NUM ) {
			
			ring->front = ring->front->next;
			tssi_dma_start(tssi->dma_chan, (unsigned int)ring->front->buf);
			ring->fu_num++;
		}
	}

	if (__dmac_channel_transmit_halt_detected(tssi->dma_chan)) {
		printk("DMA HALT\n");
		__dmac_channel_clear_transmit_halt(tssi->dma_chan);
	}

	if (__dmac_channel_address_error_detected(tssi->dma_chan)) {
		printk("DMA ADDR ERROR\n");
		__dmac_channel_clear_address_error(tssi->dma_chan);
	}

	return IRQ_HANDLED;
}

static int tssi_dma_init(unsigned int dma_chan, int size)
{
	unsigned int group, data;

	group = dma_chan / HALF_DMA_NUM;

	printk("DMA_INIT Channel : %x , group : %x\n", dma_chan , group);

	data = DMAC_DCMD_DAI |	//DCMn dest addr increment 
		DMAC_DCMD_SWDH_32 |	//src port width:32bit
		DMAC_DCMD_DWDH_32 |	 
		DMAC_DCMD_DS_16BYTE | //TSZ 32B
		DMAC_DCMD_TIE;	//transfer irq enable
        
        /* DMACn global DMA enable bit */
        REG_DMAC_DMACR(group) |= (DMAC_DMACR_DMAE | 1 << 29 | 0x3 << 8); 
        //DCKE enable DMA clock
	REG_DMAC_DMACKES(group) |= (1 << (dma_chan % 6));	
       	REG_DMAC_DCCSR(dma_chan) |= DMAC_DCCSR_NDES; 	/* DCSn */
	REG_DMAC_DCMD(dma_chan) |= data; 
	REG_DMAC_DRSR(dma_chan) |= DMAC_DRSR_RS_TSSIIN;	/* DRTn request type */

	return 0;
}


static irqreturn_t tssi_interrupt(int irq, void * dev_id)
{
	__intc_mask_irq(TSSI_IRQ);

	if ( REG_TSSI_STAT & TSSI_STAT_TRIG ) {
		tssi_read_fifo( dev_id );
	}

	if ( REG_TSSI_STAT & TSSI_STAT_OVRN ) {
		printk("pio mode tssi over run: %x\n", REG8( TSSI_STAT ));
		REG_TSSI_STAT &= ~TSSI_STAT_OVRN;		//overrun is clear BTL
	}

	REG_TSSI_STAT &= ~TSSI_STAT_TRIG;			//Trigger clear BTL
	__intc_unmask_irq(TSSI_IRQ);
	return IRQ_HANDLED;
}

static void BBDC_TSSIEnable(void)
{
	__tssi_disable();
	__tssi_soft_reset(); //clear the fifo
	__tssi_clear_state();
	/* Set the TSCTRL to 0x00 to enable all interrupts */
	BTL_WREG8(TSSI_CTRL , 0x00);

	__tssi_enable();
}

static void BBDC_TSSIDisable(void)
{

	tssi_buf_clear (&jz_tssi_ring_g );
	__tssi_disable();
	__tssi_soft_reset(); //clear the fifo
	__tssi_clear_state();

	printk("BBDC_TSSI Disable(Stop)\n");
}

static ssize_t BBDRV_Read(struct file * filp, char __user * buffer,
		size_t count, loff_t * f_pos)

{
	struct jz_tssi_t *tssi = &jz_tssi_g;
	struct jz_tssi_buf_ring_t* ring = tssi->cur_buf;
	int i, ret = 0;
	unsigned char *p = (unsigned char*)ring->rear->buf;

	count /= TSSI_PACK_SIZE;
	/*************************************************
	 * printk("\nbuffer addr: %#x ring.fu_num: %d count: %d\n",
	 * 			buffer, ring->fu_num, count);
	 **************************************************/
	mutex_lock(&tssi->buf_lock);
	if ( count > ring->fu_num )
		count = ring->fu_num;
	mutex_unlock(&tssi->buf_lock);

	TSSI_Count ++;
	if ( (TSSI_Count % 5000 ) == 0 && TSSI_Count != 0)
	{
		printk("TSSI(%d), fu_num %d, DMA_Cnt %d DMA_RCnt %d\n", TSSI_Count, count, DMA_Count, DMA_RealCount);
	}

	/* only 2 ring buffer */
	if (count > 2)
		count = 2;

	for ( i = 0; i < count; i++ ) {
		ret = copy_to_user( buffer + ( i * TSSI_PACK_SIZE),
				ring->rear->buf, TSSI_PACK_SIZE );
		if (ret)
			return -EFAULT;

		ring->rear->pos = 0;
		ring->rear = ring->rear->next;
		p = (unsigned char*)ring->rear->buf;
	}
	mutex_lock(&tssi->buf_lock);
	ring->fu_num -= count;
	mutex_unlock(&tssi->buf_lock);

	return count * TSSI_PACK_SIZE;
}

static int __init jztssi_init_module(void)
{
	struct jz_tssi_t *tssi = &jz_tssi_g;

	__cpm_start_tssi();
	tssi_buf_init( &jz_tssi_ring_g );
	tssi->cur_buf = &jz_tssi_ring_g;
	tssi->pid_num = 0;
	mutex_init(&tssi->buf_lock);
	tssi->dma_chan = jz_request_dma(DMA_ID_TSSI, "tssi", 
                                       tssi_dma_irq,IRQF_DISABLED, &jz_tssi_g);
	if ( tssi->dma_chan < 0 ) {
		printk("MPEG2-TS request irq fail!\n");
		return -1;
	}
	// enable corresponding channel clock
	__dmac_channel_enable_clk(tssi->dma_chan);
	printk("SERIAL TS driver registered (%x) DMA(%x)\n",
                                     (unsigned int)&jz_tssi_g , tssi->dma_chan);
	return 0;
}

static void __exit jztssi_cleanup_module(void)
{
	jz_free_dma(jz_tssi_g.dma_chan);
	tssi_free_buf( &jz_tssi_ring_g );
	printk("Jz TSSI module removed!\n");
}

void BBDC_Reset(void)
{
	printk("[BBDC_Reset] +\n");
	/* DTV_RESET --> LOW */
	__dtv_reset_low();

	/* DTV_RESET --> HIGH */
	mdelay(10);		//20msec
	__dtv_reset_high();
	printk("[BBDC_Reset] +\n");
}

static INT BBDRV_IOCTRL( struct file *pFile, UINT CMD, ULONG ARG);
static INT BBDRV_Open(struct inode *pINode, struct file *pFile);
static INT BBDRV_Release(struct inode *pINode, struct file *pFile);
static INT __init BBDRV_INIT(VOID);
static VOID __exit BBDRV_EXIT(VOID);


static INT BBDRV_IOCTRL( struct file *pFile, UINT CMD, ULONG ARG)
{
  	INT iRET = 0;

  	switch( CMD )
  	{
	case BBIOCTL_TSOPEN:
       			TSSI_Count = 0;
			DMA_Count = 0;
			DMA_RealCount = 0;
			BBDC_DMAStart();
			BBDC_TSSIEnable();
      		        break;
	case BBIOCTL_TSCLOSE:
			BBDC_TSSIDisable();
	      	        break;
	case BBIOCTL_TSDUMP:
			gTSSIRegPrint();
			break;
	case BBIOCTL_CHIPRESET:
			BBDC_Reset();
			break;
	case BBIOCTL_TSRESET:
			tssi_dma_reset();
			break;

    	default:
      		break;
  	}

  	return iRET;
}

static INT BBDRV_Open(struct inode *pINode, struct file *pFile)
{
	struct jz_tssi_t *tssi = &jz_tssi_g;
	
	/* TSSI Open, clear the TSSI FIFO */
	__tssi_soft_reset();
	tssi_config_filting();
		
       	tssi_dma_init(tssi->dma_chan, TSSI_PACK_SIZE);
	dump_dma_channel(tssi->dma_chan);
	print_reg();

  	return 0;
}

static INT BBDRV_Release(struct inode *pINode, struct file *pFile)
{
	struct jz_tssi_t 	*tssi = &jz_tssi_g;

	printk("[BBDRV_Release] -\n");	

	/* tsspi release */
	__cpm_stop_tssi();
	__tssi_disable();
	__tssi_dma_disable();
	__tssi_soft_reset();
	REG_DMAC_DCCSR(tssi->dma_chan) &= ~(DMAC_DCCSR_EN | /*DMAC_DCCSR_CT |*/
				DMAC_DCCSR_TT | DMAC_DCCSR_AR |
				DMAC_DCCSR_HLT);

	printk("[BBDRV_Release] +\n");	
  	return 0;
}

static INT BBDRV_Suspend(struct inode *pINode, struct file *pFile)
{
	//struct jz_tssi_t *tssi = &jz_tssi_g;
	
  	return 0;
}

static INT BBDRV_Resume(struct inode *pINode, struct file *pFile)
{
	//struct jz_tssi_t *tssi = &jz_tssi_g;

	printk("[BBDRV_Reset] \n");	
  	return 0;
}

static struct file_operations spidev_fops = {
	.owner          =	THIS_MODULE,
	.read	        =	BBDRV_Read,
	.poll	        =	NULL,
	.fasync	        =	NULL,
	.unlocked_ioctl = 	BBDRV_IOCTRL,
	.open           =	BBDRV_Open,
	.release        =	BBDRV_Release,
//	.suspend  =  BBDRV_Suspend,
//	.resume	 = 	BBDRV_Resume,

};

static INT __init BBDRV_INIT(VOID)
{
	INT iRET = 0;

	printk("[BBDRV_INIT] - TS SERIAL init -\n");	
	/* DTV_RESET --> LOW */
	__dtv_reset_init();
	__dtv_reset_low();
	__dtv_special_off();
	mdelay(10);
	/* DTV VCC EN --> Power On */
	__dtv_special_on();
	/* DTV_RESET --> HIGH */
	mdelay(10);		//20msec
	__dtv_reset_high();
  	jztssi_init_module();
  	printk("[BBDRV_INIT] - TS SERIAL init +\n");	
	iRET = register_chrdev(BBDRV_MAJOR, BBDRV_NAME, &spidev_fops);
	if (iRET != 0){
      		printk("\n################regist tssi driver failed##########\n");
       	}
	else
		printk("\n############register tssi driver success############\n");
	return iRET;
}

static VOID __exit BBDRV_EXIT(VOID)
{
	unregister_chrdev( BBDRV_MAJOR, BBDRV_NAME );
	
	jztssi_cleanup_module();
}

module_init( BBDRV_INIT );
module_exit( BBDRV_EXIT );

MODULE_DESCRIPTION("Ingenic ISDBT Demod TS interface Driver");
MODULE_LICENSE( "Dual BSD/GPL" );
