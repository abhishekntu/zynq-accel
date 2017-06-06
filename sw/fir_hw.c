#include <stdio.h>
#include <stdint.h>
#include "xparameters.h"
#include "xil_types.h"
#include "xstatus.h"
#include "xil_testmem.h"
#include "xil_io.h"
#include "platform.h"
#include "time.h"
#include <stdlib.h>
#include "sleep.h"
#include "xil_assert.h"
#include "xil_exception.h"
#include "xil_cache.h"
#include "xil_printf.h"
#include "xscugic.h"
#include "xil_mmu.h"
#include "xdmaps.h"
#include "time.h"
/***********************************************************
DMA MACROS
************************************************************/
#define DMA
#define DMA_DEVICE_ID 			XPAR_XDMAPS_1_DEVICE_ID
#define INTC_DEVICE_ID			XPAR_SCUGIC_SINGLE_DEVICE_ID

#define DMA_DONE_INTR_0			XPAR_XDMAPS_0_DONE_INTR_0
#define DMA_DONE_INTR_1			XPAR_XDMAPS_0_DONE_INTR_1
#define DMA_DONE_INTR_2			XPAR_XDMAPS_0_DONE_INTR_2
#define DMA_DONE_INTR_3			XPAR_XDMAPS_0_DONE_INTR_3
#define DMA_DONE_INTR_4			XPAR_XDMAPS_0_DONE_INTR_4
#define DMA_DONE_INTR_5			XPAR_XDMAPS_0_DONE_INTR_5
#define DMA_DONE_INTR_6			XPAR_XDMAPS_0_DONE_INTR_6
#define DMA_DONE_INTR_7			XPAR_XDMAPS_0_DONE_INTR_7
#define DMA_FAULT_INTR			XPAR_XDMAPS_0_FAULT_INTR

#define TEST_ROUNDS	1	/* Number of loops that the Dma transfers run.*/
#define TIMEOUT_LIMIT 	0x2000	/* Loop count for timeout */
/***********************************************************
Normal Functions
************************************************************/
void putnum(unsigned int num);
/***********************************************************
NUM_SAMPLES 32, 64, 128, 256, 512, 900
************************************************************/
#define NUM_SAMPLES 512
#define DMA_LENGTH NUM_SAMPLES
/***********************************************************
NUM_COEFF, dst_mem and latency
************************************************************/
#define dst_mem 0x02000000
#define latency 65
static volatile u32 *c_base 	= XPAR_COPROCESSOR_0_S_AXI_MEM0_BASEADDR + 0x00000004;
static volatile u32 *d_base 	= XPAR_COPROCESSOR_0_S_AXI_MEM1_BASEADDR + 0x00000004;
static volatile u32 *ctrl   	= XPAR_COPROCESSOR_0_S_AXI_MEM0_BASEADDR;
static volatile u32 *stat   	= XPAR_COPROCESSOR_0_S_AXI_MEM1_BASEADDR;
static volatile u32 *count		= XPAR_COPROCESSOR_0_S_AXI_MEM1_BASEADDR + 0x0000013c;
/***********************************************************
Timer base address
************************************************************/
static int *Src = XPAR_PS7_DDR_0_S_AXI_BASEADDR+0x00025000;
static int *Dst = XPAR_PS7_DDR_0_S_AXI_BASEADDR+0x01075000;

#define timer_base 0xf8f00000
#define start 0x00000001
/***********************************************************
Timer Registers
************************************************************/
static volatile int *timer_counter_l=(volatile int *)(timer_base+0x200);
static volatile int *timer_counter_h=(volatile int *)(timer_base+0x204);
static volatile int *timer_ctrl=(volatile int *)(timer_base+0x208);
/***********************************************************
DMA and GIC instances
************************************************************/
XDmaPs DmaInstance;
XScuGic GicInstance;
/***********************************************************
DMA function prototypes
************************************************************/
int load_Data_dma(u16 DeviceId, unsigned int Channel, u32 Src_dma, u32 Dst_dma);
int read_Data_dma(u16 DeviceId, unsigned int Channel, u32 Src_dma, u32 Dst_dma);
int SetupInterruptSystem(XScuGic *GicPtr, XDmaPs *DmaPtr);
void DmaDoneHandler(unsigned int Channel, XDmaPs_Cmd *DmaCmd,void *CallbackRef);
/***********************************************************
Function definitions
************************************************************/
void init_timer(volatile int *timer_ctrl, volatile int *timer_counter_l, volatile int *timer_counter_h){
	*timer_ctrl=0x0;
	*timer_counter_l=0x1;
	*timer_counter_h=0x0;
	DATA_SYNC;
}

void start_timer(volatile int *timer_ctrl){
	*timer_ctrl=*timer_ctrl | 0x00000001;
	DATA_SYNC;
}

void stop_timer(volatile int *timer_ctrl){
	*timer_ctrl=*timer_ctrl & 0xFFFFFFFE;
	DATA_SYNC;
}


void init_Data(){
	int i;
	Src[0]=0x00000100;
	for(i=1;i<NUM_SAMPLES;i++){
		Src[i]=0x00000000;
	}
}

void load_Data(){
	int i;
	for(i=0;i<NUM_SAMPLES;i++){
		    c_base[i]= Src[i];
	    }
	    DATA_SYNC;
}

void read_Data(){
	int i, value;
	for(i=0;i<NUM_SAMPLES;i++){
	    value= d_base[i];
	}
}

int main()
{
    int i,data;
    int Status;

    init_platform();
    xil_printf("---------------------\n\r");
    xil_printf("FIR Filtering on FPGA\n\r");
    xil_printf("---------------------\n\r");

    init_Data();

    init_timer(timer_ctrl, timer_counter_l, timer_counter_h);
    start_timer(timer_ctrl);

    *count = 0x00000000;
    *ctrl  = 0x00000000;

#ifndef DMA
    load_Data();
#else
    Status = load_Data_dma(DMA_DEVICE_ID, 0, Src, c_base);
#endif
    *ctrl= ( dst_mem | (NUM_SAMPLES<<12) | (latency<<4) | start );
    *stat=0x00000000;
    while(*stat!=0x1);
    stop_timer(timer_ctrl);
    xil_printf("Execution time %d us in processing %d samples--\n\r", (*timer_counter_l)/333, NUM_SAMPLES);
#ifndef DMA
    read_Data();
#else
    read_Data_dma(DMA_DEVICE_ID, 0, d_base, Dst);
#endif

    		for(i=0;i<10;i++){
                    	printf("c[%d]=%08x, d[%d]=%08x\n\r", i, (u32)c_base[i], i, (u32)d_base[i]);
            }

    	int Success;
        for(i=0;i<NUM_SAMPLES;i++){

                	if(c_base[i]!=Src[i]){
                		Success=0;
                	}else{
                		Success=1;
                	}
        }

        if(Success){
        	xil_printf("----------------------------------\n\r");
        	xil_printf("loading Successful...........\n\r");
        	xil_printf("----------------------------------\n\r");
        }else{
        	xil_printf("----------------------------------\n\r");
        	xil_printf("loading failed...............\n\r");
        	xil_printf("----------------------------------\n\r");
        }

#ifdef DMA

        for(i=0;i<NUM_SAMPLES;i++){
                	//xil_printf("c[%d]=%d, e[%d]=%d\n\r", i, c_base[i], i, e_base[i]);
                	if(Dst[i]!=d_base[i]){
                		Success=0;
                	}else{
                		Success=1;
                	}
        }

        if(Success){
        	xil_printf("----------------------------------\n\r");
        	xil_printf("Reading Successful...........\n\r");
        	xil_printf("----------------------------------\n\r");
        }else{
        	xil_printf("----------------------------------\n\r");
        	xil_printf("Reading failed...............\n\r");
        	xil_printf("----------------------------------\n\r");
        }


#endif

#ifdef DMA
        if (Status != XST_SUCCESS) {
        	xil_printf("Error: XDMaPs_Example_W_Intr failed\r\n");
        	return XST_FAILURE;
        }
        xil_printf("XDMaPs_Example_W_Intr passed\r\n");
#endif
        cleanup_platform();
        return 0;
}


/*****************************************************************************/
/*
 * Interrupt Example to test the DMA.
 ****************************************************************************/

int load_Data_dma(u16 DeviceId, unsigned int Channel, u32 Src_dma, u32 Dst_dma)
{

	int Status;
	int TestStatus;

	int TimeOutCnt;
		XDmaPs_Config *DmaCfg;
		XDmaPs *DmaInst = &DmaInstance;
		/** Setup the interrupt system.*/
		Status = SetupInterruptSystem(&GicInstance, DmaInst);
		if (Status != XST_SUCCESS) {
				return XST_FAILURE;
		}

		/** Initialize the DMA Driver*/
		DmaCfg = XDmaPs_LookupConfig(DeviceId);
		if (DmaCfg == NULL) {
			return XST_FAILURE;
		}
		Status = XDmaPs_CfgInitialize(DmaInst,DmaCfg,DmaCfg->BaseAddress);
		if (Status != XST_SUCCESS) {
			return XST_FAILURE;
		}

	volatile int Checked[XDMAPS_CHANNELS_PER_DEV];


	XDmaPs_Cmd DmaCmd;

	memset(&DmaCmd, 0, sizeof(XDmaPs_Cmd));
	DmaCmd.ChanCtrl.SrcBurstSize = 4;
	DmaCmd.ChanCtrl.SrcBurstLen = 4;
	DmaCmd.ChanCtrl.SrcInc = 1;
	DmaCmd.ChanCtrl.DstBurstSize = 4;
	DmaCmd.ChanCtrl.DstBurstLen = 4;
	DmaCmd.ChanCtrl.DstInc = 1;
	DmaCmd.BD.SrcAddr = (u32) Src_dma;
	DmaCmd.BD.DstAddr = (u32) Dst_dma;
	DmaCmd.BD.Length = DMA_LENGTH * sizeof(int);
	TestStatus = XST_SUCCESS;
	Checked[Channel] = 0;

	/* Set the Done interrupt handler */
	XDmaPs_SetDoneHandler(DmaInst,Channel,DmaDoneHandler,(void *)(Checked + Channel));
	Status = XDmaPs_Start(DmaInst, Channel, &DmaCmd, 0);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	TimeOutCnt = 0;
	/* Now the DMA is done */
	while (!Checked[Channel] && TimeOutCnt < TIMEOUT_LIMIT) {
				TimeOutCnt++;
    }
    if (TimeOutCnt >= TIMEOUT_LIMIT) {
    	TestStatus = XST_FAILURE;
    }
    if (Checked[Channel] < 0) {
    	/* DMA controller failed */
    	TestStatus = XST_FAILURE;
    }
	return TestStatus;
}

int read_Data_dma(u16 DeviceId, unsigned int Channel, u32 Src_dma, u32 Dst_dma)
{

	int Status;
	int TestStatus;

	int TimeOutCnt;
	volatile int Checked[XDMAPS_CHANNELS_PER_DEV];
	XDmaPs_Config *DmaCfg;
	XDmaPs *DmaInst = &DmaInstance;
	XDmaPs_Cmd DmaCmd;


			/** Setup the interrupt system.*/
			Status = SetupInterruptSystem(&GicInstance, DmaInst);
			if (Status != XST_SUCCESS) {
					return XST_FAILURE;
			}

			/** Initialize the DMA Driver*/
			DmaCfg = XDmaPs_LookupConfig(DeviceId);
			if (DmaCfg == NULL) {
				return XST_FAILURE;
			}
			Status = XDmaPs_CfgInitialize(DmaInst,DmaCfg,DmaCfg->BaseAddress);
			if (Status != XST_SUCCESS) {
				return XST_FAILURE;
			}

	memset(&DmaCmd, 0, sizeof(XDmaPs_Cmd));
	DmaCmd.ChanCtrl.SrcBurstSize = 4;
	DmaCmd.ChanCtrl.SrcBurstLen = 4;
	DmaCmd.ChanCtrl.SrcInc = 1;
	DmaCmd.ChanCtrl.DstBurstSize = 4;
	DmaCmd.ChanCtrl.DstBurstLen = 4;
	DmaCmd.ChanCtrl.DstInc = 1;
	DmaCmd.BD.SrcAddr = (u32) Src_dma;
	DmaCmd.BD.DstAddr = (u32) Dst_dma;
	DmaCmd.BD.Length = DMA_LENGTH * sizeof(int);
	TestStatus = XST_SUCCESS;
	Checked[Channel] = 0;

	/* Set the Done interrupt handler */
	XDmaPs_SetDoneHandler(DmaInst,Channel,DmaDoneHandler,(void *)(Checked + Channel));
	Status = XDmaPs_Start(DmaInst, Channel, &DmaCmd, 0);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	TimeOutCnt = 0;
	/* Now the DMA is done */
	while (!Checked[Channel] && TimeOutCnt < TIMEOUT_LIMIT) {
				TimeOutCnt++;
    }
    if (TimeOutCnt >= TIMEOUT_LIMIT) {
    	TestStatus = XST_FAILURE;
    }
    if (Checked[Channel] < 0) {
    	/* DMA controller failed */
    	TestStatus = XST_FAILURE;
    }
	return TestStatus;
}

/******************************************************************************/
/**
 *
 * This function connects the interrupt handler of the interrupt controller to
 * the processor.  This function is seperate to allow it to be customized for
 * each application. Each processor or RTOS may require unique processing to
 * connect the interrupt handler.
 *
 * @param	GicPtr is the GIC instance pointer.
 * @param	DmaPtr is the DMA instance pointer.
 *
 * @return	None.
 *
 * @note	None.
 *
 ****************************************************************************/
int SetupInterruptSystem(XScuGic *GicPtr, XDmaPs *DmaPtr)
{
	int Status;
	XScuGic_Config *GicConfig;


	Xil_ExceptionInit();

	/*
	 * Initialize the interrupt controller driver so that it is ready to
	 * use.
	 */
	GicConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (NULL == GicConfig) {
		return XST_FAILURE;
	}

	Status = XScuGic_CfgInitialize(GicPtr, GicConfig,
				       GicConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Connect the interrupt controller interrupt handler to the hardware
	 * interrupt handling logic in the processor.
	 */
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,
			     (Xil_ExceptionHandler)XScuGic_InterruptHandler,
			     GicPtr);

	/*
	 * Connect the device driver handlers that will be called when an interrupt
	 * for the device occurs, the device driver handler performs the specific
	 * interrupt processing for the device
	 */

	/*
	 * Connect the Fault ISR
	 */
	Status = XScuGic_Connect(GicPtr,
				 DMA_FAULT_INTR,
				 (Xil_InterruptHandler)XDmaPs_FaultISR,
				 (void *)DmaPtr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Connect the Done ISR for all 8 channels of DMA 0
	 */
	Status = XScuGic_Connect(GicPtr,
				 DMA_DONE_INTR_0,
				 (Xil_InterruptHandler)XDmaPs_DoneISR_0,
				 (void *)DmaPtr);
	Status |= XScuGic_Connect(GicPtr,
				 DMA_DONE_INTR_1,
				 (Xil_InterruptHandler)XDmaPs_DoneISR_1,
				 (void *)DmaPtr);
	Status |= XScuGic_Connect(GicPtr,
				 DMA_DONE_INTR_2,
				 (Xil_InterruptHandler)XDmaPs_DoneISR_2,
				 (void *)DmaPtr);
	Status |= XScuGic_Connect(GicPtr,
				 DMA_DONE_INTR_3,
				 (Xil_InterruptHandler)XDmaPs_DoneISR_3,
				 (void *)DmaPtr);
	Status |= XScuGic_Connect(GicPtr,
				 DMA_DONE_INTR_4,
				 (Xil_InterruptHandler)XDmaPs_DoneISR_4,
				 (void *)DmaPtr);
	Status |= XScuGic_Connect(GicPtr,
				 DMA_DONE_INTR_5,
				 (Xil_InterruptHandler)XDmaPs_DoneISR_5,
				 (void *)DmaPtr);
	Status |= XScuGic_Connect(GicPtr,
				 DMA_DONE_INTR_6,
				 (Xil_InterruptHandler)XDmaPs_DoneISR_6,
				 (void *)DmaPtr);
	Status |= XScuGic_Connect(GicPtr,
				 DMA_DONE_INTR_7,
				 (Xil_InterruptHandler)XDmaPs_DoneISR_7,
				 (void *)DmaPtr);

	if (Status != XST_SUCCESS)
		return XST_FAILURE;

	/*
	 * Enable the interrupts for the device
	 */
	XScuGic_Enable(GicPtr, DMA_DONE_INTR_0);
	XScuGic_Enable(GicPtr, DMA_DONE_INTR_1);
	XScuGic_Enable(GicPtr, DMA_DONE_INTR_2);
	XScuGic_Enable(GicPtr, DMA_DONE_INTR_3);
	XScuGic_Enable(GicPtr, DMA_DONE_INTR_4);
	XScuGic_Enable(GicPtr, DMA_DONE_INTR_5);
	XScuGic_Enable(GicPtr, DMA_DONE_INTR_6);
	XScuGic_Enable(GicPtr, DMA_DONE_INTR_7);
	XScuGic_Enable(GicPtr, DMA_FAULT_INTR);

	Xil_ExceptionEnable();

	return XST_SUCCESS;

}

/*****************************************************************************/
/**
*
* DmaDoneHandler.
*
* @param	Channel is the Channel number.
* @param	DmaCmd is the Dma Command.
* @param	CallbackRef is the callback reference data.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void DmaDoneHandler(unsigned int Channel, XDmaPs_Cmd *DmaCmd, void *CallbackRef)
{

	/* done handler */
	volatile int *Checked = (volatile int *)CallbackRef;
	int Status = 1;
	*Checked = Status;
}
