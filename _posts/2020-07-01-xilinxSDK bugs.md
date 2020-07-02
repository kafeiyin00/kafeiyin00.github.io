---
layout: post
title: xilinxSDK bugs
date: 2020-07-01
Author: jianping
categories: 
tags: [hardware]
comments: false
---

# 内存分配bug

代码运行的时候各种随机malloc 内存不足, 把steak和leap增大.拉满拉满,转到release模式. 真是个悲伤的bug.

![](https://pic.downk.cc/item/5efd84ba14195aa5947e9bd7.jpg)

![](https://pic.downk.cc/item/5efd853514195aa5947eca21.jpg)

![](https://pic.downk.cc/item/5efd85d314195aa5947efd91.jpg)

# 学会在数据传输的过程中使用位拼接

即便使用fifo+idle中断,也很难保证设备能一次性把"一句话说完",或者一次性说了几句话,因此,还是老老实实采用位拼接.

```cpp

#include <xparameters.h>
#include "xil_printf.h"
#include "sleep.h"

#include "xscugic.h"
#include "xil_exception.h"


#include "axi_uart_fifo.h"

#include "xscugic.h"
#include "xil_exception.h"

#include <vector>

#include "parseNovatel.h"


#define UART_FIFO_READ_READY_INTC_ID        XPAR_FABRIC_AXI_UART_FIFO_GPS_READ_READY_IRQ_INTR
#define INT_TYPE_RISING_EDGE    			0x03
#define INT_TYPE_HIGHLEVEL      			0x01
#define INT_TYPE_MASK           			0x03

static XScuGic INTCInst;


//data buffer
static std::vector<u8> gpsDataBuffer;

void UART_FIFO_GPS_READ_READY_INTC_Handler(void *param)
{

	// read out
	// u32 rd_fifo_valid_count = AXI_UART_FIFO_mReadReg(XPAR_AXI_UART_FIFO_GPS_S00_AXI_BASEADDR,AXI_UART_FIFO_S00_AXI_SLV_REG2_OFFSET);
	// rd_fifo_valid_count = AXI_UART_FIFO_mReadReg(XPAR_AXI_UART_FIFO_GPS_S00_AXI_BASEADDR,AXI_UART_FIFO_S00_AXI_SLV_REG2_OFFSET);

	int count = 0;

	while(AXI_UART_FIFO_ReadAvailable(XPAR_AXI_UART_FIFO_GPS_S00_AXI_BASEADDR) > 0)
	{
		u8 value = AXI_UART_FIFO_mReadReg(XPAR_AXI_UART_FIFO_GPS_S00_AXI_BASEADDR,AXI_UART_FIFO_S00_AXI_SLV_REG0_OFFSET);
		//xil_printf("%X",value);
		gpsDataBuffer.push_back(value);
		count++;
	}

	// parse data
	int tmpPos = parseNovatel(gpsDataBuffer);
	gpsDataBuffer.erase(gpsDataBuffer.begin(),gpsDataBuffer.begin()+tmpPos);
	//xil_printf("size:%d\n",gpsDataBuffer.size());
	//xil_printf("%d\n",tmpPos);

	return;
}

void IntcTypeSetup(XScuGic *InstancePtr, int intId, int intType)
{
    int mask;
    intType &= INT_TYPE_MASK;
    mask = XScuGic_DistReadReg(InstancePtr, XSCUGIC_INT_CFG_OFFSET +  (intId/16)*4);
    mask &= ~(INT_TYPE_MASK << (intId%16)*2);
    mask |= intType << ((intId%16)*2);
    XScuGic_DistWriteReg(InstancePtr, XSCUGIC_INT_CFG_OFFSET + (intId/16)*4,  mask);
}

int IntcInitFunction(u16 DeviceId)
{
       XScuGic_Config *IntcConfig;
       int status;
       // Interrupt controller initialisation
       IntcConfig = XScuGic_LookupConfig(DeviceId);
       status = XScuGic_CfgInitialize(&INTCInst, IntcConfig,  IntcConfig->CpuBaseAddress);
       if(status != XST_SUCCESS) return XST_FAILURE;
       // Call to interrupt setup
       Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
                                                         (Xil_ExceptionHandler)XScuGic_InterruptHandler,
                                                        &INTCInst);
       Xil_ExceptionEnable();
       // Connect interrupt to handler
       status = XScuGic_Connect(&INTCInst,
    		   UART_FIFO_READ_READY_INTC_ID,
                                (Xil_ExceptionHandler)UART_FIFO_GPS_READ_READY_INTC_Handler,
                                (void *)0);
       if(status != XST_SUCCESS) return XST_FAILURE;
       // Set interrupt type to rising edge
       IntcTypeSetup(&INTCInst, UART_FIFO_READ_READY_INTC_ID, INT_TYPE_HIGHLEVEL);
       // Enable interrupts in the controller
       XScuGic_Enable(&INTCInst, UART_FIFO_READ_READY_INTC_ID);



       // enable uart fifo

       AXI_UART_FIFO_Enable(XPAR_AXI_UART_FIFO_GPS_S00_AXI_BASEADDR,1);

       while(AXI_UART_FIFO_ReadAvailable(XPAR_AXI_UART_FIFO_GPS_S00_AXI_BASEADDR) > 0)
	   {
			u8 value = AXI_UART_FIFO_Read(XPAR_AXI_UART_FIFO_GPS_S00_AXI_BASEADDR);
	   }
       return XST_SUCCESS;
}


int main()
{

	//XPAR_PS7_SCUGIC_0_DEVICE_ID

	xil_printf("hello world123\n");
	IntcInitFunction(XPAR_PS7_SCUGIC_0_DEVICE_ID);
	xil_printf("hello world\n");

	gpsDataBuffer.resize(0);

	while(1)
	{
		sleep(1);

//		for(int i=0; i<1030;i++)
//		{
//			AXI_UART_FIFO_mWriteReg(XPAR_AXI_UART_FIFO_GPS_S00_AXI_BASEADDR,AXI_UART_FIFO_S00_AXI_SLV_REG1_OFFSET,'A');
//		}
//		AXI_UART_FIFO_mWriteReg(XPAR_AXI_UART_FIFO_GPS_S00_AXI_BASEADDR,AXI_UART_FIFO_S00_AXI_SLV_REG1_OFFSET,'\r');
//		AXI_UART_FIFO_mWriteReg(XPAR_AXI_UART_FIFO_GPS_S00_AXI_BASEADDR,AXI_UART_FIFO_S00_AXI_SLV_REG1_OFFSET,'\n');
////
//		//
//
//		// get read fifo valid size
		u32 rd_fifo_valid_count = AXI_UART_FIFO_mReadReg(XPAR_AXI_UART_FIFO_GPS_S00_AXI_BASEADDR,AXI_UART_FIFO_S00_AXI_SLV_REG2_OFFSET);

		xil_printf("rd_fifo Size: %d\n",rd_fifo_valid_count);

	}
	return 0;
}

```