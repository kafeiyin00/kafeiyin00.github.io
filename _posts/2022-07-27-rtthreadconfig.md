---
layout: post
title: rtthread config 
date: 2022-07-27
Author: jianping
categories: 
tags: [ide]
comments: true

---

# Contents

{:.no_toc}

* Will be replaced with the ToC, excluding the "Contents" header
{:toc}




## 1. manage the rtthread workspace
generate cubemx code in the rtthread studio root folder, then create a SConscript file

![project](https://pic.imgdb.cn/item/62e0fdd5f54cd3f937cc095f.jpg)

SConscript is as follow:

```python
import os
#引入os模块
from building import *
#导入building的所有模块

cwd = GetCurrentDir()
#获取获取当前路径，并保存至变量cwd
src  = Glob('*.c')
#获取当前目录下的所有 C 文件，并保存至src变量

# add cubemx drivers
#由于RT-Thread工程中存在部分相同函数文件，所以对src重新赋值
#文件中的stm32g4xx_it.c 、 system_stm32g4xx.c不加入构建
#其余文件按相同格式填写到下述括号内
src = Split('''
Src/stm32g4xx_hal_msp.c
Src/main.c
Src/usart.c
Src/gpio.c
''')

#创建路径列表，并保存至path中
path = [cwd]
path += [cwd + '/Inc']
#这是 RT-Thread 基于 SCons 扩展的一个方法（函数）。
group = DefineGroup('cubemx', src, depend = [''], CPPPATH = path)

Return('group')
```

Then refresh the package.


## 2. copy code from cubemx

Take an example of the uart dma receive and transmit:

in the driver/board.h:

```cpp
/*-------------------------- UART CONFIG BEGIN --------------------------*/

/** After configuring corresponding UART or UART DMA, you can use it.
 *
 * STEP 1, define macro define related to the serial port opening based on the serial port number
 *                 such as     #define BSP_USING_UART1
 *
 * STEP 2, according to the corresponding pin of serial port, define the related serial port information macro
 *                 such as     #define BSP_UART1_TX_PIN       "PA9"
 *                             #define BSP_UART1_RX_PIN       "PA10"
 *
 * STEP 3, if you want using SERIAL DMA, you must open it in the RT-Thread Settings.
 *                 RT-Thread Setting -> Components -> Device Drivers -> Serial Device Drivers -> Enable Serial DMA Mode
 *
 * STEP 4, according to serial port number to define serial port tx/rx DMA function in the board.h file
 *                 such as     #define BSP_UART1_RX_USING_DMA
 *
 */

#define BSP_USING_UART1
#define BSP_UART1_TX_PIN       "PA9"
#define BSP_UART1_RX_PIN       "PA10"
#define BSP_UART1_RX_USING_DMA
#define BSP_UART1_TX_USING_DMA

/*-------------------------- UART CONFIG END --------------------------*/

```

Then several compile fails may exist, because the dma configuration of tx is not automatic done by rtthread. The config in dma_config.h can be modified as follow:

```cpp
#define UART1_DMA_TX_IRQHandler         DMA2_Stream7_IRQHandler
#define UART1_TX_DMA_RCC                RCC_AHB1ENR_DMA2EN
#define UART1_TX_DMA_INSTANCE           DMA2_Stream7
#define UART1_TX_DMA_CHANNEL            DMA_CHANNEL_4
#define UART1_TX_DMA_IRQ                DMA2_Stream7_IRQn
```
The above config can be found in the whitepaper of STM32F7, but it quite complicated. We can generate code using CubeMX, and find the right configuration in the main func as follow:
```cpp
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE(); //important RCC_APB2ENR_USART1EN

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB7     ------> USART1_RX
    PB6     ------> USART1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA2_Stream2;// important
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;// important
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA2_Stream7;// important
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;// important
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0); // important
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);// important
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}
```
