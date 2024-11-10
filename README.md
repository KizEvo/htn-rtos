# freeRTOS Embedded System
This repository hold the source code for the Embedded System course of University of Science, reports and implementation details.
The actual board for the course is STM32F405xx and since the group has three members but only one board so the decision was to use
a different board (STM32F103C8T6) to develop and test using the HAL layers. It helps abstract the driver and hardware implementation detail so that
going from F1 to F4 the code won't change much.

The requirements were to use a RTOS on the MCU. It should have three different tasks (threads) to process data. We start the `main()`
function by initializing the necessary peripherals, creating three static tasks, queue and semaphore (as needed). Each task is 
described more in depth in the following section.

**System Configuration**

![image](https://github.com/user-attachments/assets/b6bee895-b7e2-4465-b5a9-0a9b13f7d4a1)

**GPIO:** Help in debugging the system

![image](https://github.com/user-attachments/assets/95f63264-2454-4240-8f3c-b26648f0cbd2)

**SYS**

![image](https://github.com/user-attachments/assets/83c25413-3396-4ba0-a05f-49a74c012fc7)

**Clock Configuration**

![image](https://github.com/user-attachments/assets/86e238da-a5c3-4c94-87a4-4b0622bf8251)

## Task1
**Target:** Use DMA to read from an ADC and write to the MCU EEPROM (FLASH).

**DMA**: Read ADC and write to FLASH memory region

![image](https://github.com/user-attachments/assets/ff3ade75-02d9-4605-9bec-8371382bbf93)

![image](https://github.com/user-attachments/assets/f3441dbc-efd4-4588-b36e-bedacd168b1c)

**NVIC**

![image](https://github.com/user-attachments/assets/7d63598e-1943-41b5-a6c1-8573e8028ea6)

**ADC**

![image](https://github.com/user-attachments/assets/64440aa4-cf40-4a16-8a5b-39280b4f5ac3)

![image](https://github.com/user-attachments/assets/4bb61e5c-26c7-47b4-967d-94532bc235ae)

![image](https://github.com/user-attachments/assets/0fa193cc-0e9d-4c65-b8f7-3f60f8611eb2)

**TIM4**

![image](https://github.com/user-attachments/assets/76b54786-c0da-4e55-ac8e-8c1151c7c110)

**freeRTOS** : Add a Queue and raise the priority of Task 1

![image](https://github.com/user-attachments/assets/8495d3c8-2df5-466f-a900-7c870c5d4819)

### Source code
In the `StartDefaultTask()` function, we first start the TIM4 in interrupt mode.
```c
rc = HAL_TIM_Base_Start_IT(&htim4);
assert(rc == HAL_OK && "Cannot start TIM4 clock for ADC sampling application");
```
Then initialize the necessary info to erase a page when writing to FLASH
```c
pEraseInit.Banks = FLASH_BANK_1; // This config doesn't matter since we only erasing a page
pEraseInit.NbPages = 1; // Erase 1 page
pEraseInit.PageAddress = FLASH_PAGE_X_OFFSET(50); // Macro to get the FLASH address based on the page, min is 0 and max is 63
pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES; // Type of erase operation is page
```
Following that we get into the forever loop, here we always want this Task to sleep and only wake up to process data when there is
an item in the Queue.
```c
// event.value.v hold the actual message (eg. the ADC1 data)
osEvent event = osMessageGet(ADC_QNameHandle, osWaitForever);
```
If an item exist in the Queue, the Task wakeup and process the following line of code
```c
// Unlock flash
HAL_FLASH_Unlock();
// Erase first
HAL_FLASHEx_Erase(&pEraseInit, &PageError);
// Write to flash
// HAL function to write to FLASH using CPU - HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_PAGE_X_OFFSET(50), event.value.v);
FLASH_DMA_Write(FLASH_PAGE_X_OFFSET(50), event.value.v);
// Lock flash
HAL_FLASH_Lock();
```
Based on the information of the HAL function `HAL_FLASH_Program` in the file `stm32f1xx_hal_flash_ex.c`, to be able to write to the flash we must first unlock the flash
then we erase the page we want to write to then proceed to write, after we finish writing, we lock the flash to protect it from unwanted operations.

Diving deeper into the `FLASH_DMA_Write` function, we see that it's actually based on the `HAL_FLASH_Program` function provided by the HAL. But the different is that we use
DMA to write to flash.
```c
HAL_DMA_Start(&hdma_memtomem_dma1_channel2, (uint32_t)&Data, Address, 1);
while (HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_channel2, HAL_DMA_FULL_TRANSFER, 100) != HAL_OK);
```
Once we finished writing, we can easily read from the FLASH.
```c
FLASH_Data = *(__IO uint32_t *)(FLASH_PAGE_X_OFFSET(50));
```
But how does the Queue contain item such that it let Task 1 wakeup ?
That's the jobs of TIM4 and DMA. The TIM4 will trigger an interrupt one every 500ms, which start the ADC and tell DMA controller to collect the ADC data once it finishes sampling.
The configuration for TIM4: `pCLK = 60MHz, prescaler = 600, auto-reload = true, count-period = 50000`
```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  // DO NOT TOUCH TIM2 as it's used by the OS
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM4) {
    HAL_ADC_Start_DMA(&hadc1, &ADC_Buffer, ADC_Buffer_Size);
  }
  /* USER CODE END Callback 1 */
}
```
When the DMA finish collecting the data, it will also trigger an interrupt so that we can collect the data and put it into the Queue - Task1 will wakeup after this interrupt finished.
```c
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1) {
    osMessagePut(ADC_QNameHandle, ADC_Buffer, 50);
  }
}
```
