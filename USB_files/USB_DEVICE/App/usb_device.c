/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_device.c
  * @version        : v1.0_Cube
  * @brief          : This file implements the USB Device
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
  /* USER CODE END Header */

  /* Includes ------------------------------------------------------------------*/

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

/* USER CODE BEGIN Includes */
#include "usbd_msc.h"
#include "usbd_storage_if.h"
#include "usbd_dfu.h"
#include "usbd_dfu_if.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDeviceFS;

/*
 * -- Insert your variables declaration here --
 */
 /* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*
 * -- Insert your external function declaration here --
 */
 /* USER CODE BEGIN 1 */

 /* USER CODE END 1 */

 /**
   * Init USB device Library, add supported class and start the library
   * @retval None
   */
void MX_USB_DEVICE_Init(usb_class_e class)
{
  /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */
  USBD_Descriptor_preinit(class);
  /* USER CODE END USB_DEVICE_Init_PreTreatment */

  /* Init Device Library, add supported class and start the library. */
  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
  {
    Error_Handler();
  }

  switch (class)
  {
  case USB_CLASS_CDC:
    if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)
    {
      Error_Handler();
    }
    if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK)
    {
      Error_Handler();
    }
    // NVIC_SetPriority(OTG_FS_IRQn, 0);
    // NVIC_SetPriority(DMA1_Stream5_IRQn, 10);
    break;
  case USB_CLASS_MSC:
    if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_MSC) != USBD_OK)
    {
      Error_Handler();
    }
    if (USBD_MSC_RegisterStorage(&hUsbDeviceFS, &USBD_Storage_Interface_fops_FS) != USBD_OK)
    {
      Error_Handler();
    }
    //  to make writing to FLASH possible it is needed to change priorities of interrupts:
    NVIC_SetPriority(OTG_FS_IRQn, 10);
    NVIC_SetPriority(DMA1_Stream5_IRQn, 9);

    break;
  case USB_CLASS_DFU:
    if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_DFU) != USBD_OK)
    {
      Error_Handler();
    }
    if (USBD_DFU_RegisterMedia(&hUsbDeviceFS, &USBD_DFU_fops_FS) != USBD_OK)
    {
      Error_Handler();
    }
    // NVIC_SetPriority(OTG_FS_IRQn, 0);
    // NVIC_SetPriority(DMA1_Stream5_IRQn, 10);
    break;
  default:
    break;
  }

  if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN USB_DEVICE_Init_PostTreatment */

  /* USER CODE END USB_DEVICE_Init_PostTreatment */
}

/**
  * @}
  */

  /**
    * @}
    */

