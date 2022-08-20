/*
 * CAN_Transmit_Receive.c
 *
 *  Created on: 11 juli 2021
 *      Author: Daniel Mårtensson
 */

/* Layer */
#include "Hardware.h"
#include "FreeRTOS.h"
#include  "semphr.h"
#include "peripheral/can/plib_can2.h"


/* This is a call back function e.g listener, that will be called once SAE J1939 data is going to be sent */
static void (*Callback_Function_Send)(uint32_t, uint8_t, uint8_t[]);
static void (*Callback_Function_Read)(uint32_t *, uint8_t[], bool *);

/* Platform independent library headers for CAN */
#if PROCESSOR_CHOICE == STM32
#include "main.h"
#elif PROCESSOR_CHOICE == ARDUINO
#elif PROCESSOR_CHOICE == PIC

static void CAN_callback(uintptr_t context);

#define CAN_FIFO_NUMBER_RECEIVE 1

#define CAN_FIFO_NUMBER_TRANSMIT 0

#define CAN_FIFO_NUMBER_REQUEST 2


#elif PROCESSOR_CHOICE == AVR
#elif PROCESSOR_CHOICE == QT_USB
#include "CAN_to_USB/can_to_usb.h"
#elif PROCESSOR_CHOICE == INTERNAL_CALLBACK
/* Nothing here because else statement should not be running */
#else
/* Internal fields */
static bool internal_new_message[256] = {false};
static uint8_t internal_data[256 * 8] = {0};
static uint8_t internal_DLC[256] = {0};
static uint32_t internal_ID[256] = {0};
static uint8_t buffer_index_transmit = 0;
static uint8_t buffer_index_receive = 0;

/* Internal functions */
static ENUM_J1939_STATUS_CODES Internal_Transmit(uint32_t ID, uint8_t data[], uint8_t DLC) {
    internal_ID[buffer_index_transmit] = ID;
    internal_DLC[buffer_index_transmit] = DLC;
    for (uint8_t i = 0; i < 8; i++)
        if (i < DLC)
            internal_data[buffer_index_transmit * 8 + i] = data[i];
        else
            internal_data[buffer_index_transmit * 8 + i] = 0x0;
    internal_new_message[buffer_index_transmit] = true;
    buffer_index_transmit++;                                    /* When this is 256, then it will be come 0 again */
    return STATUS_SEND_OK;
}

static void Internal_Receive(uint32_t *ID, uint8_t data[], bool *is_new_message) {
    /* Do a quick check if we are going to read message that have no data */
    if (internal_new_message[buffer_index_receive] == false) {
        *is_new_message = false;
        return;
    }

    *ID = internal_ID[buffer_index_receive];
    for (uint8_t i = 0; i < 8; i++)
        if (i < internal_DLC[buffer_index_receive])
            data[i] = internal_data[buffer_index_receive * 8 + i];
    *is_new_message = internal_new_message[buffer_index_receive];
    /* Reset */
    internal_new_message[buffer_index_receive] = false;
    internal_DLC[buffer_index_receive] = 0;
    buffer_index_receive++;                                     /* When this is 256, then it will be come 0 again */
}

#endif

ENUM_J1939_STATUS_CODES CAN_Send_Message(uint32_t ID, uint8_t data[]) {
    ENUM_J1939_STATUS_CODES status;
    #if PROCESSOR_CHOICE == STM32
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.DLC = 8;                                           /* Here we are sending 8 bytes */
    TxHeader.RTR = CAN_RTR_DATA;                                /* Data frame */
    TxHeader.IDE = CAN_ID_EXT;                                  /* We want to send an extended ID */
    TxHeader.TransmitGlobalTime = DISABLE;
    TxHeader.ExtId = ID;
    TxHeader.StdId = 0x00;                                      /* Not used */
    status = STM32_PLC_CAN_Transmit(data, &TxHeader);
    #elif PROCESSOR_CHOICE == ARDUINO
    /* Implement your CAN send 8 bytes message function for the Arduino platform */
    #elif PROCESSOR_CHOICE == PIC
    /* Implement your CAN send 8 bytes message function for the PIC platform */
    static SemaphoreHandle_t CAN_transmit_semaphore = NULL;
    if (CAN_transmit_semaphore == NULL) {
        CAN_transmit_semaphore = xSemaphoreCreateBinary();
        configASSERT(CAN_transmit_semaphore != NULL);
        CAN2_CallbackRegister(CAN_callback, (uintptr_t) CAN_transmit_semaphore, CAN_FIFO_NUMBER_TRANSMIT);
    }
    if (CAN2_MessageTransmit(ID, 8, data, CAN_FIFO_NUMBER_TRANSMIT, CAN_MSG_TX_DATA_FRAME) == false) {
        vTaskDelay(pdMS_TO_TICKS(10));
        status = STATUS_SEND_BUSY;
	 
    }
    else {
        if(xSemaphoreTake(CAN_transmit_semaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
            status = STATUS_SEND_OK;
        }
        else {
            status = STATUS_SEND_TIMEOUT;
        }
        
    }
    #elif PROCESSOR_CHOICE == AVR
    /* Implement your CAN send 8 bytes message function for the AVR platform */
    #elif PROCESSOR_CHOICE == QT_USB
    status = QT_USB_Transmit(ID, data, 8);
    #elif PROCESSOR_CHOICE == INTERNAL_CALLBACK
    /* Call our callback function */
    Callback_Function_Send(ID, 8, data);
    status = STATUS_SEND_OK;
    #else
    /* If no processor are used, use internal feedback for debugging */
    status = Internal_Transmit(ID, data, 8);
    #endif
    return status;
}

/* Send a PGN request
 * PGN: 0x00EA00 (59904)
 */
ENUM_J1939_STATUS_CODES CAN_Send_Request(uint32_t ID, uint8_t PGN[]) {
    ENUM_J1939_STATUS_CODES status;
    #if PROCESSOR_CHOICE == STM32
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.DLC = 3;                                           /* Here we are only sending 3 bytes */
    TxHeader.RTR = CAN_RTR_DATA;                                /* Data frame */
    TxHeader.IDE = CAN_ID_EXT;                                  /* We want to send an extended ID */
    TxHeader.TransmitGlobalTime = DISABLE;
    TxHeader.ExtId = ID;
    TxHeader.StdId = 0x00;                                      /* Not used */
    status = STM32_PLC_CAN_Transmit(PGN, &TxHeader);
    #elif PROCESSOR_CHOICE == ARDUINO
    /* Implement your CAN send 3 bytes message function for the Arduino platform */
    #elif PROCESSOR_CHOICE == PIC
    static SemaphoreHandle_t CAN_request_semaphore = NULL;
    if (CAN_request_semaphore == NULL) {
        CAN_request_semaphore = xSemaphoreCreateBinary();
        configASSERT(CAN_request_semaphore != NULL);
        CAN2_CallbackRegister(CAN_callback, (uintptr_t) CAN_request_semaphore, CAN_FIFO_NUMBER_REQUEST);
    }
    if (CAN2_MessageTransmit(ID, 8, PGN, CAN_FIFO_NUMBER_REQUEST, CAN_MSG_TX_DATA_FRAME) == false) {
        vTaskDelay(pdMS_TO_TICKS(10));
        status = STATUS_SEND_BUSY;
	 
    }
    else {
        if(xSemaphoreTake(CAN_request_semaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
            status = STATUS_SEND_OK;
        }
        else {
            status = STATUS_SEND_TIMEOUT;
        }
        
    }
    /* Implement your CAN send 3 bytes message function for the PIC platform */
    #elif PROCESSOR_CHOICE == AVR
    /* Implement your CAN send 3 bytes message function for the AVR platform */
    #elif PROCESSOR_CHOICE == QT_USB
    status = QT_USB_Transmit(ID, PGN, 3);                       /* PGN is always 3 bytes */
    #elif PROCESSOR_CHOICE == INTERNAL_CALLBACK
    /* Call our callback function */
    Callback_Function_Send(ID, 3, PGN);
    status = STATUS_SEND_OK;
    #else
    /* If no processor are used, use internal feedback for debugging */
    status = Internal_Transmit(ID, PGN, 3);
    #endif
    return status;
}

/* Read the current CAN-bus message. Returning false if the message has been read before, else true */
bool CAN_Read_Message(uint32_t *ID, uint8_t data[]) {
    bool is_new_message;
    #if PROCESSOR_CHOICE == STM32
    STM32_PLC_CAN_Get_ID_Data(ID, data, &is_new_message);
    #elif PROCESSOR_CHOICE == ARDUINO
    /* Implement your CAN function to get ID, data[] and the flag is_new_message here for the Arduino platform */
    #elif PROCESSOR_CHOICE == PIC
    is_new_message = false;
    static CAN_MSG_RX_ATTRIBUTE msgAttr = CAN_MSG_RX_DATA_FRAME;
    static SemaphoreHandle_t CAN_receive_semaphore = NULL;
    if (CAN_receive_semaphore == NULL) {
        CAN_receive_semaphore = xSemaphoreCreateBinary();
        configASSERT(CAN_receive_semaphore != NULL);
        CAN2_CallbackRegister(CAN_callback, (uintptr_t) CAN_receive_semaphore, CAN_FIFO_NUMBER_RECEIVE);
    }
    uint8_t rx_messageLength = 0;
    uint16_t timestamp = 0;
    if (CAN2_MessageReceive(ID, &rx_messageLength, data, &timestamp, CAN_FIFO_NUMBER_RECEIVE, &msgAttr) == true) {
        if (xSemaphoreTake(CAN_receive_semaphore, portMAX_DELAY) == pdTRUE) {
            is_new_message = true;
        }
    }
    else {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    #elif PROCESSOR_CHOICE == AVR
    /* Implement your CAN function to get ID, data[] and the flag is_new_message here for the AVR platform */
    #elif PROCESSOR_CHOICE == QT_USB
    QT_USB_Get_ID_Data(ID, data, &is_new_message);
    #elif PROCESSOR_CHOICE == INTERNAL_CALLBACK
    Callback_Function_Read(ID, data, &is_new_message);
    #else
    /* If no processor are used, use internal feedback for debugging */
    Internal_Receive(ID, data, &is_new_message);
    #endif
    return is_new_message;
}

void CAN_Set_Callback_Functions(void (*Callback_Function_Send_)(uint32_t, uint8_t, uint8_t[]), void (*Callback_Function_Read_)(uint32_t *, uint8_t[], bool *)) {
    Callback_Function_Send = Callback_Function_Send_;
    Callback_Function_Read = Callback_Function_Read_;
}

static void CAN_callback(uintptr_t context) {
    BaseType_t HigherPriorityTaskWoken;

    xSemaphoreGiveFromISR((SemaphoreHandle_t) context, &HigherPriorityTaskWoken);
    if (HigherPriorityTaskWoken == pdTRUE) {
        portEND_SWITCHING_ISR(HigherPriorityTaskWoken);
    }
}
