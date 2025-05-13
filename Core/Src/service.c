/**
  ******************************************************************************
  * @file    sample_service.c
  * @author  SRA Application Team
  * @brief   Add a sample service using a vendor specific profile
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "service.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "bluenrg_gatt_server.h"
#include "bluenrg_hal_aci.h"

#include "stm32l4xx_nucleo.h"

#include "sensors.h"

/* Private variables ---------------------------------------------------------*/
volatile int connected = FALSE;
volatile uint8_t set_connectable = 1;
volatile uint16_t connection_handle = 0;
volatile uint8_t notification_enabled = FALSE;
volatile uint8_t start_read_tx_char_handle = FALSE;
volatile uint8_t start_read_rx_char_handle = FALSE;
volatile uint8_t end_read_tx_char_handle = FALSE;
volatile uint8_t end_read_rx_char_handle = FALSE;

uint16_t tx_handle;
uint16_t rx_handle;

uint16_t sampleServHandle, TXCharHandle, RXCharHandle;
uint16_t babymonitorServHandle, environmentalCharHandle;

extern uint8_t bnrg_expansion_board;
extern BLE_RoleTypeDef BLE_Role;

/* Private macros ------------------------------------------------------------*/

/**
 * @brief
 * @param  None
 * @retval Status
 */
tBleStatus add_babymonitor_service(void)
{
  tBleStatus ret;

  const uint8_t babymonitor_uuid[2] = {0x66,0x9a};
  const uint8_t enviromental_uuid[2] = {0x67,0x9a};

  ret = aci_gatt_add_serv(UUID_TYPE_16, babymonitor_uuid, PRIMARY_SERVICE, 8, &babymonitorServHandle);

  if (ret != BLE_STATUS_SUCCESS) goto fail;

  ret = aci_gatt_add_char(babymonitorServHandle, UUID_TYPE_16, enviromental_uuid, 4, CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, 0,
                           16, 1, &environmentalCharHandle);

  if (ret != BLE_STATUS_SUCCESS) goto fail;

  PRINTF("BabyMonitor Service added.\n");
  return BLE_STATUS_SUCCESS;

fail:
  PRINTF("Error while adding BabyMonitor Service.\n");
  return BLE_STATUS_ERROR ;
}

/**
 * @brief  Make the device connectable
 * @param  None
 * @retval None
 */
void Make_Connection(void)
{
  tBleStatus ret;

  // Server code - update to include the baby monitor service UUID
  const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'B','a','b','y','M','o','n','i','t','o','r'};

  // Get the baby monitor UUID for advertising
  const uint8_t babymonitor_uuid[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe0,0xf2,0x73,0xd9};

  // Create a shorter representation for advertising - using just the most significant bytes
  uint8_t service_uuid_list[3];
  service_uuid_list[0] = AD_TYPE_16_BIT_SERV_UUID; // Type: Complete list of 16-bit service UUIDs
  service_uuid_list[1] = babymonitor_uuid[0];      // LSB of UUID
  service_uuid_list[2] = babymonitor_uuid[1];      // MSB of UUID

  /* disable scan response */
  hci_le_set_scan_resp_data(0, NULL);

  PRINTF("General Discoverable Mode ");
  /*
  Advertising_Event_Type, Adv_Interval_Min, Adv_Interval_Max, Address_Type, Adv_Filter_Policy,
  Local_Name_Length, Local_Name, Service_Uuid_Length, Service_Uuid_List, Slave_Conn_Interval_Min,
  Slave_Conn_Interval_Max
  */
  ret = aci_gap_set_discoverable(ADV_DATA_TYPE, ADV_INTERV_MIN, ADV_INTERV_MAX, PUBLIC_ADDR,
      NO_WHITE_LIST_USE, 12, local_name, 3, service_uuid_list, 0, 0);
  PRINTF("%d\n", ret);
}

/**
 * @brief  Sends environmental data over BLE
 * @param  env_data Pointer to environmental data structure
 * @retval None
 * @note   This function assumes the connection and notification checks have already been performed
 */
void send_environmental_data(EnvData_t* env_data)
{
  // Temperature data (requires 2 bytes according to BLE spec)
  uint8_t env_buffer[4];
  env_buffer[0] = (uint8_t)(env_data->temperature & 0xFF);        // LSB
  env_buffer[1] = (uint8_t)((env_data->temperature >> 8) & 0xFF); // MSB

  env_buffer[2] = (uint8_t)(env_data->humidity & 0xFF);        // LSB
  env_buffer[3] = (uint8_t)((env_data->humidity >> 8) & 0xFF); // MSB

  // Send envionmental data
  aci_gatt_update_char_value(babymonitorServHandle, environmentalCharHandle, 0, 4, env_buffer);

  // Print debug information
  printf("Environmental data sent - Temp: %d.%d°C, Humidity: %d.%d%%\r\n",
         env_data->temperature/10, (uint8_t)env_data->temperature%10,
         env_data->humidity/10, env_data->humidity%10);
}


/**
 * @brief  Discovery TX characteristic handle by UUID 128 bits
 * @param  None
 * @retval None
 */
void startReadTXCharHandle(void)
{
  if (!start_read_tx_char_handle)
  {
    PRINTF("Start reading TX Char Handle\n");

    const uint8_t charUuid128_TX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe1,0xf2,0x73,0xd9};
    aci_gatt_disc_charac_by_uuid(connection_handle, 0x0001, 0xFFFF, UUID_TYPE_128, charUuid128_TX);
    start_read_tx_char_handle = TRUE;
  }
}

/**
 * @brief  Discovery RX characteristic handle by UUID 128 bits
 * @param  None
 * @retval None
 */
void startReadRXCharHandle(void)
{
  if (!start_read_rx_char_handle)
  {
    PRINTF("Start reading RX Char Handle\n");

    const uint8_t charUuid128_RX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe2,0xf2,0x73,0xd9};
    aci_gatt_disc_charac_by_uuid(connection_handle, 0x0001, 0xFFFF, UUID_TYPE_128, charUuid128_RX);
    start_read_rx_char_handle = TRUE;
  }
}

/**
 * @brief  This function is used to receive data related to the sample service
 *         (received over the air from the remote board).
 * @param  data_buffer : pointer to store in received data
 * @param  Nb_bytes : number of bytes to be received
 * @retval None
 */
void receiveData(uint8_t* data_buffer, uint8_t Nb_bytes)
{
  BSP_LED_Toggle(LED2);

  for(int i = 0; i < Nb_bytes; i++) {
    printf("%c", data_buffer[i]);
  }
  fflush(stdout);
}

/**
 * @brief  This function is used to send data related to the sample service
 *         (to be sent over the air to the remote board).
 * @param  data_buffer : pointer to data to be sent
 * @param  Nb_bytes : number of bytes to send
 * @retval None
 */
void sendData(uint8_t* data_buffer, uint8_t Nb_bytes)
{
  aci_gatt_update_char_value(sampleServHandle,TXCharHandle, 0, Nb_bytes, data_buffer);
}

/**
 * @brief  Enable notification
 * @param  None
 * @retval None
 */
void enableNotification(void)
{
  uint8_t client_char_conf_data[] = {0x01, 0x00}; // Enable notifications

  uint32_t tickstart = HAL_GetTick();

  while(aci_gatt_write_charac_descriptor(connection_handle, tx_handle+2, 2, client_char_conf_data)==BLE_STATUS_NOT_ALLOWED){
    /* Radio is busy */
    if ((HAL_GetTick() - tickstart) > (10*HCI_DEFAULT_TIMEOUT_MS)) break;
  }
  notification_enabled = TRUE;
}


/**
 * @brief  This function is called when an attribute gets modified
 * @param  handle : handle of the attribute
 * @param  data_length : size of the modified attribute data
 * @param  att_data : pointer to the modified attribute data
 * @retval None
 */
void Attribute_Modified_CB(uint16_t handle, uint8_t data_length, uint8_t *att_data)
{
  if (handle == environmentalCharHandle + 2) {
    if(att_data[0] == 0x01) {
      notification_enabled = TRUE;
    }
  }
}

/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  addr : Address of peer device
 * @param  handle : Connection handle
 * @retval None
 */
void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{
  connected = TRUE;
  connection_handle = handle;

  printf("Connected to device:");
  for(int i = 5; i > 0; i--){
    printf("%02X-", addr[i]);
  }
  printf("%02X\n", addr[0]);
}

/**
 * @brief  This function is called when the peer device get disconnected.
 * @param  None
 * @retval None
 */
void GAP_DisconnectionComplete_CB(void)
{
  connected = FALSE;

  printf("Disconnected\n");
  /* Make the device connectable again. */
  set_connectable = TRUE;
  notification_enabled = FALSE;
  start_read_tx_char_handle = FALSE;
  start_read_rx_char_handle = FALSE;
  end_read_tx_char_handle = FALSE;
  end_read_rx_char_handle = FALSE;
}

/**
 * @brief  This function is called when there is a notification from the sever.
 * @param  attr_handle Handle of the attribute
 * @param  attr_len    Length of attribute value in the notification
 * @param  attr_value  Attribute value in the notification
 * @retval None
 */
void GATT_Notification_CB(uint16_t attr_handle, uint8_t attr_len, uint8_t *attr_value)
{
  if (attr_handle == tx_handle+1) {
    receiveData(attr_value, attr_len);
  }
}

/**
 * @brief  This function is called whenever there is an ACI event to be processed.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  pData  Pointer to the ACI packet
 * @retval None
 */
void user_notify(void * pData)
{
  hci_uart_pckt *hci_pckt = pData;
  /* obtain event packet */
  hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;

  if(hci_pckt->type != HCI_EVENT_PKT)
    return;

  switch(event_pckt->evt){

  case EVT_DISCONN_COMPLETE:
    {
      GAP_DisconnectionComplete_CB();
    }
    break;

  case EVT_LE_META_EVENT:
    {
      evt_le_meta_event *evt = (void *)event_pckt->data;

      switch(evt->subevent){
      case EVT_LE_CONN_COMPLETE:
        {
          evt_le_connection_complete *cc = (void *)evt->data;
          GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);
        }
        break;
      }
    }
    break;

  case EVT_VENDOR:
    {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;
      switch(blue_evt->ecode){

      case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
          evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
          Attribute_Modified_CB(evt->attr_handle, evt->data_length, evt->att_data);
        break;
      case EVT_BLUE_GATT_NOTIFICATION:
        {
          evt_gatt_attr_notification *evt = (evt_gatt_attr_notification*)blue_evt->data;
          GATT_Notification_CB(evt->attr_handle, evt->event_data_length - 2, evt->attr_value);
        }
        break;
      case EVT_BLUE_GATT_DISC_READ_CHAR_BY_UUID_RESP:
        break;

      case EVT_BLUE_GATT_PROCEDURE_COMPLETE:
        break;
      }
    }
    break;
  }
}
