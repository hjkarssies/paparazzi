/*
 * Copyright (C) nilay
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/percevite_wifi/percevite_wifi.h"
 * @author nilay
 * wifi ssid broadcast TODO: extend
 */

#ifndef PERCEVITE_WIFI_H
#define PERCEVITE_WIFI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mcu_periph/uart.h"

#define ESP_UART_PORT uart2

// Enter ID for drone number (1/2)
// #define SELF_ID 1 // Defined in bebop.xml
// #define SIM

#define ESP_MAX_LEN 50
#define MAX_DRONES 5  

typedef enum {
  ACK_FRAME = 0,
  DATA_FRAME
} frame_type_t;

// decoder state
typedef enum {
  ESP_SYNC = 0,
  ESP_DRONE_INFO,
  ESP_DRONE_DATA,
  ESP_ERR_CHK,  // TODO: shift to crc
  ESP_RX_OK,
  ESP_RX_ERR
} esp_state_t;

typedef struct __attribute__((packed)) {
  float x;
  float y;
} vec2f_t;

typedef struct __attribute__((packed)) {
  vec2f_t pos;
  float vel;
  float head;
} drone_data_t;

typedef struct __attribute__((packed)) {
  uint8_t drone_id;
  uint8_t packet_type;
  uint8_t packet_length;
} drone_info_t;

// encapsulated in $ and * 
typedef struct __attribute__((packed)){
  drone_info_t info; 
  drone_data_t data;
} uart_packet_t;

extern drone_data_t dr_data[MAX_DRONES];

extern void percevite_wifi_rx_event(void);
extern void percevite_wifi_init(void);
extern void percevite_wifi_tx_loop(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif

