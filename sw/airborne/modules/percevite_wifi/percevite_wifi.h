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
 * wifi ssid broadcast
 */

#ifndef PERCEVITE_WIFI_H
#define PERCEVITE_WIFI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mcu_periph/uart.h"

#define ESP_UART_PORT uart2

// Enter ID for drone number (0/1)
#define SELF_ID 3

#define ESP_MAX_LEN 50 // lat,long,alt,bearing = 51 bytes max (28 currently)
#define MAX_DRONES 5   // maximum drones in ESP32's range

typedef enum {
  ACK_FRAME = 0,
  DATA_FRAME
} frame_type_t;

// decoder state
typedef enum {
  ESP_SYNC = 0,
  ESP_DRONE_INFO,
  ESP_DRONE_DATA,
  ESP_ERR_CHK,
  ESP_RX_OK,
  ESP_RX_ERR
} esp_state_t;

typedef struct __attribute__((packed)) {
  float x;
  float y;
  float z;
} vec3f_t;

typedef struct __attribute__((packed)) {
  vec3f_t pos;
  float heading;
  vec3f_t vel;
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

// struct drone_status_t drone_status[MAX_DRONES];

extern void esp_event_uart_rx(void);
extern void uart_esp_init(void);
extern void uart_esp_loop(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif

