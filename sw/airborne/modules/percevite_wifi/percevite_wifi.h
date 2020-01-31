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

#include "mcu_periph/uart.h"

#define ESP_UART_PORT uart2

// Enter ID for drone number (0/1)
#define SELF_ID 1

#define ESP_MAX_LEN 50 // lat,long,alt,bearing = 51 bytes max (28 currently)
#define MAX_DRONES 2   // maximum drones in ESP32's range

// NULL terminate default str
// char array for sending drone lat_long_alt_bearing
// char rx_buf_str[MAX_DRONES][ESP_MAX_LEN] = {'0'};

// generic JEVOIS message structure
struct esp_msg_t {
  uint8_t type;
  uint8_t id;
  char str[ESP_MAX_LEN];
};

// decoder state
enum esp_state {
  ESP_SYNC = 0,
  ESP_ID,
  ESP_RX_MSG,
  ESP_RX_OK,
  ESP_RX_ERR
};

struct drone_status_t {
  float north;
  float east;
  float down;
  float heading;

  // reserve mid-bytes
  char north_str[7];
  char east_str[7];
  char down_str[7];
  char heading_str[7];
};

struct drone_status_t drone_status[MAX_DRONES];

// similar to jevois struct
struct esp_t {
  enum esp_state state; // decoder state
  char buf[ESP_MAX_LEN]; // temp buffer
  uint8_t idx; // temp buffer index
  struct esp_msg_t msg; // last decoded message
  bool data_available; // new data to report
};

struct esp_t esp;

extern void esp_event_uart_rx(void);
extern void uart_esp_init(void);
extern void uart_esp_loop(void);

#endif

