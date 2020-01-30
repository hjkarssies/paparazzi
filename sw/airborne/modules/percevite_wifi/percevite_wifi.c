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
 * @file "modules/percevite_wifi/percevite_wifi.c"
 * @author nilay
 * wifi ssid broadcast
 */

#include "modules/percevite_wifi/percevite_wifi.h"
#include "subsystems/datalink/telemetry.h"
#include <stdio.h>

uint8_t str1_len = 50;
uint8_t str2_len = 50;
char str1[50] = {};
char str2[50] = {};

static void msg_cb(struct transport_tx *trans, struct link_device *dev) {
  pprz_msg_send_PERCEVITE_WIFI(trans, dev, AC_ID, str1_len, str1, str2_len, str2);
  printf("drone1: %s, drone2: %s\n", str1, str2);
}

void uart_esp_init() {
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PERCEVITE_WIFI, msg_cb);
}
void uart_esp_loop() {}



