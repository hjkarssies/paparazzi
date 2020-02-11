/*
 * Copyright (C) nilay994
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
 * @author nilay_994@hotmail.com
 * wifi ssid broadcast
 */

/**************************************************************************************
| Startbytes(0-1)  | DroneID (2) | PacketType (3) | PacketLength (4) | Pos x,y,z (5 - 16) | heading (17 - 21) | Vel x,y,z (21 - 32) | 33                    |
|------------------|-------------|----------------|------------------|--------------------|-------------------|---------------------|-----------------------|
| 0x24, 0xB2 (hex) | 0x00 NA     | ACK/DATA       | 0 to 64          | 4 bytes (float)    | 4 bytes (float)   | 4 bytes (float)     | checksum (drone data) |
| UART append      | Drone Info  | Drone Info     | Drone Info       | Drone Data         | Drone Data        | Drone Data          | UART append           |
 * ************************************************************************************/

#include <stdio.h>
#include <string.h>

#include "modules/percevite_wifi/percevite_wifi.h"
#include "subsystems/datalink/telemetry.h"
#include "state.h"

// #define DBG

drone_data_t dr_data[MAX_DRONES];
uint8_t esp_state = ESP_SYNC;

// tx: finally send a hex array to esp32
static uint8_t esp_send_string(uint8_t *s, uint8_t len) {

	// augment start bytes
	uart_put_byte(&(ESP_UART_PORT), 0, '$');
	uart_put_byte(&(ESP_UART_PORT), 0, 178);

	uint8_t checksum = 0;

	// maximum of 255 bytes
 	uint8_t i = 0;
	for (i = 0; i < len; i ++) {
		uart_put_byte(&(ESP_UART_PORT), 0, s[i]);
		
		// TODO: remove when full checksum
		if (i > 2) {
			checksum += s[i];
		}
	}
	#ifdef DBG
		printf("appended checksum while bbp tx: 0x%02x\n", checksum);
	#endif
	
	uart_put_byte(&(ESP_UART_PORT), 0, checksum);

	return (i+3);
}

// tx: send struct to esp32
static void tx_struct(uart_packet_t *uart_packet) {

	uint8_t tx_string[ESP_MAX_LEN] = {0};

	//uart_packet_t = drone_info_t + drone_data_t;

	// copy packed struct into a string
	memcpy(tx_string, uart_packet, sizeof(uart_packet_t));

	#ifdef DBG
	printf("ssid should be:\n");
	for (int i = 0; i < sizeof(uart_packet_t); i++) {
		printf("0x%02x,", tx_string[i]);
	}
	printf("\n*******\n");
	#endif
	// send "stringed" struct
	esp_send_string(tx_string, sizeof(uart_packet_t));
	
}

// rx: print struct received after checksum match
static void print_drone_struct(uart_packet_t *uart_packet_rx) {
	// printf("info->drone_id: %d, info->packet_type: %d, info->packet_length: %d\n"
	// 			 "dat->pos.x: %f, dat->pos.y: %f, dat->pos.z: %f\n"
	// 			 "dat->heading: %f\n"
	// 			 "dat->vel.x: %f, dat->vel.y: %f, dat->vel.z: %f\n",
	// 			 	uart_packet_rx->info.drone_id, uart_packet_rx->info.packet_type, uart_packet_rx->info.packet_length,
	// 				uart_packet_rx->data.pos.x, uart_packet_rx->data.pos.y,	uart_packet_rx->data.pos.z,
	// 				uart_packet_rx->data.heading,
	// 				uart_packet_rx->data.vel.x, uart_packet_rx->data.vel.y, uart_packet_rx->data.vel.z);
	printf("info->drone_id: %d, dat->pos.x: %f\n", uart_packet_rx->info.drone_id, uart_packet_rx->data.pos.x);
}

uint8_t localbuf[ESP_MAX_LEN] = {0};
// rx: parse other drone ids that are reported by esp32
static void esp_parse(uint8_t c) {

	static uint8_t byte_ctr = 0;
	static uint8_t drone_id = 0;
	static uint8_t packet_length = 0;
	static uint8_t packet_type = 0;
	static uint8_t checksum = 0;
	static uint8_t prev_char = 0;
	
	#ifdef DBG
  printf("esp_state: %d, char rxed: 0x%02x\n", esp_state, c);
	#endif
  
	switch (esp_state) {
    case ESP_SYNC: {

			/* first char, sync string */
			if (c == '$') {
				byte_ctr = byte_ctr + 1;
    	}

			/* second char: are you really the start of the packet? */
			if ((byte_ctr == 1) && (c == 178)) {
				byte_ctr = byte_ctr + 1;
				esp_state = ESP_DRONE_INFO;
			}
		} break;

    case ESP_DRONE_INFO: {
			if (byte_ctr == 2) {
				/* take note of packet length */
				drone_id = c;
				byte_ctr = byte_ctr + 1;
			} else if (byte_ctr == 3) {
				/* take note of packet type */
				packet_type = c;
				byte_ctr = byte_ctr + 1;
			} else if (byte_ctr == 4) {
				/* take note of drone ID */
				packet_length = c;
				byte_ctr = byte_ctr + 1;

				// info frame populated!! 
				// printf("[uart] packet_length: %d, packet_type: %d, drone_id: %d\n", packet_length, packet_type, drone_id);

				if (packet_type == ACK_FRAME && packet_length == 4) {
					// TODO: esp received ssid change signal and sent you ack, 
					// indicate that on bool pprz esp ping? 
					esp_state = ESP_RX_OK;
					byte_ctr = 0;
				}

				/* packet length will always be shorter than padded struct, create some leeway */
				else if ((packet_type == DATA_FRAME) && (packet_length >= (sizeof(drone_data_t)-5))) {
					esp_state = ESP_DRONE_DATA;
				} else if (packet_length > ESP_MAX_LEN) {
					printf("[uart-err] Packet unexpectedly long \n");
					esp_state = ESP_RX_ERR;
				}	else {
					// do nothing?!
				} 
			} else {
				// do nothing?!
			}
			
		} break;

		case ESP_DRONE_DATA: {
			// TODO: remove when full checksum
			uint8_t st_byte_pos = 5;

			if (byte_ctr < packet_length) {
				/* fill a localbuf and calculate local checksum */
				localbuf[byte_ctr - st_byte_pos] = c;

				checksum += localbuf[byte_ctr - st_byte_pos];
				
				byte_ctr = byte_ctr + 1;

			}

			/* after receiving the msg, terminate ssid string */
			if (byte_ctr == packet_length) {
				byte_ctr = 0;
				esp_state = ESP_ERR_CHK;
			}
		} break;

		case ESP_ERR_CHK: {
			/* check if last packet matches your checksum */
			if (c == checksum) {
				#ifdef DBG
				printf("[uart] checksum matched!\n");
				#endif
				esp_state = ESP_RX_OK;
			}
			else {
				esp_state = ESP_RX_ERR;
			}
		} // no break statement required;

    case ESP_RX_OK: {

			#ifdef DBG
			printf("\n [uart] received string: ");
			// print string
			for (int i = 0; i<packet_length; i++) {
				printf("0x%02x,", localbuf[i]);
			}
			#endif

			/* checksum matches, proceed to populate the info struct */
			uart_packet_t uart_packet_rx = {
				.info = {
					.drone_id = drone_id,
					.packet_type = packet_type,
					.packet_length = packet_length,
				},			
			};
			/* checksum matches, proceed to populate the data struct */
			memcpy(&dr_data[drone_id], &localbuf, sizeof(drone_data_t));
			uart_packet_rx.data = dr_data[drone_id];

			print_drone_struct(&uart_packet_rx);

			checksum = 0;

			/* reset state machine */
			esp_state = ESP_SYNC;

		} break;
    case ESP_RX_ERR: {
			#ifdef DBG
			printf("[uart] ESP_RX_ERR\n");
			#endif
			byte_ctr = 0;
			checksum = 0;
			/* reset state machine, string terminated earlier than expected */
			esp_state = ESP_SYNC;
    } break;
    default: {
			byte_ctr = 0;
			checksum = 0;
			esp_state = ESP_SYNC;
		} break;
  }

	/* reset state machine asap, when start pattern is observed */
	if (prev_char == '$' && c == 178) {
		byte_ctr = 2;
		esp_state = ESP_DRONE_INFO;
  }

	/* for start byte pattern check */
	prev_char = c;

}

// rx: event based UART polling function 
void esp_event_uart_rx(void) {
	// // Look for data on serial link and send to parser
	while (uart_char_available(&(ESP_UART_PORT))) {
		uint8_t ch = uart_getch(&(ESP_UART_PORT));
		esp_parse(ch);
	}
}

// init: clear all data for all drones
static void clear_drone_status(void) {
	for (uint8_t id = 0; id < MAX_DRONES; id++) {
		// initialize at tropical waters of eastern Altanic ocean, facing the artic
		dr_data[id].pos.x = 0;
		dr_data[id].pos.y = 0;
		dr_data[id].pos.z = 0;
		dr_data[id].heading = 0;
		dr_data[id].vel.x = 0;
		dr_data[id].vel.y = 0;
		dr_data[id].vel.z = 0;
	}
}

// init: uart esp32 bbp
void uart_esp_init() {
	
	/* reset receive buffer state machine */
	esp_state = ESP_SYNC;

	/* check pprz message for drone1 and drone2 (sort of like esp heartbeat) */
	// register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PERCEVITE_WIFI, msg_cb);

	clear_drone_status();

	printf("sizeof(uart_packet_t) = %d\n", sizeof(uart_packet_t));
	printf("sizeof(drone_data_t) = %d\n", sizeof(drone_data_t));
	printf("sizeof(drone_info_t) = %d\n", sizeof(drone_info_t));
}

// frequency: twice every second
void uart_esp_loop() {

	// TODO: send these over instead of hardcoded
	struct NedCoor_f *optipos = stateGetPositionNed_f();
	struct NedCoor_f *optivel = stateGetSpeedNed_f();
	struct FloatEulers *att = stateGetNedToBodyEulers_f();

	static float ctr_trm = 1.4;

	uart_packet_t uart_packet = {
		.info = {
			.drone_id = SELF_ID,
			.packet_type = DATA_FRAME,
			.packet_length = 2 + sizeof(uart_packet_t),
		},
		.data = {
			.pos = {
				.x = -1.53,
				.y = -346.234,
				.z = 23455.234,
			},
			.heading = -452.12,
			.vel = {
				.x = -1.53,
				.y = -346.234,
				.z = 23455.234,
			},
		},
	};

	uart_packet.data.pos.x = ctr_trm;

	tx_struct(&uart_packet);

	ctr_trm = ctr_trm + 1;

	// mutex, don't tx to esp when ack is being sent
  // if (esp.state!= ESP_RX_OK) {
		
	//}	

}

/*

// TODO: checksum?
// Send an ack to esp, that other drone's data was successfully received
static uint8_t esp_send_ack(void)
{
	// start bytes
	uart_put_byte(&(ESP_UART_PORT), 0, '$');
	uart_put_byte(&(ESP_UART_PORT), 0, 178);

	uint8_t s[2] = {0x04, ACK_FRAME};
	uint8_t i = 0; // maximum of 255 bytes
	for (i=0; i<2; i++) {
		uart_put_byte(&(ESP_UART_PORT), 0, (uint8_t)(s[i]));
	}

	// end byte
	// uart_put_byte(&(ESP_UART_PORT), 0, '*');

	return (i+2);
}

// static void msg_cb(struct transport_tx *trans, struct link_device *dev) {

// 	// char buf1[10] = {0};
// 	// char buf2[10] = {0};
  
// 	// // TODO: debug, index out of bounds, send messages for drone1 and drone2
// 	// pprz_msg_send_PERCEVITE_WIFI(trans, dev, AC_ID, 
// 	// 								strlen(drone_status[0].), idrone_status[0].east_str, 
// 	// 								strlen(dr_status[1].pos.x), gcvt(dr_status[1].pos.x, 6, buf2));

// 	// DEBUG: 
// 	// printf("east_len: %d, east_str: %s\n", strlen(drone_status[0].east_str), drone_status[1].east_str);
// }

*/