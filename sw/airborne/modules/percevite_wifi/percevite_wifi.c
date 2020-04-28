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
| Startbytes(0-1)  | DroneID (2) | PacketType (3) | PacketLength (4) | Pos x,y (5 - 12)   | head (13-16)      |  vel (17 - 20)      | checksum (21)         |
|------------------|-------------|----------------|------------------|--------------------|-------------------|---------------------|-----------------------|
| 0x24, 0xB2 (hex) | 0x00 NA     | ACK/DATA       | 0 to 64          | 4 bytes (float)    | 4 bytes (float)   | 4 bytes (float)     | checksum (drone data) |
| UART append      | Drone Info  | Drone Info     | Drone Info       | Drone Data         | Drone Data        | Drone Data          | UART auto-append      |
 *************************************************************************************/

/* printf, fprintf */
#include <stdio.h>

/* memset, memcpy */
#include <string.h>

/* float sys get time */
#include "mcu_periph/sys_time.h"

/* gps functions */
#include "state.h"

#include "modules/percevite_wifi/percevite_wifi.h"

/* add pprz messages later */
// #include "subsystems/datalink/telemetry.h"

// #define DBG

FILE *drone_data_f;

// tx: finally send a hex array to esp32
static uint8_t send_to_esp(uint8_t *s, uint8_t len) {

	// augment start bytes
	uart_put_byte(&(ESP_UART_PORT), 0, '$');
	uart_put_byte(&(ESP_UART_PORT), 0, 178);

	uint8_t checksum = 0;

	// maximum of 255 bytes
 	uint8_t i = 0;
	for (i = 0; i < len; i ++) {
		uart_put_byte(&(ESP_UART_PORT), 0, s[i]);
		checksum += s[i];
	}	
	uart_put_byte(&(ESP_UART_PORT), 0, checksum);

	#ifdef DBG
	printf("appended checksum while bbp tx: 0x%02x\n", checksum);
	#endif

	return (i+3);
}

// tx: send struct to esp32
static void tx_struct(uart_packet_t *uart_packet_tx) {

	printf("[tx] id: %d, x: %f, y: %f, vel: %f, head: %f\n", 
		uart_packet_tx->info.drone_id,
		uart_packet_tx->data.pos.x, uart_packet_tx->data.pos.y,
		uart_packet_tx->data.vel,   uart_packet_tx->data.head);

	/* uart_packet_t = drone_info_t + drone_data_t */

	uint8_t tx_string[ESP_MAX_LEN] = {0};
	// copy packed struct into a string
	memcpy(tx_string, uart_packet_tx, sizeof(uart_packet_t));

	#ifdef DBG
	// check via wireshark
	printf("ESP SSID should be:\n");
	for (int i = 0; i < sizeof(uart_packet_t); i++) {
		printf("0x%02x,", tx_string[i]);
	}
	printf(" + checksum ****\n");
	#endif
	
	// send "stringed" struct
	send_to_esp(tx_string, sizeof(uart_packet_t));
}

// rx: print struct received after checksum match
static void print_drone_struct(uart_packet_t *uart_packet_rx) {
	printf("[rx] id: %d, x: %f, y: %f, vel: %f, head: %f\n", 
					uart_packet_rx->info.drone_id,
					uart_packet_rx->data.pos.x, uart_packet_rx->data.pos.y,
					uart_packet_rx->data.vel, uart_packet_rx->data.head);
}


// rx: parse other drone ids that are reported by esp32
static void esp_parse(uint8_t c) {

	// start state-machine for syncing to esp32
	static uint8_t esp_state = ESP_SYNC;

	// temp data-buffer to store espdata
	static uint8_t databuf[ESP_MAX_LEN] = {0};

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
				/* take note of drone id */
				drone_id = c;
				byte_ctr = byte_ctr + 1;
			} else if (byte_ctr == 3) {
				/* take note of packet type */
				packet_type = c;
				byte_ctr = byte_ctr + 1;
			} else if (byte_ctr == 4) {
				/* take note of packet length */
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

				// packet-length = 3 bytes of info + 10 bytes of data
				else if ((packet_type == DATA_FRAME) && (packet_length >= (sizeof(drone_data_t) + sizeof(drone_info_t)))) {
					// overwrite old checksum, start afresh
					checksum = drone_id + packet_type + packet_length;
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
			// start byte = 3 bytes from drone info + 2 packet sync bytes
			const uint8_t st_byte_pos = sizeof(drone_info_t) + 2;

			if (byte_ctr < packet_length) {
				/* fill a databuf from zero and calculate data+info checksum */
				databuf[byte_ctr - st_byte_pos] = c;
				checksum += databuf[byte_ctr - st_byte_pos];	
				byte_ctr = byte_ctr + 1;
			}

			/* after receiving the msg, terminate ssid string */
			if (byte_ctr == packet_length) {
				byte_ctr = 0;
				esp_state = ESP_ERR_CHK;
			}
		} break;

		case ESP_ERR_CHK: {
			/* take in the last byte and check if it matches data+info checksum */
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
			printf("[uart] received string: ");
			// print string
			for (int i = 0; i < packet_length; i++) {
				printf("0x%02x,", databuf[i]);
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
			/* hope this is atomic, vo reads from externed dr_data */
			memcpy(&dr_data[drone_id], &databuf, sizeof(drone_data_t));

			/* now revise the entire packet.. for dropout logs later */
			uart_packet_rx.data = dr_data[drone_id];
			print_drone_struct(&uart_packet_rx);
		
			/* reset state machine */
			memset(databuf, 0, ESP_MAX_LEN);
			checksum = 0;
			esp_state = ESP_SYNC;
		} break;

    case ESP_RX_ERR: {
			#ifdef DBG
			printf("[uart] ESP_RX_ERR\n");
			#endif
			memset(databuf, 0, ESP_MAX_LEN);
			byte_ctr = 0;
			checksum = 0;
			/* reset state machine, string terminated earlier than expected */
			esp_state = ESP_SYNC;
    } break;

    default: {
			memset(databuf, 0, ESP_MAX_LEN);
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

// uart rx: event based polling function for data rxed from esp32
void percevite_wifi_rx_event(void) {
	// // Look for data on serial link and send to parser
	while (uart_char_available(&(ESP_UART_PORT))) {
		uint8_t ch = uart_getch(&(ESP_UART_PORT));
		esp_parse(ch);
	}
}

// init: clear all data for all drones
static void clear_drone_status(void) {
	for (uint8_t id = 0; id < MAX_DRONES; id++) {
		// initialize all drones in the tropical waters of eastern Altanic ocean, facing the artic
		dr_data[id].pos.x = 0.0;
		dr_data[id].pos.y = 0.0;
		dr_data[id].vel   = 0.0;
		dr_data[id].head  = 0.0;
	}
}

void percevite_wifi_init() {

	drone_data_f = fopen("drone_data.csv", "w+");

	/* check pprz message for drone1 and drone2 (sort of like esp heartbeat) */
	// register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PERCEVITE_WIFI, msg_cb);

	clear_drone_status();

	/* GPS init
	// already done at GPS init
	struct LlaCoor_i *lla_ref = stateGetPositionLla_i();
	struct LtpDef_i *ltp_def;
	ltp_def_from_lla_i(&ltp_def, &lla_ref);
	stateSetLocalOrigin_i(&ltp_def);
	*/
	
	printf("-----------------Drone ID: [%d]-----------------\n", SELF_ID);

	struct LlaCoor_f *lla_ref_f = stateGetPositionLla_f();
  bool gps_valid_startup = stateIsLocalCoordinateValid();
	printf("GPS initialized with [%d] at lat: %f, long: %f \n", 
					gps_valid_startup, 
					lla_ref_f->lat * 180.0/3.142, lla_ref_f->lon * 180.0/3.142);
}

// change ssid ten times every second
void percevite_wifi_tx_loop() {
	bool gps_valid = stateIsLocalCoordinateValid();
	if (gps_valid) {
		struct NedCoor_f *gpspos = stateGetPositionNed_f();
		float gpsvel = stateGetHorizontalSpeedNorm_f();
		struct FloatEulers *att = stateGetNedToBodyEulers_f();

		#ifndef SIM
		dr_data[SELF_ID].pos.x = gpspos->x;
		dr_data[SELF_ID].pos.y = gpspos->y;
		dr_data[SELF_ID].vel   = gpsvel;   
		dr_data[SELF_ID].head  = att->psi;
		#endif

		// hacky ways to make them not null terminated...
		uart_packet_t uart_packet_tx = {
			.info = {
				.drone_id = SELF_ID,
				.packet_type = DATA_FRAME,
				.packet_length = 2 + sizeof(uart_packet_t),  // 19 byte frame + 2 sync bytes
			},
			.data = {
				.pos = {
					.x = dr_data[SELF_ID].pos.x + 0.1,
					.y = dr_data[SELF_ID].pos.y + 0.1,
				},
				.vel = dr_data[SELF_ID].vel + 0.01,
				.head = dr_data[SELF_ID].head + 0.01,
			},
		};

		/* send to esp32 for changing its ssid */
		tx_struct(&uart_packet_tx);

		// TODO: send_data_for_vo(&dr_data);

		// LOG: MAXDRONES for me = 2 (can't use ID 0x00 packet terminates..):(
		// fmt: x,y,vel,head,time
		for (int id = 1; id < 3; id++) {
			fprintf(drone_data_f, "%f,%f,", dr_data[id].pos.x, dr_data[id].pos.y);
			fprintf(drone_data_f, "%f,%f,", dr_data[id].vel, dr_data[id].head);
		}
		fprintf(drone_data_f, "%f\n", get_sys_time_float());
	}

	// mutex, don't tx to esp when ack is being sent
  // if (esp.state!= ESP_RX_OK) {
		
	//}

}