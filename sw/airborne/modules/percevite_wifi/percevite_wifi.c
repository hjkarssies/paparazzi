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

/**************************************************************************************
 * 
 * | Startbyte(0) | DroneID (1-2) | North (3-8)     | East (9-14)     | Down (15-20)    | Heading (20-25) | Endbytes (26-27) |
 * |--------------|---------------|-----------------|-----------------|-----------------|-----------------|------------------|
 * | '(' (char)   | 1 bytes (int) | 6 bytes (float) | 6 bytes (float) | 6 bytes (float) | 6 bytes (float) | '**'0 (char)     |
 * |                                                                                                                         |
 * |--------------ESP-------------|--------------PPRZ----------PPRZ-----------------PPRZ----------------------PPRZ-----------|
 *  This firmware only sends the middle 4*6 bytes to ESP, header of the packet is encoded by ESP32
 * ************************************************************************************/

#include <stdio.h>
#include <string.h>

#include "modules/percevite_wifi/percevite_wifi.h"
#include "subsystems/datalink/telemetry.h"
#include "state.h"


// utility function to send a string to esp (TODO: maybe NL-CR-LF is not sent)
static void esp_send_string(char *s)
{
	uint8_t i = 0;
	while (s[i]) {
		uart_put_byte(&(ESP_UART_PORT), 0, (uint8_t)(s[i]));
		i++;
	}
	uart_put_byte(&(ESP_UART_PORT), 0, '\n');
}



static void parse_packet(int id, char *s) {
	/* CAUTION: strncpy and snprintf have different lengths and starting ptr */

	// North
	snprintf(drone_status[id].north_str, 7, "%s", s);
	drone_status[id].north_str[6] = 0;
	drone_status[id].north = strtof(drone_status[id].north_str, NULL);
	printf("North: %s, len: %d, %f\n", drone_status[id].north_str, strlen(drone_status[id].north_str), drone_status[id].north);

	// East 
	snprintf(drone_status[id].east_str, 7, "%s", &s[6]);
	drone_status[id].east_str[6] = 0;
	drone_status[id].east = strtof(drone_status[id].east_str, NULL);
	printf("East: %s, len: %d, %f\n", drone_status[id].east_str, strlen(drone_status[id].east_str), drone_status[id].east);

	// Down
	snprintf(drone_status[id].down_str, 7, "%s", &s[12]);
	drone_status[id].down_str[6] = 0;
	drone_status[id].down = strtof(drone_status[id].down_str, NULL);
	printf("Down: %s, %f\n", drone_status[id].down_str, drone_status[id].down); 

	// heading 
	snprintf(drone_status[id].heading_str, 7, "%s", &s[18]);
	drone_status[id].heading_str[6] = 0;
	drone_status[id].heading = strtof(drone_status[id].heading_str, NULL);
	printf("Heading: %s, %f\n", drone_status[id].heading_str, drone_status[id].heading);

	// char ack[4] = {'$', 'O', 'K', 0};
	// esp_send_string(ack);
}

// state machine: raw message parsing function /* struct esp_t *esp, */
static void esp_parse(char c) {

	static uint8_t byte_ctr = 0;

  // NOTE: CR and LF are two seperate chars
  // printf("esp.state: %d, char rxed: %c\n", esp.state, c);
  switch (esp.state) {
    case ESP_SYNC:
			/* first char, sync string */
			if (c == '$') {
					esp.state = ESP_ID;	
    } break;

    case ESP_ID: {  /* take note of drone ID */
					/* only accept if received char is a number */
					if (c > 47 && c < 58) {
						/* only first 2 bytes 00-99 (100) drones for now */
						char tmp_str[3] = {'0','0', '\0'};
						tmp_str[byte_ctr] = c;
						byte_ctr = byte_ctr + 1;

						/* if received two bytes after start of packet */
						if (byte_ctr == 2) {
							/* convert 3 bytes with terminated string to int */
							esp.msg.id = (uint8_t) atoi(tmp_str);

							/* reset for bytectr for next state */
							byte_ctr = 0;  

							/* switch state machine */
							esp.state = ESP_RX_MSG;
						}
					}
					else {
						esp.state = ESP_RX_ERR;
						byte_ctr = 0;
					} 
	} break;
	case ESP_RX_MSG: {  
						/* if received terminate or non number ascii before atleast 10 bytes are received, trigger ERROR */
						if (((c < 48) || (c > 57)) && (c!='.') && (c!= '-') && (byte_ctr < 10)) {
							esp.state = ESP_RX_ERR;
							byte_ctr = 0;
						}

						/* after receiving the msg, terminate ssid string */
						if ((c=='\0' || c=='\n' || c=='\r' || c=='*') && (byte_ctr > 10)) {
							esp.msg.str[byte_ctr] = '\0';
							byte_ctr = 0;
							/* received message, now switch state machine */
							esp.state = ESP_RX_OK; 
						}

						/* record ssid until full lat,long,alt are stored in esp.msg.str */
						/* don't change state machine until str terminate char is received */
						else {
							esp.msg.str[byte_ctr] = c;
							byte_ctr = byte_ctr + 1;
						}
		} break;
    case ESP_RX_OK: {
						/* string is okay, print it out and reset the state machine */
						printf("esp.state: %d, esp.msg.id: %d, esp.msg.str: %s\n", esp.state, esp.msg.id, esp.msg.str);

						/* populate drone status structure */
						parse_packet(esp.msg.id, esp.msg.str);
						
						/* reset state machine */
						esp.state = ESP_SYNC;

		} break;
    case ESP_RX_ERR: {
						printf("ESP_RX_ERR: string terminated before drone info\n");
						byte_ctr = 0;
						/* reset state machine, string terminated earlier than expected */
						esp.state = ESP_SYNC;
    } break;
    default: {
						byte_ctr = 0;
						esp.state = ESP_SYNC;
		} break;
  }
}

// event based UART polling function 
void esp_event_uart_rx(void)
{
	// Look for data on serial link and send to parser
	while (uart_char_available(&(ESP_UART_PORT))) {
		uint8_t ch = uart_getch(&(ESP_UART_PORT));
		esp_parse(ch);
	}
}


static void msg_cb(struct transport_tx *trans, struct link_device *dev) {
  
	// TODO: debug, index out of bounds, send messages for drone1 and drone2
	pprz_msg_send_PERCEVITE_WIFI(trans, dev, AC_ID, 
									strlen(drone_status[0].east_str), drone_status[0].east_str, 
									strlen(drone_status[1].east_str), drone_status[1].east_str);

	// DEBUG: 
	// printf("east_len: %d, east_str: %s\n", strlen(drone_status[0].east_str), drone_status[1].east_str);
}


static void clear_drone_status(void) {
	for (uint8_t id = 0; id < MAX_DRONES; id++) {
		// initialize at tropical waters of eastern Altanic ocean, facing the artic
		drone_status[id].north = 0;
		drone_status[id].east = 0;
		drone_status[id].down = 0;
		drone_status[id].heading = 0;
		// NULL terminate all strings 
		memset(drone_status[id].north_str,  0, 7);
		memset(drone_status[id].east_str,   0, 7);
		memset(drone_status[id].down_str,   0, 7);
		memset(drone_status[id].heading_str,0, 7);
	}
}

void uart_esp_init() {
	/* reset receive buffer state machine */
	esp.state = ESP_SYNC;

	/* check pprz message for drone1 and drone2 (sort of like esp heartbeat) */
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PERCEVITE_WIFI, msg_cb);

	clear_drone_status();
}


// frequency: 2 Hz
void uart_esp_loop() {

	// TODO: send these over instead of hardcoded
	struct NedCoor_f *optipos = stateGetPositionNed_f();
	struct NedCoor_f *optivel = stateGetSpeedNed_f();
	struct FloatEulers *att = stateGetNedToBodyEulers_f();

	// 5 NED Heading and end string
	#define TX_STRING_LEN (1 + (6*4) + 3)
	char tx_string[TX_STRING_LEN] = {0};

	// start and end bytes
	tx_string[0] = '$';
	tx_string[TX_STRING_LEN-3] = '*';
	tx_string[TX_STRING_LEN-2] = '*';
	tx_string[TX_STRING_LEN-1] = 0;

		// removebeforeflight
		static int ctr1 = 0;
		static int ctr2 = 0;
		if (ctr1 % 3 == 0) {
			ctr2 = ctr2 + 1;
		}
		ctr1 = ctr1 + 1;

	sprintf(drone_status[SELF_ID].north_str, "%06.2f", 00.0000);
	strncpy(&tx_string[1+0], drone_status[SELF_ID].north_str, 6);

	sprintf(drone_status[SELF_ID].east_str, "%06.2f", 22.53453);
	strncpy(&tx_string[1+6], drone_status[SELF_ID].east_str, 6);

		// remove before flight 
		sprintf(drone_status[SELF_ID].east_str, "%03d", ctr2);
		strncpy(&tx_string[1+6], drone_status[SELF_ID].east_str, 3);

	sprintf(drone_status[SELF_ID].down_str, "%06.2f", 33.53453);
	strncpy(&tx_string[1+12], drone_status[SELF_ID].down_str, 6);

	sprintf(drone_status[SELF_ID].heading_str, "%06.2f", 44.53453);
	strncpy(&tx_string[1+18], drone_status[SELF_ID].heading_str, 6);

	// DEBUG: 
	printf("ssid should be: %s\n", tx_string);

	// mutex, don't tx to esp when ack is being sent
  //if (esp.state!= ESP_RX_OK) {
		esp_send_string(tx_string);
	//}

}



