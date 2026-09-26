/**
* @file main.c
* @author cristian sandu (sanducristian@gmail.com)
* @brief
* @version 0.1
* @date 09-13-2022
* @copyright Copyright (c) 2022
*/
/********** Includes ******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
/***** Free RTOS includes */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
/***** ESP based includes */
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "esp_check.h"
/***** ESP drivers includes */
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define SERVO1_GPIO          4
#define SERVO2_GPIO          5
#define SERVO3_GPIO          6

#define SERVO_FREQ_HZ        50
#define SERVO_PERIOD_US      20000

#define SERVO_MIN_US         1000
#define SERVO_CENTER_US      1500
#define SERVO_MAX_US         2000

#define SERVO_TIMER          LEDC_TIMER_0
#define SERVO_MODE           LEDC_LOW_SPEED_MODE
#define SERVO_CHANNEL        LEDC_CHANNEL_0

#define SERVO_RES            LEDC_TIMER_14_BIT
#define SERVO_MAX_DUTY       (1 << 14)

#define DUTY_RES             LEDC_TIMER_14_BIT
#define DUTY_MAX             ((1 << 14) - 1)

#define SERVO_UART           UART_NUM_1
#define SERVO_DATA_GPIO      17
#define SERVO_TX_GPIO        17 // Change to your chosen GPIO
#define SERVO_RX_GPIO        18
#define SERVO_TX_EN_GPIO     16
#define SERVO_RX_EN_GPIO     15
#define SERVO_BAUD           1000000

#define SCS_BROADCAST        0xFE
#define SCS_MAX_ID           253
#define SCS_INST_PING        0x01
#define SCS_INST_READ        0x02
#define SCS_INST_WRITE       0x03
#define SCS_TORQUE_EN        40 // 0x28
#define SCS_GOAL_POS         42 // 0x2A
#define SCS_REG_ID           5
#define SCS_REG_LOCK         48
#define SCS_RX_TIMEOUT_MS    25
#define SCS_PRESENT_POSITION 56
#define SCS_PRESENT_LEN      12
#define SCS_DISCOVERY_START  0
#define SCS_DISCOVERY_LEN    40

/********** Global defines ************************************************************************/
#define TAG "SERVO-TEST-MAIN    "


/********** Global typedefs ***********************************************************************/
typedef struct {
	uint8_t id;

	/* Command */
	uint16_t target_position;
	uint16_t target_speed;

	/* Feedback */
	uint8_t  error;
	uint16_t position;
	int16_t  speed;
	int16_t  load;
	float    voltage;
	uint8_t  temperature;
	uint8_t  moving;
	int16_t  current;

	bool online;
} scs15_servo_t;


typedef struct {
	/*
     * ID used to address the servo.
     * Set this before calling scs15_get_discovery().
     */
	uint8_t id;

	/* Identification */
	uint8_t firmware_major;
	uint8_t firmware_minor;
	uint8_t endian;
	uint8_t servo_version_major;
	uint8_t servo_version_minor;

	/* Communication */
	uint8_t  baud_code;
	uint32_t baud_rate;
	uint8_t  status_return_level;

	/* Mechanical limits */
	uint16_t min_position;
	uint16_t max_position;

	/* Protection limits */
	uint8_t  max_temperature;
	float    max_voltage;
	float    min_voltage;
	uint16_t max_torque;

	/* Configuration */
	uint8_t phase;
	uint8_t unloading_conditions;
	uint8_t led_alarm_conditions;
	uint8_t mode;

	/* Position controller */
	uint8_t position_p;
	uint8_t position_d;

	/* Motor behaviour */
	uint8_t  cw_deadband;
	uint8_t  ccw_deadband;
	uint16_t offset;

	/* Overload protection */
	uint8_t protective_torque;
	uint8_t protection_time;
	uint8_t overload_torque;

	/*
     * Keep complete raw EEPROM block.
     * Useful for firmware variants / debugging.
     */
	uint8_t raw[SCS_DISCOVERY_LEN];

	bool online;

} scs15_servo_full_t;


/********** Global forward function declarations **************************************************/
static esp_err_t scs_write_register(uint8_t id, uint8_t address, const uint8_t* data, uint8_t data_len);
static esp_err_t scs_read_status(uint8_t expected_id, uint8_t* servo_error);
static esp_err_t scs_tx_packet(uint8_t id, uint8_t instruction, const uint8_t* params, uint8_t param_len);
static esp_err_t scs_set_position_all(uint16_t position);
static esp_err_t scs_get_data(uint8_t id, scs15_servo_t* data);
static void      scs_print_data(const scs15_servo_t* servo);


static void servo_set(ledc_channel_t channel, int percent) {
	int pulse_us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * percent / 100;

	uint32_t duty = (uint32_t)pulse_us * DUTY_MAX / 20000;

	ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty));
	ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}


static void servo_set_us(uint32_t pulse_us) {
	uint32_t duty = (pulse_us * SERVO_MAX_DUTY) / SERVO_PERIOD_US;
	ESP_LOGI(TAG, "Pulse: %lu us, duty: %lu", (unsigned long)pulse_us, (unsigned long)duty);
	ESP_ERROR_CHECK(ledc_set_duty(SERVO_MODE, SERVO_CHANNEL, duty));
	ESP_ERROR_CHECK(ledc_update_duty(SERVO_MODE, SERVO_CHANNEL));
}

static void servos_set(int percent) {
	servo_set(LEDC_CHANNEL_0, percent);
	servo_set(LEDC_CHANNEL_1, percent);
	servo_set(LEDC_CHANNEL_2, percent);
}


static inline uint16_t scs_u16_be(const uint8_t* p) {
	return ((uint16_t)p[0] << 8) | p[1];
}


static uint32_t scs_baud_from_code(uint8_t code) {
	switch (code) {
	case 0:
		return 1000000;
	case 1:
		return 500000;
	case 2:
		return 250000;
	case 3:
		return 128000;
	case 4:
		return 115200;
	case 5:
		return 76800;
	case 6:
		return 57600;
	case 7:
		return 38400;
	default:
		return 0;
	}
}

static void scs15_print_discovery(
    const scs15_servo_full_t* servo) {
	if (servo == NULL) {
		return;
	}

	if (!servo->online) {
		ESP_LOGW(TAG, "Servo ID %u OFFLINE", servo->id);
		return;
	}

	ESP_LOGI(TAG, "======== SCS15 ID %u ========", servo->id);
	ESP_LOGI(TAG, "  - Firmware       : %u.%u", servo->firmware_major, servo->firmware_minor);
	ESP_LOGI(TAG, "  - Servo version  : %u.%u", servo->servo_version_major, servo->servo_version_minor);
	ESP_LOGI(TAG, "  - Endian         : %u", servo->endian);
	ESP_LOGI(TAG, "  - ID             : %u", servo->id);
	if (servo->baud_rate != 0) {
		ESP_LOGI(TAG, "  - Baud           : %lu (code %u)", (unsigned long)servo->baud_rate, servo->baud_code);
	} else {
		ESP_LOGI(TAG, "  - Baud           : unknown code %u", servo->baud_code);
	}
	ESP_LOGI(TAG, "  - Return level   : %u", servo->status_return_level);
	ESP_LOGI(TAG, "  - Position limit : %u .. %u", servo->min_position, servo->max_position);
	ESP_LOGI(TAG, "  - Temperature max: %u C", servo->max_temperature);
	ESP_LOGI(TAG, "  - Voltage range  : %.1f .. %.1f V", servo->min_voltage, servo->max_voltage);
	ESP_LOGI(TAG, "  - Max torque     : %u (%.1f%%)", servo->max_torque, servo->max_torque / 10.0f);
	ESP_LOGI(TAG, "  - Position PID   : P=%u D=%u", servo->position_p, servo->position_d);
	ESP_LOGI(TAG, "  - Deadband       : CW=%u CCW=%u", servo->cw_deadband, servo->ccw_deadband);
	ESP_LOGI(TAG, "  - Offset         : %u", servo->offset);
	ESP_LOGI(TAG, "  - Phase          : 0x%02X", servo->phase);
	ESP_LOGI(TAG, "  - Unload mask    : 0x%02X", servo->unloading_conditions);
	ESP_LOGI(TAG, "  - LED alarm mask : 0x%02X", servo->led_alarm_conditions);
	ESP_LOGI(TAG, "  - Protect torque : %u%%", servo->protective_torque);
	ESP_LOGI(TAG, "  - Protect time   : %u", servo->protection_time);
	ESP_LOGI(TAG, "  - Overload torque: %u%%", servo->overload_torque);
}

static esp_err_t scs15_get_discovery(scs15_servo_full_t* servo) {
	if (servo == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	if (servo->id > SCS_MAX_ID) {
		return ESP_ERR_INVALID_ARG;
	}

	uint8_t rx[96];
	uint8_t params[2] = {
		SCS_DISCOVERY_START,
		SCS_DISCOVERY_LEN
	};
	servo->online = false;

	// Remove stale data before starting transaction.
	esp_err_t err = uart_flush_input(SERVO_UART);
	if (err != ESP_OK) {
		ESP_LOGW(TAG, "UART RX flush failed: %s", esp_err_to_name(err));
	}

	// Send: FF FF ID 04 READ 00 28 CHECKSUM
	err = scs_tx_packet(servo->id, SCS_INST_READ, params, sizeof(params));
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Servo %u discovery request failed", servo->id);
		return err;
	}

	// Read TX echo + servo response.
	int len = uart_read_bytes(SERVO_UART, rx, sizeof(rx), pdMS_TO_TICKS(20));
	if (len <= 0) {
		ESP_LOGW(TAG, "Servo %u: no discovery response", servo->id);
		return ESP_ERR_TIMEOUT;
	}

	// ESP_LOGD(TAG, "Discovery RX: %d bytes", len);
	// ESP_LOG_BUFFER_HEXDUMP(TAG, rx, len, ESP_LOG_DEBUG);

	// Status packet length field: DATA bytes + ERROR + CHECKSUM = 40 + 2
	const uint8_t expected_length = SCS_DISCOVERY_LEN + 2;
	for (int i = 0; i < len - 5; i++) {
		if (rx[i] != 0xFF || rx[i + 1] != 0xFF) {
			continue;
		}

		if (rx[i + 2] != servo->id) {
			continue;
		}

		if (rx[i + 3] != expected_length) {
			continue;
		}

		const int packet_len = rx[i + 3] + 4;
		if ((i + packet_len) > len) {
			ESP_LOGW(TAG, "Incomplete discovery packet");
			return ESP_ERR_INVALID_SIZE;
		}

		// Verify checksum.
		uint8_t sum = 0;
		for (int j = i + 2; j < i + packet_len - 1; j++) {
			sum += rx[j];
		}

		uint8_t expected_checksum = (uint8_t)~sum;
		uint8_t received_checksum = rx[i + packet_len - 1];
		if (expected_checksum != received_checksum) {
			ESP_LOGW(TAG, "Discovery checksum error: RX=%02X expected=%02X", received_checksum, expected_checksum);
			continue;
		}

		// Status/error byte.
		uint8_t status = rx[i + 4];
		if (status != 0) {
			ESP_LOGW(TAG, "Servo %u status: 0x%02X", servo->id, status);
		}

		// EEPROM starts immediately after status byte.
		const uint8_t* data = &rx[i + 5];
		memcpy(servo->raw, data, SCS_DISCOVERY_LEN);

		// offsets from https://github.com/workloads/scservo/blob/main/src/SMS_STS.h

		// Addresses 0 .. 4
		servo->firmware_major = data[0];
		servo->firmware_minor = data[1];
		servo->endian         = data[2];

		servo->servo_version_major = data[3];
		servo->servo_version_minor = data[4];

		// Communication
		servo->id        = data[5];
		servo->baud_code = data[6];
		servo->baud_rate = scs_baud_from_code(data[6]);

		servo->status_return_level = data[8];

		// Mechanical limits
		servo->min_position = scs_u16_be(&data[9]);
		servo->max_position = scs_u16_be(&data[11]);

		// Protection
		servo->max_temperature = data[13];
		servo->max_voltage     = data[14] / 10.0f;
		servo->min_voltage     = data[15] / 10.0f;
		servo->max_torque      = scs_u16_be(&data[16]);

		// Configuration
		servo->phase                = data[18];
		servo->unloading_conditions = data[19];
		servo->led_alarm_conditions = data[20];
		servo->position_p           = data[21];
		servo->position_d           = data[22];

		// Deadbands
		servo->cw_deadband  = data[26];
		servo->ccw_deadband = data[27];
		servo->offset       = scs_u16_be(&data[31]);
		servo->mode         = data[33];

		// Overload protection
		servo->protective_torque = data[37];
		servo->protection_time   = data[38];
		servo->overload_torque   = data[39];
		servo->online            = true;
		return ESP_OK;
	}

	ESP_LOGW(TAG, "Servo %u: no valid discovery packet", servo->id);
	return ESP_ERR_NOT_FOUND;
}

static void scs_print_data(const scs15_servo_t* servo) {
	if (servo == NULL) {
		return;
	}

	if (!servo->online) {
		ESP_LOGW(TAG, "Servo ID %u OFFLINE", servo->id);
		return;
	}

	ESP_LOGI(TAG, "Servo ID %u", servo->id);
	ESP_LOGI(TAG, "  Error       : 0x%02X", servo->error);
	ESP_LOGI(TAG, "  Moving      : %u", servo->moving);
	ESP_LOGI(TAG, "  Position    : %u", servo->position);
	ESP_LOGI(TAG, "  Speed       : %d", servo->speed);
	ESP_LOGI(TAG, "  Load        : %d", servo->load);
	ESP_LOGI(TAG, "  Voltage     : %.1f V", servo->voltage);
	//ESP_LOGI(TAG, "  Current     : %.1f A", servo->current);
	ESP_LOGI(TAG, "  Temperature : %u C", servo->temperature);
}

static esp_err_t scs_get_data(uint8_t id, scs15_servo_t* servo) {
	if (!servo) {
		return ESP_ERR_INVALID_ARG;
	}

	esp_err_t err = ESP_OK;
	uint8_t   rx[64];
	uint8_t   params[2] = {
		SCS_PRESENT_POSITION,
		SCS_PRESENT_LEN
	};

	// Clear old data before sending the request. GPIO18 will also receive the TX echo.
	ESP_ERROR_CHECK(uart_flush_input(SERVO_UART));

	err = scs_tx_packet(id, SCS_INST_READ, params, sizeof(params));
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "READ request failed");
		return err;
	}

	// Read echo + servo response.
	int len = uart_read_bytes(SERVO_UART, rx, sizeof(rx), pdMS_TO_TICKS(10));
	if (len <= 0) {
		ESP_LOGW(TAG, "Servo %u: no response", id);
		servo->id     = id;
		servo->online = false;
		return ESP_ERR_TIMEOUT;
	}

	//ESP_LOGI(TAG, "RX %d bytes:", len);
	//ESP_LOG_BUFFER_HEXDUMP(TAG, rx, len, ESP_LOG_INFO);

	/* Response packet:
     * FF FF ID LENGTH ERROR DATA... CHECKSUM
     * LENGTH = number_of_data_bytes + 2
     */
	const uint8_t expected_length = SCS_PRESENT_LEN + 2;
	for (int i = 0; i + SCS_PRESENT_LEN + 6 <= len; i++) {
		if (rx[i] != 0xFF || rx[i + 1] != 0xFF) {
			continue;
		}
		if (rx[i + 2] != id) {
			continue;
		}

		// This also rejects our TX echo, because READ request LENGTH = 4.
		if (rx[i + 3] != expected_length) {
			continue;
		}

		const int packet_len = rx[i + 3] + 4;
		if ((i + packet_len) > len) {
			ESP_LOGW(TAG, "Incomplete servo response");
			return ESP_ERR_INVALID_SIZE;
		}

		// Check checksum.
		uint8_t sum = 0;
		for (int j = i + 2; j < i + packet_len - 1; j++) {
			sum += rx[j];
		}

		uint8_t expected_checksum = ~sum;
		uint8_t received_checksum = rx[i + packet_len - 1];
		if (expected_checksum != received_checksum) {
			ESP_LOGW(TAG, "Bad checksum: RX=%02X expected=%02X", received_checksum, expected_checksum);
			continue;
		}

		const uint8_t* data  = &rx[i + 5];
		servo->id            = id;
		servo->error         = rx[i + 4];
		servo->position      = ((uint16_t)data[0] << 8) | data[1];
		uint16_t speed_raw   = ((uint16_t)data[2] << 8) | data[3];
		uint16_t load_raw    = ((uint16_t)data[4] << 8) | data[5];
		servo->speed         = (speed_raw & 0x8000) ? -(int16_t)(speed_raw & 0x7FFF) : (int16_t)speed_raw;
		servo->load          = (load_raw & 0x0400) ? -(int16_t)(load_raw & 0x03FF) : (int16_t)(load_raw & 0x03FF);
		servo->voltage       = data[6] / 10.0f;
		servo->temperature   = data[7];
		servo->moving        = data[8];
		uint16_t current_raw = ((uint16_t)data[9] << 8) | data[10];
		servo->current       = (current_raw & 0x8000) ? -(int16_t)(current_raw & 0x7FFF) : (int16_t)current_raw;
		servo->online        = true;
		return ESP_OK;
	}

	ESP_LOGW(TAG, "No valid response from servo %u", id);
	return ESP_ERR_NOT_FOUND;
}


static inline esp_err_t scs_tx_enable(void) {
	esp_err_t err = gpio_output_enable(SERVO_DATA_GPIO);
	if (err != ESP_OK) {
		ESP_LOGI(TAG, "GPIO: Failed to enable the comm TX as output");
	}
	uart_set_pin(
	    SERVO_UART,
	    SERVO_TX_GPIO,
	    UART_PIN_NO_CHANGE, // keep GPIO18 RX unchanged
	    UART_PIN_NO_CHANGE,
	    UART_PIN_NO_CHANGE);
	return err;
}

static inline esp_err_t scs_tx_disable(void) {
	// TX becomes high impedance -- UART RX can still listen to the same GPIO.
	esp_err_t err = gpio_output_disable(SERVO_DATA_GPIO);
	if (err != ESP_OK) {
		ESP_LOGI(TAG, "GPIO: Failed to disable the comm TX as output");
	}
	return err;
}

static esp_err_t scs_tx_packet(uint8_t id, uint8_t instruction, const uint8_t* params, uint8_t param_len) {
	if (param_len > 25)
		return ESP_ERR_INVALID_ARG;

	const uint8_t length     = param_len + 2;
	const size_t  packet_len = param_len + 6;
	uint8_t       packet[32] = { 0xff, 0xff, id, length, instruction, 0x00 };
	uint16_t      checksum   = id + length + instruction;

	// Copy data from the params to the payload packet
	for (uint8_t i = 0; i < param_len; i++) {
		packet[5 + i] = params[i];
		checksum     += params[i];
	}
	// Set the checksum
	packet[5 + param_len] = ~(checksum & 0xFF);

	scs_tx_enable();
	const int written = uart_write_bytes(SERVO_UART, packet, packet_len);
	if (written < 0) {
		ESP_LOGE(TAG, "UART write failed");
		return ESP_FAIL;
	}
	if ((size_t)written != packet_len) {
		ESP_LOGE(TAG, "UART incomplete write: %d/%u bytes", written, (unsigned)packet_len);
		return ESP_FAIL;
	}

	esp_err_t err = uart_wait_tx_done(SERVO_UART, pdMS_TO_TICKS(20));
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "UART TX timeout: %s", esp_err_to_name(err));
		scs_tx_disable();
		return err;
	}
	scs_tx_disable();
	return ESP_OK;
}


static esp_err_t scs_write_register(uint8_t id, uint8_t address, const uint8_t* data, uint8_t data_len) {
	if (data == NULL || data_len == 0 || data_len > 24) {
		return ESP_ERR_INVALID_ARG;
	}

	uint8_t params[25] = { address, 0x00 };
	for (uint8_t i = 0; i < data_len; i++) {
		params[i + 1] = data[i];
	}

	esp_err_t err = scs_tx_packet(id, SCS_INST_WRITE, params, data_len + 1);
	if (err != ESP_OK) {
		return err;
	}

	/* A normal WRITE may generate a status packet.
     * We do not need it here because set_id() verifies the result with PING afterward.
     */
	vTaskDelay(pdMS_TO_TICKS(1));
	uart_flush_input(SERVO_UART);

	return ESP_OK;
}



static esp_err_t scs_ping(uint8_t id) {
	uint8_t   servo_error = 0;
	esp_err_t err         = ESP_OK;

	if (id > SCS_MAX_ID) {
		return ESP_ERR_INVALID_ARG;
	}

	// Remove any old status packets before sending PING.
	uart_flush_input(SERVO_UART);

	err = scs_tx_packet(id, SCS_INST_PING, NULL, 0);
	if (err != ESP_OK) {
		return err;
	}

	// Remove any old status packets before sending PING.
	uart_flush_input(SERVO_UART);

	err = scs_read_status(id, &servo_error);
	if (err != ESP_OK) {
		return err;
	}
	if (servo_error != 0) {
		ESP_LOGW(TAG, "Servo ID %u responds, status=0x%02X", id, servo_error);
		return ESP_ERR_INVALID_RESPONSE;
	}
	return ESP_OK;
}

static esp_err_t scs_discard_echo(size_t packet_len) {
	uint8_t dummy[32];

	if (packet_len > sizeof(dummy)) {
		return ESP_ERR_INVALID_ARG;
	}

	int received = uart_read_bytes(SERVO_UART, dummy, packet_len, pdMS_TO_TICKS(5));
	if (received != packet_len) {
		ESP_LOGW(TAG, "Echo length %d/%u", received, (unsigned)packet_len);
		return ESP_ERR_TIMEOUT;
	}
	return ESP_OK;
}


static esp_err_t scs_read_status(uint8_t expected_id, uint8_t* servo_error) {
	uint8_t rx[33];

	int len = uart_read_bytes(SERVO_UART, rx, sizeof(rx), pdMS_TO_TICKS(SCS_RX_TIMEOUT_MS));
	if (len == 0) {
		if (servo_error)
			*servo_error = 3;
		return ESP_ERR_TIMEOUT;
	}
	if (len < 0) {
		ESP_LOGW(TAG, "Error read bytes from UART. Length %d, status=0x%04X", len, len);
		if (servo_error)
			*servo_error = 2;
		return ESP_FAIL;
	}

	//ESP_LOGI(TAG, "Received %d bytes from device %u", len, expected_id);
	//ESP_LOG_BUFFER_HEXDUMP(TAG, rx, len, ESP_LOG_INFO);

	for (int i = 0; i <= len - 6; i++) {
		if (rx[i] != 0xFF || rx[i + 1] != 0xFF) { // Validate the header
			ESP_LOGW(TAG, "   - Packet invalid header: %02x%02x", rx[i], rx[i + 1]);
			continue;
		}

		const uint8_t id            = rx[i + 2];
		const uint8_t packet_length = rx[i + 3];
		if (id != expected_id) { // Validate that it is the expected ID
			ESP_LOGW(TAG, "   - Packet invalid ID: %02x and length %u", rx[i + 2], rx[i + 3]);
			continue;
		}

		const int total_length = packet_length + 4;
		if ((i + total_length) > len) { // Check if we have the entire packet read, if failed then just exit
			ESP_LOGW(TAG, "   - Packet too large %u", rx[i + 3]);
			continue;
		}

		// Let's verify the check sum
		uint16_t checksum = 0;
		for (int j = i + 2; j < i + total_length - 1; j++) {
			checksum += rx[j];
		}
		uint8_t expected_checksum = ~(checksum & 0xFF);
		uint8_t received_checksum = rx[i + total_length - 1];
		if (expected_checksum != received_checksum) { // Check the checksum to determine any transmission errors
			ESP_LOGW(TAG, "Bad checksum from ID %u", id);
			continue;
		}
		if (servo_error != NULL) {
			*servo_error = rx[i + 4];
		}
		return ESP_OK;
	}
	ESP_LOGW(TAG, "Bad responde from ID %u", expected_id);
	return ESP_ERR_INVALID_RESPONSE;
}



static esp_err_t scs_set_id(uint8_t old_id, uint8_t new_id) {
	if (old_id > SCS_MAX_ID || new_id > SCS_MAX_ID) {
		return ESP_ERR_INVALID_ARG;
	}

	if (old_id == new_id) {
		ESP_LOGI(TAG, "Servo already has ID %u", old_id);
		return ESP_OK;
	}

	// Make sure old servo exists.
	if (scs_ping(old_id) != ESP_OK) {
		ESP_LOGE(TAG, "Servo ID %u not found", old_id);
		return ESP_ERR_NOT_FOUND;
	}

	// Do not create an ID collision.
	if (scs_ping(new_id) == ESP_OK) {
		ESP_LOGE(TAG, "ID %u already exists", new_id);
		return ESP_ERR_INVALID_STATE;
	}

	ESP_LOGI(TAG, "Changing servo ID %u -> %u", old_id, new_id);

	// Unlock EEPROM.
	uint8_t unlock = 0;
	ESP_RETURN_ON_ERROR(scs_write_register(old_id, SCS_REG_LOCK, &unlock, 1), TAG, "Failed to unlock EEPROM");

	// Change ID.
	ESP_RETURN_ON_ERROR(scs_write_register(old_id, SCS_REG_ID, &new_id, 1), TAG, "Failed to write new ID");

	// Servo now responds using NEW ID.
	uint8_t lock = 1;
	ESP_RETURN_ON_ERROR(scs_write_register(new_id, SCS_REG_LOCK, &lock, 1), TAG, "Failed to lock EEPROM");
	vTaskDelay(pdMS_TO_TICKS(10));

	// Verify new ID.
	if (scs_ping(new_id) != ESP_OK) {
		ESP_LOGE(TAG, "ID change verification failed");
		return ESP_FAIL;
	}

	ESP_LOGI(TAG, "Servo ID changed successfully: %u -> %u", old_id, new_id);
	return ESP_OK;
}


static int scs_scan(void) {
	int                found = 0;
	scs15_servo_full_t servo;
	ESP_LOGI(TAG, "Scanning SCS bus...");
	for (uint16_t id = 0; id <= SCS_MAX_ID; id++) {
		esp_err_t err = scs_ping((uint8_t)id);
		if (err == ESP_OK) {
			ESP_LOGI(TAG, "Found servo ID %u", id);
			servo.id = id;
			scs15_get_discovery(&servo);
			scs15_print_discovery(&servo);
			found++;
		}
	}

	ESP_LOGI(TAG, "Scan complete: %d servo(s) found", found);
	return found;
}



static esp_err_t scs_write(uint8_t address, const uint8_t* data, uint8_t data_len) {
	if (data == NULL || data_len > 25) {
		ESP_LOGE(TAG, "Invalid SCS write arguments");
		return ESP_ERR_INVALID_ARG;
	}
	uint8_t  length     = data_len + 3;
	uint8_t  packet[32] = { 0xff, 0xff, SCS_BROADCAST, length, SCS_INST_WRITE, address };
	uint16_t checksum   = SCS_BROADCAST + length + SCS_INST_WRITE + address;

	// Copy the payload to the payload packet
	for (int i = 0; i < data_len; i++) {
		packet[6 + i] = data[i];
		checksum     += data[i];
	}

	packet[6 + data_len] = ~(checksum & 0xFF);
	int packet_len       = 7 + data_len;

	// Activate the bus
	scs_tx_enable();

	const int written = uart_write_bytes(SERVO_UART, packet, packet_len);
	if (written < 0) {
		ESP_LOGE(TAG, "UART write failed");
		return ESP_FAIL;
	}
	if ((size_t)written != packet_len) {
		ESP_LOGE(TAG, "UART incomplete write: %d/%u bytes", written, (unsigned)packet_len);
		return ESP_FAIL;
	}

	esp_err_t err = uart_wait_tx_done(SERVO_UART, pdMS_TO_TICKS(20));
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "UART TX completion failed: %s", esp_err_to_name(err));
		scs_tx_disable();
		return err;
	}
	scs_tx_disable();
	return ESP_OK;
}



static esp_err_t scs_enable_torque(void) {
	uint8_t enable = 1;
	return scs_write(SCS_TORQUE_EN, &enable, 1);
}

static esp_err_t scs_set_position(uint8_t id, uint16_t position) {
	if (id > SCS_MAX_ID || position > 1023) {
		return ESP_ERR_INVALID_ARG;
	}
	const uint16_t speed   = 1000;
	uint8_t        data[6] = {
		(position >> 8) & 0xFF, position & 0xFF,
		0x00, 0x00,
		(speed >> 8) & 0xFF, speed & 0xFF
	};

	return scs_write_register(id, SCS_GOAL_POS, data, sizeof(data));
}


static esp_err_t scs_set_position_all(uint16_t position) {
	// Saturate position
	if (position > 1023) {
		position = 1023;
	}

	/* SCSCL uses high byte first.
     *
     * Registers:
     * 42-43 position
     * 44-45 time
     * 46-47 speed
     */

	uint16_t speed   = 1000;
	uint8_t  data[6] = {
		(position >> 8) & 0xFF, position & 0xFF,
		0x00, 0x00, // Time
		(speed >> 8) & 0xFF, speed & 0xFF
	};

	esp_err_t err = scs_write(SCS_GOAL_POS, data, sizeof(data));
	if (err == ESP_OK) {
		ESP_LOGI(TAG, "Position = %u", position);
	} else {
		ESP_LOGE(TAG, "ERROR. Failed to set position to %u", position);
	}
	return err;
}

void app_main(void) {
	ledc_timer_config_t timer = {
		.speed_mode      = SERVO_MODE,
		.timer_num       = SERVO_TIMER,
		.duty_resolution = SERVO_RES,
		.freq_hz         = SERVO_FREQ_HZ,
		.clk_cfg         = LEDC_AUTO_CLK,
	};
	uart_config_t uart_config = {
		.baud_rate  = SERVO_BAUD,
		.data_bits  = UART_DATA_8_BITS,
		.parity     = UART_PARITY_DISABLE,
		.stop_bits  = UART_STOP_BITS_1,
		.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT,
	};

	const int gpio[3] = {
		SERVO1_GPIO,
		SERVO2_GPIO,
		SERVO3_GPIO
	};

	ESP_LOGI(TAG, "Initialise MCU modules ...");


	//*****************************************************
	// Step 0-C: Show FreeRTOS parameters
	//*****************************************************
#define TAG_FOS "FreeRTOS"
	ESP_LOGI(TAG_FOS, "configUSE_PREEMPTION                 : %7d", configUSE_PREEMPTION);
	ESP_LOGI(TAG_FOS, "configUSE_PORT_OPTIMISED_TASK_SELECTION : %7d", configUSE_PORT_OPTIMISED_TASK_SELECTION);
	ESP_LOGI(TAG_FOS, "configTICK_RATE_HZ                   : %7d", configTICK_RATE_HZ);
	ESP_LOGI(TAG_FOS, "configMAX_PRIORITIES                 : %7d", configMAX_PRIORITIES);
	ESP_LOGI(TAG_FOS, "configMINIMAL_STACK_SIZE             : %7d", configMINIMAL_STACK_SIZE);
	ESP_LOGI(TAG_FOS, "configMAX_TASK_NAME_LEN              : %7d", configMAX_TASK_NAME_LEN);
	ESP_LOGI(TAG_FOS, "configUSE_16_BIT_TICKS               : %7d", configUSE_16_BIT_TICKS);
	ESP_LOGI(TAG_FOS, "configIDLE_SHOULD_YIELD              : %7d", configIDLE_SHOULD_YIELD);
	ESP_LOGI(TAG_FOS, "configUSE_CO_ROUTINES                : %7d", configUSE_CO_ROUTINES);
	ESP_LOGI(TAG_FOS, "configUSE_TASK_NOTIFICATIONS         : %7d", configUSE_TASK_NOTIFICATIONS);
	ESP_LOGI(TAG_FOS, "configTASK_NOTIFICATION_ARRAY_ENTRIES: %7d", configTASK_NOTIFICATION_ARRAY_ENTRIES);
	ESP_LOGI(TAG_FOS, "configUSE_MUTEXES                    : %7d", configUSE_MUTEXES);
	ESP_LOGI(TAG_FOS, "configUSE_RECURSIVE_MUTEXES          : %7d", configUSE_RECURSIVE_MUTEXES);
	ESP_LOGI(TAG_FOS, "configUSE_COUNTING_SEMAPHORES        : %7d", configUSE_COUNTING_SEMAPHORES);
	ESP_LOGI(TAG_FOS, "configUSE_ALTERNATIVE_API            : %7d", configUSE_ALTERNATIVE_API);
	ESP_LOGI(TAG_FOS, "configQUEUE_REGISTRY_SIZE            : %7d", configQUEUE_REGISTRY_SIZE);
	ESP_LOGI(TAG_FOS, "configUSE_QUEUE_SETS                 : %7d", configUSE_QUEUE_SETS);
	ESP_LOGI(TAG_FOS, "configUSE_TIME_SLICING               : %7d", configUSE_TIME_SLICING);
	ESP_LOGI(TAG_FOS, "configUSE_NEWLIB_REENTRANT           : %7d", configUSE_NEWLIB_REENTRANT);
	ESP_LOGI(TAG_FOS, "configENABLE_BACKWARD_COMPATIBILITY  : %7d", configENABLE_BACKWARD_COMPATIBILITY);
	ESP_LOGI(TAG_FOS, "configNUM_THREAD_LOCAL_STORAGE_POINTERS : %7d", configNUM_THREAD_LOCAL_STORAGE_POINTERS);
	/* Memory allocation related definitions. */
	ESP_LOGI(TAG_FOS, "configSUPPORT_STATIC_ALLOCATION      : %7d", configSUPPORT_STATIC_ALLOCATION);
	ESP_LOGI(TAG_FOS, "configSUPPORT_DYNAMIC_ALLOCATION     : %7d", configSUPPORT_DYNAMIC_ALLOCATION);
	//ESP_LOGI(TAG_FOS, "configTOTAL_HEAP_SIZE              : %7d", configTOTAL_HEAP_SIZE);
	ESP_LOGI(TAG_FOS, "configAPPLICATION_ALLOCATED_HEAP     : %7d", configAPPLICATION_ALLOCATED_HEAP);
	ESP_LOGI(TAG_FOS, "configSTACK_ALLOCATION_FROM_SEPARATE_HEAP : %7d", configSTACK_ALLOCATION_FROM_SEPARATE_HEAP);
	/* Hook function related definitions. */
	ESP_LOGI(TAG_FOS, "configUSE_IDLE_HOOK                  : %7d", configUSE_IDLE_HOOK);
	ESP_LOGI(TAG_FOS, "configUSE_TICK_HOOK                  : %7d", configUSE_TICK_HOOK);
	ESP_LOGI(TAG_FOS, "configCHECK_FOR_STACK_OVERFLOW       : %7d", configCHECK_FOR_STACK_OVERFLOW);
	ESP_LOGI(TAG_FOS, "configUSE_MALLOC_FAILED_HOOK         : %7d", configUSE_MALLOC_FAILED_HOOK);
	ESP_LOGI(TAG_FOS, "configUSE_DAEMON_TASK_STARTUP_HOOK   : %7d", configUSE_DAEMON_TASK_STARTUP_HOOK);

	ESP_LOGI(TAG_FOS, "configUSE_TIMERS                     : %7d", configUSE_TIMERS);
	ESP_LOGI(TAG_FOS, "configTIMER_TASK_PRIORITY            : %7d", configTIMER_TASK_PRIORITY);
	ESP_LOGI(TAG_FOS, "configTIMER_QUEUE_LENGTH             : %7d", configTIMER_QUEUE_LENGTH);
	ESP_LOGI(TAG_FOS, "configTIMER_TASK_STACK_DEPTH         : %7d", configTIMER_TASK_STACK_DEPTH);
	ESP_LOGI(TAG_FOS, "configKERNEL_INTERRUPT_PRIORITY      : %7d", configKERNEL_INTERRUPT_PRIORITY);
	//ESP_LOGI(TAG_FOS, "configMAX_SYSCALL_INTERRUPT_PRIORITY : %7d", configMAX_SYSCALL_INTERRUPT_PRIORITY);


	ESP_LOGI(TAG, "Configuring timers ...");
	ESP_ERROR_CHECK(ledc_timer_config(&timer));


	ESP_LOGI(TAG, "Configuring UART ...");
	ESP_ERROR_CHECK(uart_driver_install(SERVO_UART, 256, 0, 0, NULL, 0));
	ESP_ERROR_CHECK(uart_param_config(SERVO_UART, &uart_config));
	ESP_ERROR_CHECK(uart_set_pin(
	    SERVO_UART,
	    SERVO_TX_GPIO,
	    SERVO_RX_GPIO,
	    UART_PIN_NO_CHANGE,
	    UART_PIN_NO_CHANGE));


	for (int i = 0; i < 3; i++) {
		ledc_channel_config_t ch = {
			.gpio_num   = gpio[i],
			.speed_mode = LEDC_LOW_SPEED_MODE,
			.channel    = LEDC_CHANNEL_0 + i,
			.timer_sel  = LEDC_TIMER_0,
			.duty       = 0,
			.hpoint     = 0
		};

		ESP_ERROR_CHECK(ledc_channel_config(&ch));
	}

	ESP_LOGI(TAG, "SCS15 brodcast enable torque ");
	vTaskDelay(pdMS_TO_TICKS(500));
	scs_enable_torque();


	ESP_LOGI(TAG, "SCS15 start scanning ... ");
	scs_scan();


	ESP_LOGI(TAG, "Servo test started on GPIO %d", SERVO1_GPIO);
	// scs_set_position(3, 0);
	// vTaskDelay(pdMS_TO_TICKS(500));
	// scs_set_position(3, 1000);
	// vTaskDelay(pdMS_TO_TICKS(500));
	// scs_set_position(3, 0);
	// vTaskDelay(pdMS_TO_TICKS(500));


	// uint8_t id = 0;
	// while (1) {
	// 	ESP_LOGI(TAG, "SCS15 cycle for ID: %u", id);
	// 	scs_set_position(id, 0);
	// 	vTaskDelay(pdMS_TO_TICKS(500));

	// 	scs_set_position(id, 1000);
	// 	vTaskDelay(pdMS_TO_TICKS(500));
	// 	id++;
	// 	if (id == 255)
	// 		break;
	// }

	while (1) {
		scs15_servo_t servo3;

		ESP_LOGI(TAG, "Toggle");
		servos_set(0);
		scs_set_position_all(0);
		servo_set_us(SERVO_MIN_US);
		vTaskDelay(pdMS_TO_TICKS(1000));

		servos_set(50);
		scs_set_position_all(512);
		servo_set_us(SERVO_CENTER_US);
		vTaskDelay(pdMS_TO_TICKS(1000));

		servos_set(100);
		scs_set_position_all(1023);
		servo_set_us(SERVO_MAX_US);
		vTaskDelay(pdMS_TO_TICKS(1000));

		scs_get_data(3, &servo3);
		scs_print_data(&servo3);

		servos_set(50);
		scs_set_position_all(512);
		servo_set_us(SERVO_CENTER_US);
		vTaskDelay(pdMS_TO_TICKS(1000));

		scs_get_data(3, &servo3);
		scs_print_data(&servo3);

		servos_set(0);
		scs_set_position_all(0);
		servo_set_us(SERVO_MIN_US);
		vTaskDelay(pdMS_TO_TICKS(1000));

		scs_get_data(3, &servo3);
		scs_print_data(&servo3);

		vTaskDelay(pdMS_TO_TICKS(10000));
	}
}
