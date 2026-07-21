/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "CrsfRc.hpp"
#include "CrsfParser.hpp"
#include "Crc8.hpp"

#include <cstdio>
#include <fcntl.h>

#include <uORB/topics/debug_value.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

#define CRSF_BAUDRATE 420000

CrsfRc::CrsfRc(const char *device) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(device))
{
	if (device) {
		strncpy(_device, device, sizeof(_device) - 1);
		_device[sizeof(_device) - 1] = '\0';
	}
}

CrsfRc::~CrsfRc()
{
	perf_free(_cycle_interval_perf);
	perf_free(_publish_interval_perf);
}

int CrsfRc::task_spawn(int argc, char *argv[])
{
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	const char *device_name = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_name = myoptarg;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return PX4_ERROR;
	}

	if (!device_name) {
		PX4_ERR("Valid device required");
		return PX4_ERROR;
	}

	CrsfRc *instance = new CrsfRc(device_name);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	instance->ScheduleNow();

	return PX4_OK;
}

void CrsfRc::Run()
{
	if (should_exit()) {
		ScheduleClear();

		if (_uart) {
			(void) _uart->close();
			delete _uart;
			_uart = nullptr;
		}

		exit_and_cleanup();
		return;
	}

	if (_uart == nullptr) {
		// Create the UART port instance
		_uart = new Serial(_device);

		if (_uart == nullptr) {
			PX4_ERR("Error creating serial device %s", _device);
			px4_sleep(1);
			return;
		}
	}

	if (! _uart->isOpen()) {
		// Configure the desired baudrate if one was specified by the user.
		// Otherwise the default baudrate will be used.
		if (! _uart->setBaudrate(CRSF_BAUDRATE)) {
			PX4_ERR("Error setting baudrate to %u on %s", CRSF_BAUDRATE, _device);
			px4_sleep(1);
			return;
		}

		// Open the UART. If this is successful then the UART is ready to use.
		if (! _uart->open()) {
			PX4_ERR("Error opening serial device  %s", _device);
			px4_sleep(1);
			return;
		}

		if (board_rc_swap_rxtx(_device)) {
			_uart->setSwapRxTxMode();
		}

		if (board_rc_singlewire(_device)) {
			_is_singlewire = true;
			_uart->setSingleWireMode();
		}

		PX4_INFO("Crsf serial opened sucessfully");

		if (_is_singlewire) {
			PX4_INFO("Crsf serial is single wire. Telemetry disabled");
		}

		_uart->flush();

		Crc8Init(0xd5);

		_input_rc.rssi_dbm = NAN;
		_input_rc.link_quality = -1;

		CrsfParser_Init();
	}

	const hrt_abstime time_now_us = hrt_absolute_time();
	perf_count_interval(_cycle_interval_perf, time_now_us);

	// Read all available data from the serial RC input UART
	int new_bytes = _uart->readAtLeast(&_rcs_buf[0], RC_MAX_BUFFER_SIZE, 1, 100);

	if (new_bytes > 0) {
		_bytes_rx += new_bytes;

		// Load new bytes into the CRSF parser buffer
		CrsfParser_LoadBuffer(_rcs_buf, new_bytes);

		// Scan the parse buffer for messages, one at a time
		CrsfPacket_t new_crsf_packet;

		while (CrsfParser_TryParseCrsfPacket(&new_crsf_packet, &_packet_parser_statistics)) {
			switch (new_crsf_packet.message_type) {
			case CRSF_MESSAGE_TYPE_RC_CHANNELS:
				_input_rc.timestamp_last_signal = time_now_us;
				_last_packet_seen = time_now_us;

				for (int i = 0; i < CRSF_CHANNEL_COUNT; i++) {
					_input_rc.values[i] = new_crsf_packet.channel_data.channels[i];
				}

				break;

			case CRSF_MESSAGE_TYPE_LINK_STATISTICS:
				_last_packet_seen = time_now_us;
				_input_rc.rssi_dbm = -(float)new_crsf_packet.link_statistics.uplink_rssi_1;
				_input_rc.link_quality = new_crsf_packet.link_statistics.uplink_link_quality;
				break;

			default:
				break;
			}
		}

		if (_param_rc_crsf_tel_en.get() && !_is_singlewire) {
			debug_value_s debug_value;

			if (_debug_value_sub.update(&debug_value) && debug_value.ind == collision_warning_debug_index) {
				const float severity = math::constrain(debug_value.value, 0.f, 3.f);
				_last_collision_warning_update = hrt_absolute_time();
				_collision_warning_severity = (int16_t)roundf(severity * 10.f);
			}

			int16_t collision_warning_severity = _collision_warning_severity;

			if (_last_collision_warning_update == 0
			    || hrt_elapsed_time(&_last_collision_warning_update) > collision_warning_timeout) {
				collision_warning_severity = 0;
			}

			if (collision_warning_severity > 0) {
				_collision_warning_was_active = true;
				_collision_warning_clear_frames_remaining = collision_warning_clear_frames;

			} else if (_collision_warning_was_active && _collision_warning_clear_frames_remaining == 0) {
				_collision_warning_clear_frames_remaining = collision_warning_clear_frames;
			}

			const bool send_warning_frame = collision_warning_severity > 0 || _collision_warning_clear_frames_remaining > 0;

			if (send_warning_frame && _input_rc.timestamp > _telemetry_update_last + telemetry_interval_warning) {
				vehicle_status_s vehicle_status;

				if (_vehicle_status_sub.copy(&vehicle_status)) {
					const char *flight_mode = "(unknown)";

					switch (vehicle_status.nav_state) {
					case vehicle_status_s::NAVIGATION_STATE_MANUAL:
						flight_mode = "Manual";
						break;

					case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
						flight_mode = "Altitude";
						break;

					case vehicle_status_s::NAVIGATION_STATE_POSCTL:
						flight_mode = "Position";
						break;

					case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
						flight_mode = "Return";
						break;

					case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
						flight_mode = "Mission";
						break;

					case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
					case vehicle_status_s::NAVIGATION_STATE_DESCEND:
					case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
					case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
					case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
					case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
						flight_mode = "Auto";
						break;

					/*case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL:
						flight_mode = "Failure";
						break;*/

					case vehicle_status_s::NAVIGATION_STATE_ACRO:
						flight_mode = "Acro";
						break;

					case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
						flight_mode = "Terminate";
						break;

					case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
						flight_mode = "Offboard";
						break;

					case vehicle_status_s::NAVIGATION_STATE_STAB:
						flight_mode = "Stabilized";
						break;

					default:
						flight_mode = "Unknown";
					}

					if (collision_warning_severity > 0) {
						char warning_flight_mode[16] {};
						const int severity_level = math::constrain((int)roundf(collision_warning_severity / 10.f), 1, 3);
						_collision_warning_sequence = (_collision_warning_sequence + 1) % 10;
						const char sequence_id = (char)('A' + _collision_warning_sequence);
						snprintf(warning_flight_mode, sizeof(warning_flight_mode), "CW%d%c %s", severity_level, sequence_id, flight_mode);

						this->SendTelemetryFlightMode(warning_flight_mode);

					} else {
						this->SendTelemetryFlightMode(flight_mode);
					}
				}

				if (collision_warning_severity <= 0 && _collision_warning_clear_frames_remaining > 0) {
					_collision_warning_clear_frames_remaining--;

					if (_collision_warning_clear_frames_remaining == 0) {
						_collision_warning_was_active = false;
					}
				}

				_telemetry_update_last = _input_rc.timestamp;
			}
		}
	}

	// If no communication
	if (time_now_us - _last_packet_seen > 100_ms) {
		// Invalidate link statistics
		_input_rc.rssi_dbm = NAN;
		_input_rc.link_quality = -1;
	}

	// If we have not gotten RC updates specifically
	if (time_now_us - _input_rc.timestamp_last_signal > 50_ms) {
		_input_rc.rc_lost = 1;
		_input_rc.rc_failsafe = 1;

	} else {
		_input_rc.rc_lost = 0;
		_input_rc.rc_failsafe = 0;
	}

	_input_rc.channel_count = CRSF_CHANNEL_COUNT;
	_input_rc.rssi = -1;
	_input_rc.rc_ppm_frame_length = 0;
	_input_rc.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_CRSF;
	_input_rc.timestamp = hrt_absolute_time();
	_input_rc_pub.publish(_input_rc);

	perf_count(_publish_interval_perf);

	ScheduleDelayed(4_ms);
}

/**
 * write an uint8_t value to a buffer at a given offset and increment the offset
 */
static inline void write_uint8_t(uint8_t *buf, int &offset, uint8_t value)
{
	buf[offset++] = value;
}

void CrsfRc::WriteFrameHeader(uint8_t *buf, int &offset, const crsf_frame_type_t type, const uint8_t payload_size)
{
	write_uint8_t(buf, offset, 0xc8); // this got changed from the address to the sync byte
	write_uint8_t(buf, offset, payload_size + 2);
	write_uint8_t(buf, offset, (uint8_t)type);
}

void CrsfRc::WriteFrameCrc(uint8_t *buf, int &offset, const int buf_size)
{
	// CRC does not include the address and length
	write_uint8_t(buf, offset, Crc8Calc(buf + 2, buf_size - 3));
}

bool CrsfRc::SendTelemetryFlightMode(const char *flight_mode)
{
	const int max_length = 16;
	int length = strlen(flight_mode) + 1;

	if (length > max_length) {
		length = max_length;
	}

	uint8_t buf[max_length + 4];
	int offset = 0;
	WriteFrameHeader(buf, offset, crsf_frame_type_t::flight_mode, length);
	memcpy(buf + offset, flight_mode, length);
	offset += length;
	buf[offset - 1] = 0; // ensure null-terminated string
	WriteFrameCrc(buf, offset, length + 4);
	return _uart->write((void *) buf, (size_t) offset);
}

int CrsfRc::print_status()
{
	if (_device[0] != '\0') {
		PX4_INFO("UART device: %s", _device);
		PX4_INFO("UART RX bytes: %"  PRIu32, _bytes_rx);
	}

	if (_is_singlewire) {
		PX4_INFO("Telemetry disabled: Singlewire RC port");

	} else {
		PX4_INFO("Telemetry: %s", _param_rc_crsf_tel_en.get() ? "yes" : "no");
	}

	perf_print_counter(_cycle_interval_perf);
	perf_print_counter(_publish_interval_perf);

	PX4_INFO_RAW("Disposed bytes: %" PRIu32 "\n", _packet_parser_statistics.disposed_bytes);
	PX4_INFO_RAW("Valid known packet CRCs: %" PRIu32 "\n", _packet_parser_statistics.crcs_valid_known_packets);
	PX4_INFO_RAW("Valid unknown packet CRCs: %" PRIu32 "\n", _packet_parser_statistics.crcs_valid_unknown_packets);
	PX4_INFO_RAW("Invalid CRCs: %" PRIu32 "\n", _packet_parser_statistics.crcs_invalid);
	PX4_INFO_RAW("Invalid known packet sizes: %" PRIu32 "\n", _packet_parser_statistics.invalid_known_packet_sizes);
	PX4_INFO_RAW("Invalid unknown packet sizes: %" PRIu32 "\n", _packet_parser_statistics.invalid_unknown_packet_sizes);

	return 0;
}

int CrsfRc::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CrsfRc::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module parses the CRSF RC uplink protocol and generates CRSF downlink telemetry data

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("crsf_rc", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "RC device", true);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int crsf_rc_main(int argc, char *argv[])
{
	return CrsfRc::main(argc, argv);
}
