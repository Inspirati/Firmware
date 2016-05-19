/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *   Author: Robert Dickenson
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

/**
 * @file esc.c
 *
 * Utility for viewing esc ORB publications, specifically for
 * development work on the AUAV ESC35 connected via CAN bus.
 *
 * @author Robert Dickenson
 *
 */

#include <px4_config.h>
#include <termios.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <apps/nsh.h>
#include <fcntl.h>
#include <poll.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include <uORB/topics/esc_status.h>
#include <uORB/topics/esc_report.h>
#define esc_report esc_report_s
#define esc_status esc_status_s

/*
struct esc_report_s {
	uint8_t esc_vendor;
	uint32_t esc_errorcount;
	int32_t esc_rpm;
	float esc_voltage;
	float esc_current;
	float esc_temperature;
	float esc_setpoint;
	uint16_t esc_setpoint_raw;
	uint16_t esc_address;
	uint16_t esc_version;
	uint16_t esc_state;
};

struct esc_status_s {
	uint16_t counter;
	uint64_t timestamp;
	uint8_t esc_count;
	uint8_t esc_connectiontype;
	struct esc_report_s esc[8];
#ifdef __cplusplus
	static const uint8_t CONNECTED_ESC_MAX = 8;
	static const uint8_t ESC_VENDOR_GENERIC = 0;
	static const uint8_t ESC_VENDOR_MIKROKOPTER = 1;
	static const uint8_t ESC_VENDOR_GRAUPNER_HOTT = 2;
	static const uint8_t ESC_CONNECTION_TYPE_PPM = 0;
	static const uint8_t ESC_CONNECTION_TYPE_SERIAL = 1;
	static const uint8_t ESC_CONNECTION_TYPE_ONESHOOT = 2;
	static const uint8_t ESC_CONNECTION_TYPE_I2C = 3;
	static const uint8_t ESC_CONNECTION_TYPE_CAN = 4;
#endif
};
 */

#include <uORB/topics/esc_report.h>
#define mag_report esc_report_s
/*
struct esc_report_s {
		uint64_t timestamp;
		uint64_t error_count;
		float x;
		float y;
		float z;
		float range_ga;
		float scaling;
		float temperature;
		int16_t x_raw;
		int16_t y_raw;
		int16_t z_raw;
};
 */

int esc1(void);
int esc2(void);

int
esc1(void)
{
	struct esc_status erb;
	int esc_fd = orb_subscribe(ORB_ID(esc_status));

	while (true) {

		bool updated = false;
		orb_check(esc_fd, &updated);

		if (updated) {
			orb_copy(ORB_ID(esc_status), esc_fd, &erb);
/*
	uint16_t counter;
	uint64_t timestamp;
	uint8_t esc_count;
	uint8_t esc_connectiontype;
--	struct esc_report_s esc[8];
		uint8_t esc_vendor;
		uint32_t esc_errorcount;
		int32_t esc_rpm;
		float esc_voltage;
		float esc_current;
		float esc_temperature;
		float esc_setpoint;
		uint16_t esc_setpoint_raw;
		uint16_t esc_address;
		uint16_t esc_version;
		uint16_t esc_state;
 */
            printf("esc %u %u %u ", erb.counter, erb.esc_count, erb.esc_connectiontype);
            printf(": %u %u %u ", erb.esc[0].esc_vendor, erb.esc[0].esc_errorcount, erb.esc[0].esc_rpm);
            printf(": %u %u %u %u", erb.esc[0].esc_address, erb.esc[0].esc_setpoint_raw, erb.esc[0].esc_version, erb.esc[0].esc_state);
            printf("\n");
            printf("esc %8.4f %8.4f", (double)erb.esc[0].esc_voltage, (double)erb.esc[0].esc_current);
            printf(": %8.4f %8.4f",   (double)erb.esc[0].esc_temperature, (double)erb.esc[0].esc_setpoint);
            printf("\n");
        }

		struct pollfd fds;
		int ret;
		fds.fd = 0; /* stdin */
		fds.events = POLLIN;
		ret = poll(&fds, 1, 0);
		if (ret > 0) {
			char c;

			read(0, &c, 1);
			switch (c) {
			case 0x03: // ctrl-c
			case 0x1b: // esc
			case 'c':
			case 'q':
				return OK;
			}
		}
	}
	return OK;
}

int
esc2(void)
{
	struct esc_report erb;
	int esc_fd = orb_subscribe(ORB_ID(esc_report));

	while (true) {

		bool updated = false;
		orb_check(esc_fd, &updated);

		if (updated) {
			orb_copy(ORB_ID(esc_report), esc_fd, &erb);
/*
	uint8_t esc_vendor;
	uint32_t esc_errorcount;
	int32_t esc_rpm;
	float esc_voltage;
	float esc_current;
	float esc_temperature;
	float esc_setpoint;
	uint16_t esc_setpoint_raw;
	uint16_t esc_address;
	uint16_t esc_version;
	uint16_t esc_state;
 */
            printf("esc %u %u %u %u %u %u %u", erb.esc_vendor, erb.esc_errorcount, erb.esc_rpm, erb.esc_address, erb.esc_setpoint_raw, erb.esc_version, erb.esc_state);
            printf("\n");
            printf("esc %8.4f %8.4f %8.4f %8.4f", (double)erb.esc_voltage, (double)erb.esc_current, (double)erb.esc_temperature, (double)erb.esc_setpoint);
            printf("\n");
        }

		struct pollfd fds;
		int ret;
		fds.fd = 0; /* stdin */
		fds.events = POLLIN;
		ret = poll(&fds, 1, 0);
		if (ret > 0) {
			char c;

			read(0, &c, 1);
			switch (c) {
			case 0x03: // ctrl-c
			case 0x1b: // esc
			case 'c':
			case 'q':
				return OK;
			}
		}
	}
	return OK;
}

#if 0
#define MAX_SAMPLES 100
static struct esc_status erb[MAX_SAMPLES];
//static uint64_t last_timestamp;

int
esc2(void)
{
	int i = 0;

	int esc_fd = orb_subscribe(ORB_ID(esc_status));

	while (i < MAX_SAMPLES) {

		bool updated = false;
		orb_check(esc_fd, &updated);

		if (updated) {
			orb_copy(ORB_ID(esc_report), esc_fd, &erb[i++]);
		}
#if 0
		struct pollfd fds;
		int ret;
		fds.fd = 0; // stdin
		fds.events = POLLIN;
		ret = poll(&fds, 1, 0);
		if (ret > 0) {
			char c;
			read(0, &c, 1);
			switch (c) {
			case 0x03: // ctrl-c
			case 0x1b: // esc
			case 'c':
			case 'q':
				return OK;
			}
		}
#endif
	}

/*
	uint32_t total_time = 0;
	last_timestamp = erb[0].timestamp;

	for (i = 0; i < MAX_SAMPLES; i++) {

//			printf("magxyz %d %d %d ", erb[i].x_raw, erb[i].y_raw, erb[i].z_raw);
			printf("magxyz %8.4f %8.4f %8.4f ", (double)erb[i].x, (double)erb[i].y, (double)erb[i].z);

			uint8_t cnt0 = (erb[i].error_count & 0x000000FF00000000) >> 32;
			uint8_t cnt1 = (erb[i].error_count & 0x00000000FF000000) >> 24;
			uint8_t cnt2 = (erb[i].error_count & 0x0000000000FF0000) >> 16;
			uint8_t cnt3 = (erb[i].error_count & 0x000000000000FF00) >> 8;
			uint8_t cnt4 = (erb[i].error_count & 0x00000000000000FF);
			printf("[%x %x %x %x %x] ", cnt0, cnt1, cnt2, cnt3, cnt4);

			uint32_t mag_interval = (erb[i].error_count & 0xFFFFFF0000000000) >> 40;
			printf("%u  ", mag_interval);
			if ((mag_interval) < 5000) {
				printf("* ");
			}
			if ((mag_interval) > 15000) {
				printf("! ");
			}

			uint32_t orb_interval = erb[i].timestamp - last_timestamp;
			if (orb_interval != mag_interval) {
				printf("orb ", orb_interval); // missed some orb publications
			}
			last_timestamp = erb[i].timestamp;
			total_time += orb_interval;

			if (erb[i].x_raw == erb[i-1].x_raw && erb[i].y_raw == erb[i-1].y_raw && erb[i].z_raw == erb[i-1].z_raw ) {
				printf("dup "); // duplicate magnetometer xyz data
			}

			printf("\n");
	}
	double t = (double)total_time / 1000000.0;
	printf("total_time: %.3f s\n", t);
 */
    return OK;
}
#endif

__EXPORT int esc_main(int argc, char *argv[]);

int
esc_main(int argc, char *argv[])
{
	if (argc < 2) {
		esc1();
	} else {
		int reps = atoi(argv[1]);
		if (reps > 20 || reps < 1) {
			reps = 1;
		}
		while (reps--) {
			esc2();
		}
	}
	return OK;
}
