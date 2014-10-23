/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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
 * @file px4_daemon_app.c
 * daemon application example for PX4 autopilot
 * 
 * @author Example User <mail@example.com>
 */

#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/time.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>
#include <poll.h>
#include <string.h>
#include <uORB/topics/cubie_pos.h>
#include <mavlink/mavlink_log.h>
 
/* file handle that will be used for publishing */
static int cubie_pos_handle;

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

/**
 * daemon management function.
 */
__EXPORT int cubie_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int cubie_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason)
		warnx("%s\n", reason);
	errx(1, "usage: cubie {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 * 
 * The actual stack size should be set in the call
 * to task_create().
 */
int cubie_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		daemon_task = task_spawn_cmd("cubie",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_DEFAULT,
					 4096,
					 cubie_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		
		int mavlink_fd;
		mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
		mavlink_log_info(mavlink_fd, "[cubie] started on %s", argv[2]);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");
		} else {
			warnx("\tnot started\n");
		}
		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int cubie_thread_main(int argc, char *argv[])
{
	//int mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	//fd_set rfds;
	//struct timeval tv;
	int retval;

	warnx("[cubie] starting\n");

	thread_running = true;

	/* advertise debug value */
	/*struct debug_key_value_s dbgx = { .key = "dbgx", .value = 0.0f };
	struct debug_key_value_s dbgy = { .key = "dbgy", .value = 0.0f };
	struct debug_key_value_s dbgz = { .key = "dbgz", .value = 0.0f };
	orb_advert_t pub_dbgx = orb_advertise(ORB_ID(debug_key_value), &dbgx);
	orb_advert_t pub_dbgy = orb_advertise(ORB_ID(debug_key_value), &dbgy);
	orb_advert_t pub_dbgz = orb_advertise(ORB_ID(debug_key_value), &dbgz);*/
	
	//Init
	int uart2 = open(argv[1], O_RDONLY | O_NOCTTY);
	if(uart2 >= 0)
	{
		warnx("port is open\n");
	}
	else
	{
		warnx("port is NOT open\n");
		return 1;
	}
	
	//Set baud rate
	struct termios uart_config;
	tcgetattr(uart2, &uart_config);
	cfsetispeed(&uart_config, B57600);
	cfsetospeed(&uart_config, B57600);
	tcsetattr(uart2, TCSANOW, &uart_config);
	
	struct pollfd fds = { .fd = uart2, .events = POLLIN };
	
	uint8_t buffer[32];
	uint8_t buf_size = 0;
	int16_t x = 0, y = 0, z = 0;
	uint16_t ux = 0, uy = 0, uz = 0;

	//warnx("1");
	
	/* generate the initial data for first publication */
	struct cubie_pos_s pos = { .x = 0.0f, .y = 0.0f, .z = 0.0f };
 
	/* advertise the topic and make the initial publication */
	cubie_pos_handle = orb_advertise(ORB_ID(cubie_position), &pos);
	
	while (!thread_should_exit)
	{
		//dbgx.value = 1.0f;
		//orb_publish(ORB_ID(debug_key_value), pub_dbgx, &dbgx);
		//warnx("2");
		for(int i = 0; i < 4; )
		{
			/*tv.tv_sec = 0;
			tv.tv_usec = 100;
			retval = select(uart2 + 1, &rfds, NULL, NULL, &tv);*/
			retval = poll(&fds, 1, 1000);
		//dbgx.value = 2.0f;
		//orb_publish(ORB_ID(debug_key_value), pub_dbgx, &dbgx);
			if(retval == -1)
			{
				warnx("poll() error");
				goto DeInit;
			}
		//dbgx.value = 3.0f;
		//orb_publish(ORB_ID(debug_key_value), pub_dbgx, &dbgx);
			if(thread_should_exit)
			{
				goto DeInit;
			}

			if(retval != 1) continue;

			warnx("bytes read: %d", read(uart2, buffer, 1));
			if((unsigned char)buffer[0] == (unsigned char)0x1F) i++;
			else i = 0;
			//warnx("i= %d char= %X", i, (unsigned char)buffer[0]);
		}
		/*dbgx.value = 4.0f;
		orb_publish(ORB_ID(debug_key_value), pub_dbgx, &dbgx);*/
		while(buf_size < 6)
		{
		/*dbgx.value = 5.0f;
		orb_publish(ORB_ID(debug_key_value), pub_dbgx, &dbgx);*/
			/*tv.tv_sec = 0;
			tv.tv_usec = 100;
			retval = select(uart2 + 1, &rfds, NULL, NULL, &tv);*/
			retval = poll(&fds, 1, 1000);
			if(retval == -1)
			{
				warnx("poll() error");
				goto DeInit;
			}
			if(thread_should_exit)
			{
				goto DeInit;
			}

			if(retval != 1) continue;

			//warnx(".");
			buf_size += read(uart2, buffer + buf_size, 6 - buf_size);
		}
		//TODO: checksum
		//mavlink_log_info(mavlink_fd, "buf: %x %x %x %x %x %x", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);
		ux = *(uint16_t *)(buffer);
		uy = *(uint16_t *)(buffer + 2);
		uz = *(uint16_t *)(buffer + 4);
		/*ux = (ux >> 8) | (ux << 8);
		uy = (uy >> 8) | (uy << 8);
		uz = (uz >> 8) | (uz << 8);*/
		x = *(int16_t *)(&ux);
		y = *(int16_t *)(&uy);
		z = *(int16_t *)(&uz);
		pos.x = x;
		pos.y = y;
		pos.z = z;
		//mavlink_log_info(mavlink_fd, "pos: %.3f %.3f %.3f", pos.x, pos.y, pos.z);
		//warnx("x=%03d y=%03d z=%03d", buffer, buffer + 2, buffer + 4);
		//warnx("x=%03d y=%03d z=%03d", x, y, z);
		pos.x *= 0.001f;
		pos.y *= 0.001f;
		pos.z *= 0.001f;
		/* publish the new data structure */
		orb_publish(ORB_ID(cubie_position), cubie_pos_handle, &pos);
		/*dbgx.value = (float)x / 100.0f;
		dbgy.value = (float)y / 100.0f;
		dbgz.value = (float)z / 100.0f;
		orb_publish(ORB_ID(debug_key_value), pub_dbgx, &dbgx);
		orb_publish(ORB_ID(debug_key_value), pub_dbgy, &dbgy);
		orb_publish(ORB_ID(debug_key_value), pub_dbgz, &dbgz);*/
		buf_size = 0;
	}

	//DeInit
DeInit:
	warnx("[cubie] exiting.\n");
	close(uart2);

	thread_running = false;

	return 0;
}
