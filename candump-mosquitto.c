/*
 * candump.c
 *
 * Copyright (c) 2017 Giovanni Grieco
 * Copyright (c) 2002-2009 Volkswagen Group Electronic Research
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of Volkswagen nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * Alternatively, provided that this notice is retained in full, this
 * software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2, in which case the provisions of the
 * GPL apply INSTEAD OF those given above.
 *
 * The provided data structures and external interfaces from this code
 * are not restricted to be used by modules with a GPL compatible license.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * Send feedback to <linux-can@vger.kernel.org>
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>
#include <ctype.h>
#include <libgen.h>
#include <time.h>
#include <errno.h>
#include <string.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "terminal.h"
#include "lib.h"

#include <mosquitto.h>

#define CAN_INTERFACE "can0"
#define MAXIFNAMES 30 /* size of receive name index to omit ioctls */
#define ANYDEV "any"  /* name of interface to receive from any CAN interface */

static char devname[MAXIFNAMES][IFNAMSIZ+1];
static int  dindex[MAXIFNAMES];
static int  max_devname_len; /* to prevent frazzled device name output */ 
const int canfd_on = 1;

static struct timeval time_count;

static volatile int running = 1;

void sigterm(int signo)
{
	running = 0;
}

int idx2dindex(int ifidx, int socket) {
	int i;
	struct ifreq ifr;

	for (i=0; i < MAXIFNAMES; i++) {
		if (dindex[i] == ifidx)
			return i;
	}

	/* create new interface index cache entry */
	/* remove index cache zombies first */
	for (i=0; i < MAXIFNAMES; i++) {
		if (dindex[i]) {
			ifr.ifr_ifindex = dindex[i];
			if (ioctl(socket, SIOCGIFNAME, &ifr) < 0)
				dindex[i] = 0;
		}
	}

	for (i=0; i < MAXIFNAMES; i++)
		if (!dindex[i]) /* free entry */
			break;

	dindex[i] = ifidx;

	ifr.ifr_ifindex = ifidx;
	if (ioctl(socket, SIOCGIFNAME, &ifr) < 0)
		perror("SIOCGIFNAME");

	if (max_devname_len < strlen(ifr.ifr_name))
		max_devname_len = strlen(ifr.ifr_name);

	strcpy(devname[i], ifr.ifr_name);

	return i;
}

void mosquitto_connect_callback(struct mosquitto *mosq, void *userdata, int result)
{
	if (!result) {
		mosquitto_subscribe(mosq, NULL, "$SYS/formatted/lap_finished", 0);
	}
}

void mosquitto_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message)
{
	if (message->payloadlen) {
		if ((int *)message->payload) {
			gettimeofday(&time_count, NULL);
		} 			
	}
}

int main(int argc, char **argv)
{
	fd_set rdfs;
	int sock;
	unsigned char view = 0;
	int count = 0;
	char *ptr = CAN_INTERFACE; 
	struct sockaddr_can addr;
	char ctrlmsg[CMSG_SPACE(sizeof(struct timeval)) + CMSG_SPACE(sizeof(__u32))];
	struct iovec iov;
	struct msghdr msg;
	struct canfd_frame frame;
	int nbytes, maxdlen;
	struct ifreq ifr;
	bool clean_session = true;	
	int idx;

	struct mosquitto *mosq = NULL;
	char *host = "localhost";
	int port = 1883;
	int keepalive = 33;

	// initialize time counter
	gettimeofday(&time_count, NULL);

 	mosquitto_lib_init();
	mosq = mosquitto_new("candump", clean_session, NULL);
	mosquitto_connect_callback_set(mosq, mosquitto_connect_callback);
	mosquitto_message_callback_set(mosq, mosquitto_message_callback);
	mosquitto_connect(mosq, host, port, keepalive);

	signal(SIGTERM, sigterm);
	signal(SIGHUP, sigterm);
	signal(SIGINT, sigterm);
	
	sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (sock < 0) {
		perror("socket");
		return 1;
	}

	nbytes = strlen(ptr);

	addr.can_family = AF_CAN;

	memset(&ifr.ifr_name, 0, sizeof(ifr.ifr_name));
	strncpy(ifr.ifr_name, ptr, nbytes);

	if (strcmp(ANYDEV, ifr.ifr_name)) {
		if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
			perror("SIOCGIFINDEX");
			exit(1);
		}
		addr.can_ifindex = ifr.ifr_ifindex;
	} else
		addr.can_ifindex = 0; /* any can interface */

	/* try to switch the socket into CAN FD mode */
	setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on));

	if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return 1;
	}

	/* these settings are static and can be held out of the hot path */
	iov.iov_base = &frame;
	msg.msg_name = &addr;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_control = &ctrlmsg;

	while (running) {
		FD_ZERO(&rdfs);
		FD_SET(sock, &rdfs);

		if (FD_ISSET(sock, &rdfs)) {
			/* these settings may be modified by recvmsg() */
			iov.iov_len = sizeof(frame);
			msg.msg_namelen = sizeof(addr);
			msg.msg_controllen = sizeof(ctrlmsg);  
			msg.msg_flags = 0;

			nbytes = recvmsg(sock, &msg, 0);
			idx = idx2dindex(addr.can_ifindex, sock);

			if ((size_t)nbytes == CAN_MTU)
				maxdlen = CAN_MAX_DLEN;
			else if ((size_t)nbytes == CANFD_MTU)
				maxdlen = CANFD_MAX_DLEN;
			else {
				fprintf(stderr, "read: incomplete CAN frame\n");
				return 1;
			}

			if (count && (--count == 0))
				running = 0;

			struct timeval mosq_te;
			gettimeofday(&mosq_te, NULL);
			long long ms = mosq_te.tv_sec*1000LL + mosq_te.tv_usec/1000;
			long long ref_ms = time_count.tv_sec*1000LL + time_count.tv_usec/1000;
			long long delta_ms = ms - ref_ms;
				
			char json[100]; // da fare l'alloc della memoria ad hoc
			sprintf(json, "{id:%u,time:%lld,data:[", frame.can_id, delta_ms);
			char tmp[10];
			int i = 0;
			for (i = 0; i < 8; i++) {
				sprintf(tmp, "%d,", frame.data[i]);
				strcat(json, tmp);
			}
			strcat(json, "]}");
			mosquitto_publish(mosq, NULL, "data/raw", 100, &json, 0, false);
			mosquitto_loop(mosq, 0, 1);				
			
			// canN output on stdout
			printf("  %*s  ", max_devname_len, devname[idx]);
			fprint_long_canframe(stdout, &frame, NULL, view, maxdlen);
			printf("\n");
		}
	}

	mosquitto_destroy(mosq);
	mosquitto_lib_cleanup();

	close(sock);

	return 0;
}
