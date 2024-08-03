/**
 * Copyright (c) David Hubbard 2024
 * Licensed under the GNU Affero General Public License, version 3
 * A copy of LICENSE.txt should have accompanied this file.
 *
 * How to use:
 * 1. Download a copy of libusb-1.0.0.dylib
 *    Or clone https://github.com/libusb/libusb and build it with XCode
 *    The looooong path in the next step is due to where XCode keeps libusb-1.0.0.dylib
 *
 * 2. Compile usb2sock: */
//    gcc -o usb2sock -g -Wall usb2sock.o -c usb2sock.c \
//        -L$HOME/Library/Developer/Xcode/DerivedData/libusb*/Build/Products/Debug \
//        -Xlinker -rpath -Xlinker $HOME/Library/Developer/Xcode/DerivedData/libusb*/Build/Products/Debug \
//        -lusb-1.0.0
/*
 * 3. Run: ./usb2sock
 * 4. It should prompt you: Please plug in goggles [0 tcp]
 * 5. It also puts the VLC MRL in the terminal
 * 6. Copy the VLC MRL and launch VLC
 * 7. Open Network (or press Command-N)
 * 8. Paste the MRL: tcp/h264://127.0.0.1:18080
 * 9. Click OK
 * 10. Click the play button. The VLC seek bar should show an animation while it waits for bytes.
 * 11. Power up the drone and the goggles
 * 12. Wait for the goggles to connect to the drone
 *     (The goggles get confused if usb2sock talks to them before they are fully booted,
 *     you may end up in a loop between "Please plug in goggles [1 tcp]" and
 *     "video signal: OFF". Just unplug/replug the goggles if they are confused.
 * 13. Plug in the goggles
 * 14. usb2sock starts streaming: video [1 tcp] packet      1: 41K
 * 15. vlc will display the video
 * 16. You can unplug the goggles anytime, usb2sock will just wait for them to come back
 * 17. Type Ctrl-C to shut down usb2sock, or close the terminal
 * 18. Go fly some more
 */
#include "include/usb2sock.h"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include "../../libusb/libusb/libusb.h"

static void logcb(libusb_context * ctx, enum libusb_log_level level, const char * str)
{
	(void) ctx;
	printf("<%d>%s", level, str);
}

volatile int connected = 0;
static int set_connected_flag(libusb_context *ctx,
        libusb_device *device, libusb_hotplug_event event, void *user_data)
{
	(void) user_data;
	switch (event) {
	case LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED:
		printf(" +hotplug\n");
		connected = 1;
		break;
	case LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT:
		printf(" -hotplug\n");
		connected = 0;
		break;
	}
	return 0;
}

enum {
	GOGGLES_WANT_CFG = 1,
	VENDOR_ID_DJI = 0x2ca3,
	DEVICE_ID_GOGGLES = 0x001f,
	WAIT_FOR_HOTPLUG = 0xca0,
	TCP_SERVER_PORT = 18080,
};

int tune_io_timeout_ms = 250;
int sockfd = -1;
int clientlist[1024];
unsigned clientlist_use = 0;
unsigned char buf[128*1024];

void close_client(unsigned i)
{
	close(clientlist[i]);
	// shift down all entries in clientlist[] that are i or higher
	clientlist_use--;
	memmove(&clientlist[i], &clientlist[i + 1], clientlist_use - i);
}

void accept_any_connection()
{
	struct sockaddr_in saddr;
	socklen_t saddrlen = sizeof(saddr);
	int clientfd = accept(sockfd, (struct sockaddr *) &saddr, &saddrlen);
	if (clientfd < 0) {
		int r = errno;
		// sockfd is F_SETFL, O_NONBLOCK so accept will just return an error instead of blocking
		if (r == EWOULDBLOCK || r == EAGAIN) return;
		printf("accept tcp conn failed: %d %s\n", r, strerror(r));
		return;
	}

	char clientaddr[INET6_ADDRSTRLEN + INET_ADDRSTRLEN];
	if (!inet_ntop(AF_INET, &(saddr.sin_addr), clientaddr, sizeof(clientaddr))) {
		printf("inet_ntop client failed: %d %s\n", errno, strerror(errno));
	}
	snprintf(clientaddr + strlen(clientaddr), sizeof(clientaddr) - 1 - strlen(clientaddr),
		":%u", (unsigned) ntohs(saddr.sin_port));
	clientaddr[sizeof(clientaddr) - 1] = 0;
	if (saddrlen != sizeof(saddr)) {
		printf("accepted \"%s\" wrong size: %d want %zu\n", clientaddr, (int) saddrlen, sizeof(saddr));
		return;
	}
	if (clientlist_use >= sizeof(clientlist) / sizeof(clientlist[0])) {
		printf("client %s: too many clients\n", clientaddr);
		close(clientfd);
		return;
	}

	if (fcntl(clientfd, F_SETFL, O_NONBLOCK)) {
		printf("O_NONBLOCK client failed: %d %s\n", errno, strerror(errno));
		close(clientfd);
	}

	printf(" client%u %s\n", clientlist_use, clientaddr);
	clientlist[clientlist_use] = clientfd;
	clientlist_use++;
}

void recv_and_discard()
{
	char discard[256];
	for (unsigned i = 0; i < clientlist_use;) {
		ssize_t n = read(clientlist[i], discard, sizeof(discard));
		if (n < 0) {
			int r = errno;
			if (r == EWOULDBLOCK || r == EAGAIN) {
				// read buffers are empty, move to next client
				i++;
				continue;
			}
			// read failed, close socket
			close_client(i);
			continue;
		} else if (n == 0) {
			// client closed socket
			close_client(i);
			continue;
		}
		// keep receiving and discarding
	}
}

void send_to_socket(int buflen)
{
	// This is designed to be nonblocking, favoring usb which is *the* blocking i/o
	// If tcp clients can receive data, it is shoved at them as fast as possible
	//
	// But if anything goes wrong, disconnect the client and get back to the usb stream
	int ofs = 0;
	for (unsigned i = 0; i < clientlist_use;) {
		ssize_t n = write(clientlist[i], buf + ofs, buflen - ofs);
		if (n < 0) {
			int r = errno;
			if (r != EWOULDBLOCK && r != EAGAIN) {
				// write failed, close socket and move to next client
				close_client(i);
				ofs = 0;
				continue;
			}
			// write buffer full, move to next client
		} else if (n > 0) {
			// keep writing until buf is sent or EAGAIN is received
			ofs += n;
			if (ofs < buflen) continue;
		}
		// move to next client
		ofs = 0;
		i++;
	}
}

libusb_hotplug_callback_handle hotplug;
int start_stream(libusb_context * ctx, libusb_device_handle * usbdev,
	int inEndpoint, int outEndpoint)
{
	int dbg = 0;

	static unsigned char MAGIC_PACKET[] = "RMVT";

	// Two cases:
	// 1. New usb connection, goggles state is "waiting for MAGIC_PACKET"
	// 2. Existing usb connection, so this program probably was restarted
	//    - goggles state is "streaming" and the MAGIC_PACKET gets ignored
	//    - sending the MAGIC_PACKET will time out but the goggles stream data
	//      happily, so just ignore the time out error
	//    - anyway, if the goggles really aren't streaming, then the best way
	//      to detect what's going on is a libusb_bulk_transfer(in).
	int success = 0;
	libusb_set_debug(ctx, LIBUSB_LOG_LEVEL_ERROR);  // silence warning for this bulk transfer
	int r = libusb_bulk_transfer(usbdev, outEndpoint,
		MAGIC_PACKET, sizeof(MAGIC_PACKET) - 1, &success, 50);
	libusb_set_debug(ctx, LIBUSB_LOG_LEVEL_WARNING);  // unsilence warnings
	if (r == LIBUSB_ERROR_TIMEOUT && success == 0) {
		if (dbg) printf("send(magic): sent %d, %s, goggles already connected\n",
			success, libusb_strerror(r));
	} else if (r == LIBUSB_ERROR_IO) {
		printf(" send:io\n");
		return WAIT_FOR_HOTPLUG;
	} else if (r) {
		printf("send(magic): sent %d, %s\n", success, libusb_strerror(r));
		return r;	// weird error
	} else if (success != sizeof(MAGIC_PACKET) - 1) {
		printf("send(magic): sent %d want %d\n", success, (int) sizeof(MAGIC_PACKET) - 1);
		return LIBUSB_ERROR_TIMEOUT;
	}

	// This is not really complicated. Mainly all the logic here is to make usb2sock
	// resilient when macOS's charming usb subsystem returns random errors, or when
	// the goggles are not ready yet.
	//
	// It just eternally asks the goggles for some data libusb_bulk_transfer() and
	// if any data came, shove it out to anyone connected via tcp send_to_socket().
	uint32_t count = 0;
	for (;; count++) {
		accept_any_connection();
		recv_and_discard();
		success = 0;
		r = libusb_bulk_transfer(usbdev, inEndpoint, buf, sizeof(buf),
			&success, tune_io_timeout_ms);
		if (r == LIBUSB_ERROR_TIMEOUT && success == 0) {
			printf("\e[A\r\e[Kvideo signal: OFF");
			fflush(stdout);
			usleep(500 * 1000);	// 0.5s: go slowly until the signal returns
			count = 0;
		} else if (r == LIBUSB_ERROR_IO) {
			printf(" wait:io\n");
			return WAIT_FOR_HOTPLUG;
		} else if (r == LIBUSB_ERROR_NO_DEVICE) {
			printf(" wait:no dev\n");
			return WAIT_FOR_HOTPLUG;
		} else if (r == LIBUSB_ERROR_NOT_FOUND) {
			printf(" wait:not found\n");
			return WAIT_FOR_HOTPLUG;
		} else if (r != 0) {
			printf("rx: got %d, %d %s\n", success, r, libusb_strerror(r));
			return r;	// rx broke
		} else {
			fflush(stdout);
			printf("\r\e[Kvideo [%u tcp] packet %6u: %dK", clientlist_use, count, success / 1024);
			fflush(stdout);
			// write data to anyone connected via tcp
			send_to_socket(success);
		}
	}
	// "return 0" never happens: this program eternally pumps data from usb
	return 0;
}

int start_with_a_device(libusb_context * ctx, libusb_device * dev)
{
	int dbg = 0;

	struct libusb_device_descriptor desc;
	int r = libusb_get_device_descriptor(dev, &desc);
	if (r != 0) {
		printf("get_device_descriptor failed: %d %s\n", r, libusb_strerror(r));
		return r;
	}

	if (desc.idVendor != VENDOR_ID_DJI || desc.idProduct != DEVICE_ID_GOGGLES) {
		//printf("no: %04x:%04x\n", desc.idVendor, desc.idProduct);
		return VENDOR_ID_DJI;
	}

	libusb_device_handle * usbdev;
	r = libusb_open(dev, &usbdev);
	if (r) {
		printf("libusb_open %04x:%04x failed: %d %s\n",
			desc.idVendor, desc.idProduct, r, libusb_strerror(r));
		return WAIT_FOR_HOTPLUG;
	}
	int cfg = -1;
	r = libusb_get_configuration(usbdev, &cfg);
	if (r) {
		printf("%04x:%04x get_configuration failed: %d %s\n",
			desc.idVendor, desc.idProduct, r, libusb_strerror(r));
	} else if (cfg == 0) {
		printf("%04x:%04x cfg = %d need to set cfg %d\n",
			desc.idVendor, desc.idProduct, cfg, GOGGLES_WANT_CFG);
		r = libusb_set_configuration(usbdev, GOGGLES_WANT_CFG);
		if (r) {
			printf("%04x:%04x set_configuration(%d) failed: %d %s\n",
				desc.idVendor, desc.idProduct, GOGGLES_WANT_CFG,
				r, libusb_strerror(r));
			return r;
		}
	} else {
		if (dbg) printf("%04x:%04x cfg = %d\n", desc.idVendor, desc.idProduct, cfg);
	}
	struct libusb_config_descriptor * cdesc;
	r = libusb_get_config_descriptor(dev, 0, &cdesc);
	if (r) {
		printf("%04x:%04x get_config_descriptor failed: %d %s\n",
			desc.idVendor, desc.idProduct, r, libusb_strerror(r));
		return r;
	}
	int find_FF_43 = -1;
	int inEndpoint = -1;
	int outEndpoint = -1;
	for (uint32_t i = 0; i < cdesc->bNumInterfaces; i++) {
		const struct libusb_interface * iface = &cdesc->interface[i];
		if (iface->num_altsetting == 0)
			printf("%04x:%04x iface %u no alt\n", desc.idVendor, desc.idProduct, i);
		for (uint32_t j = 0; j < iface->num_altsetting; j++) {
			if (dbg) printf("%04x:%04x iface %u alt %u\n", desc.idVendor, desc.idProduct, i, j);
			const struct libusb_interface_descriptor * alt = &iface->altsetting[j];
			if (dbg) printf("   cls: %02x.%02x proto: %02x\n",
				alt->bInterfaceClass, alt->bInterfaceSubClass, alt->bInterfaceProtocol);
			int firstIn = -1;
			int firstOut = -1;
			for (uint32_t k = 0; k < alt->bNumEndpoints; k++) {
				const struct libusb_endpoint_descriptor * dep = &alt->endpoint[k];
				if (dbg) printf("          ep[%u] addr=%02x | %s\n", k,
					dep->bEndpointAddress & 0x0f,
					(dep->bEndpointAddress & LIBUSB_ENDPOINT_IN) ? "in" : "out");
				if ((dep->bEndpointAddress & LIBUSB_ENDPOINT_IN) && firstIn == -1) {
					firstIn = dep->bEndpointAddress;
				}
				if (!(dep->bEndpointAddress & LIBUSB_ENDPOINT_IN) && firstOut == -1) {
					firstOut = dep->bEndpointAddress;
				}
			}
			if (j == 0 && alt->bInterfaceClass == 0xff && alt->bInterfaceSubClass == 0x43) {
				if (firstIn != -1 && firstOut != -1) {
					find_FF_43 = i;
					inEndpoint = firstIn;
					outEndpoint = firstOut;
				}
			} else if (alt->bInterfaceClass == 0xff && alt->bInterfaceSubClass == 0x43) {
				printf("%04x:%04x iface %u alt 0 ok, but nAlt = %02x unexpected, will keep going\n",
					desc.idVendor, desc.idProduct, i, iface->num_altsetting);
			}
		}
	}

	r = libusb_claim_interface(usbdev, find_FF_43);
	if (r) {
		printf("libusb_claim %04x:%04x failed: %d\n", desc.idVendor, desc.idProduct, r);
		libusb_close(usbdev);
		return r;
	}
	if (dbg) printf("%04x:%04x claimed %d ok\n", desc.idVendor, desc.idProduct, find_FF_43);

	// Because libusb runs in userspace, the get_configuration() value can change at any time up until
	// libusb_claim_interface. This is just good hygiene to double check the value
	cfg = -1;
	r = libusb_get_configuration(usbdev, &cfg);
	if (r) {
		printf("libusb_get_configuration %04x:%04x failed: %d\n", desc.idVendor, desc.idProduct, r);
		libusb_close(usbdev);
		return r;
	}
	if (cfg != 1 || desc.bNumConfigurations != 1) {
		printf("%04x:%04x cfg = %d nCfgs = %02x unexpected, will keep going\n",
			desc.idVendor, desc.idProduct, cfg, desc.bNumConfigurations);
	}
	r = start_stream(ctx, usbdev, inEndpoint, outEndpoint);
	libusb_close(usbdev);
	return r;
}

int start_usb2sock(void)
{
	struct libusb_init_option options[] = {
		{
			// user can override this with LIBUSB_DEBUG=NNN
			.option = LIBUSB_OPTION_LOG_LEVEL,
			//.value.ival = LIBUSB_LOG_LEVEL_DEBUG,
			.value.ival = LIBUSB_LOG_LEVEL_WARNING,
		}
	};
	libusb_set_log_cb(/*ctx=*/ 0, logcb, LIBUSB_LOG_CB_GLOBAL);
	libusb_context * ctx;
	int r = libusb_init_context(&ctx, options, sizeof(options) / sizeof(options[0]));
	if (r < 0) {
		printf("libusb_init_context: %d %s\n", r, libusb_strerror(r));
		return r;
	}
	if (libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
		// even though macOS can do hotplug notifications
		// this program does just fine without them - here they are, for what it's worth
		r = libusb_hotplug_register_callback(ctx,
			LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT,
			/*flags=*/ 0, VENDOR_ID_DJI, DEVICE_ID_GOGGLES,
			/*dev_class=*/ LIBUSB_HOTPLUG_MATCH_ANY,
			set_connected_flag, /*user_data=*/ 0, &hotplug);
	}

	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) {
		printf("socket(AF_INET) tcp socket failed: %d %s\n", errno, strerror(errno));
		return 1;
	}
	r = 1;  // turn on SO_REUSEADDR
	if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (const void *) &r, sizeof(r))) {
		printf("SO_REUSEADDR: tcp socket failed: %d %s\n", errno, strerror(errno));
		return 1;
	}
	r = 1;  // turn on SO_NOSIGPIPE
	if (setsockopt(sockfd, SOL_SOCKET, SO_NOSIGPIPE, (const void *) &r, sizeof(r))) {
		printf("SO_NOSIGPIPE: tcp socket failed: %d %s\n", errno, strerror(errno));
		return 1;
	}
	struct sockaddr_in saddr;
	memset(&saddr, 0, sizeof(saddr));
	saddr.sin_family = AF_INET;
	// When listening on a port, set the host ip address where there is an open port
	if (0) saddr.sin_addr.s_addr = htonl(INADDR_ANY);	// bind to all host ip addresses
		else inet_pton(AF_INET, "127.0.0.1", &saddr.sin_addr);	// bind to 127.0.0.1
	saddr.sin_port = htons(TCP_SERVER_PORT);

	if (bind(sockfd, (struct sockaddr *) &saddr, sizeof(saddr))) {
		printf("bind tcp socket failed: %d %s\n", errno, strerror(errno));
		return 1;
	}

	if (listen(sockfd, 5)) {
		printf("listen tcp socket failed: %d %s\n", errno, strerror(errno));
		return 1;
	}

	if (fcntl(sockfd, F_SETFL, O_NONBLOCK)) {
		printf("O_NONBLOCK tcp socket failed: %d %s\n", errno, strerror(errno));
		return 1;
	}

	for (;;) {
		accept_any_connection();
		recv_and_discard();
		libusb_device ** devs;
		ssize_t cnt = libusb_get_device_list(ctx, &devs);
		r = 0;
		if (cnt >= 0) {
			for (ssize_t i = 0; devs[i] && i < cnt; i++) {
				r = start_with_a_device(ctx, devs[i]);
				if (r == VENDOR_ID_DJI) continue;	// not the correct device
				break;
			}
			libusb_free_device_list(devs, /*unref_devices=*/ 1);
			if (r == 0) break;	// something did a "return 0", time to exit
			if (r != WAIT_FOR_HOTPLUG && r != VENDOR_ID_DJI) {
				// something failed
				break;
			}
			printf("\r\e[KPlease plug in goggles [%u tcp] \"tcp/h264://127.0.0.1:18080\"",
				clientlist_use);
			fflush(stdout);
			// Wait for a hotplug event
			usleep(500 * 1000);
		}
	}

	libusb_exit(ctx);
	return r;
}

int main(int argc, char ** argv) {
	(void)argc;
	(void)argv;
	return start_usb2sock();
}
