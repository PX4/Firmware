/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file netman.cpp
 * Network Manager driver.
 *
 * @author David Sidrane
 */

#include <px4_platform_common/px4_config.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <ctype.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fsutils/ipcfg.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <arpa/inet.h>
#include <px4_platform_common/shutdown.h>

constexpr char DEFAULT_NETMAN_CONFIG[] = "/fs/microsd/net.cfg";
#if defined(CONFIG_NETINIT_DHCPC)
#  define DEFAULT_PROTO    IPv4PROTO_FALLBACK
#  define DEFAULT_IP      0XC0A80003  // 192.168.0.3
#else
#  define DEFAULT_PROTO   IPv4PROTO_STATIC
#  define DEFAULT_IP      CONFIG_NETINIT_IPADDR
#endif
#define DEFAULT_NETMASK   CONFIG_NETINIT_NETMASK
#define DEFAULT_ROUTER    CONFIG_NETINIT_DRIPADDR
#define DEFAULT_DNS       CONFIG_NETINIT_DNSIPADDR

static void usage(const char *reason);
__BEGIN_DECLS
__EXPORT int  netman_main(int argc, char *argv[]);
__END_DECLS

class net_params
{
private:

	class ipl
	{
		const char *_keyword;
	public:

		union {
			int32_t  l;
			uint32_t u;
			struct in_addr a;
			uint8_t b[sizeof(int32_t) + 1];
			enum ipv4cfg_bootproto_e e;
		};

		const char *keyword() { return _keyword;}
		ipl() = delete;
		ipl(const char *w)	{ _keyword = w;}

		const char *to_str()
		{
			return inet_ntoa(a);
		}

		const char *name()
		{
			b[arraySize(b) - 1] = '\0';
			return (const char *)b;
		}

		void set_name(const char *name)
		{
			unsigned int i;

			for (i = 0; i < arraySize(b) - 1; i++) {
				b[i] = name[i];
			}

			b[i] = '\0';
		}

		const char *protocol()
		{
			return e == IPv4PROTO_STATIC ? "static" : (e  == IPv4PROTO_DHCP) ? "dhcp" : "fallback";
		}

		const char *parseProtocol(const char *ps)
		{
			char *p = strstr(ps, "dhcp");

			if (p) {
				e = IPv4PROTO_DHCP;

			} else {

				p = strstr(ps, "static");

				if (p) {
					e = IPv4PROTO_STATIC;

				} else {

					p = strstr(ps, "fallback");

					if (p) {
						e = IPv4PROTO_FALLBACK;
					}
				}
			}

			return ps;
		}


		const char *parse(const char *cp)
		{
			u = inet_addr(cp);
			return cp;
		}

		const char *parse(const char *buffer, const char *end)
		{
			char *ps = strstr(buffer, keyword());

			if (ps) {
				int len = strlen(keyword());

				if (ps + len < end) {
					ps += len;
					isalpha(*ps) ? parseProtocol(ps) : parse(ps);

				} else {
					ps = nullptr;
				}
			}

			return ps;
		}
	};


public:

	ipl device{"DEVICE="};
	ipl proto{"BOOTPROTO="};
	ipl netmask{"NETMASK="};
	ipl ipaddr{"IPADDR="};
	ipl router{"ROUTER="};
	ipl dnsaddr{"DNS="};


	net_params() = default;

	~net_params() {}

	net_params &operator = (const ipv4cfg_s &ipcfg)
	{
		proto.e  =    ipcfg.proto;
		ipaddr.u  =   ipcfg.ipaddr;
		netmask.u =   ipcfg.netmask;
		router.u  =   ipcfg.router;
		dnsaddr.u =   ipcfg.dnsaddr;
		return *this;
	}


	int read(const char *netdev)
	{
		struct ipv4cfg_s ipcfg;
		int rv = ipcfg_read(netdev, (FAR struct ipcfg_s *) &ipcfg, AF_INET);

		if (rv == -EINVAL ||
		    (rv == OK  && (ipcfg.proto > IPv4PROTO_FALLBACK || ipcfg.ipaddr == 0xffffffff))) {
			// Build a default
			ipcfg.ipaddr  = HTONL(DEFAULT_IP);
			ipcfg.netmask = HTONL(DEFAULT_NETMASK);
			ipcfg.router  = HTONL(DEFAULT_ROUTER);
			ipcfg.dnsaddr = HTONL(DEFAULT_DNS);
			ipcfg.proto   = DEFAULT_PROTO;
			rv = -ENOENT;
		}

		device.set_name(netdev);
		*this = ipcfg;
		return rv;
	}

	int write()
	{
		struct ipv4cfg_s ipcfg;
		ipcfg.proto   = proto.e;
		ipcfg.ipaddr  = ipaddr.u;
		ipcfg.netmask = netmask.u;
		ipcfg.router  = router.u;
		ipcfg.dnsaddr = dnsaddr.u;
		return ipcfg_write(device.name(), (FAR struct ipcfg_s *) &ipcfg, AF_INET);
	}
};

int save(const char *path, const char *netdev)
{

	net_params config;
	constexpr int lsz = 80;
	char line[lsz + 1];
	int len;

	int rv = config.read(netdev);

	int fd =  fileno(stdout);

	if (path != nullptr) {
		fd = open(path, O_CREAT | O_WRONLY | O_TRUNC, PX4_O_MODE_666);
	}

	if (fd < 0) {
		PX4_ERR("Can not create file %s", path);
		goto errout;
	}

	len = snprintf(line,  lsz, "%s%s\n", config.device.keyword(), netdev);

	if (len != write(fd, line, len)) {
		goto errout;
	}

	len = snprintf(line,  lsz, "%s%s\n",  config.proto.keyword(), config.proto.protocol());

	if (len != write(fd, line, len)) {
		goto errout;
	}

	len = snprintf(line,  lsz, "%s%s\n", config.netmask.keyword(), config.netmask.to_str());

	if (len != write(fd, line, len)) {
		goto errout;
	}

	len = snprintf(line,  lsz, "%s%s\n", config.ipaddr.keyword(), config.ipaddr.to_str());

	if (len != write(fd, line, len)) {
		goto errout;
	}

	len = snprintf(line,  lsz, "%s%s\n", config.router.keyword(), config.router.to_str());

	if (len != write(fd, line, len)) {
		goto errout;
	}

	len = snprintf(line,  lsz, "%s%s\n", config.dnsaddr.keyword(), config.dnsaddr.to_str());

	if (len != write(fd, line, len)) {
		rv = -errno;

	} else {
		close(fd);
		return rv;
	}

errout: {
		rv = -errno;

		if (fd >= 0) {
			close(fd);
		}

		return rv;
	}
}

int update(const char *path, const char *netdev)
{
	net_params config;
	struct stat sb;
	FAR char *lines = nullptr;
	int fd = -1;
	int rv = OK;
	off_t fbuf_size;

	// First do we have a binary config stored?

	rv = config.read(netdev);

	if (rv == -ENOENT) {
		goto write_reboot;
	}

	// Is there a config file update?

	if (stat(path, &sb) < 0) {
		return 0;
	}

	// Allocate file size plus a null.

	fbuf_size = sb.st_size + 1;
	lines = (char *) malloc(fbuf_size);

	if (!lines) {
		return -errno;
	}

	// Null Fill buffer

	memset(lines, 0, fbuf_size);

	fd = open(path, O_RDONLY);

	if (fd < 0) {
		rv = -errno;
		goto errout;
	}

	if (read(fd, lines, sb.st_size) != sb.st_size) {
		rv = -errno;
		goto errout;
	}

	close(fd);
	fd = -1;
	unlink(path);

	config.proto.parse(lines, &lines[sb.st_size - 1]);
	config.netmask.parse(lines, &lines[sb.st_size - 1]);
	config.ipaddr.parse(lines, &lines[sb.st_size - 1]);
	config.router.parse(lines, &lines[sb.st_size - 1]);
	config.dnsaddr.parse(lines, &lines[sb.st_size - 1]);

write_reboot:
	rv = config.write();

	if (rv < 0) {
		PX4_ERR("Network could not be saved!");
		return -errno;
	}


	PX4_INFO("Network settings updated, rebooting....\n");


	// Ensure the message is seen.

	sleep(1);

	px4_reboot_request(false);

	while (1) { px4_usleep(1); } // this command should not return on success

errout:

	if (lines) {
		free(lines);
	}

	if (fd >= 0) {
		close(fd);
	}

	return rv;
}

static void usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
  ### Description
  Network configuration manager saves the network settings in non-volatile
  memory. On boot the `update` option will be run. If a network configuration
  does not exist. The default setting will be saved in non-volatile and the
  system rebooted.
  On Subsequent boots, the `update` option will check for the existence of
  `net.cfg` in the root of the SD Card.  It will saves the network settings
  from `net.cfg` in non-volatile memory, delete the file and reboot the system.

  The `save` option will `net.cfg` on the SD Card. Use this to edit the settings.
  The  `show` option will display the network settings  to the console.

  ### Examples
  $ netman save           # Save the parameters to the SD card.
  $ netman show           # display current settings.
  $ netman update -i eth0 # do an update
)DESCR_STR");
    PRINT_MODULE_USAGE_NAME("netman", "system");
    PRINT_MODULE_USAGE_COMMAND_DESCR("show", "Display the current persistent network settings to the console.");
    PRINT_MODULE_USAGE_COMMAND_DESCR("update","Check SD card for net.cfg and update network persistent network settings.");
    PRINT_MODULE_USAGE_COMMAND_DESCR("save", "Save the current network parameters to the SD card.");
    PRINT_MODULE_USAGE_PARAM_STRING('i',"eth0", nullptr, "Set the interface name", true);
}

int netman_main(int argc, char *argv[])
{
  const char *path = DEFAULT_NETMAN_CONFIG;
  const char *netdev = "eth0";
  int ch;
  int rv = 1;

  if (argc < 2) {
    usage(nullptr);
    return 1;
  }

  int myoptind = 1;
  const char *myoptarg = nullptr;

  while ((ch = px4_getopt(argc, argv, "i:", &myoptind, &myoptarg)) != EOF) {
    switch (ch) {

      case 'i':
        netdev = myoptarg;
          break;

     default:
      usage(nullptr);
      return rv;
    }
  }

  if (myoptind >= argc) {
    usage(nullptr);
    return rv;
  }

  if (strcmp("save", argv[myoptind]) == 0)
      {
        rv = save(path, netdev);
      }
  else if (strcmp("update", argv[myoptind]) == 0)
      {
      rv = update(path, netdev);
      }
  else if (strcmp("show", argv[myoptind]) == 0)
      {
      rv = save(nullptr, netdev);
      }
  return rv;
}
