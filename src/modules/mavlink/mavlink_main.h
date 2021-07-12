/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * @file mavlink_main.h
 *
 * MAVLink 2.0 protocol interface definition.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#pragma once

#include <pthread.h>
#include <stdbool.h>

#ifdef __PX4_NUTTX
#include <nuttx/fs/fs.h>
#else
#include <arpa/inet.h>
#include <drivers/device/device.h>
#include <sys/socket.h>
#endif

#if defined(CONFIG_NET) || !defined(__PX4_NUTTX)
#include <net/if.h>
#include <netinet/in.h>
#endif

#include <containers/List.hpp>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/radio_status.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_status.h>

#include "mavlink_command_sender.h"
#include "mavlink_events.h"
#include "mavlink_messages.h"
#include "mavlink_receiver.h"
#include "mavlink_shell.h"
#include "mavlink_ulog.h"

#define DEFAULT_BAUD_RATE       57600
#define DEFAULT_DEVICE_NAME     "/dev/ttyS1"

#define HASH_PARAM              "_HASH_CHECK"

#if defined(CONFIG_NET) || defined(__PX4_POSIX)
# define MAVLINK_UDP
# define DEFAULT_REMOTE_PORT_UDP 14550 ///< GCS port per MAVLink spec
#endif // CONFIG_NET || __PX4_POSIX

enum class Protocol {
	SERIAL = 0,
#if defined(MAVLINK_UDP)
	UDP,
#endif // MAVLINK_UDP
};

using namespace time_literals;

class Mavlink final : public ModuleParams
{

public:
	/**
	 * Constructor
	 */
	Mavlink();

	/**
	 * Destructor, also kills the mavlinks task.
	 */
	~Mavlink();

	/**
	* Start the mavlink task.
	 *
	 * @return OK on success.
	 */
	static int		start(int argc, char *argv[]);

	/**
	 * Display the mavlink status.
	 */
	void			display_status();

	/**
	 * Display the status of all enabled streams.
	 */
	void			display_status_streams();

	static int		stream_command(int argc, char *argv[]);

	static int		instance_count();

	static Mavlink		*new_instance();

	static Mavlink 		*get_instance_for_device(const char *device_name);

	mavlink_message_t 	*get_buffer() { return &_mavlink_buffer; }

	mavlink_status_t 	*get_status() { return &_mavlink_status; }

	/**
	 * Set the MAVLink version
	 *
	 * Currently supporting v1 and v2
	 *
	 * @param version MAVLink version
	 */
	void			set_proto_version(unsigned version);

	static int		destroy_all_instances();

	static int		get_status_all_instances(bool show_streams_status);

	static bool		serial_instance_exists(const char *device_name, Mavlink *self);

	static bool		component_was_seen(int system_id, int component_id, Mavlink *self = nullptr);

	static void		forward_message(const mavlink_message_t *msg, Mavlink *self);

	int			get_uart_fd() const { return _uart_fd; }

	/**
	 * Get the MAVLink system id.
	 *
	 * @return The system ID of this vehicle
	 */
	int			get_system_id() const { return mavlink_system.sysid; }

	/**
	 * Get the MAVLink component id.
	 *
	 * @return The component ID of this vehicle
	 */
	int			get_component_id() const { return mavlink_system.compid; }

	const char *_device_name{DEFAULT_DEVICE_NAME};

	enum MAVLINK_MODE {
		MAVLINK_MODE_NORMAL = 0,
		MAVLINK_MODE_CUSTOM,
		MAVLINK_MODE_ONBOARD,
		MAVLINK_MODE_OSD,
		MAVLINK_MODE_MAGIC,
		MAVLINK_MODE_CONFIG,
		MAVLINK_MODE_IRIDIUM,
		MAVLINK_MODE_MINIMAL,
		MAVLINK_MODE_EXTVISION,
		MAVLINK_MODE_EXTVISIONMIN,
		MAVLINK_MODE_GIMBAL,
		MAVLINK_MODE_ONBOARD_LOW_BANDWIDTH,
		MAVLINK_MODE_COUNT
	};

	enum BROADCAST_MODE {
		BROADCAST_MODE_OFF = 0,
		BROADCAST_MODE_ON,
		BROADCAST_MODE_MULTICAST
	};

	enum FLOW_CONTROL_MODE {
		FLOW_CONTROL_OFF = 0,
		FLOW_CONTROL_AUTO,
		FLOW_CONTROL_ON
	};

	static const char *mavlink_mode_str(enum MAVLINK_MODE mode)
	{
		switch (mode) {
		case MAVLINK_MODE_NORMAL:
			return "Normal";

		case MAVLINK_MODE_CUSTOM:
			return "Custom";

		case MAVLINK_MODE_ONBOARD:
			return "Onboard";

		case MAVLINK_MODE_OSD:
			return "OSD";

		case MAVLINK_MODE_MAGIC:
			return "Magic";

		case MAVLINK_MODE_CONFIG:
			return "Config";

		case MAVLINK_MODE_IRIDIUM:
			return "Iridium";

		case MAVLINK_MODE_MINIMAL:
			return "Minimal";

		case MAVLINK_MODE_EXTVISION:
			return "ExtVision";

		case MAVLINK_MODE_EXTVISIONMIN:
			return "ExtVisionMin";

		case MAVLINK_MODE_GIMBAL:
			return "Gimbal";

		case MAVLINK_MODE_ONBOARD_LOW_BANDWIDTH:
			return "OnboardLowBandwidth";

		default:
			return "Unknown";
		}
	}

	enum MAVLINK_MODE	get_mode() { return _mode; }

	bool			get_hil_enabled() { return _hil_enabled; }

	bool			get_use_hil_gps() { return _param_mav_usehilgps.get(); }

	bool			get_forward_externalsp() { return _param_mav_fwdextsp.get(); }

	bool			get_flow_control_enabled() { return _flow_control_mode; }

	bool			get_forwarding_on() { return _forwarding_on; }

	bool			is_connected() { return _tstatus.heartbeat_type_gcs; }

#if defined(MAVLINK_UDP)
	static Mavlink 		*get_instance_for_network_port(unsigned long port);

	bool			broadcast_enabled() { return _mav_broadcast == BROADCAST_MODE_ON; }

	bool			multicast_enabled() { return _mav_broadcast == BROADCAST_MODE_MULTICAST; }
#endif // MAVLINK_UDP

	/**
	 * Set the boot complete flag on all instances
	 *
	 * Setting the flag unblocks parameter transmissions, which are gated
	 * beforehand to ensure that the system is fully initialized.
	 */
	static void		set_boot_complete();

	/**
	 * Get the free space in the transmit buffer
	 *
	 * @return free space in the UART TX buffer
	 */
	unsigned		get_free_tx_buf();

	static int		start_helper(int argc, char *argv[]);

	/**
	 * Enable / disable Hardware in the Loop simulation mode.
	 *
	 * @param hil_enabled	The new HIL enable/disable state.
	 * @return		OK if the HIL state changed, ERROR if the
	 *			requested change could not be made or was
	 *			redundant.
	 */
	int			set_hil_enabled(bool hil_enabled);

	/**
	 * Set manual input generation mode
	 *
	 * Set to true to generate RC_INPUT messages on the system bus from
	 * MAVLink messages.
	 *
	 * @param generation_enabled If set to true, generate RC_INPUT messages
	 */
	void			set_generate_virtual_rc_input(bool generation_enabled) { _generate_rc = generation_enabled; }

	/**
	 * Set communication protocol for this mavlink instance
	 */
	void 			set_protocol(Protocol p) { _protocol = p; }

	/**
	 * Get the manual input generation mode
	 *
	 * @return true if manual inputs should generate RC data
	 */
	bool			should_generate_virtual_rc_input() { return _generate_rc; }

	/**
	 * This is the beginning of a MAVLINK_START_UART_SEND/MAVLINK_END_UART_SEND transaction
	 */
	void 			send_start(int length);

	/**
	 * Buffer bytes to send out on the link.
	 */
	void			send_bytes(const uint8_t *buf, unsigned packet_len);

	/**
	 * Flush the transmit buffer and send one MAVLink packet
	 */
	void             	send_finish();

	/**
	 * Resend message as is, don't change sequence number and CRC.
	 */
	void			resend_message(mavlink_message_t *msg) { _mavlink_resend_uart(_channel, msg); }

	void			handle_message(const mavlink_message_t *msg);

	int			get_instance_id() const { return _instance_id; }

	/**
	 * Enable / disable hardware flow control.
	 *
	 * @param enabled	True if hardware flow control should be enabled
	 */
	int			setup_flow_control(enum FLOW_CONTROL_MODE enabled);

	mavlink_channel_t	get_channel() const { return _channel; }

	void			configure_stream_threadsafe(const char *stream_name, float rate = -1.0f);

	orb_advert_t		*get_mavlink_log_pub() { return &_mavlink_log_pub; }

	/**
	 * Send a status text with loglevel INFO
	 *
	 * @param string the message to send (will be capped by mavlink max string length)
	 */
	void			send_statustext_info(const char *string);

	/**
	 * Send a status text with loglevel CRITICAL
	 *
	 * @param string the message to send (will be capped by mavlink max string length)
	 */
	void			send_statustext_critical(const char *string);

	/**
	 * Send a status text with loglevel EMERGENCY
	 *
	 * @param string the message to send (will be capped by mavlink max string length)
	 */
	void			send_statustext_emergency(const char *string);

	/**
	 * Send a status text with loglevel, the difference from mavlink_log_xxx() is that message sent
	 * only on this mavlink connection. Useful for reporting communication specific, not system-wide info
	 * only to client interested in it. Message will be not sent immediately but queued in buffer as
	 * for mavlink_log_xxx().
	 *
	 * @param string the message to send (will be capped by mavlink max string length)
	 * @param severity the log level
	 */
	void			send_statustext(unsigned char severity, const char *string);

	/**
	 * Send the capabilities of this autopilot in terms of the MAVLink spec
	 */
	bool 			send_autopilot_capabilities();

	/**
	 * Send the protocol version of MAVLink
	 */
	void			send_protocol_version();

	List<MavlinkStream *> &get_streams() { return _streams; }

	float			get_rate_mult() const { return _rate_mult; }

	float			get_baudrate() { return _baudrate; }

	/* Functions for waiting to start transmission until message received. */
	void			set_has_received_messages(bool received_messages) { _received_messages = received_messages; }
	bool			get_has_received_messages() { return _received_messages; }
	void			set_wait_to_transmit(bool wait) { _wait_to_transmit = wait; }
	bool			get_wait_to_transmit() { return _wait_to_transmit; }
	bool			should_transmit() { return (_transmitting_enabled && _boot_complete && (!_wait_to_transmit || (_wait_to_transmit && _received_messages))); }

	bool			message_buffer_write(const void *ptr, int size);

	void			lockMessageBufferMutex(void) { pthread_mutex_lock(&_message_buffer_mutex); }
	void			unlockMessageBufferMutex(void) { pthread_mutex_unlock(&_message_buffer_mutex); }

	/**
	 * Count transmitted bytes
	 */
	void			count_txbytes(unsigned n) { _bytes_tx += n; };

	/**
	 * Count bytes not transmitted because of errors
	 */
	void			count_txerrbytes(unsigned n) { _bytes_txerr += n; };

	/**
	 * Count received bytes
	 */
	void			count_rxbytes(unsigned n) { _bytes_rx += n; };

	/**
	 * Get the receive status of this MAVLink link
	 */
	telemetry_status_s	&telemetry_status() { return _tstatus; }
	void                    telemetry_status_updated() { _tstatus_updated = true; }

	void			set_telemetry_status_type(uint8_t type) { _tstatus.type = type; }

	void			update_radio_status(const radio_status_s &radio_status);

	unsigned		get_system_type() { return _param_mav_type.get(); }

	Protocol 		get_protocol() const { return _protocol; }

	int 			get_socket_fd() { return _socket_fd; };

	bool			_task_should_exit{false};	/**< Mavlink task should exit iff true. */

#if defined(MAVLINK_UDP)
	unsigned short		get_network_port() { return _network_port; }

	unsigned short		get_remote_port() { return _remote_port; }

	const in_addr		query_netmask_addr(const int socket_fd, const ifreq &ifreq);

	const in_addr		compute_broadcast_addr(const in_addr &host_addr, const in_addr &netmask_addr);

	struct sockaddr_in 	&get_client_source_address() { return _src_addr; }

	void			set_client_source_initialized() { _src_addr_initialized = true; }

	bool			get_client_source_initialized() { return _src_addr_initialized; }
#endif

	uint64_t		get_start_time() { return _mavlink_start_time; }

	static bool		boot_complete() { return _boot_complete; }

	bool			is_usb_uart() { return _is_usb_uart; }

	int			get_data_rate()		{ return _datarate; }
	void			set_data_rate(int rate) { if (rate > 0) { _datarate = rate; } }

	unsigned		get_main_loop_delay() const { return _main_loop_delay; }

	/** get the Mavlink shell. Create a new one if there isn't one. It is *always* created via MavlinkReceiver thread.
	 *  Returns nullptr if shell cannot be created */
	MavlinkShell		*get_shell();
	/** close the Mavlink shell if it is open */
	void			close_shell();

	/** get ulog streaming if active, nullptr otherwise */
	MavlinkULog		*get_ulog_streaming() { return _mavlink_ulog; }
	void			try_start_ulog_streaming(uint8_t target_system, uint8_t target_component)
	{
		if (_mavlink_ulog) { return; }

		_mavlink_ulog = MavlinkULog::try_start(_datarate, 0.7f, target_system, target_component);
	}
	void			request_stop_ulog_streaming()
	{
		if (_mavlink_ulog) { _mavlink_ulog_stop_requested.store(true); }
	}

	const events::SendProtocol &get_events_protocol() const { return _events; };
	bool ftp_enabled() const { return _ftp_on; }

	bool hash_check_enabled() const { return _param_mav_hash_chk_en.get(); }
	bool forward_heartbeats_enabled() const { return _param_mav_hb_forw_en.get(); }
	bool odometry_loopback_enabled() const { return _param_mav_odom_lp.get(); }

	bool failure_injection_enabled() const { return _param_sys_failure_injection_enabled.get(); }

	struct ping_statistics_s {
		uint64_t last_ping_time;
		uint32_t last_ping_seq;
		uint32_t dropped_packets;
		float last_rtt;
		float mean_rtt;
		float max_rtt;
		float min_rtt;
	};

	/**
	 * Get the ping statistics of this MAVLink link
	 */
	struct ping_statistics_s &get_ping_statistics() { return _ping_stats; }

	static hrt_abstime &get_first_start_time() { return _first_start_time; }

	bool radio_status_critical() const { return _radio_status_critical; }

private:
	MavlinkReceiver 	_receiver;
	int			_instance_id{-1};

	bool			_transmitting_enabled{true};
	bool			_transmitting_enabled_commanded{false};
	bool			_first_heartbeat_sent{false};

	orb_advert_t		_mavlink_log_pub{nullptr};

	uORB::Publication<vehicle_command_ack_s> _vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};
	uORB::PublicationMulti<telemetry_status_s> _telemetry_status_pub{ORB_ID(telemetry_status)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _vehicle_command_ack_sub{ORB_ID(vehicle_command_ack)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	bool			_task_running{true};
	static bool		_boot_complete;

	static constexpr int	MAVLINK_MIN_INTERVAL{1500};
	static constexpr int	MAVLINK_MAX_INTERVAL{10000};
	static constexpr float	MAVLINK_MIN_MULTIPLIER{0.0005f};

	mavlink_message_t	_mavlink_buffer {};
	mavlink_status_t	_mavlink_status {};

	/* states */
	bool			_hil_enabled{false};		/**< Hardware In the Loop mode */
	bool			_generate_rc{false};		/**< Generate RC messages from manual input MAVLink messages */
	bool			_is_usb_uart{false};		/**< Port is USB */
	bool			_wait_to_transmit{false};  	/**< Wait to transmit until received messages. */
	bool			_received_messages{false};	/**< Whether we've received valid mavlink messages. */

	unsigned		_main_loop_delay{1000};	/**< mainloop delay, depends on data rate */

	List<MavlinkStream *>		_streams;

	MavlinkShell		*_mavlink_shell{nullptr};
	MavlinkULog		*_mavlink_ulog{nullptr};
	static events::EventBuffer	*_event_buffer;
	events::SendProtocol		_events{*_event_buffer, *this};

	px4::atomic_bool	_mavlink_ulog_stop_requested{false};

	MAVLINK_MODE 		_mode{MAVLINK_MODE_NORMAL};

	mavlink_channel_t	_channel{MAVLINK_COMM_0};

	bool			_forwarding_on{false};
	bool			_ftp_on{false};
	bool			_use_software_mav_throttling{false};

	int			_uart_fd{-1};

	int			_baudrate{57600};
	int			_datarate{1000};		///< data rate for normal streams (attitude, position, etc.)
	float			_rate_mult{1.0f};

	bool			_radio_status_available{false};
	bool			_radio_status_critical{false};
	float			_radio_status_mult{1.0f};

	/**
	 * If the queue index is not at 0, the queue sending
	 * logic will send parameters from the current index
	 * to len - 1, the end of the param list.
	 */
	unsigned int		_mavlink_param_queue_index{0};

	bool			_mavlink_link_termination_allowed{false};

	char			*_subscribe_to_stream{nullptr};
	float			_subscribe_to_stream_rate{0.0f};  ///< rate of stream to subscribe to (0=disable, -1=unlimited, -2=default)
	bool			_udp_initialised{false};

	FLOW_CONTROL_MODE	_flow_control_mode{Mavlink::FLOW_CONTROL_OFF};

	uint64_t		_last_write_success_time{0};
	uint64_t		_last_write_try_time{0};
	uint64_t		_mavlink_start_time{0};
	int32_t			_protocol_version_switch{-1};
	int32_t			_protocol_version{0};

	unsigned		_bytes_tx{0};
	unsigned		_bytes_txerr{0};
	unsigned		_bytes_rx{0};
	hrt_abstime		_bytes_timestamp{0};

#if defined(MAVLINK_UDP)
	BROADCAST_MODE		_mav_broadcast {BROADCAST_MODE_OFF};

	sockaddr_in		_myaddr {};
	sockaddr_in		_src_addr {};
	sockaddr_in		_bcast_addr {};

	bool			_src_addr_initialized{false};
	bool			_broadcast_address_found{false};
	bool			_broadcast_address_not_found_warned{false};
	bool			_broadcast_failed_warned{false};

	unsigned short		_network_port{14556};
	unsigned short		_remote_port{DEFAULT_REMOTE_PORT_UDP};
#endif // MAVLINK_UDP

	uint8_t			_buf[MAVLINK_MAX_PACKET_LEN] {};
	unsigned		_buf_fill{0};

	bool			_tx_buffer_low{false};

	const char 		*_interface_name{nullptr};

	int			_socket_fd{-1};
	Protocol		_protocol{Protocol::SERIAL};

	radio_status_s		_rstatus {};
	telemetry_status_s	_tstatus {};
	bool                    _tstatus_updated{false};

	ping_statistics_s	_ping_stats {};

	struct mavlink_message_buffer {
		int write_ptr;
		int read_ptr;
		int size;
		char *data;
	};

	mavlink_message_buffer	_message_buffer {};

	pthread_mutex_t		_message_buffer_mutex {};
	pthread_mutex_t		_send_mutex {};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MAV_SYS_ID>) _param_mav_sys_id,
		(ParamInt<px4::params::MAV_COMP_ID>) _param_mav_comp_id,
		(ParamInt<px4::params::MAV_PROTO_VER>) _param_mav_proto_ver,
		(ParamInt<px4::params::MAV_SIK_RADIO_ID>) _param_sik_radio_id,
		(ParamInt<px4::params::MAV_TYPE>) _param_mav_type,
		(ParamBool<px4::params::MAV_USEHILGPS>) _param_mav_usehilgps,
		(ParamBool<px4::params::MAV_FWDEXTSP>) _param_mav_fwdextsp,
		(ParamBool<px4::params::MAV_HASH_CHK_EN>) _param_mav_hash_chk_en,
		(ParamBool<px4::params::MAV_HB_FORW_EN>) _param_mav_hb_forw_en,
		(ParamBool<px4::params::MAV_ODOM_LP>) _param_mav_odom_lp,
		(ParamInt<px4::params::MAV_RADIO_TOUT>)      _param_mav_radio_timeout,
		(ParamInt<px4::params::SYS_HITL>) _param_sys_hitl,
		(ParamBool<px4::params::SYS_FAILURE_EN>) _param_sys_failure_injection_enabled
	)

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": tx run elapsed")};                      /**< loop performance counter */
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": tx run interval")};           /**< loop interval performance counter */
	perf_counter_t _send_byte_error_perf{perf_alloc(PC_COUNT, MODULE_NAME": send_bytes error")};           /**< send bytes error count */

	void			mavlink_update_parameters();

	int mavlink_open_uart(const int baudrate = DEFAULT_BAUD_RATE,
			      const char *uart_name = DEFAULT_DEVICE_NAME,
			      const FLOW_CONTROL_MODE flow_control = FLOW_CONTROL_AUTO);

	static constexpr unsigned RADIO_BUFFER_CRITICAL_LOW_PERCENTAGE = 25;
	static constexpr unsigned RADIO_BUFFER_LOW_PERCENTAGE = 35;
	static constexpr unsigned RADIO_BUFFER_HALF_PERCENTAGE = 50;

	static hrt_abstime _first_start_time;

	/**
	 * Configure a single stream.
	 * @param stream_name
	 * @param rate streaming rate in Hz, -1 = unlimited rate
	 * @return 0 on success, <0 on error
	 */
	int configure_stream(const char *stream_name, const float rate = -1.0f);

	/**
	 * Configure default streams according to _mode for either all streams or only a single
	 * stream.
	 * @param configure_single_stream: if nullptr, configure all streams, else only a single stream
	 * @return 0 on success, <0 on error
	 */
	int configure_streams_to_default(const char *configure_single_stream = nullptr);

	int message_buffer_init(int size);

	void message_buffer_destroy();

	int message_buffer_count();

	int message_buffer_is_empty() const { return (_message_buffer.read_ptr == _message_buffer.write_ptr); }

	int message_buffer_get_ptr(void **ptr, bool *is_part);

	void message_buffer_mark_read(int n) { _message_buffer.read_ptr = (_message_buffer.read_ptr + n) % _message_buffer.size; }

	void pass_message(const mavlink_message_t *msg);

	void publish_telemetry_status();

	void check_requested_subscriptions();

	/**
	 * Reconfigure a SiK radio if requested by MAV_SIK_RADIO_ID
	 *
	 * This convenience function allows to re-configure a connected
	 * SiK radio without removing it from the main system harness.
	 */
	void configure_sik_radio();

	/**
	 * Update rate mult so total bitrate will be equal to _datarate.
	 */
	void update_rate_mult();

#if defined(MAVLINK_UDP)
	void find_broadcast_address();

	void init_udp();
#endif // MAVLINK_UDP


	void set_channel();

	bool set_instance_id();

	/**
	 * Main mavlink task.
	 */
	int task_main(int argc, char *argv[]);

	// Disallow copy construction and move assignment.
	Mavlink(const Mavlink &) = delete;
	Mavlink operator=(const Mavlink &) = delete;
};
