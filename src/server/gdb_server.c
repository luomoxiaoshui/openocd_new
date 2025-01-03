// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007-2010 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 by Broadcom Corporation                            *
 *   Evan Hunter - ehunter@broadcom.com                                    *
 *                                                                         *
 *   Copyright (C) ST-Ericsson SA 2011                                     *
 *   michel.jaouen@stericsson.com : smp minimum support                    *
 *                                                                         *
 *   Copyright (C) 2013 Andes Technology                                   *
 *   Hsiangkai Wang <hkwang@andestech.com>                                 *
 *                                                                         *
 *   Copyright (C) 2013 Franck Jullien                                     *
 *   elec4fun@gmail.com                                                    *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <target/breakpoints.h>
#include <target/target_request.h>
#include <target/register.h>
#include <target/target.h>
#include <target/target_type.h>
#include <target/semihosting_common.h>
#include "server.h"
#include <flash/nor/core.h>
#include "gdb_server.h"
#include <target/image.h>
#include <jtag/jtag.h>
#include "rtos/rtos.h"
#include "target/smp.h"

/**
 * @file
 * GDB server implementation.
 *
 * This implements the GDB Remote Serial Protocol, over TCP connections,
 * giving GDB access to the JTAG or other hardware debugging facilities
 * found in most modern embedded processors.
 */

enum gdb_output_flag {
	/* GDB doesn't accept 'O' packets */
	GDB_OUTPUT_NO,
	/* GDB accepts 'O' packets */
	GDB_OUTPUT_ALL,
};

struct target_desc_format {
	char *tdesc;
	uint32_t tdesc_length;
};

/* private connection data for GDB */
struct gdb_connection {
	char buffer[GDB_BUFFER_SIZE + 1]; /* Extra byte for null-termination */
	char *buf_p;
	int buf_cnt;
	bool ctrl_c;
	enum target_state frontend_state;
	struct image *vflash_image;
	bool closed;
	bool busy;
	int noack_mode;
	/* set flag to true if you want the next stepi to return immediately.
	 * allowing GDB to pick up a fresh set of register values from the target
	 * without modifying the target state. */
	bool sync;
	/* We delay reporting memory write errors until next step/continue or memory
	 * write. This improves performance of gdb load significantly as the GDB packet
	 * can be replied immediately and a new GDB packet will be ready without delay
	 * (ca. 10% or so...). */
	bool mem_write_error;
	/* with extended-remote it seems we need to better emulate attach/detach.
	 * what this means is we reply with a W stop reply after a kill packet,
	 * normally we reply with a S reply via gdb_last_signal_packet.
	 * as a side note this behaviour only effects gdb > 6.8 */
	bool attached;
	/* set when extended protocol is used */
	bool extended_protocol;
	/* temporarily used for target description support */
	struct target_desc_format target_desc;
	/* temporarily used for thread list support */
	char *thread_list;
	/* flag to mask the output from gdb_log_callback() */
	enum gdb_output_flag output_flag;
};

#if 0
#define _DEBUG_GDB_IO_
#endif

static struct gdb_connection *current_gdb_connection;

static int gdb_breakpoint_override;
static enum breakpoint_type gdb_breakpoint_override_type;

static int gdb_error(struct connection *connection, int retval);
static char *gdb_port;
static char *gdb_port_next;

static void gdb_log_callback(void *priv, const char *file, unsigned line,
		const char *function, const char *string);

static void gdb_sig_halted(struct connection *connection);

/* number of gdb connections, mainly to suppress gdb related debugging spam
 * in helper/log.c when no gdb connections are actually active */
int gdb_actual_connections;

/* set if we are sending a memory map to gdb
 * via qXfer:memory-map:read packet */
/* enabled by default*/
static int gdb_use_memory_map = 1;
/* enabled by default*/
static int gdb_flash_program = 1;

/* if set, data aborts cause an error to be reported in memory read packets
 * see the code in gdb_read_memory_packet() for further explanations.
 * Disabled by default.
 */
static int gdb_report_data_abort;
/* If set, errors when accessing registers are reported to gdb. Disabled by
 * default. */
static int gdb_report_register_access_error;

/* set if we are sending target descriptions to gdb
 * via qXfer:features:read packet */
/* enabled by default */
static int gdb_use_target_description = 1;

/* current processing free-run type, used by file-I/O */
static char gdb_running_type;

static int gdb_last_signal(struct target *target)
{
	switch (target->debug_reason) {
		case DBG_REASON_DBGRQ:
			return 0x2;		/* SIGINT */
		case DBG_REASON_BREAKPOINT:
		case DBG_REASON_WATCHPOINT:
		case DBG_REASON_WPTANDBKPT:
			return 0x05;	/* SIGTRAP */
		case DBG_REASON_SINGLESTEP:
			return 0x05;	/* SIGTRAP */
		case DBG_REASON_EXC_CATCH:
			return 0x05;
		case DBG_REASON_NOTHALTED:
			return 0x0;		/* no signal... shouldn't happen */
		default:
			LOG_USER("undefined debug reason %d - target needs reset",
					target->debug_reason);
			return 0x0;
	}
}

static int check_pending(struct connection *connection,
		int timeout_s, int *got_data)
{
	/* a non-blocking socket will block if there is 0 bytes available on the socket,
	 * but return with as many bytes as are available immediately
	 */
	struct timeval tv;
	fd_set read_fds;
	struct gdb_connection *gdb_con = connection->priv;
	int t;
	if (!got_data)
		got_data = &t;
	*got_data = 0;

	if (gdb_con->buf_cnt > 0) {
		*got_data = 1;
		return ERROR_OK;
	}

	FD_ZERO(&read_fds);
	FD_SET(connection->fd, &read_fds);

	tv.tv_sec = timeout_s;
	tv.tv_usec = 0;
	if (socket_select(connection->fd + 1, &read_fds, NULL, NULL, &tv) == 0) {
		/* This can typically be because a "monitor" command took too long
		 * before printing any progress messages
		 */
		if (timeout_s > 0)
			return ERROR_GDB_TIMEOUT;
		else
			return ERROR_OK;
	}
	*got_data = FD_ISSET(connection->fd, &read_fds) != 0;
	return ERROR_OK;
}

static int gdb_get_char_inner(struct connection *connection, int *next_char)
{
	struct gdb_connection *gdb_con = connection->priv;
	int retval = ERROR_OK;

#ifdef _DEBUG_GDB_IO_
	char *debug_buffer;
#endif
	for (;; ) {
		if (connection->service->type != CONNECTION_TCP)
			gdb_con->buf_cnt = read(connection->fd, gdb_con->buffer, GDB_BUFFER_SIZE);
		else {
			retval = check_pending(connection, 1, NULL);
			if (retval != ERROR_OK)
				return retval;
			gdb_con->buf_cnt = read_socket(connection->fd,
					gdb_con->buffer,
					GDB_BUFFER_SIZE);
		}

		if (gdb_con->buf_cnt > 0)
			break;
		if (gdb_con->buf_cnt == 0) {
			LOG_DEBUG("GDB connection closed by the remote client");
			gdb_con->closed = true;
			return ERROR_SERVER_REMOTE_CLOSED;
		}

#ifdef _WIN32
		errno = WSAGetLastError();

		switch (errno) {
			case WSAEWOULDBLOCK:
				usleep(1000);
				break;
			case WSAECONNABORTED:
				gdb_con->closed = true;
				return ERROR_SERVER_REMOTE_CLOSED;
			case WSAECONNRESET:
				gdb_con->closed = true;
				return ERROR_SERVER_REMOTE_CLOSED;
			default:
				LOG_ERROR("read: %d", errno);
				exit(-1);
		}
#else
		switch (errno) {
			case EAGAIN:
				usleep(1000);
				break;
			case ECONNABORTED:
				gdb_con->closed = true;
				return ERROR_SERVER_REMOTE_CLOSED;
			case ECONNRESET:
				gdb_con->closed = true;
				return ERROR_SERVER_REMOTE_CLOSED;
			default:
				LOG_ERROR("read: %s", strerror(errno));
				gdb_con->closed = true;
				return ERROR_SERVER_REMOTE_CLOSED;
		}
#endif
	}

#ifdef _DEBUG_GDB_IO_
	debug_buffer = strndup(gdb_con->buffer, gdb_con->buf_cnt);
	LOG_DEBUG("received '%s'", debug_buffer);
	free(debug_buffer);
#endif

	gdb_con->buf_p = gdb_con->buffer;
	gdb_con->buf_cnt--;
	*next_char = *(gdb_con->buf_p++);
	if (gdb_con->buf_cnt > 0)
		connection->input_pending = true;
	else
		connection->input_pending = false;
#ifdef _DEBUG_GDB_IO_
	LOG_DEBUG("returned char '%c' (0x%2.2x)", *next_char, *next_char);
#endif

	return retval;
}

/**
 * The cool thing about this fn is that it allows buf_p and buf_cnt to be
 * held in registers in the inner loop.
 *
 * For small caches and embedded systems this is important!
 */
static inline int gdb_get_char_fast(struct connection *connection,
		int *next_char, char **buf_p, int *buf_cnt)
{
	int retval = ERROR_OK;

	if ((*buf_cnt)-- > 0) {
		*next_char = **buf_p;
		(*buf_p)++;
		if (*buf_cnt > 0)
			connection->input_pending = true;
		else
			connection->input_pending = false;

#ifdef _DEBUG_GDB_IO_
		LOG_DEBUG("returned char '%c' (0x%2.2x)", *next_char, *next_char);
#endif

		return ERROR_OK;
	}

	struct gdb_connection *gdb_con = connection->priv;
	gdb_con->buf_p = *buf_p;
	gdb_con->buf_cnt = *buf_cnt;
	retval = gdb_get_char_inner(connection, next_char);
	*buf_p = gdb_con->buf_p;
	*buf_cnt = gdb_con->buf_cnt;

	return retval;
}

static int gdb_get_char(struct connection *connection, int *next_char)
{
	struct gdb_connection *gdb_con = connection->priv;
	return gdb_get_char_fast(connection, next_char, &gdb_con->buf_p, &gdb_con->buf_cnt);
}

static int gdb_putback_char(struct connection *connection, int last_char)
{
	struct gdb_connection *gdb_con = connection->priv;

	if (gdb_con->buf_p > gdb_con->buffer) {
		*(--gdb_con->buf_p) = last_char;
		gdb_con->buf_cnt++;
	} else
		LOG_ERROR("BUG: couldn't put character back");

	return ERROR_OK;
}

/* The only way we can detect that the socket is closed is the first time
 * we write to it, we will fail. Subsequent write operations will
 * succeed. Shudder! */
static int gdb_write(struct connection *connection, void *data, int len)
{
	struct gdb_connection *gdb_con = connection->priv;
	if (gdb_con->closed) {
		LOG_DEBUG("GDB socket marked as closed, cannot write to it.");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	if (connection_write(connection, data, len) == len)
		return ERROR_OK;

	LOG_WARNING("Error writing to GDB socket. Dropping the connection.");
	gdb_con->closed = true;
	return ERROR_SERVER_REMOTE_CLOSED;
}

/* 用于记录来自GDB的传入数据包。它根据日志级别和数据包的内容决定如何记录这些信息
 *     日志级别检查:用来检查当前的日志级别是否为调试级别,如果不是直接返回，不执行后续的日志记录操作
 *	   获取目标对象：调用get_target_from_connection函数，从连接中获取对应的目标对象target，通常是指正在调试的目标设备或进程
 *	   避免打印非可打印字符：
 *	       计算数据包的长度 packet_len
 *		   调用 find_nonprint_char 函数查找数据包中的第一个非可打印字符的位置，并将其存储在 nonprint 中
 *     处理包含非可打印字符的数据包：
 *			查找冒号 (:)：使用 memchr 在数据包的前50个字符中查找冒号。某些GDB协议的数据包可能以冒号分隔命令和数据
 *			判断是否有可打印的前缀：如果找到了冒号且非可打印字符出现在冒号之后，则认为数据包有可打印的前缀
 *			记录日志：如果数据包有可打印的前缀，则记录前缀部分并指出剩余部分为二进制数据，同时给出二进制数据的长度。
 *				如果整个数据包都是不可打印字符，则简单地记录数据包的总长度，并说明它是二进制数据
 *		处理全为可打印字符的数据包：如果数据包中所有字符都是可打印的，则直接记录整个数据包的内容
 */
static void gdb_log_incoming_packet(struct connection *connection, char *packet)
{
	if (!LOG_LEVEL_IS(LOG_LVL_DEBUG))
		return;

	struct target *target = get_target_from_connection(connection);

	/* Avoid dumping non-printable characters to the terminal */
	const unsigned packet_len = strlen(packet);
	const char *nonprint = find_nonprint_char(packet, packet_len);
	if (nonprint) {
		/* Does packet at least have a prefix that is printable?
		 * Look within the first 50 chars of the packet. */
		const char *colon = memchr(packet, ':', MIN(50, packet_len));
		const bool packet_has_prefix = (colon);
		const bool packet_prefix_printable = (packet_has_prefix && nonprint > colon);

		if (packet_prefix_printable) {
			const unsigned int prefix_len = colon - packet + 1;  /* + 1 to include the ':' */
			const unsigned int payload_len = packet_len - prefix_len;
			LOG_TARGET_DEBUG(target, "received packet: %.*s<binary-data-%u-bytes>", prefix_len,
				packet, payload_len);
		} else {
			LOG_TARGET_DEBUG(target, "received packet: <binary-data-%u-bytes>", packet_len);
		}
	} else {
		/* All chars printable, dump the packet as is */
		LOG_TARGET_DEBUG(target, "received packet: %s", packet);
	}
}

static void gdb_log_outgoing_packet(struct connection *connection, char *packet_buf,
	unsigned int packet_len, unsigned char checksum)
{
	if (!LOG_LEVEL_IS(LOG_LVL_DEBUG))
		return;

	struct target *target = get_target_from_connection(connection);

	if (find_nonprint_char(packet_buf, packet_len))
		LOG_TARGET_DEBUG(target, "sending packet: $<binary-data-%u-bytes>#%2.2x",
			packet_len, checksum);
	else
		LOG_TARGET_DEBUG(target, "sending packet: $%.*s#%2.2x", packet_len, packet_buf,
			checksum);
}
/* gdb_put_packet_inner:用于向GDB发送数据包，并处理与GDB之间的通信细节
 *		char *buffer：指向要发送的数据包内容的字符串
 * 流程：
 *		校验和计算：初始化my_checksum为0，遍历 buffer 中的每个字符，并将其累加到 my_checksum 中
 *  	检查未处理的输入数据:
 *			无限循环：通过 for (;;) 创建一个无限循环，直到满足特定条件才退出
 *			检查待处理数据：调用 check_pending 函数检查连接中是否有未处理的数据。check_pending 返回 ERROR_OK 表示没有错误，gotdata 为真表示有未处理的数据
 *			读取字符：如果有未处理的数据，则调用 gdb_get_char 读取一个字符并存储在 reply 中
 *			处理 $ 字符：如果读取到的字符是 $，这可能是一个新数据包的开始
 *			丢弃其他字符：如果读取到的字符不是 $，则记录一条警告信息，说明丢弃了一个意外的字符，并继续循环
 *		处理要发送的数据包：
 *			无限循环：创建一个无限循环，直到满足特定条件才退出	
 *			记录传出的数据包：gdb_log_outgoing_packet
 *			数据包格式化与发送：
 *				初始化 local_buffer：创建一个大小为1024字节的缓冲区 local_buffer，并将第一个字符设置为 $，这是GDB协议中数据包的开始标志。
 *				小数据包优化：如果数据包长度加上校验和部分（# 和两位十六进制校验和）不超过 local_buffer 的大小，则将整个数据包（包括校验和）复制到 local_buffer 中，并通过一次 gdb_write 调用发送出去。
 *				大数据包处理：如果数据包过大，不能一次性放入 local_buffer，则分三次调用 gdb_write：
 *					发送 $ 符号、发送数据包内容、发送校验和部分（# 和两位十六进制校验和）
 *			处理ACK/NACK和其他响应：
 *				检查 noack_mode：如果GDB连接处于无ACK模式（noack_mode），则直接退出循环，因为不需要等待确认
 *				读取响应字符：调用 gdb_get_char 从连接中读取一个字符并存储在 reply 中	
 *			处理不同的响应字符
 *				正确认可 (+)：如果收到 +，表示GDB正确接收了数据包，记录日志并退出循环
 *				负确认 (-)：如果收到 -，表示GDB没有正确接收数据包，设置 output_flag 为 GDB_OUTPUT_NO 以停止发送输出包，并记录日志，发出警告信息
 *				中断请求 (0x3)：
 *					如果收到 Ctrl-C（ASCII码0x3），设置 ctrl_c 标志为 true，记录日志，并继续读取下一个字符。
 *					根据后续字符的不同，分别处理：
 *						+：记录日志并退出循环。
 *						-：设置 output_flag 为 GDB_OUTPUT_NO，记录日志，发出警告信息。
 *						$：记录错误日志，假设数据包已正确接收，并将 $ 字符重新放回输入队列。
 *						其他字符：记录错误日志，关闭连接并返回错误码 ERROR_SERVER_REMOTE_CLOSED
 *				未接收到ACK ($)：如果收到 $，记录错误日志，假设数据包已正确接收，并将 $ 字符重新放回输入队列
 *				未知字符：如果收到其他未知字符，记录错误日志，关闭连接并返回错误码 ERROR_SERVER_REMOTE_CLOSED
 *		检查连接状态:检查 gdb_con->closed 标志，如果连接已关闭，则返回错误码 ERROR_SERVER_REMOTE_CLOSED
 *
 */
static int gdb_put_packet_inner(struct connection *connection,
		char *buffer, int len)
{
	int i;
	unsigned char my_checksum = 0;
	int reply;
	int retval;
	struct gdb_connection *gdb_con = connection->priv;

	for (i = 0; i < len; i++)
		my_checksum += buffer[i];

//这个预处理器指令使得下面的代码块只在定义了 _DEBUG_GDB_IO_ 宏时编译。这意味着这些调试代码不会出现在非调试版本的程序中
#ifdef _DEBUG_GDB_IO_
	/*
	 * At this point we should have nothing in the input queue from GDB,
	 * however sometimes '-' is sent even though we've already received
	 * an ACK (+) for everything we've sent off.
	 */
	int gotdata;
	for (;; ) {
		retval = check_pending(connection, 0, &gotdata);
		if (retval != ERROR_OK)
			return retval;
		if (!gotdata)
			break;
		retval = gdb_get_char(connection, &reply);
		if (retval != ERROR_OK)
			return retval;
		if (reply == '$') {
			/* fix a problem with some IAR tools */
			gdb_putback_char(connection, reply);
			LOG_DEBUG("Unexpected start of new packet");
			break;
		}

		LOG_WARNING("Discard unexpected char %c", reply);
	}
#endif

	while (1) {
		gdb_log_outgoing_packet(connection, buffer, len, my_checksum);

		char local_buffer[1024];
		local_buffer[0] = '$';
		if ((size_t)len + 4 <= sizeof(local_buffer)) {
			/* performance gain on smaller packets by only a single call to gdb_write() */
			memcpy(local_buffer + 1, buffer, len++);
			len += snprintf(local_buffer + len, sizeof(local_buffer) - len, "#%02x", my_checksum);
			retval = gdb_write(connection, local_buffer, len);
			if (retval != ERROR_OK)
				return retval;
		} else {
			/* larger packets are transmitted directly from caller supplied buffer
			 * by several calls to gdb_write() to avoid dynamic allocation */
			snprintf(local_buffer + 1, sizeof(local_buffer) - 1, "#%02x", my_checksum);
			retval = gdb_write(connection, local_buffer, 1);
			if (retval != ERROR_OK)
				return retval;
			retval = gdb_write(connection, buffer, len);
			if (retval != ERROR_OK)
				return retval;
			retval = gdb_write(connection, local_buffer + 1, 3);
			if (retval != ERROR_OK)
				return retval;
		}

		if (gdb_con->noack_mode)
			break;

		retval = gdb_get_char(connection, &reply);
		if (retval != ERROR_OK)
			return retval;

		if (reply == '+') {
			gdb_log_incoming_packet(connection, "+");
			break;
		} else if (reply == '-') {
			/* Stop sending output packets for now */
			gdb_con->output_flag = GDB_OUTPUT_NO;
			gdb_log_incoming_packet(connection, "-");
			LOG_WARNING("negative reply, retrying");
		} else if (reply == 0x3) {
			gdb_con->ctrl_c = true;
			gdb_log_incoming_packet(connection, "<Ctrl-C>");
			retval = gdb_get_char(connection, &reply);
			if (retval != ERROR_OK)
				return retval;
			if (reply == '+') {
				gdb_log_incoming_packet(connection, "+");
				break;
			} else if (reply == '-') {
				/* Stop sending output packets for now */
				gdb_con->output_flag = GDB_OUTPUT_NO;
				gdb_log_incoming_packet(connection, "-");
				LOG_WARNING("negative reply, retrying");
			} else if (reply == '$') {
				LOG_ERROR("GDB missing ack(1) - assumed good");
				gdb_putback_char(connection, reply);
				return ERROR_OK;
			} else {
				LOG_ERROR("unknown character(1) 0x%2.2x in reply, dropping connection", reply);
				gdb_con->closed = true;
				return ERROR_SERVER_REMOTE_CLOSED;
			}
		} else if (reply == '$') {
			LOG_ERROR("GDB missing ack(2) - assumed good");
			gdb_putback_char(connection, reply);
			return ERROR_OK;
		} else {
			LOG_ERROR("unknown character(2) 0x%2.2x in reply, dropping connection",
				reply);
			gdb_con->closed = true;
			return ERROR_SERVER_REMOTE_CLOSED;
		}
	}
	if (gdb_con->closed)
		return ERROR_SERVER_REMOTE_CLOSED;

	return ERROR_OK;
}

int gdb_put_packet(struct connection *connection, char *buffer, int len)
{
	struct gdb_connection *gdb_con = connection->priv;
	gdb_con->busy = true;
	int retval = gdb_put_packet_inner(connection, buffer, len);
	gdb_con->busy = false;

	/* we sent some data, reset timer for keep alive messages */
	kept_alive();

	return retval;
}

static inline int fetch_packet(struct connection *connection,
		int *checksum_ok, int noack, int *len, char *buffer)
{
	unsigned char my_checksum = 0;
	char checksum[3];
	int character;
	int retval = ERROR_OK;

	struct gdb_connection *gdb_con = connection->priv;
	my_checksum = 0;
	int count = 0;
	count = 0;

	/* move this over into local variables to use registers and give the
	 * more freedom to optimize */
	char *buf_p = gdb_con->buf_p;
	int buf_cnt = gdb_con->buf_cnt;

	for (;; ) {
		/* The common case is that we have an entire packet with no escape chars.
		 * We need to leave at least 2 bytes in the buffer to have
		 * gdb_get_char() update various bits and bobs correctly.
		 */
		if ((buf_cnt > 2) && ((buf_cnt + count) < *len)) {
			/* The compiler will struggle a bit with constant propagation and
			 * aliasing, so we help it by showing that these values do not
			 * change inside the loop
			 */
			int i;
			char *buf = buf_p;
			int run = buf_cnt - 2;
			i = 0;
			int done = 0;
			while (i < run) {
				character = *buf++;
				i++;
				if (character == '#') {
					/* Danger! character can be '#' when esc is
					 * used so we need an explicit boolean for done here. */
					done = 1;
					break;
				}

				if (character == '}') {
					/* data transmitted in binary mode (X packet)
					 * uses 0x7d as escape character */
					my_checksum += character & 0xff;
					character = *buf++;
					i++;
					my_checksum += character & 0xff;
					buffer[count++] = (character ^ 0x20) & 0xff;
				} else {
					my_checksum += character & 0xff;
					buffer[count++] = character & 0xff;
				}
			}
			buf_p += i;
			buf_cnt -= i;
			if (done)
				break;
		}
		if (count > *len) {
			LOG_ERROR("packet buffer too small");
			retval = ERROR_GDB_BUFFER_TOO_SMALL;
			break;
		}

		retval = gdb_get_char_fast(connection, &character, &buf_p, &buf_cnt);
		if (retval != ERROR_OK)
			break;

		if (character == '#')
			break;

		if (character == '}') {
			/* data transmitted in binary mode (X packet)
			 * uses 0x7d as escape character */
			my_checksum += character & 0xff;

			retval = gdb_get_char_fast(connection, &character, &buf_p, &buf_cnt);
			if (retval != ERROR_OK)
				break;

			my_checksum += character & 0xff;
			buffer[count++] = (character ^ 0x20) & 0xff;
		} else {
			my_checksum += character & 0xff;
			buffer[count++] = character & 0xff;
		}
	}

	gdb_con->buf_p = buf_p;
	gdb_con->buf_cnt = buf_cnt;

	if (retval != ERROR_OK)
		return retval;

	*len = count;

	retval = gdb_get_char(connection, &character);
	if (retval != ERROR_OK)
		return retval;
	checksum[0] = character;
	retval = gdb_get_char(connection, &character);
	if (retval != ERROR_OK)
		return retval;
	checksum[1] = character;
	checksum[2] = 0;

	if (!noack)
		*checksum_ok = (my_checksum == strtoul(checksum, NULL, 16));

	return ERROR_OK;
}

static int gdb_get_packet_inner(struct connection *connection,
		char *buffer, int *len)
{
	int character;
	int retval;
	struct gdb_connection *gdb_con = connection->priv;

	while (1) {
		do {
			retval = gdb_get_char(connection, &character);
			if (retval != ERROR_OK)
				return retval;

#ifdef _DEBUG_GDB_IO_
			LOG_DEBUG("character: '%c'", character);
#endif

			switch (character) {
				case '$':
					break;
				case '+':
					gdb_log_incoming_packet(connection, "+");
					/* According to the GDB documentation
					 * (https://sourceware.org/gdb/onlinedocs/gdb/Packet-Acknowledgment.html):
					 * "gdb sends a final `+` acknowledgment of the stub's `OK`
					 * response, which can be safely ignored by the stub."
					 * However OpenOCD server already is in noack mode at this
					 * point and instead of ignoring this it was emitting a
					 * warning. This code makes server ignore the first ACK
					 * that will be received after going into noack mode,
					 * warning only about subsequent ACK's. */
					if (gdb_con->noack_mode > 1) {
						LOG_WARNING("acknowledgment received, but no packet pending");
					} else if (gdb_con->noack_mode) {
						LOG_DEBUG("Received first acknowledgment after entering noack mode. Ignoring it.");
						gdb_con->noack_mode = 2;
					}
					break;
				case '-':
					gdb_log_incoming_packet(connection, "-");
					LOG_WARNING("negative acknowledgment, but no packet pending");
					break;
				case 0x3:
					gdb_log_incoming_packet(connection, "<Ctrl-C>");
					gdb_con->ctrl_c = true;
					*len = 0;
					return ERROR_OK;
				default:
					LOG_WARNING("ignoring character 0x%x", character);
					break;
			}
		} while (character != '$');

		int checksum_ok = 0;
		/* explicit code expansion here to get faster inlined code in -O3 by not
		 * calculating checksum */
		if (gdb_con->noack_mode) {
			retval = fetch_packet(connection, &checksum_ok, 1, len, buffer);
			if (retval != ERROR_OK)
				return retval;
		} else {
			retval = fetch_packet(connection, &checksum_ok, 0, len, buffer);
			if (retval != ERROR_OK)
				return retval;
		}

		if (gdb_con->noack_mode) {
			/* checksum is not checked in noack mode */
			break;
		}
		if (checksum_ok) {
			retval = gdb_write(connection, "+", 1);
			if (retval != ERROR_OK)
				return retval;
			break;
		}
	}
	if (gdb_con->closed)
		return ERROR_SERVER_REMOTE_CLOSED;

	return ERROR_OK;
}

static int gdb_get_packet(struct connection *connection, char *buffer, int *len)
{
	struct gdb_connection *gdb_con = connection->priv;
	gdb_con->busy = true;
	int retval = gdb_get_packet_inner(connection, buffer, len);
	gdb_con->busy = false;
	return retval;
}

static int gdb_output_con(struct connection *connection, const char *line)
{
	char *hex_buffer;
	int bin_size;

	bin_size = strlen(line);

	hex_buffer = malloc(bin_size * 2 + 2);
	if (!hex_buffer)
		return ERROR_GDB_BUFFER_TOO_SMALL;

	hex_buffer[0] = 'O';
	size_t pkt_len = hexify(hex_buffer + 1, (const uint8_t *)line, bin_size,
		bin_size * 2 + 1);
	int retval = gdb_put_packet(connection, hex_buffer, pkt_len + 1);

	free(hex_buffer);
	return retval;
}

static int gdb_output(struct command_context *context, const char *line)
{
	/* this will be dumped to the log and also sent as an O packet if possible */
	LOG_USER_N("%s", line);
	return ERROR_OK;
}

static void gdb_signal_reply(struct target *target, struct connection *connection)
{
	struct gdb_connection *gdb_connection = connection->priv;
	char sig_reply[65];
	char stop_reason[32];
	char current_thread[25];
	int sig_reply_len;
	int signal_var;

	rtos_update_threads(target);

	if (target->debug_reason == DBG_REASON_EXIT) {
		sig_reply_len = snprintf(sig_reply, sizeof(sig_reply), "W00");
	} else {
		struct target *ct;
		if (target->rtos) {
			target->rtos->current_threadid = target->rtos->current_thread;
			target->rtos->gdb_target_for_threadid(connection, target->rtos->current_threadid, &ct);
		} else {
			ct = target;
		}

		if (gdb_connection->ctrl_c) {
			signal_var = 0x2;
		} else
			signal_var = gdb_last_signal(ct);

		stop_reason[0] = '\0';
		if (ct->debug_reason == DBG_REASON_WATCHPOINT) {
			enum watchpoint_rw hit_wp_type;
			target_addr_t hit_wp_address;

			if (watchpoint_hit(ct, &hit_wp_type, &hit_wp_address) == ERROR_OK) {

				switch (hit_wp_type) {
					case WPT_WRITE:
						snprintf(stop_reason, sizeof(stop_reason),
								"watch:%08" TARGET_PRIxADDR ";", hit_wp_address);
						break;
					case WPT_READ:
						snprintf(stop_reason, sizeof(stop_reason),
								"rwatch:%08" TARGET_PRIxADDR ";", hit_wp_address);
						break;
					case WPT_ACCESS:
						snprintf(stop_reason, sizeof(stop_reason),
								"awatch:%08" TARGET_PRIxADDR ";", hit_wp_address);
						break;
					default:
						break;
				}
			}
		}

		current_thread[0] = '\0';
		if (target->rtos)
			snprintf(current_thread, sizeof(current_thread), "thread:%" PRIx64 ";",
					target->rtos->current_thread);

		sig_reply_len = snprintf(sig_reply, sizeof(sig_reply), "T%2.2x%s%s",
				signal_var, stop_reason, current_thread);

		gdb_connection->ctrl_c = false;
	}

	gdb_put_packet(connection, sig_reply, sig_reply_len);
	gdb_connection->frontend_state = TARGET_HALTED;
}

static void gdb_fileio_reply(struct target *target, struct connection *connection)
{
	struct gdb_connection *gdb_connection = connection->priv;
	char fileio_command[256];
	int command_len;
	bool program_exited = false;

	if (strcmp(target->fileio_info->identifier, "open") == 0)
		sprintf(fileio_command, "F%s,%" PRIx64 "/%" PRIx64 ",%" PRIx64 ",%" PRIx64, target->fileio_info->identifier,
				target->fileio_info->param_1,
				target->fileio_info->param_2 + 1,	/* len + trailing zero */
				target->fileio_info->param_3,
				target->fileio_info->param_4);
	else if (strcmp(target->fileio_info->identifier, "close") == 0)
		sprintf(fileio_command, "F%s,%" PRIx64, target->fileio_info->identifier,
				target->fileio_info->param_1);
	else if (strcmp(target->fileio_info->identifier, "read") == 0)
		sprintf(fileio_command, "F%s,%" PRIx64 ",%" PRIx64 ",%" PRIx64, target->fileio_info->identifier,
				target->fileio_info->param_1,
				target->fileio_info->param_2,
				target->fileio_info->param_3);
	else if (strcmp(target->fileio_info->identifier, "write") == 0)
		sprintf(fileio_command, "F%s,%" PRIx64 ",%" PRIx64 ",%" PRIx64, target->fileio_info->identifier,
				target->fileio_info->param_1,
				target->fileio_info->param_2,
				target->fileio_info->param_3);
	else if (strcmp(target->fileio_info->identifier, "lseek") == 0)
		sprintf(fileio_command, "F%s,%" PRIx64 ",%" PRIx64 ",%" PRIx64, target->fileio_info->identifier,
				target->fileio_info->param_1,
				target->fileio_info->param_2,
				target->fileio_info->param_3);
	else if (strcmp(target->fileio_info->identifier, "rename") == 0)
		sprintf(fileio_command, "F%s,%" PRIx64 "/%" PRIx64 ",%" PRIx64 "/%" PRIx64, target->fileio_info->identifier,
				target->fileio_info->param_1,
				target->fileio_info->param_2 + 1,	/* len + trailing zero */
				target->fileio_info->param_3,
				target->fileio_info->param_4 + 1);	/* len + trailing zero */
	else if (strcmp(target->fileio_info->identifier, "unlink") == 0)
		sprintf(fileio_command, "F%s,%" PRIx64 "/%" PRIx64, target->fileio_info->identifier,
				target->fileio_info->param_1,
				target->fileio_info->param_2 + 1);	/* len + trailing zero */
	else if (strcmp(target->fileio_info->identifier, "stat") == 0)
		sprintf(fileio_command, "F%s,%" PRIx64 "/%" PRIx64 ",%" PRIx64, target->fileio_info->identifier,
				target->fileio_info->param_1,
				target->fileio_info->param_2,
				target->fileio_info->param_3);
	else if (strcmp(target->fileio_info->identifier, "fstat") == 0)
		sprintf(fileio_command, "F%s,%" PRIx64 ",%" PRIx64, target->fileio_info->identifier,
				target->fileio_info->param_1,
				target->fileio_info->param_2);
	else if (strcmp(target->fileio_info->identifier, "gettimeofday") == 0)
		sprintf(fileio_command, "F%s,%" PRIx64 ",%" PRIx64, target->fileio_info->identifier,
				target->fileio_info->param_1,
				target->fileio_info->param_2);
	else if (strcmp(target->fileio_info->identifier, "isatty") == 0)
		sprintf(fileio_command, "F%s,%" PRIx64, target->fileio_info->identifier,
				target->fileio_info->param_1);
	else if (strcmp(target->fileio_info->identifier, "system") == 0)
		sprintf(fileio_command, "F%s,%" PRIx64 "/%" PRIx64, target->fileio_info->identifier,
				target->fileio_info->param_1,
				target->fileio_info->param_2 + 1);	/* len + trailing zero */
	else if (strcmp(target->fileio_info->identifier, "exit") == 0) {
		/* If target hits exit syscall, report to GDB the program is terminated.
		 * In addition, let target run its own exit syscall handler. */
		program_exited = true;
		sprintf(fileio_command, "W%02" PRIx64, target->fileio_info->param_1);
	} else {
		LOG_DEBUG("Unknown syscall: %s", target->fileio_info->identifier);

		/* encounter unknown syscall, continue */
		gdb_connection->frontend_state = TARGET_RUNNING;
		target_resume(target, 1, 0x0, 0, 0);
		return;
	}

	command_len = strlen(fileio_command);
	gdb_put_packet(connection, fileio_command, command_len);

	if (program_exited) {
		/* Use target_resume() to let target run its own exit syscall handler. */
		gdb_connection->frontend_state = TARGET_RUNNING;
		target_resume(target, 1, 0x0, 0, 0);
	} else {
		gdb_connection->frontend_state = TARGET_HALTED;
		rtos_update_threads(target);
	}
}

static void gdb_frontend_halted(struct target *target, struct connection *connection)
{
	struct gdb_connection *gdb_connection = connection->priv;

	/* In the GDB protocol when we are stepping or continuing execution,
	 * we have a lingering reply. Upon receiving a halted event
	 * when we have that lingering packet, we reply to the original
	 * step or continue packet.
	 *
	 * Executing monitor commands can bring the target in and
	 * out of the running state so we'll see lots of TARGET_EVENT_XXX
	 * that are to be ignored.
	 */
	if (gdb_connection->frontend_state == TARGET_RUNNING) {
		/* stop forwarding log packets! */
		gdb_connection->output_flag = GDB_OUTPUT_NO;

		/* check fileio first */
		if (target_get_gdb_fileio_info(target, target->fileio_info) == ERROR_OK)
			gdb_fileio_reply(target, connection);
		else
			gdb_signal_reply(target, connection);
	}
}

static int gdb_target_callback_event_handler(struct target *target,
		enum target_event event, void *priv)
{
	struct connection *connection = priv;
	struct gdb_service *gdb_service = connection->service->priv;

	if (gdb_service->target != target)
		return ERROR_OK;

	switch (event) {
		case TARGET_EVENT_GDB_HALT:
			gdb_frontend_halted(target, connection);
			break;
		case TARGET_EVENT_HALTED:
			target_call_event_callbacks(target, TARGET_EVENT_GDB_END);
			break;
		default:
			break;
	}

	return ERROR_OK;
}

static int gdb_new_connection(struct connection *connection)
{
	struct gdb_connection *gdb_connection = malloc(sizeof(struct gdb_connection));
	struct target *target;
	int retval;
	int initial_ack;

	target = get_target_from_connection(connection);
	connection->priv = gdb_connection;
	connection->cmd_ctx->current_target = target;

	/* initialize gdb connection information */
	gdb_connection->buf_p = gdb_connection->buffer;
	gdb_connection->buf_cnt = 0;
	gdb_connection->ctrl_c = false;
	gdb_connection->frontend_state = TARGET_HALTED;
	gdb_connection->vflash_image = NULL;
	gdb_connection->closed = false;
	gdb_connection->busy = false;
	gdb_connection->noack_mode = 0;
	gdb_connection->sync = false;
	gdb_connection->mem_write_error = false;
	gdb_connection->attached = true;
	gdb_connection->extended_protocol = false;
	gdb_connection->target_desc.tdesc = NULL;
	gdb_connection->target_desc.tdesc_length = 0;
	gdb_connection->thread_list = NULL;
	gdb_connection->output_flag = GDB_OUTPUT_NO;

	/* send ACK to GDB for debug request */
	gdb_write(connection, "+", 1);

	/* output goes through gdb connection */
	command_set_output_handler(connection->cmd_ctx, gdb_output, connection);

	/* we must remove all breakpoints registered to the target as a previous
	 * GDB session could leave dangling breakpoints if e.g. communication
	 * timed out.
	 */
	breakpoint_clear_target(target);
	watchpoint_clear_target(target);

	/* Since version 3.95 (gdb-19990504), with the exclusion of 6.5~6.8, GDB
	 * sends an ACK at connection with the following comment in its source code:
	 * "Ack any packet which the remote side has already sent."
	 * LLDB does the same since the first gdb-remote implementation.
	 * Remove the initial ACK from the incoming buffer.
	 */
	retval = gdb_get_char(connection, &initial_ack);
	if (retval != ERROR_OK)
		return retval;

	if (initial_ack != '+')
		gdb_putback_char(connection, initial_ack);

	target_call_event_callbacks(target, TARGET_EVENT_GDB_ATTACH);

	if (target->rtos) {
		/* clean previous rtos session if supported*/
		if (target->rtos->type->clean)
			target->rtos->type->clean(target);

		/* update threads */
		rtos_update_threads(target);
	}

	if (gdb_use_memory_map) {
		/* Connect must fail if the memory map can't be set up correctly.
		 *
		 * This will cause an auto_probe to be invoked, which is either
		 * a no-op or it will fail when the target isn't ready(e.g. not halted).
		 */
		for (unsigned int i = 0; i < flash_get_bank_count(); i++) {
			struct flash_bank *p;
			p = get_flash_bank_by_num_noprobe(i);
			if (p->target != target)
				continue;
			retval = get_flash_bank_by_num(i, &p);
			if (retval != ERROR_OK) {
				LOG_ERROR("Connect failed. Consider setting up a gdb-attach event for the target "
						"to prepare target for GDB connect, or use 'gdb_memory_map disable'.");
				return retval;
			}
		}
	}

	gdb_actual_connections++;
	log_printf_lf(all_targets->next ? LOG_LVL_INFO : LOG_LVL_DEBUG,
			__FILE__, __LINE__, __func__,
			"New GDB Connection: %d, Target %s, state: %s",
			gdb_actual_connections,
			target_name(target),
			target_state_name(target));

	if (!target_was_examined(target)) {
		LOG_ERROR("Target %s not examined yet, refuse gdb connection %d!",
				  target_name(target), gdb_actual_connections);
		gdb_actual_connections--;
		return ERROR_TARGET_NOT_EXAMINED;
	}

	if (target->state != TARGET_HALTED)
		LOG_WARNING("GDB connection %d on target %s not halted",
					gdb_actual_connections, target_name(target));

	/* DANGER! If we fail subsequently, we must remove this handler,
	 * otherwise we occasionally see crashes as the timer can invoke the
	 * callback fn.
	 *
	 * register callback to be informed about target events */
	target_register_event_callback(gdb_target_callback_event_handler, connection);

	log_add_callback(gdb_log_callback, connection);

	return ERROR_OK;
}

static int gdb_connection_closed(struct connection *connection)
{
	struct target *target;
	struct gdb_connection *gdb_connection = connection->priv;

	target = get_target_from_connection(connection);

	/* we're done forwarding messages. Tear down callback before
	 * cleaning up connection.
	 */
	log_remove_callback(gdb_log_callback, connection);

	gdb_actual_connections--;
	LOG_DEBUG("GDB Close, Target: %s, state: %s, gdb_actual_connections=%d",
		target_name(target),
		target_state_name(target),
		gdb_actual_connections);

	/* see if an image built with vFlash commands is left */
	if (gdb_connection->vflash_image) {
		image_close(gdb_connection->vflash_image);
		free(gdb_connection->vflash_image);
		gdb_connection->vflash_image = NULL;
	}

	/* if this connection registered a debug-message receiver delete it */
	delete_debug_msg_receiver(connection->cmd_ctx, target);

	free(connection->priv);
	connection->priv = NULL;

	target_unregister_event_callback(gdb_target_callback_event_handler, connection);

	target_call_event_callbacks(target, TARGET_EVENT_GDB_END);

	target_call_event_callbacks(target, TARGET_EVENT_GDB_DETACH);

	return ERROR_OK;
}

static void gdb_send_error(struct connection *connection, uint8_t the_error)
{
	char err[4];
	snprintf(err, 4, "E%2.2X", the_error);
	gdb_put_packet(connection, err, 3);
}

static int gdb_last_signal_packet(struct connection *connection,
		char const *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	struct gdb_connection *gdb_con = connection->priv;
	char sig_reply[4];
	int signal_var;

	if (!gdb_con->attached) {
		/* if we are here we have received a kill packet
		 * reply W stop reply otherwise gdb gets very unhappy */
		gdb_put_packet(connection, "W00", 3);
		return ERROR_OK;
	}

	signal_var = gdb_last_signal(target);

	snprintf(sig_reply, 4, "S%2.2x", signal_var);
	gdb_put_packet(connection, sig_reply, 3);

	return ERROR_OK;
}

static inline int gdb_reg_pos(struct target *target, int pos, int len)
{
	if (target->endianness == TARGET_LITTLE_ENDIAN)
		return pos;
	else
		return len - 1 - pos;
}

/* Convert register to string of bytes. NB! The # of bits in the
 * register might be non-divisible by 8(a byte), in which
 * case an entire byte is shown.
 *
 * NB! the format on the wire is the target endianness
 *
 * The format of reg->value is little endian
 *
 */
static void gdb_str_to_target(struct target *target,
		char *tstr, struct reg *reg)
{
	int i;

	uint8_t *buf;
	int buf_len;
	buf = reg->value;
	buf_len = DIV_ROUND_UP(reg->size, 8);

	for (i = 0; i < buf_len; i++) {
		int j = gdb_reg_pos(target, i, buf_len);
		tstr += sprintf(tstr, "%02x", buf[j]);
	}
}

/* copy over in register buffer */
static void gdb_target_to_reg(struct target *target,
		char const *tstr, int str_len, uint8_t *bin)
{
	if (str_len % 2) {
		LOG_ERROR("BUG: gdb value with uneven number of characters encountered");
		exit(-1);
	}

	int i;
	for (i = 0; i < str_len; i += 2) {
		unsigned t;
		if (sscanf(tstr + i, "%02x", &t) != 1) {
			LOG_ERROR("BUG: unable to convert register value");
			exit(-1);
		}

		int j = gdb_reg_pos(target, i/2, str_len/2);
		bin[j] = t;
	}
}

static int gdb_get_registers_packet(struct connection *connection,
		char const *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	struct reg **reg_list;
	int reg_list_size;
	int retval;
	int reg_packet_size = 0;
	char *reg_packet;
	char *reg_packet_p;
	int i;

#ifdef _DEBUG_GDB_IO_
	LOG_DEBUG("-");
#endif

	if ((target->rtos) && (rtos_get_gdb_reg_list(connection) == ERROR_OK))
		return ERROR_OK;

	retval = target_get_gdb_reg_list(target, &reg_list, &reg_list_size,
			REG_CLASS_GENERAL);
	if (retval != ERROR_OK)
		return gdb_error(connection, retval);

	for (i = 0; i < reg_list_size; i++) {
		if (!reg_list[i] || reg_list[i]->exist == false || reg_list[i]->hidden)
			continue;
		reg_packet_size += DIV_ROUND_UP(reg_list[i]->size, 8) * 2;
	}

	assert(reg_packet_size > 0);

	reg_packet = malloc(reg_packet_size + 1); /* plus one for string termination null */
	if (!reg_packet)
		return ERROR_FAIL;

	reg_packet_p = reg_packet;

	for (i = 0; i < reg_list_size; i++) {
		if (!reg_list[i] || reg_list[i]->exist == false || reg_list[i]->hidden)
			continue;
		if (!reg_list[i]->valid) {
			retval = reg_list[i]->type->get(reg_list[i]);
			if (retval != ERROR_OK && gdb_report_register_access_error) {
				LOG_DEBUG("Couldn't get register %s.", reg_list[i]->name);
				free(reg_packet);
				free(reg_list);
				return gdb_error(connection, retval);
			}
		}
		gdb_str_to_target(target, reg_packet_p, reg_list[i]);
		reg_packet_p += DIV_ROUND_UP(reg_list[i]->size, 8) * 2;
	}

#ifdef _DEBUG_GDB_IO_
	{
		char *reg_packet_p_debug;
		reg_packet_p_debug = strndup(reg_packet, reg_packet_size);
		LOG_DEBUG("reg_packet: %s", reg_packet_p_debug);
		free(reg_packet_p_debug);
	}
#endif

	gdb_put_packet(connection, reg_packet, reg_packet_size);
	free(reg_packet);

	free(reg_list);

	return ERROR_OK;
}

static int gdb_set_registers_packet(struct connection *connection,
		char const *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	int i;
	struct reg **reg_list;
	int reg_list_size;
	int retval;
	char const *packet_p;

#ifdef _DEBUG_GDB_IO_
	LOG_DEBUG("-");
#endif

	/* skip command character */
	packet++;
	packet_size--;

	if (packet_size % 2) {
		LOG_WARNING("GDB set_registers packet with uneven characters received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	retval = target_get_gdb_reg_list(target, &reg_list, &reg_list_size,
			REG_CLASS_GENERAL);
	if (retval != ERROR_OK)
		return gdb_error(connection, retval);

	packet_p = packet;
	for (i = 0; i < reg_list_size; i++) {
		uint8_t *bin_buf;
		if (!reg_list[i] || !reg_list[i]->exist || reg_list[i]->hidden)
			continue;
		int chars = (DIV_ROUND_UP(reg_list[i]->size, 8) * 2);

		if (packet_p + chars > packet + packet_size)
			LOG_ERROR("BUG: register packet is too small for registers");

		bin_buf = malloc(DIV_ROUND_UP(reg_list[i]->size, 8));
		gdb_target_to_reg(target, packet_p, chars, bin_buf);

		retval = reg_list[i]->type->set(reg_list[i], bin_buf);
		if (retval != ERROR_OK && gdb_report_register_access_error) {
			LOG_DEBUG("Couldn't set register %s.", reg_list[i]->name);
			free(reg_list);
			free(bin_buf);
			return gdb_error(connection, retval);
		}

		/* advance packet pointer */
		packet_p += chars;

		free(bin_buf);
	}

	/* free struct reg *reg_list[] array allocated by get_gdb_reg_list */
	free(reg_list);

	gdb_put_packet(connection, "OK", 2);

	return ERROR_OK;
}

static int gdb_get_register_packet(struct connection *connection,
	char const *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	char *reg_packet;
	int reg_num = strtoul(packet + 1, NULL, 16);
	struct reg **reg_list;
	int reg_list_size;
	int retval;

#ifdef _DEBUG_GDB_IO_
	LOG_DEBUG("-");
#endif

	if ((target->rtos) && (rtos_get_gdb_reg(connection, reg_num) == ERROR_OK))
		return ERROR_OK;

	retval = target_get_gdb_reg_list_noread(target, &reg_list, &reg_list_size,
			REG_CLASS_ALL);
	if (retval != ERROR_OK)
		return gdb_error(connection, retval);

	if ((reg_list_size <= reg_num) || !reg_list[reg_num] ||
		!reg_list[reg_num]->exist || reg_list[reg_num]->hidden) {
		LOG_ERROR("gdb requested a non-existing register (reg_num=%d)", reg_num);
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	if (!reg_list[reg_num]->valid) {
		retval = reg_list[reg_num]->type->get(reg_list[reg_num]);
		if (retval != ERROR_OK && gdb_report_register_access_error) {
			LOG_DEBUG("Couldn't get register %s.", reg_list[reg_num]->name);
			free(reg_list);
			return gdb_error(connection, retval);
		}
	}

	reg_packet = calloc(DIV_ROUND_UP(reg_list[reg_num]->size, 8) * 2 + 1, 1); /* plus one for string termination null */

	gdb_str_to_target(target, reg_packet, reg_list[reg_num]);

	gdb_put_packet(connection, reg_packet, DIV_ROUND_UP(reg_list[reg_num]->size, 8) * 2);

	free(reg_list);
	free(reg_packet);

	return ERROR_OK;
}

static int gdb_set_register_packet(struct connection *connection,
	char const *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	char *separator;
	int reg_num = strtoul(packet + 1, &separator, 16);
	struct reg **reg_list;
	int reg_list_size;
	int retval;

#ifdef _DEBUG_GDB_IO_
	LOG_DEBUG("-");
#endif

	if (*separator != '=') {
		LOG_ERROR("GDB 'set register packet', but no '=' following the register number");
		return ERROR_SERVER_REMOTE_CLOSED;
	}
	size_t chars = strlen(separator + 1);
	uint8_t *bin_buf = malloc(chars / 2);
	gdb_target_to_reg(target, separator + 1, chars, bin_buf);

	if ((target->rtos) &&
			(rtos_set_reg(connection, reg_num, bin_buf) == ERROR_OK)) {
		free(bin_buf);
		gdb_put_packet(connection, "OK", 2);
		return ERROR_OK;
	}

	retval = target_get_gdb_reg_list_noread(target, &reg_list, &reg_list_size,
			REG_CLASS_ALL);
	if (retval != ERROR_OK) {
		free(bin_buf);
		return gdb_error(connection, retval);
	}

	if ((reg_list_size <= reg_num) || !reg_list[reg_num] ||
		!reg_list[reg_num]->exist || reg_list[reg_num]->hidden) {
		LOG_ERROR("gdb requested a non-existing register (reg_num=%d)", reg_num);
		free(bin_buf);
		free(reg_list);
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	if (chars != (DIV_ROUND_UP(reg_list[reg_num]->size, 8) * 2)) {
		LOG_ERROR("gdb sent %zu bits for a %" PRIu32 "-bit register (%s)",
				chars * 4, reg_list[reg_num]->size, reg_list[reg_num]->name);
		free(bin_buf);
		free(reg_list);
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	gdb_target_to_reg(target, separator + 1, chars, bin_buf);

	retval = reg_list[reg_num]->type->set(reg_list[reg_num], bin_buf);
	if (retval != ERROR_OK && gdb_report_register_access_error) {
		LOG_DEBUG("Couldn't set register %s.", reg_list[reg_num]->name);
		free(bin_buf);
		free(reg_list);
		return gdb_error(connection, retval);
	}

	gdb_put_packet(connection, "OK", 2);

	free(bin_buf);
	free(reg_list);

	return ERROR_OK;
}

/* No attempt is made to translate the "retval" to
 * GDB speak. This has to be done at the calling
 * site as no mapping really exists.
 */
static int gdb_error(struct connection *connection, int retval)
{
	LOG_DEBUG("Reporting %i to GDB as generic error", retval);
	gdb_send_error(connection, EFAULT);
	return ERROR_OK;
}

/* We don't have to worry about the default 2 second timeout for GDB packets,
 * because GDB breaks up large memory reads into smaller reads.
 */
 /* 负责解析 GDB 发送的二进制格式的内存读取请求，从目标设备的内存中读取数据，并将读取到的数据以十六进制格式返回给 GDB
  * addr：表示要读取的起始地址（64 位无符号整数）
  * len：表示要读取的数据长度（32 位无符号整数）
  * buffer：用于存储从目标设备读取的原始数据
  * hex_buffer：用于存储转换后的十六进制字符串，最终发送给 GDB
  * 
  */
static int gdb_read_memory_packet(struct connection *connection,
		char const *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	char *separator;
	uint64_t addr = 0;
	uint32_t len = 0;

	uint8_t *buffer;
	char *hex_buffer;

	int retval = ERROR_OK;

	/* skip command character */
	packet++;

	addr = strtoull(packet, &separator, 16);

	if (*separator != ',') {
		LOG_ERROR("incomplete read memory packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	len = strtoul(separator + 1, NULL, 16);

	if (!len) {
		LOG_WARNING("invalid read memory packet received (len == 0)");
		gdb_put_packet(connection, "", 0);
		return ERROR_OK;
	}

	//为读取的数据分配一个 len 大小的缓冲区 buffer。这个缓冲区将用于存储从目标设备读取的原始数据
	buffer = malloc(len);

	LOG_DEBUG("addr: 0x%16.16" PRIx64 ", len: 0x%8.8" PRIx32 "", addr, len);

	retval = ERROR_NOT_IMPLEMENTED;
	if (target->rtos)
		retval = rtos_read_buffer(target, addr, len, buffer);
	if (retval == ERROR_NOT_IMPLEMENTED)
		retval = target_read_buffer(target, addr, len, buffer);

	if ((retval != ERROR_OK) && !gdb_report_data_abort) {
		/* TODO : Here we have to lie and send back all zero's lest stack traces won't work.
		 * At some point this might be fixed in GDB, in which case this code can be removed.
		 *
		 * OpenOCD developers are acutely aware of this problem, but there is nothing
		 * gained by involving the user in this problem that hopefully will get resolved
		 * eventually
		 *
		 * http://sourceware.org/cgi-bin/gnatsweb.pl? \
		 * cmd = view%20audit-trail&database = gdb&pr = 2395
		 *
		 * For now, the default is to fix up things to make current GDB versions work.
		 * This can be overwritten using the gdb_report_data_abort <'enable'|'disable'> command.
		 */
		memset(buffer, 0, len);
		retval = ERROR_OK;
	}

	if (retval == ERROR_OK) {
		/* malloc(len * 2 + 1)：为转换后的十六进制字符串分配足够的空间。每个字节需要两个字符来表示，因此 len * 2 是所需的字符数，再加上一个终止符 \0。
		 * hexify：将 buffer 中的原始数据转换为十六进制字符串，并存储在 hex_buffer 中。pkt_len 是转换后字符串的实际长度。
		 * gdb_put_packet：将转换后的十六进制字符串发送给 GDB。
         * free(hex_buffer)：释放 hex_buffer 的内存
		 */
		hex_buffer = malloc(len * 2 + 1);

		size_t pkt_len = hexify(hex_buffer, buffer, len, len * 2 + 1);

		gdb_put_packet(connection, hex_buffer, pkt_len);

		free(hex_buffer);
	} else
		retval = gdb_error(connection, retval);

	free(buffer);

	return retval;
}

static int gdb_write_memory_packet(struct connection *connection,
		char const *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	char *separator;
	uint64_t addr = 0;
	uint32_t len = 0;

	uint8_t *buffer;
	int retval;

	/* skip command character */
	packet++;

	addr = strtoull(packet, &separator, 16);

	if (*separator != ',') {
		LOG_ERROR("incomplete write memory packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	len = strtoul(separator + 1, &separator, 16);

	if (*(separator++) != ':') {
		LOG_ERROR("incomplete write memory packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	buffer = malloc(len);

	LOG_DEBUG("addr: 0x%" PRIx64 ", len: 0x%8.8" PRIx32 "", addr, len);

	if (unhexify(buffer, separator, len) != len)
		LOG_ERROR("unable to decode memory packet");

	retval = ERROR_NOT_IMPLEMENTED;
	if (target->rtos)
		retval = rtos_write_buffer(target, addr, len, buffer);
	if (retval == ERROR_NOT_IMPLEMENTED)
		retval = target_write_buffer(target, addr, len, buffer);

	if (retval == ERROR_OK)
		gdb_put_packet(connection, "OK", 2);
	else
		retval = gdb_error(connection, retval);

	free(buffer);

	return retval;
}

/* 负责解析 GDB 发送的二进制格式的内存写入请求，并将数据写入目标设备的内存中
 * separator：用于解析 packet 中的分隔符，指向解析后的下一个字符，即地址部分的结束位置
 * fast_limit：定义了一个阈值，表示当写入数据长度大于或等于这个值时，GDB 会立即确认写入操作，以加快下载速度
 * 实现流程：
 * 		跳过命令字符
 *		解析地址
 *		检查分隔符：检查 separator 指向的字符是否为逗号（,），这是地址和长度之间的分隔符
 *		解析长度：
 *		检查分隔符：检查 separator 指向的字符是否为冒号（:），这是长度和数据之间的分隔符
 *		获取 GDB 连接对象：
 *		检查内存写入错误：
 *		快速确认大写入：发送一个确认包（"OK"）给 GDB，表示写入操作已经开始。这可以加速下载过程，因为 GDB 可以在等待确认的同时继续发送新的数据包
 *		实际写入内存:
 *			记录调试信息，显示要写入的地址和长度
 *			如果目标设备运行了实时操作系统（RTOS），尝试使用 RTOS 特定的写入函数 rtos_write_buffer 来写入数据
 *			如果 rtos_write_buffer 不可用（返回 ERROR_NOT_IMPLEMENTED），则使用通用的 target_write_buffer 函数来写入数据
 *			gdb_connection->mem_write_error = true：如果写入操作失败，设置内存写入错误标志，以便后续处理
 *		处理小写入:
 *			对于小于 fast_limit 的写入操作，GDB 不会立即确认写入。因此，需要在写入完成后发送确认包或错误信息
 *			如果写入失败，发送错误信息给 GDB，并清除内存写入错误标志
 *			如果写入成功，发送确认包（"OK"）给 GDB
 *			
 */
static int gdb_write_memory_binary_packet(struct connection *connection,
		char const *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	char *separator;
	uint64_t addr = 0;
	uint32_t len = 0;

	int retval = ERROR_OK;
	/* Packets larger than fast_limit bytes will be acknowledged instantly on
	 * the assumption that we're in a download and it's important to go as fast
	 * as possible. */
	uint32_t fast_limit = 8;

	/* skip command character */
	packet++;

	addr = strtoull(packet, &separator, 16);

	if (*separator != ',') {
		LOG_ERROR("incomplete write memory binary packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	len = strtoul(separator + 1, &separator, 16);

	if (*(separator++) != ':') {
		LOG_ERROR("incomplete write memory binary packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	struct gdb_connection *gdb_connection = connection->priv;

	if (gdb_connection->mem_write_error)
		retval = ERROR_FAIL;

	if (retval == ERROR_OK) {
		if (len >= fast_limit) {
			/* By replying the packet *immediately* GDB will send us a new packet
			 * while we write the last one to the target.
			 * We only do this for larger writes, so that users who do something like:
			 * p *((int*)0xdeadbeef)=8675309
			 * will get immediate feedback that that write failed.
			 */
			gdb_put_packet(connection, "OK", 2);
		}
	//处理内存写入错误:发送错误信息给 GDB，并根据 retval 设置相应的错误码;清除内存写入错误标志
	} else {
		retval = gdb_error(connection, retval);
		/* now that we have reported the memory write error, we can clear the condition */
		gdb_connection->mem_write_error = false;
		if (retval != ERROR_OK)
			return retval;
	}

	if (len) {
		LOG_DEBUG("addr: 0x%" PRIx64 ", len: 0x%8.8" PRIx32 "", addr, len);

		retval = ERROR_NOT_IMPLEMENTED;
		if (target->rtos)
			retval = rtos_write_buffer(target, addr, len, (uint8_t *)separator);
		if (retval == ERROR_NOT_IMPLEMENTED)
			retval = target_write_buffer(target, addr, len, (uint8_t *)separator);

		if (retval != ERROR_OK)
			gdb_connection->mem_write_error = true;
	}

	if (len < fast_limit) {
		if (retval != ERROR_OK) {
			gdb_error(connection, retval);
			gdb_connection->mem_write_error = false;
		} else {
			gdb_put_packet(connection, "OK", 2);
		}
	}

	return ERROR_OK;
}

static int gdb_step_continue_packet(struct connection *connection,
		char const *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	int current = 0;
	uint64_t address = 0x0;
	int retval = ERROR_OK;

	LOG_DEBUG("-");

	if (packet_size > 1)
		address = strtoull(packet + 1, NULL, 16);
	else
		current = 1;

	gdb_running_type = packet[0];
	if (packet[0] == 'c') {
		LOG_DEBUG("continue");
		/* resume at current address, don't handle breakpoints, not debugging */
		retval = target_resume(target, current, address, 0, 0);
	} else if (packet[0] == 's') {
		LOG_DEBUG("step");
		/* step at current or address, don't handle breakpoints */
		retval = target_step(target, current, address, 0);
	}
	return retval;
}

/* 解析gdb发来的断点或监视断点的数据包，根据数据包内容配置目标设备的断点、观察点
 * connection：与gdb客户端连接的结构体
 * packet：gdb发来的数据包的内容
 * packet_size：数据包的大小
 */
static int gdb_breakpoint_watchpoint_packet(struct connection *connection,
		char const *packet, int packet_size)
{	
	//初始化：获取目标设备、软件断点、读观察点
	struct target *target = get_target_from_connection(connection); 
	int type;
	enum breakpoint_type bp_type = BKPT_SOFT /* dummy init to avoid warning */; 
	enum watchpoint_rw wp_type = WPT_READ /* dummy init to avoid warning */;
	uint64_t address;
	uint32_t size;
	char *separator;
	int retval;

	LOG_DEBUG("[%s]", target_name(target));

	//解析数据包类型：第一个字段
	//strtoul字符串转化成16进制
	type = strtoul(packet + 1, &separator, 16);

	if (type == 0)	/* memory breakpoint */
		bp_type = BKPT_SOFT;
	else if (type == 1)	/* hardware breakpoint */
		bp_type = BKPT_HARD;
	else if (type == 2)	/* write watchpoint */
		wp_type = WPT_WRITE;
	else if (type == 3)	/* read watchpoint */
		wp_type = WPT_READ;
	else if (type == 4)	/* access watchpoint */
		wp_type = WPT_ACCESS;
	else {
		LOG_ERROR("invalid gdb watch/breakpoint type(%d), dropping connection", type);
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	//如果配置启用了断点类型重写，则将断点类型强制设置为gdb_breakpoint_override_type
	if (gdb_breakpoint_override && ((bp_type == BKPT_SOFT) || (bp_type == BKPT_HARD)))
		bp_type = gdb_breakpoint_override_type;
	
	//检查数据包格式：数据包字段应使用，分割
	if (*separator != ',') {
		LOG_ERROR("incomplete breakpoint/watchpoint packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	//解析断点、监视断点地址
	address = strtoull(separator + 1, &separator, 16);

	if (*separator != ',') {
		LOG_ERROR("incomplete breakpoint/watchpoint packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}
	
	//解析断点、监视断点大小
	size = strtoul(separator + 1, &separator, 16);

	switch (type) {
		case 0:
		case 1:  /* hardware breakpoint */
			if (packet[0] == 'Z') {  //添加断点
				retval = breakpoint_add(target, address, size, bp_type);
				if (retval == ERROR_NOT_IMPLEMENTED) {   //当前目标设备不支持断点类型
					/* Send empty reply to report that breakpoints of this type are not supported */
					gdb_put_packet(connection, "", 0);  //返回空响应
				} else if (retval != ERROR_OK) {
					retval = gdb_error(connection, retval);
					if (retval != ERROR_OK)
						return retval;
				} else
					gdb_put_packet(connection, "OK", 2);
			} else {     //移除断点
				breakpoint_remove(target, address);
				gdb_put_packet(connection, "OK", 2);
			}
			break;
		case 2:
		case 3:
		case 4:  /* access watchpoint */
		{
			if (packet[0] == 'Z') {  //添加监视断点：地址、大小、类型、起始掩码、结束掩码
				retval = watchpoint_add(target, address, size, wp_type, 0, 0xffffffffu);
				if (retval == ERROR_NOT_IMPLEMENTED) {
					/* Send empty reply to report that watchpoints of this type are not supported */
					gdb_put_packet(connection, "", 0);
				} else if (retval != ERROR_OK) {
					retval = gdb_error(connection, retval);
					if (retval != ERROR_OK)
						return retval;
				} else
					gdb_put_packet(connection, "OK", 2);
			} else {
				watchpoint_remove(target, address);
				gdb_put_packet(connection, "OK", 2);
			}
			break;
		}
		default:
			break;
	}

	return ERROR_OK;
}

/* print out a string and allocate more space as needed,
 * mainly used for XML at this point
 */
static __attribute__ ((format (PRINTF_ATTRIBUTE_FORMAT, 5, 6))) void xml_printf(int *retval,
		char **xml, int *pos, int *size, const char *fmt, ...)
{
	if (*retval != ERROR_OK)
		return;
	int first = 1;

	for (;; ) {
		if ((!*xml) || (!first)) {
			/* start by 0 to exercise all the code paths.
			 * Need minimum 2 bytes to fit 1 char and 0 terminator. */

			*size = *size * 2 + 2;
			char *t = *xml;
			*xml = realloc(*xml, *size);
			if (!*xml) {
				free(t);
				*retval = ERROR_SERVER_REMOTE_CLOSED;
				return;
			}
		}

		va_list ap;
		int ret;
		va_start(ap, fmt);
		ret = vsnprintf(*xml + *pos, *size - *pos, fmt, ap);
		va_end(ap);
		if ((ret > 0) && ((ret + 1) < *size - *pos)) {
			*pos += ret;
			return;
		}
		/* there was just enough or not enough space, allocate more. */
		first = 0;
	}
}

static int decode_xfer_read(char const *buf, char **annex, int *ofs, unsigned int *len)
{
	/* Locate the annex. */
	const char *annex_end = strchr(buf, ':');
	if (!annex_end)
		return ERROR_FAIL;

	/* After the read marker and annex, qXfer looks like a
	 * traditional 'm' packet. */
	char *separator;
	*ofs = strtoul(annex_end + 1, &separator, 16);

	if (*separator != ',')
		return ERROR_FAIL;

	*len = strtoul(separator + 1, NULL, 16);

	/* Extract the annex if needed */
	if (annex) {
		*annex = strndup(buf, annex_end - buf);
		if (!*annex)
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int compare_bank(const void *a, const void *b)
{
	struct flash_bank *b1, *b2;
	b1 = *((struct flash_bank **)a);
	b2 = *((struct flash_bank **)b);

	if (b1->base == b2->base)
		return 0;
	else if (b1->base > b2->base)
		return 1;
	else
		return -1;
}

static int gdb_memory_map(struct connection *connection,
		char const *packet, int packet_size)
{
	/* We get away with only specifying flash here. Regions that are not
	 * specified are treated as if we provided no memory map(if not we
	 * could detect the holes and mark them as RAM).
	 * Normally we only execute this code once, but no big deal if we
	 * have to regenerate it a couple of times.
	 */

	struct target *target = get_target_from_connection(connection);
	struct flash_bank *p;
	char *xml = NULL;
	int size = 0;
	int pos = 0;
	int retval = ERROR_OK;
	struct flash_bank **banks;
	int offset;
	int length;
	char *separator;
	target_addr_t ram_start = 0;
	unsigned int target_flash_banks = 0;

	/* skip command character */
	packet += 23;

	offset = strtoul(packet, &separator, 16);
	length = strtoul(separator + 1, &separator, 16);

	xml_printf(&retval, &xml, &pos, &size, "<memory-map>\n");

	/* Sort banks in ascending order.  We need to report non-flash
	 * memory as ram (or rather read/write) by default for GDB, since
	 * it has no concept of non-cacheable read/write memory (i/o etc).
	 */
	banks = malloc(sizeof(struct flash_bank *)*flash_get_bank_count());

	for (unsigned int i = 0; i < flash_get_bank_count(); i++) {
		p = get_flash_bank_by_num_noprobe(i);
		if (p->target != target)
			continue;
		retval = get_flash_bank_by_num(i, &p);
		if (retval != ERROR_OK) {
			free(banks);
			gdb_error(connection, retval);
			return retval;
		}
		banks[target_flash_banks++] = p;
	}

	qsort(banks, target_flash_banks, sizeof(struct flash_bank *),
		compare_bank);

	for (unsigned int i = 0; i < target_flash_banks; i++) {
		unsigned sector_size = 0;
		unsigned group_len = 0;

		p = banks[i];

		if (ram_start < p->base)
			xml_printf(&retval, &xml, &pos, &size,
				"<memory type=\"ram\" start=\"" TARGET_ADDR_FMT "\" "
				"length=\"" TARGET_ADDR_FMT "\"/>\n",
				ram_start, p->base - ram_start);

		/* Report adjacent groups of same-size sectors.  So for
		 * example top boot CFI flash will list an initial region
		 * with several large sectors (maybe 128KB) and several
		 * smaller ones at the end (maybe 32KB).  STR7 will have
		 * regions with 8KB, 32KB, and 64KB sectors; etc.
		 */
		for (unsigned int j = 0; j < p->num_sectors; j++) {

			/* Maybe start a new group of sectors. */
			if (sector_size == 0) {
				if (p->sectors[j].offset + p->sectors[j].size > p->size) {
					LOG_WARNING("The flash sector at offset 0x%08" PRIx32
						" overflows the end of %s bank.",
						p->sectors[j].offset, p->name);
					LOG_WARNING("The rest of bank will not show in gdb memory map.");
					break;
				}
				target_addr_t start;
				start = p->base + p->sectors[j].offset;
				xml_printf(&retval, &xml, &pos, &size,
					"<memory type=\"flash\" "
					"start=\"" TARGET_ADDR_FMT "\" ",
					start);
				sector_size = p->sectors[j].size;
				group_len = sector_size;
			} else {
				group_len += sector_size; /* equal to p->sectors[j].size */
			}

			/* Does this finish a group of sectors?
			 * If not, continue an already-started group.
			 */
			if (j < p->num_sectors - 1
					&& p->sectors[j + 1].size == sector_size
					&& p->sectors[j + 1].offset == p->sectors[j].offset + sector_size
					&& p->sectors[j + 1].offset + p->sectors[j + 1].size <= p->size)
				continue;

			xml_printf(&retval, &xml, &pos, &size,
				"length=\"0x%x\">\n"
				"<property name=\"blocksize\">"
				"0x%x</property>\n"
				"</memory>\n",
				group_len,
				sector_size);
			sector_size = 0;
		}

		ram_start = p->base + p->size;
	}

	if (ram_start != 0)
		xml_printf(&retval, &xml, &pos, &size,
			"<memory type=\"ram\" start=\"" TARGET_ADDR_FMT "\" "
			"length=\"" TARGET_ADDR_FMT "\"/>\n",
			ram_start, target_address_max(target) - ram_start + 1);
	/* ELSE a flash chip could be at the very end of the address space, in
	 * which case ram_start will be precisely 0 */

	free(banks);

	xml_printf(&retval, &xml, &pos, &size, "</memory-map>\n");

	if (retval != ERROR_OK) {
		free(xml);
		gdb_error(connection, retval);
		return retval;
	}

	if (offset + length > pos)
		length = pos - offset;

	char *t = malloc(length + 1);
	t[0] = 'l';
	memcpy(t + 1, xml + offset, length);
	gdb_put_packet(connection, t, length + 1);

	free(t);
	free(xml);
	return ERROR_OK;
}

static const char *gdb_get_reg_type_name(enum reg_type type)
{
	switch (type) {
		case REG_TYPE_BOOL:
			return "bool";
		case REG_TYPE_INT:
			return "int";
		case REG_TYPE_INT8:
			return "int8";
		case REG_TYPE_INT16:
			return "int16";
		case REG_TYPE_INT32:
			return "int32";
		case REG_TYPE_INT64:
			return "int64";
		case REG_TYPE_INT128:
			return "int128";
		case REG_TYPE_UINT:
			return "uint";
		case REG_TYPE_UINT8:
			return "uint8";
		case REG_TYPE_UINT16:
			return "uint16";
		case REG_TYPE_UINT32:
			return "uint32";
		case REG_TYPE_UINT64:
			return "uint64";
		case REG_TYPE_UINT128:
			return "uint128";
		case REG_TYPE_CODE_PTR:
			return "code_ptr";
		case REG_TYPE_DATA_PTR:
			return "data_ptr";
		case REG_TYPE_FLOAT:
			return "float";
		case REG_TYPE_IEEE_SINGLE:
			return "ieee_single";
		case REG_TYPE_IEEE_DOUBLE:
			return "ieee_double";
		case REG_TYPE_ARCH_DEFINED:
			return "int"; /* return arbitrary string to avoid compile warning. */
	}

	return "int"; /* "int" as default value */
}

static int lookup_add_arch_defined_types(char const **arch_defined_types_list[], const char *type_id,
					int *num_arch_defined_types)
{
	int tbl_sz = *num_arch_defined_types;

	if (type_id && (strcmp(type_id, ""))) {
		for (int j = 0; j < (tbl_sz + 1); j++) {
			if (!((*arch_defined_types_list)[j])) {
				(*arch_defined_types_list)[tbl_sz++] = type_id;
				*arch_defined_types_list = realloc(*arch_defined_types_list,
								sizeof(char *) * (tbl_sz + 1));
				(*arch_defined_types_list)[tbl_sz] = NULL;
				*num_arch_defined_types = tbl_sz;
				return 1;
			} else {
				if (!strcmp((*arch_defined_types_list)[j], type_id))
					return 0;
			}
		}
	}

	return -1;
}

static int gdb_generate_reg_type_description(struct target *target,
		char **tdesc, int *pos, int *size, struct reg_data_type *type,
		char const **arch_defined_types_list[], int *num_arch_defined_types)
{
	int retval = ERROR_OK;

	if (type->type_class == REG_TYPE_CLASS_VECTOR) {
		struct reg_data_type *data_type = type->reg_type_vector->type;
		if (data_type->type == REG_TYPE_ARCH_DEFINED) {
			if (lookup_add_arch_defined_types(arch_defined_types_list, data_type->id,
							num_arch_defined_types))
				gdb_generate_reg_type_description(target, tdesc, pos, size, data_type,
								arch_defined_types_list,
								num_arch_defined_types);
		}
		/* <vector id="id" type="type" count="count"/> */
		xml_printf(&retval, tdesc, pos, size,
				"<vector id=\"%s\" type=\"%s\" count=\"%" PRIu32 "\"/>\n",
				type->id, type->reg_type_vector->type->id,
				type->reg_type_vector->count);

	} else if (type->type_class == REG_TYPE_CLASS_UNION) {
		struct reg_data_type_union_field *field;
		field = type->reg_type_union->fields;
		while (field) {
			struct reg_data_type *data_type = field->type;
			if (data_type->type == REG_TYPE_ARCH_DEFINED) {
				if (lookup_add_arch_defined_types(arch_defined_types_list, data_type->id,
								num_arch_defined_types))
					gdb_generate_reg_type_description(target, tdesc, pos, size, data_type,
									arch_defined_types_list,
									num_arch_defined_types);
			}

			field = field->next;
		}
		/* <union id="id">
		 *  <field name="name" type="type"/> ...
		 * </union> */
		xml_printf(&retval, tdesc, pos, size,
				"<union id=\"%s\">\n",
				type->id);

		field = type->reg_type_union->fields;
		while (field) {
			xml_printf(&retval, tdesc, pos, size,
					"<field name=\"%s\" type=\"%s\"/>\n",
					field->name, field->type->id);

			field = field->next;
		}

		xml_printf(&retval, tdesc, pos, size,
				"</union>\n");

	} else if (type->type_class == REG_TYPE_CLASS_STRUCT) {
		struct reg_data_type_struct_field *field;
		field = type->reg_type_struct->fields;

		if (field->use_bitfields) {
			/* <struct id="id" size="size">
			 *  <field name="name" start="start" end="end"/> ...
			 * </struct> */
			xml_printf(&retval, tdesc, pos, size,
					"<struct id=\"%s\" size=\"%" PRIu32 "\">\n",
					type->id, type->reg_type_struct->size);
			while (field) {
				xml_printf(&retval, tdesc, pos, size,
						"<field name=\"%s\" start=\"%" PRIu32 "\" end=\"%" PRIu32 "\" type=\"%s\" />\n",
						field->name, field->bitfield->start, field->bitfield->end,
						gdb_get_reg_type_name(field->bitfield->type));

				field = field->next;
			}
		} else {
			while (field) {
				struct reg_data_type *data_type = field->type;
				if (data_type->type == REG_TYPE_ARCH_DEFINED) {
					if (lookup_add_arch_defined_types(arch_defined_types_list, data_type->id,
									num_arch_defined_types))
						gdb_generate_reg_type_description(target, tdesc, pos, size, data_type,
										arch_defined_types_list,
										num_arch_defined_types);
				}
			}

			/* <struct id="id">
			 *  <field name="name" type="type"/> ...
			 * </struct> */
			xml_printf(&retval, tdesc, pos, size,
					"<struct id=\"%s\">\n",
					type->id);
			while (field) {
				xml_printf(&retval, tdesc, pos, size,
						"<field name=\"%s\" type=\"%s\"/>\n",
						field->name, field->type->id);

				field = field->next;
			}
		}

		xml_printf(&retval, tdesc, pos, size,
				"</struct>\n");

	} else if (type->type_class == REG_TYPE_CLASS_FLAGS) {
		/* <flags id="id" size="size">
		 *  <field name="name" start="start" end="end"/> ...
		 * </flags> */
		xml_printf(&retval, tdesc, pos, size,
				"<flags id=\"%s\" size=\"%" PRIu32 "\">\n",
				type->id, type->reg_type_flags->size);

		struct reg_data_type_flags_field *field;
		field = type->reg_type_flags->fields;
		while (field) {
			xml_printf(&retval, tdesc, pos, size,
					"<field name=\"%s\" start=\"%" PRIu32 "\" end=\"%" PRIu32 "\" type=\"%s\" />\n",
					field->name, field->bitfield->start, field->bitfield->end,
					gdb_get_reg_type_name(field->bitfield->type));

			field = field->next;
		}

		xml_printf(&retval, tdesc, pos, size,
				"</flags>\n");

	}

	return ERROR_OK;
}

/* Get a list of available target registers features. feature_list must
 * be freed by caller.
 */
static int get_reg_features_list(struct target *target, char const **feature_list[], int *feature_list_size,
		struct reg **reg_list, int reg_list_size)
{
	int tbl_sz = 0;

	/* Start with only one element */
	*feature_list = calloc(1, sizeof(char *));

	for (int i = 0; i < reg_list_size; i++) {
		if (reg_list[i]->exist == false || reg_list[i]->hidden)
			continue;

		if (reg_list[i]->feature
			&& reg_list[i]->feature->name
			&& (strcmp(reg_list[i]->feature->name, ""))) {
			/* We found a feature, check if the feature is already in the
			 * table. If not, allocate a new entry for the table and
			 * put the new feature in it.
			 */
			for (int j = 0; j < (tbl_sz + 1); j++) {
				if (!((*feature_list)[j])) {
					(*feature_list)[tbl_sz++] = reg_list[i]->feature->name;
					*feature_list = realloc(*feature_list, sizeof(char *) * (tbl_sz + 1));
					(*feature_list)[tbl_sz] = NULL;
					break;
				} else {
					if (!strcmp((*feature_list)[j], reg_list[i]->feature->name))
						break;
				}
			}
		}
	}

	if (feature_list_size)
		*feature_list_size = tbl_sz;

	return ERROR_OK;
}

/* Create a register list that's the union of all the registers of the SMP
 * group this target is in. If the target is not part of an SMP group, this
 * returns the same as target_get_gdb_reg_list_noread().
 */
static int smp_reg_list_noread(struct target *target,
		struct reg **combined_list[], int *combined_list_size,
		enum target_register_class reg_class)
{
	if (!target->smp)
		return target_get_gdb_reg_list_noread(target, combined_list,
				combined_list_size, REG_CLASS_ALL);

	unsigned int combined_allocated = 256;
	struct reg **local_list = malloc(combined_allocated * sizeof(struct reg *));
	if (!local_list) {
		LOG_ERROR("malloc(%zu) failed", combined_allocated * sizeof(struct reg *));
		return ERROR_FAIL;
	}
	unsigned int local_list_size = 0;

	struct target_list *head;
	foreach_smp_target(head, target->smp_targets) {
		if (!target_was_examined(head->target))
			continue;

		struct reg **reg_list = NULL;
		int reg_list_size;
		int result = target_get_gdb_reg_list_noread(head->target, &reg_list,
				&reg_list_size, reg_class);
		if (result != ERROR_OK) {
			free(local_list);
			return result;
		}
		for (int i = 0; i < reg_list_size; i++) {
			bool found = false;
			struct reg *a = reg_list[i];
			if (a->exist) {
				/* Nested loop makes this O(n^2), but this entire function with
				 * 5 RISC-V targets takes just 2ms on my computer. Fast enough
				 * for me. */
				for (unsigned int j = 0; j < local_list_size; j++) {
					struct reg *b = local_list[j];
					if (!strcmp(a->name, b->name)) {
						found = true;
						if (a->size != b->size) {
							LOG_ERROR("SMP register %s is %d bits on one "
									"target, but %d bits on another target.",
									a->name, a->size, b->size);
							free(reg_list);
							free(local_list);
							return ERROR_FAIL;
						}
						break;
					}
				}
				if (!found) {
					LOG_DEBUG("[%s] %s not found in combined list", target_name(target), a->name);
					if (local_list_size >= combined_allocated) {
						combined_allocated *= 2;
						local_list = realloc(local_list, combined_allocated * sizeof(struct reg *));
						if (!local_list) {
							LOG_ERROR("realloc(%zu) failed", combined_allocated * sizeof(struct reg *));
							return ERROR_FAIL;
						}
					}
					local_list[local_list_size] = a;
					local_list_size++;
				}
			}
		}
		free(reg_list);
	}

	if (local_list_size == 0) {
		LOG_ERROR("Unable to get register list");
		free(local_list);
		return ERROR_FAIL;
	}

	/* Now warn the user about any registers that weren't found in every target. */
	foreach_smp_target(head, target->smp_targets) {
		if (!target_was_examined(head->target))
			continue;

		struct reg **reg_list = NULL;
		int reg_list_size;
		int result = target_get_gdb_reg_list_noread(head->target, &reg_list,
				&reg_list_size, reg_class);
		if (result != ERROR_OK) {
			free(local_list);
			return result;
		}
		for (unsigned int i = 0; i < local_list_size; i++) {
			bool found = false;
			struct reg *a = local_list[i];
			for (int j = 0; j < reg_list_size; j++) {
				struct reg *b = reg_list[j];
				if (b->exist && !strcmp(a->name, b->name)) {
					found = true;
					break;
				}
			}
			if (!found) {
				LOG_WARNING("Register %s does not exist in %s, which is part of an SMP group where "
					    "this register does exist.",
					    a->name, target_name(head->target));
			}
		}
		free(reg_list);
	}

	*combined_list = local_list;
	*combined_list_size = local_list_size;
	return ERROR_OK;
}

static int gdb_generate_target_description(struct target *target, char **tdesc_out)
{
	int retval = ERROR_OK;
	struct reg **reg_list = NULL;
	int reg_list_size;
	char const *architecture;
	char const **features = NULL;
	int feature_list_size = 0;
	char *tdesc = NULL;
	int pos = 0;
	int size = 0;


	retval = smp_reg_list_noread(target, &reg_list, &reg_list_size,
			REG_CLASS_ALL);

	if (retval != ERROR_OK) {
		LOG_ERROR("get register list failed");
		retval = ERROR_FAIL;
		goto error;
	}

	if (reg_list_size <= 0) {
		LOG_ERROR("get register list failed");
		retval = ERROR_FAIL;
		goto error;
	}

	/* Get a list of available target registers features */
	retval = get_reg_features_list(target, &features, &feature_list_size, reg_list, reg_list_size);
	if (retval != ERROR_OK) {
		LOG_ERROR("Can't get the registers feature list");
		retval = ERROR_FAIL;
		goto error;
	}

	/* If we found some features associated with registers, create sections */
	int current_feature = 0;

	xml_printf(&retval, &tdesc, &pos, &size,
			"<?xml version=\"1.0\"?>\n"
			"<!DOCTYPE target SYSTEM \"gdb-target.dtd\">\n"
			"<target version=\"1.0\">\n");

	/* generate architecture element if supported by target */
	architecture = target_get_gdb_arch(target);
	if (architecture)
		xml_printf(&retval, &tdesc, &pos, &size,
				"<architecture>%s</architecture>\n", architecture);

	/* generate target description according to register list */
	if (features) {
		while (features[current_feature]) {
			char const **arch_defined_types = NULL;
			int num_arch_defined_types = 0;

			arch_defined_types = calloc(1, sizeof(char *));
			xml_printf(&retval, &tdesc, &pos, &size,
					"<feature name=\"%s\">\n",
					features[current_feature]);

			int i;
			for (i = 0; i < reg_list_size; i++) {

				if (reg_list[i]->exist == false || reg_list[i]->hidden)
					continue;

				if (strcmp(reg_list[i]->feature->name, features[current_feature]))
					continue;

				const char *type_str;
				if (reg_list[i]->reg_data_type) {
					if (reg_list[i]->reg_data_type->type == REG_TYPE_ARCH_DEFINED) {
						/* generate <type... first, if there are architecture-defined types. */
						if (lookup_add_arch_defined_types(&arch_defined_types,
										reg_list[i]->reg_data_type->id,
										&num_arch_defined_types))
							gdb_generate_reg_type_description(target, &tdesc, &pos, &size,
											reg_list[i]->reg_data_type,
											&arch_defined_types,
											&num_arch_defined_types);

						type_str = reg_list[i]->reg_data_type->id;
					} else {
						/* predefined type */
						type_str = gdb_get_reg_type_name(
								reg_list[i]->reg_data_type->type);
					}
				} else {
					/* Default type is "int" */
					type_str = "int";
				}

				xml_printf(&retval, &tdesc, &pos, &size,
						"<reg name=\"%s\"", reg_list[i]->name);
				xml_printf(&retval, &tdesc, &pos, &size,
						" bitsize=\"%" PRIu32 "\"", reg_list[i]->size);
				xml_printf(&retval, &tdesc, &pos, &size,
						" regnum=\"%" PRIu32 "\"", reg_list[i]->number);
				if (reg_list[i]->caller_save)
					xml_printf(&retval, &tdesc, &pos, &size,
							" save-restore=\"yes\"");
				else
					xml_printf(&retval, &tdesc, &pos, &size,
							" save-restore=\"no\"");

				xml_printf(&retval, &tdesc, &pos, &size,
						" type=\"%s\"", type_str);

				if (reg_list[i]->group)
					xml_printf(&retval, &tdesc, &pos, &size,
							" group=\"%s\"", reg_list[i]->group);

				xml_printf(&retval, &tdesc, &pos, &size,
						"/>\n");
			}

			xml_printf(&retval, &tdesc, &pos, &size,
					"</feature>\n");

			current_feature++;
			free(arch_defined_types);
		}
	}

	xml_printf(&retval, &tdesc, &pos, &size,
			"</target>\n");

error:
	free(features);
	free(reg_list);

	if (retval == ERROR_OK)
		*tdesc_out = tdesc;
	else
		free(tdesc);

	return retval;
}

static int gdb_get_target_description_chunk(struct target *target, struct target_desc_format *target_desc,
		char **chunk, int32_t offset, uint32_t length)
{
	if (!target_desc) {
		LOG_ERROR("Unable to Generate Target Description");
		return ERROR_FAIL;
	}

	char *tdesc = target_desc->tdesc;
	uint32_t tdesc_length = target_desc->tdesc_length;

	if (!tdesc) {
		int retval = gdb_generate_target_description(target, &tdesc);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to Generate Target Description");
			return ERROR_FAIL;
		}

		tdesc_length = strlen(tdesc);
	}

	char transfer_type;

	if (length < (tdesc_length - offset))
		transfer_type = 'm';
	else
		transfer_type = 'l';

	*chunk = malloc(length + 2);
	if (!*chunk) {
		LOG_ERROR("Unable to allocate memory");
		return ERROR_FAIL;
	}

	(*chunk)[0] = transfer_type;
	if (transfer_type == 'm') {
		strncpy((*chunk) + 1, tdesc + offset, length);
		(*chunk)[1 + length] = '\0';
	} else {
		strncpy((*chunk) + 1, tdesc + offset, tdesc_length - offset);
		(*chunk)[1 + (tdesc_length - offset)] = '\0';

		/* After gdb-server sends out last chunk, invalidate tdesc. */
		free(tdesc);
		tdesc = NULL;
		tdesc_length = 0;
	}

	target_desc->tdesc = tdesc;
	target_desc->tdesc_length = tdesc_length;

	return ERROR_OK;
}

static int gdb_target_description_supported(struct target *target, int *supported)
{
	int retval = ERROR_OK;
	struct reg **reg_list = NULL;
	int reg_list_size = 0;
	char const **features = NULL;
	int feature_list_size = 0;

	char const *architecture = target_get_gdb_arch(target);

	retval = target_get_gdb_reg_list_noread(target, &reg_list,
			&reg_list_size, REG_CLASS_ALL);
	if (retval != ERROR_OK) {
		LOG_ERROR("get register list failed");
		goto error;
	}

	if (reg_list_size <= 0) {
		LOG_ERROR("get register list failed");
		retval = ERROR_FAIL;
		goto error;
	}

	/* Get a list of available target registers features */
	retval = get_reg_features_list(target, &features, &feature_list_size, reg_list, reg_list_size);
	if (retval != ERROR_OK) {
		LOG_ERROR("Can't get the registers feature list");
		goto error;
	}

	if (supported) {
		if (architecture || feature_list_size)
			*supported = 1;
		else
			*supported = 0;
	}

error:
	free(features);

	free(reg_list);

	return retval;
}

static int gdb_generate_thread_list(struct target *target, char **thread_list_out)
{
	struct rtos *rtos = target->rtos;
	int retval = ERROR_OK;
	char *thread_list = NULL;
	int pos = 0;
	int size = 0;

	xml_printf(&retval, &thread_list, &pos, &size,
		   "<?xml version=\"1.0\"?>\n"
		   "<threads>\n");

	if (rtos) {
		for (int i = 0; i < rtos->thread_count; i++) {
			struct thread_detail *thread_detail = &rtos->thread_details[i];

			if (!thread_detail->exists)
				continue;

			if (thread_detail->thread_name_str)
				xml_printf(&retval, &thread_list, &pos, &size,
					   "<thread id=\"%" PRIx64 "\" name=\"%s\">",
					   thread_detail->threadid,
					   thread_detail->thread_name_str);
			else
				xml_printf(&retval, &thread_list, &pos, &size,
					   "<thread id=\"%" PRIx64 "\">", thread_detail->threadid);

			if (thread_detail->thread_name_str)
				xml_printf(&retval, &thread_list, &pos, &size,
					   "Name: %s", thread_detail->thread_name_str);

			if (thread_detail->extra_info_str) {
				if (thread_detail->thread_name_str)
					xml_printf(&retval, &thread_list, &pos, &size,
						   ", ");
				xml_printf(&retval, &thread_list, &pos, &size,
					   "%s", thread_detail->extra_info_str);
			}

			xml_printf(&retval, &thread_list, &pos, &size,
				   "</thread>\n");
		}
	}

	xml_printf(&retval, &thread_list, &pos, &size,
		   "</threads>\n");

	if (retval == ERROR_OK)
		*thread_list_out = thread_list;
	else
		free(thread_list);

	return retval;
}

//用于从目标设备的线程列表中提取指定范围的线程信息，并将其转换为 XML 格式返回给 GDB 客户端。且负责处理分块传输的逻辑，确保数据可以逐步发送
static int gdb_get_thread_list_chunk(struct target *target, char **thread_list,
		char **chunk, int32_t offset, uint32_t length)
{
	if (!*thread_list) {
		int retval = gdb_generate_thread_list(target, thread_list);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to Generate Thread List");
			return ERROR_FAIL;
		}
	}

	size_t thread_list_length = strlen(*thread_list);
	char transfer_type;

	length = MIN(length, thread_list_length - offset);
	if (length < (thread_list_length - offset))
		transfer_type = 'm';
	else
		transfer_type = 'l';

	*chunk = malloc(length + 2 + 3);
	/* Allocating extra 3 bytes prevents false positive valgrind report
	 * of strlen(chunk) word access:
	 * Invalid read of size 4
	 * Address 0x4479934 is 44 bytes inside a block of size 45 alloc'd */
	if (!*chunk) {
		LOG_ERROR("Unable to allocate memory");
		return ERROR_FAIL;
	}

	(*chunk)[0] = transfer_type;
	strncpy((*chunk) + 1, (*thread_list) + offset, length);
	(*chunk)[1 + length] = '\0';

	/* After gdb-server sends out last chunk, invalidate thread list. */
	if (transfer_type == 'l') {
		free(*thread_list);
		*thread_list = NULL;
	}

	return ERROR_OK;
}

//用于处理来自GDB客户端的查询（query）命令。它根据不同的查询命令执行相应的操作，并返回适当的响应给GDB客户端
static int gdb_query_packet(struct connection *connection,
		char const *packet, int packet_size)
{
	struct command_context *cmd_ctx = connection->cmd_ctx;
	struct gdb_connection *gdb_connection = connection->priv;
	struct target *target = get_target_from_connection(connection);

	/* 解码命令：将 qRcmd, 后面的十六进制编码的字符串解码为普通字符串。
     * 执行命令：使用 Jim_EvalObj 执行解码后的命令，并捕获输出。
     * 处理结果：根据命令执行的结果，发送相应的响应给GDB客户端。如果命令成功执行且有输出，则将输出转换为十六进制格式并发送；否则发送 OK 或错误信息
	 */
	if (strncmp(packet, "qRcmd,", 6) == 0) {
		if (packet_size > 6) {
			Jim_Interp *interp = cmd_ctx->interp;
			char *cmd;
			cmd = malloc((packet_size - 6) / 2 + 1);
			size_t len = unhexify((uint8_t *)cmd, packet + 6, (packet_size - 6) / 2);
			cmd[len] = 0;

			/* We want to print all debug output to GDB connection */
			gdb_connection->output_flag = GDB_OUTPUT_ALL;
			target_call_timer_callbacks_now();
			/* some commands need to know the GDB connection, make note of current
			 * GDB connection. */
			current_gdb_connection = gdb_connection;

			struct target *saved_target_override = cmd_ctx->current_target_override;
			cmd_ctx->current_target_override = NULL;

			struct command_context *old_context = Jim_GetAssocData(interp, "context");
			Jim_DeleteAssocData(interp, "context");
			int retval = Jim_SetAssocData(interp, "context", NULL, cmd_ctx);
			if (retval == JIM_OK) {
				retval = Jim_EvalObj(interp, Jim_NewStringObj(interp, cmd, -1));
				Jim_DeleteAssocData(interp, "context");
			}
			int inner_retval = Jim_SetAssocData(interp, "context", NULL, old_context);
			if (retval == JIM_OK)
				retval = inner_retval;

			cmd_ctx->current_target_override = saved_target_override;

			current_gdb_connection = NULL;
			target_call_timer_callbacks_now();
			gdb_connection->output_flag = GDB_OUTPUT_NO;
			free(cmd);
			if (retval == JIM_RETURN)
				retval = interp->returnCode;
			int lenmsg;
			const char *cretmsg = Jim_GetString(Jim_GetResult(interp), &lenmsg);
			char *retmsg;
			if (lenmsg && cretmsg[lenmsg - 1] != '\n') {
				retmsg = alloc_printf("%s\n", cretmsg);
				lenmsg++;
			} else {
				retmsg = strdup(cretmsg);
			}
			if (!retmsg)
				return ERROR_GDB_BUFFER_TOO_SMALL;

			if (retval == JIM_OK) {
				if (lenmsg) {
					char *hex_buffer = malloc(lenmsg * 2 + 1);
					if (!hex_buffer) {
						free(retmsg);
						return ERROR_GDB_BUFFER_TOO_SMALL;
					}

					size_t pkt_len = hexify(hex_buffer, (const uint8_t *)retmsg, lenmsg,
											lenmsg * 2 + 1);
					gdb_put_packet(connection, hex_buffer, pkt_len);
					free(hex_buffer);
				} else {
					gdb_put_packet(connection, "OK", 2);
				}
			} else {
				if (lenmsg)
					gdb_output_con(connection, retmsg);
				gdb_send_error(connection, retval);
			}
			free(retmsg);
			return ERROR_OK;
		}
		gdb_put_packet(connection, "OK", 2);
		return ERROR_OK;
	} else if (strncmp(packet, "qCRC:", 5) == 0) {
	/* 解析地址和长度：从命令中提取内存地址和长度。
     * 计算校验和：调用 target_checksum_memory 计算指定内存区域的校验和。
     * 发送响应：如果计算成功，发送包含校验和的响应；否则发送错误信息
	 */
		if (packet_size > 5) {
			int retval;
			char gdb_reply[10];
			char *separator;
			uint32_t checksum;
			target_addr_t addr = 0;
			uint32_t len = 0;

			/* skip command character */
			packet += 5;

			addr = strtoull(packet, &separator, 16);

			if (*separator != ',') {
				LOG_ERROR("incomplete read memory packet received, dropping connection");
				return ERROR_SERVER_REMOTE_CLOSED;
			}

			len = strtoul(separator + 1, NULL, 16);

			retval = target_checksum_memory(target, addr, len, &checksum);

			if (retval == ERROR_OK) {
				snprintf(gdb_reply, 10, "C%8.8" PRIx32 "", checksum);
				gdb_put_packet(connection, gdb_reply, 9);
			} else {
				retval = gdb_error(connection, retval);
				if (retval != ERROR_OK)
					return retval;
			}

			return ERROR_OK;
		}
	} else if (strncmp(packet, "qSupported", 10) == 0) {
	/* 检测支持的功能：检查目标设备是否支持目标描述、内存映射等功能
     * 构建响应：根据检测结果构建支持的功能列表，并发送给GDB客户端
	 */
		/* we currently support packet size and qXfer:memory-map:read (if enabled)
		 * qXfer:features:read is supported for some targets */
		int retval = ERROR_OK;
		char *buffer = NULL;
		int pos = 0;
		int size = 0;
		int gdb_target_desc_supported = 0;

		/* we need to test that the target supports target descriptions */
		retval = gdb_target_description_supported(target, &gdb_target_desc_supported);
		if (retval != ERROR_OK) {
			LOG_INFO("Failed detecting Target Description Support, disabling");
			gdb_target_desc_supported = 0;
		}

		/* support may be disabled globally */
		if (gdb_use_target_description == 0) {
			if (gdb_target_desc_supported)
				LOG_WARNING("Target Descriptions Supported, but disabled");
			gdb_target_desc_supported = 0;
		}

		xml_printf(&retval,
			&buffer,
			&pos,
			&size,
			"PacketSize=%x;qXfer:memory-map:read%c;qXfer:features:read%c;qXfer:threads:read+;QStartNoAckMode+;vContSupported+",
			GDB_BUFFER_SIZE,
			((gdb_use_memory_map == 1) && (flash_get_bank_count() > 0)) ? '+' : '-',
			(gdb_target_desc_supported == 1) ? '+' : '-');

		if (retval != ERROR_OK) {
			gdb_send_error(connection, 01);
			return ERROR_OK;
		}

		gdb_put_packet(connection, buffer, strlen(buffer));
		free(buffer);

		return ERROR_OK;
	/* 读取内存映射：如果存在flash bank，则调用 gdb_memory_map 函数读取内存映射信息
	 */
	} else if ((strncmp(packet, "qXfer:memory-map:read::", 23) == 0)
		   && (flash_get_bank_count() > 0))
		return gdb_memory_map(connection, packet, packet_size);
	else if (strncmp(packet, "qXfer:features:read:", 20) == 0) {
	/* 读取目标描述：解析命令中的偏移量和长度，调用 gdb_get_target_description_chunk 获取目标描述的一部分，并发送给GDB客户端
	 */
		char *xml = NULL;
		int retval = ERROR_OK;

		int offset;
		unsigned int length;

		/* skip command character */
		packet += 20;

		if (decode_xfer_read(packet, NULL, &offset, &length) < 0) {
			gdb_send_error(connection, 01);
			return ERROR_OK;
		}

		/* Target should prepare correct target description for annex.
		 * The first character of returned xml is 'm' or 'l'. 'm' for
		 * there are *more* chunks to transfer. 'l' for it is the *last*
		 * chunk of target description.
		 */
		retval = gdb_get_target_description_chunk(target, &gdb_connection->target_desc,
				&xml, offset, length);
		if (retval != ERROR_OK) {
			gdb_error(connection, retval);
			return retval;
		}

		gdb_put_packet(connection, xml, strlen(xml));

		free(xml);
		return ERROR_OK;
	} else if (strncmp(packet, "qXfer:threads:read:", 19) == 0) {
		/* 读取线程列表：解析命令中的偏移量和长度，调用 gdb_get_thread_list_chunk 获取线程列表的一部分，并发送给GDB客户端
		 * xml：用于存储生成的 XML 格式的线程列表
		 * 实现流程：
		 * 		跳过命令前缀：将 packet 指针向前移动 19 个字符，跳过命令前缀，以便后续处理偏移量和长度参数
		 *		解析偏移量和长度：调用 decode_xfer_read 函数解析命令中的偏移量（offset）和长度（length）
		 *		获取线程列表分块：调用 gdb_get_thread_list_chunk 函数获取线程列表的一部分。
		 *				该函数会根据指定的 offset 和 length 从 target 的线程列表中提取相应的数据，并将其转换为 XML 格式，存储在 xml 中
		 *				gdb_connection->thread_list 是一个缓存，用于存储整个线程列表
		 *		处理错误：如果 gdb_get_thread_list_chunk 返回错误码，则调用 gdb_error 发送错误信息给 GDB 客户端，并返回错误码
		 *		发送响应：如果 gdb_get_thread_list_chunk 成功执行，则使用 gdb_put_packet 函数将生成的 XML 数据发送给 GDB 客户端
		 *		释放资源：	释放分配给 xml 的内存，避免内存泄漏	
		 */
		char *xml = NULL;
		int retval = ERROR_OK;

		int offset;
		unsigned int length;

		/* skip command character */
		packet += 19;

		if (decode_xfer_read(packet, NULL, &offset, &length) < 0) {
			gdb_send_error(connection, 01);
			return ERROR_OK;
		}

		/* Target should prepare correct thread list for annex.
		 * The first character of returned xml is 'm' or 'l'. 'm' for
		 * there are *more* chunks to transfer. 'l' for it is the *last*
		 * chunk of target description.
		 */
		retval = gdb_get_thread_list_chunk(target, &gdb_connection->thread_list,
						   &xml, offset, length);
		if (retval != ERROR_OK) {
			gdb_error(connection, retval);
			return retval;
		}

		gdb_put_packet(connection, xml, strlen(xml));

		free(xml);
		return ERROR_OK;
	} else if (strncmp(packet, "QStartNoAckMode", 15) == 0) {
	/* 启用无ACK模式：设置 noack_mode 标志为 1，并发送 OK 响应
	 */
		gdb_connection->noack_mode = 1;
		gdb_put_packet(connection, "OK", 2);
		return ERROR_OK;
	} else if (target->type->gdb_query_custom) {
	/* 处理自定义查询命令：如果目标设备支持自定义查询命令，则调用相应的处理函数
	 */
		char *buffer = NULL;
		int ret = target->type->gdb_query_custom(target, packet, &buffer);
		gdb_put_packet(connection, buffer, strlen(buffer));
		return ret;
	}

	//如果没有匹配的查询命令，默认发送空响应
	gdb_put_packet(connection, "", 0);
	return ERROR_OK;
}

//处理GDB发送的vCont命令，用于控制目标设备的执行流程，允许GDB请求目标设备继续执行、单步执行、暂停等操作
static bool gdb_handle_vcont_packet(struct connection *connection, const char *packet, int packet_size)
{	
	//获取GDB连接和目标设备
	struct gdb_connection *gdb_connection = connection->priv;
	struct target *target = get_target_from_connection(connection);
	const char *parse = packet;
	int retval;

	/* query for vCont supported */
	/* 处理vCont?查询:如果命令以?开头，表示GDB在查询支持的vCont操作
	 * 		检查目标设备是否支持单步执行（step）。如果支持，则返回支持的操作列表
	 *      发送响应给GDB，告知支持的操作
	 *			这里返回"vCont;c;C;s;S"，表示支持以下操作：
	 *				c：继续执行（不带信号）
	 *				C：继续执行并传递信号
	 *				s：单步执行（不带信号）
	 *				S：单步执行并传递信号
	 *		处理带分隔符的命令:如果命令以分号开头，表示命令中包含额外的参数或选项。为了简化解析，跳过分号，并减少packet_size
	 *      
	 */
	if (parse[0] == '?') {
		if (target->type->step) { 
			/* gdb doesn't accept c without C and s without S */
			gdb_put_packet(connection, "vCont;c;C;s;S", 13);
			return true;
		}
		return false;
	}

	if (parse[0] == ';') {
		++parse;
		--packet_size;
	}

	/* simple case, a continue packet */
	/* 处理GDB发送的vCont c命令，用于继续执行目标设备,即恢复目标设备的运行
	 *		设置标志和日志记录:设置全局变量gdb_running_type为'c'，表示当前正在处理continue命令
	 *		设置输出标志:设置GDB连接的输出标志为GDB_OUTPUT_ALL，表示允许输出所有调试信息
	 *		调用目标设备的继续执行函数:
	 *		处理目标设备未暂停的情况：如果目标设备在请求恢复时并未暂停，记录一条信息，告知用户目标设备的状态
	 *		尝试同步目标设备的内部状态：如果继续执行操作失败，调用target_poll函数尝试同步目标设备的内部状态
	 * 		更新前端状态：即使继续执行操作失败，仍然将前端状态设置为TARGET_RUNNING，以保持与GDB的预期状态一致
	 *		调用事件回调：调用事件回调函数，通知系统GDB已经开始执行。这可以触发其他模块或插件执行相应的操作
	 */
	if (parse[0] == 'c') {
		gdb_running_type = 'c';
		LOG_DEBUG("target %s continue", target_name(target));
		gdb_connection->output_flag = GDB_OUTPUT_ALL;
		retval = target_resume(target, 1, 0, 0, 0);
		if (retval == ERROR_TARGET_NOT_HALTED)
			LOG_INFO("target %s was not halted when resume was requested", target_name(target));

		/* poll target in an attempt to make its internal state consistent */
		if (retval != ERROR_OK) {
			retval = target_poll(target);
			if (retval != ERROR_OK)
				LOG_DEBUG("error polling target %s after failed resume", target_name(target));
		}

		/*
		 * We don't report errors to gdb here, move frontend_state to
		 * TARGET_RUNNING to stay in sync with gdb's expectation of the
		 * target state
		 */
		gdb_connection->frontend_state = TARGET_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_GDB_START);

		return true;
	}

	/* single-step or step-over-breakpoint */
	/* 处理GDB发送的vCont s命令，用于单步执行目标设备
	 * 		设置标志:设置全局变量gdb_running_type为's'，表示当前正在处理step命令
	 *		获取目标设备和线程ID:
	 *			ct：初始化一个指向目标设备的指针ct，初始值为target
	 *			parse++ 和 packet_size--：跳过命令中的s字符，并减少packet_size
	 *		解析线程ID:
	 *			parse[0] == ':'：如果命令中包含冒号（:），表示后面跟着线程ID
	 *			strtoll：将字符串转换为64位整数，提取线程ID。endp指向转换后剩余的字符串部分
	 *			packet_size -= endp - parse：更新packet_size，确保后续解析时不会误读已解析的部分
	 *			thread_id = 0：如果没有提供线程ID，则默认为0，表示操作所有线程
	 *		处理RTOS线程管理:
	 *			target->rtos：检查目标设备是否运行RTOS（实时操作系统）。
	 *			rtos_update_threads：更新RTOS的线程信息，确保线程状态是最新的。
	 *			gdb_target_for_threadid：根据提供的线程ID，获取对应的目标设备指针ct。
	 *			fake_step = true：如果要单步执行的线程不是当前的RTOS线程，则设置fake_step为true，表示需要模拟单步执行
	 *		处理解析后的其他参数:
	 *			';'：如果命令中包含分号（;），表示后面可能有其他参数
	 *			'c'：如果接下来的字符是'c'，表示请求仅单步执行当前核心（core），并保持其他核心暂停
	 *			':'：如果接下来的字符是冒号（:），表示后面跟着线程ID
	 *			strtoll：提取线程ID并进行比较，如果与之前解析的thread_id相同，则记录调试信息，表示请求仅单步执行当前核心
	 *			current_pc = 2：注释掉的代码，表示在某些目标架构（如aarch64）中，可以通过设置current_pc为2来实现仅单步执行当前核心的功能
	 */
	if (parse[0] == 's') {
		gdb_running_type = 's';
		bool fake_step = false;

		struct target *ct = target;
		int current_pc = 1;
		int64_t thread_id;
		parse++;
		packet_size--;
		if (parse[0] == ':') {
			char *endp;
			parse++;
			packet_size--;
			thread_id = strtoll(parse, &endp, 16);
			if (endp) {
				packet_size -= endp - parse;
				parse = endp;
			}
		} else {
			thread_id = 0;
		}

		if (target->rtos) {
			/* FIXME: why is this necessary? rtos state should be up-to-date here already! */
			rtos_update_threads(target);

			target->rtos->gdb_target_for_threadid(connection, thread_id, &ct);

			/*
			 * check if the thread to be stepped is the current rtos thread
			 * if not, we must fake the step
			 */
			if (target->rtos->current_thread != thread_id)
				fake_step = true;
		}

		if (parse[0] == ';') {
			++parse;
			--packet_size;

			if (parse[0] == 'c') {
				parse += 1;

				/* check if thread-id follows */
				if (parse[0] == ':') {
					int64_t tid;
					parse += 1;

					tid = strtoll(parse, NULL, 16);
					if (tid == thread_id) {
						/*
						 * Special case: only step a single thread (core),
						 * keep the other threads halted. Currently, only
						 * aarch64 target understands it. Other target types don't
						 * care (nobody checks the actual value of 'current')
						 * and it doesn't really matter. This deserves
						 * a symbolic constant and a formal interface documentation
						 * at a later time.
						 */
						LOG_DEBUG("request to step current core only");
						/* uncomment after checking that indeed other targets are safe */
						/*current_pc = 2;*/
					}
				}
			}
		}

		/* 记录调试信息
		 * 设置gdb_connection结构中的output_flag成员为GDB_OUTPUT_ALL，即接下来的操作需要向GDB发送所有的输出信息
		 * 调用target_call_event_callbacks函数，通知所有已注册的回调函数，当前发生了TARGET_EVENT_GDB_START事件，用于通知GDB会话已经开始
		 */
		LOG_DEBUG("target %s single-step thread %"PRIx64, target_name(ct), thread_id);
		gdb_connection->output_flag = GDB_OUTPUT_ALL;
		target_call_event_callbacks(ct, TARGET_EVENT_GDB_START);

		/*
		 * work around an annoying gdb behaviour: when the current thread
		 * is changed in gdb, it assumes that the target can follow and also
		 * make the thread current. This is an assumption that cannot hold
		 * for a real target running a multi-threading OS. We just fake
		 * the step to not trigger an internal error in gdb. See
		 * https://sourceware.org/bugzilla/show_bug.cgi?id=22925 for details
		 */
		 /* GDB有一个行为，当在GDB中切换当前线程时，它假设目标系统也能够同步地切换到相同的线程作为当前线程
		  * 然而，在一个多线程操作系统的真实目标上，这种假设并不总是成立
		  * 因此，代码提供了一种方法来绕过这个问题
		  * 如果fake_step变量为真，那么代码将执行以下操作：
		  * 	再次使用LOG_DEBUG宏记录一条调试信息，这次是为了记录伪造的单步步进操作。
		  * 	使用snprintf格式化一个字符串，模拟从目标接收到的信号响应。
		  *			这里的"T05"表示信号5（SIGTRAP），这是用来告诉GDB程序已经到达断点或完成了单步指令执行
		  *			thread:后面跟着的是线程ID的十六进制表示，用来告知GDB哪个线程触发了这个信号
		  *		通过gdb_put_packet函数将构造好的响应包发送给GDB
		  *		设置gdb_connection的output_flag为GDB_OUTPUT_NO，这可能意味着停止进一步的输出
		  */
		if (fake_step) {
			int sig_reply_len;
			char sig_reply[128];

			LOG_DEBUG("fake step thread %"PRIx64, thread_id);

			sig_reply_len = snprintf(sig_reply, sizeof(sig_reply),
									"T05thread:%016"PRIx64";", thread_id);

			gdb_put_packet(connection, sig_reply, sig_reply_len);
			gdb_connection->output_flag = GDB_OUTPUT_NO;

			return true;
		}

		/* support for gdb_sync command */
		/* GDB 同步 (gdb_sync) 处理:
		 *		检查 gdb_sync 标志：如果 gdb_connection->sync 为真，表示GDB发出了同步请求。
		 *		重置 sync 标志：将 gdb_connection->sync 设置为 false，以确保这个标志不会再次触发。
		 *		检查目标状态：
		 *			如果目标（ct）处于停止状态（TARGET_HALTED），则记录一条调试信息，表明忽略单步步进命令，并告知GDB现在会从目标获取寄存器状态。
		 *			调用 gdb_sig_halted 函数向GDB发送一个信号，通知它目标已经停止。
		 *			设置 gdb_connection->output_flag 为 GDB_OUTPUT_NO，阻止进一步的日志包转发。
		 *			如果目标不是停止状态，则设置前端状态为运行中（TARGET_RUNNING）。
		 *			返回 true：结束函数调用，表示成功处理了同步请求 	
		 */
		if (gdb_connection->sync) {
			gdb_connection->sync = false;
			if (ct->state == TARGET_HALTED) {
				LOG_DEBUG("stepi ignored. GDB will now fetch the register state "
								"from the target.");
				gdb_sig_halted(connection);
				gdb_connection->output_flag = GDB_OUTPUT_NO;
			} else
				gdb_connection->frontend_state = TARGET_RUNNING;
			return true;
		}

		/*  执行单步步进
		 *	错误处理:如果 target_step 返回 ERROR_TARGET_NOT_HALTED，说明尝试单步步进时目标没有处于停止状态
		 *  成功处理：
		 *		轮询目标：如果 target_step 成功（返回 ERROR_OK），则调用 target_poll 函数来查询目标的状态
		 *		日志记录：如果 target_poll 失败，记录一条调试信息
		 *		发送信号回复：调用 gdb_signal_reply 函数向GDB发送信号信息，告知其目标的状态变化
		 *		停止日志转发：设置 gdb_connection->output_flag 为 GDB_OUTPUT_NO，防止进一步的日志信息被转发给GDB
		 *		设置前端状态：如果 target_step 没有成功，设置前端状态为 TARGET_RUNNING，表示目标可能仍在运行
		 */
		retval = target_step(ct, current_pc, 0, 0);
		if (retval == ERROR_TARGET_NOT_HALTED)
			LOG_INFO("target %s was not halted when step was requested", target_name(ct));

		/* if step was successful send a reply back to gdb */
		if (retval == ERROR_OK) {
			retval = target_poll(ct);
			if (retval != ERROR_OK)
				LOG_DEBUG("error polling target %s after successful step", target_name(ct));
			/* send back signal information */
			gdb_signal_reply(ct, connection);
			/* stop forwarding log packets! */
			gdb_connection->output_flag = GDB_OUTPUT_NO;
		} else
			gdb_connection->frontend_state = TARGET_RUNNING;
		return true;
	}
	LOG_ERROR("Unknown vCont packet");
	return false;
}

static char *next_hex_encoded_field(const char **str, char sep)
{
	size_t hexlen;
	const char *hex = *str;
	if (hex[0] == '\0')
		return NULL;

	const char *end = strchr(hex, sep);
	if (!end)
		hexlen = strlen(hex);
	else
		hexlen = end - hex;
	*str = hex + hexlen + 1;

	if (hexlen % 2 != 0) {
		/* Malformed hex data */
		return NULL;
	}

	size_t count = hexlen / 2;
	char *decoded = malloc(count + 1);
	if (!decoded)
		return NULL;

	size_t converted = unhexify((void *)decoded, hex, count);
	if (converted != count) {
		free(decoded);
		return NULL;
	}

	decoded[count] = '\0';
	return decoded;
}

/* handle extended restart packet */
static void gdb_restart_inferior(struct connection *connection, const char *packet, int packet_size)
{
	struct gdb_connection *gdb_con = connection->priv;
	struct target *target = get_target_from_connection(connection);

	breakpoint_clear_target(target);
	watchpoint_clear_target(target);
	command_run_linef(connection->cmd_ctx, "ocd_gdb_restart %s",
			target_name(target));
	/* set connection as attached after reset */
	gdb_con->attached = true;
	/*  info rtos parts */
	gdb_thread_packet(connection, packet, packet_size);
}

static bool gdb_handle_vrun_packet(struct connection *connection, const char *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	const char *parse = packet;

	/* Skip "vRun" */
	parse += 4;

	if (parse[0] != ';')
		return false;
	parse++;

	/* Skip first field "filename"; don't know what to do with it. */
	free(next_hex_encoded_field(&parse, ';'));

	char *cmdline = next_hex_encoded_field(&parse, ';');
	while (cmdline) {
		char *arg = next_hex_encoded_field(&parse, ';');
		if (!arg)
			break;
		char *new_cmdline = alloc_printf("%s %s", cmdline, arg);
		free(cmdline);
		free(arg);
		cmdline = new_cmdline;
	}

	if (cmdline) {
		if (target->semihosting) {
			LOG_INFO("GDB set inferior command line to '%s'", cmdline);
			free(target->semihosting->cmdline);
			target->semihosting->cmdline = cmdline;
		} else {
			LOG_INFO("GDB set inferior command line to '%s' but semihosting is unavailable", cmdline);
			free(cmdline);
		}
	}

	gdb_restart_inferior(connection, packet, packet_size);
	gdb_put_packet(connection, "S00", 3);
	return true;
}

/* 处理GDB发送的以'v'开头的扩展命令包
 * 参数：connection：指向当前连接的结构体，包含了与GDB通信所需的所有信息。
 * packet：指向接收到的数据包内容
 * packet_size：数据包的大小
 */
static int gdb_v_packet(struct connection *connection,
		char const *packet, int packet_size)
{
	//获取私有数据和目标设备:从connection结构体中提取出私有数据，即与GDB连接相关的特定信息
	struct gdb_connection *gdb_connection = connection->priv;
	int result;

	struct target *target = get_target_from_connection(connection);

	/* 处理vCont命令:用于控制目标设备的执行，支持多种操作，如继续运行、单步执行、暂停等
	 *		使用strncmp检查数据包是否以"vCont"开头，
	 *		如果匹配，将指针packet向前移动5个字符，并减少packet_size，以便处理剩余的命令参数
	 *		调用gdb_handle_vcont_packet函数来具体处理vCont命令,如果处理成功，handled将为true；否则为false
	 *		如果handled为false，发送一个空响应给GDB，表示该命令未被处理
	 */
	if (strncmp(packet, "vCont", 5) == 0) {
		bool handled;

		packet += 5;
		packet_size -= 5;

		handled = gdb_handle_vcont_packet(connection, packet, packet_size);
		if (!handled)
			gdb_put_packet(connection, "", 0);

		return ERROR_OK;
	}

	/* 处理vRun命令：用于启动目标设备的执行，通常在调试会话开始时使用。可以指定启动时的行为，如加载初始程序、设置断点等
	 * 		使用strncmp检查数据包是否以"vRun"开头
	 *		如果匹配，直接调用gdb_handle_vrun_packet函数来处理vRun命令。如果处理成功，handled将为true；否则为false
	 *      如果handled为false，发送一个空响应给GDB，表示该命令未被处理
	 */
	if (strncmp(packet, "vRun", 4) == 0) {
		bool handled;

		handled = gdb_handle_vrun_packet(connection, packet, packet_size);
		if (!handled)
			gdb_put_packet(connection, "", 0);

		return ERROR_OK;
	}

	/* if flash programming disabled - send a empty reply */
	/* 检查flash编程是否启用:
	 * 如果gdb_flash_program为0，表示闪存编程被禁用
	 * 在这种情况下，直接发送一个空响应给GDB，并返回ERROR_OK，表示处理完成
	 */
	if (gdb_flash_program == 0) {
		gdb_put_packet(connection, "", 0);
		return ERROR_OK;
	}

	/* 处理vFlashErase命令:用于指定要擦除的闪存地址和长度
	 * 格式为vFlashErase:addr,length
	 * 		解析命令参数:
	 *			将指针parse指向命令参数部分（跳过"vFlashErase:"前缀）
	 *          检查命令是否完整，如果命令为空或格式不正确，记录错误并关闭连接
	 *			使用strtoul函数将地址和长度从字符串转换为无符号长整型
	 *			确保地址和长度之间有一个逗号分隔符，并且没有多余的字符	
	 *      设置脏标志:
	 *			调用flash_set_dirty()函数标记Flash为dirty状态，表示需要重新写入。可以防止在多次调用flash_write时出现问题
	 *			触发事件回调:触发TARGET_EVENT_GDB_FLASH_ERASE_START事件,允许目标设备执行任何必要的预擦除操作
	 *			执行擦除操作:调用flash_erase_address_range函数执行实际的擦除操作。参数包括目标设备、是否强制擦除（false表示非强制）、起始地址和长度
	 *			触发事件回调:触发TARGET_EVENT_GDB_FLASH_ERASE_END事件。这允许目标设备执行任何必要的擦除后的处理操作
	 *			处理擦除结果:如果擦除操作成功，发送"OK"响应给GDB;如果擦除操作失败，发送I/O错误响应给GDB，并记录详细的错误信息
	 *
	 */
	if (strncmp(packet, "vFlashErase:", 12) == 0) {
		unsigned long addr;
		unsigned long length;

		char const *parse = packet + 12;
		if (*parse == '\0') {
			LOG_ERROR("incomplete vFlashErase packet received, dropping connection");
			return ERROR_SERVER_REMOTE_CLOSED;
		}

		addr = strtoul(parse, (char **)&parse, 16);

		if (*(parse++) != ',' || *parse == '\0') {
			LOG_ERROR("incomplete vFlashErase packet received, dropping connection");
			return ERROR_SERVER_REMOTE_CLOSED;
		}

		length = strtoul(parse, (char **)&parse, 16);

		if (*parse != '\0') {
			LOG_ERROR("incomplete vFlashErase packet received, dropping connection");
			return ERROR_SERVER_REMOTE_CLOSED;
		}

		/* assume all sectors need erasing - stops any problems
		 * when flash_write is called multiple times */
		flash_set_dirty();

		/* perform any target specific operations before the erase */
		target_call_event_callbacks(target,
			TARGET_EVENT_GDB_FLASH_ERASE_START);

		/* vFlashErase:addr,length messages require region start and
		 * end to be "block" aligned ... if padding is ever needed,
		 * GDB will have become dangerously confused.
		 */
		result = flash_erase_address_range(target, false, addr,
			length);

		/* perform any target specific operations after the erase */
		target_call_event_callbacks(target,
			TARGET_EVENT_GDB_FLASH_ERASE_END);

		/* perform erase */
		if (result != ERROR_OK) {
			/* GDB doesn't evaluate the actual error number returned,
			 * treat a failed erase as an I/O error
			 */
			gdb_send_error(connection, EIO);
			LOG_ERROR("flash_erase returned %i", result);
		} else
			gdb_put_packet(connection, "OK", 2);

		return ERROR_OK;
	}

	/* 处理vFlashWrite命令：用于指定要写入闪存的数据，格式为vFlashWrite:addr:data
	 * 		解析命令参数:
	 *			将指针parse指向命令参数部分（跳过"vFlashWrite:"前缀）
	 *          检查命令是否完整，如果命令为空或格式不正确，记录错误并关闭连接
	 *			使用strtoul函数将地址从字符串转换为无符号长整型
	 *			确保地址和长度之间有一个冒号分隔符，并且没有多余的字符
	 *			计算实际数据的长度	
	 *			创建或更新内存映像：如果当前没有创建映像，则分配内存并调用image_open函数初始化一个新的映像
	 *               image_open函数通常用于打开或创建一个映像文件，这里使用空字符串和"build"模式来创建一个临时的内存映像
	 *			添加数据到映像：image_add_section函数将接收到的数据添加到内存映像的一个新节区中
	 *			 	 参数包括：gdb_connection->vflash_image指向内存映像的指针，(uint8_t const *)parse指向数据的指针
	 *			发送响应：发送"OK"响应给GDB，表示写入操作成功完成。2表示响应字符串的长度
	 *
	 */
	if (strncmp(packet, "vFlashWrite:", 12) == 0) {
		int retval;
		unsigned long addr;
		unsigned long length;
		char const *parse = packet + 12;

		if (*parse == '\0') {
			LOG_ERROR("incomplete vFlashErase packet received, dropping connection");
			return ERROR_SERVER_REMOTE_CLOSED;
		}
		addr = strtoul(parse, (char **)&parse, 16);
		if (*(parse++) != ':') {
			LOG_ERROR("incomplete vFlashErase packet received, dropping connection");
			return ERROR_SERVER_REMOTE_CLOSED;
		}
		length = packet_size - (parse - packet);

		/* create a new image if there isn't already one */
		if (!gdb_connection->vflash_image) {
			gdb_connection->vflash_image = malloc(sizeof(struct image));
			image_open(gdb_connection->vflash_image, "", "build");
		}

		/* create new section with content from packet buffer */
		retval = image_add_section(gdb_connection->vflash_image,
				addr, length, 0x0, (uint8_t const *)parse);
		if (retval != ERROR_OK)
			return retval;

		gdb_put_packet(connection, "OK", 2);

		return ERROR_OK;
	}

	/* 处理vFlashDone命令：用于指示GDB已完成数据传输，并请求将这些数据写入目标设备的Flash，
	 * 		初始化变量:written用于存储实际写入闪存的字节数
	 *		调用事件回调:调用目标设备的事件回调函数，通知系统即将开始闪存写入操作
	 *			TARGET_EVENT_GDB_FLASH_WRITE_START在写入开始前调用
	 *			TARGET_EVENT_GDB_FLASH_WRITE_END：在写入结束后调用
	 *		执行闪存写入:调用此函数将内存映像中的数据写入目标设备的闪存
	 *		处理写入结果:如果写入操作失败，根据具体的错误码采取不同的处理方式
	 *			ERROR_FLASH_DST_OUT_OF_BANK：如果错误是由于目标地址超出闪存范围，发送"E.memtype"错误响应
	 *			其他错误：发送通用的I/O错误响应
	 *		清理资源:关闭并释放内存映像资源、释放分配的内存、将指针置为NULL
	 */
	if (strncmp(packet, "vFlashDone", 10) == 0) {
		uint32_t written;

		/* process the flashing buffer. No need to erase as GDB
		 * always issues a vFlashErase first. */
		target_call_event_callbacks(target,
				TARGET_EVENT_GDB_FLASH_WRITE_START);
		result = flash_write(target, gdb_connection->vflash_image,
			&written, false);
		target_call_event_callbacks(target,
			TARGET_EVENT_GDB_FLASH_WRITE_END);
		if (result != ERROR_OK) {
			if (result == ERROR_FLASH_DST_OUT_OF_BANK)
				gdb_put_packet(connection, "E.memtype", 9);
			else
				gdb_send_error(connection, EIO);
		} else {
			LOG_DEBUG("wrote %u bytes from vFlash image to flash", (unsigned)written);
			gdb_put_packet(connection, "OK", 2);
		}

		image_close(gdb_connection->vflash_image);
		free(gdb_connection->vflash_image);
		gdb_connection->vflash_image = NULL;

		return ERROR_OK;
	}

	gdb_put_packet(connection, "", 0);
	return ERROR_OK;
}

static int gdb_detach(struct connection *connection)
{
	/*
	 * Only reply "OK" to GDB
	 * it will close the connection and this will trigger a call to
	 * gdb_connection_closed() that will in turn trigger the event
	 * TARGET_EVENT_GDB_DETACH
	 */
	return gdb_put_packet(connection, "OK", 2);
}

/* The format of 'F' response packet is
 * Fretcode,errno,Ctrl-C flag;call-specific attachment
 */
static int gdb_fileio_response_packet(struct connection *connection,
		char const *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	char *separator;
	char *parsing_point;
	int fileio_retcode = strtoul(packet + 1, &separator, 16);
	int fileio_errno = 0;
	bool fileio_ctrl_c = false;
	int retval;

	LOG_DEBUG("-");

	if (*separator == ',') {
		parsing_point = separator + 1;
		fileio_errno = strtoul(parsing_point, &separator, 16);
		if (*separator == ',') {
			if (*(separator + 1) == 'C') {
				/* TODO: process ctrl-c */
				fileio_ctrl_c = true;
			}
		}
	}

	LOG_DEBUG("File-I/O response, retcode: 0x%x, errno: 0x%x, ctrl-c: %s",
			fileio_retcode, fileio_errno, fileio_ctrl_c ? "true" : "false");

	retval = target_gdb_fileio_end(target, fileio_retcode, fileio_errno, fileio_ctrl_c);
	if (retval != ERROR_OK)
		return ERROR_FAIL;

	/* After File-I/O ends, keep continue or step */
	if (gdb_running_type == 'c')
		retval = target_resume(target, 1, 0x0, 0, 0);
	else if (gdb_running_type == 's')
		retval = target_step(target, 1, 0x0, 0);
	else
		retval = ERROR_FAIL;

	if (retval != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static void gdb_log_callback(void *priv, const char *file, unsigned line,
		const char *function, const char *string)
{
	struct connection *connection = priv;
	struct gdb_connection *gdb_con = connection->priv;

	if (gdb_con->output_flag == GDB_OUTPUT_NO)
		/* No out allowed */
		return;

	if (gdb_con->busy) {
		/* do not reply this using the O packet */
		return;
	}

	gdb_output_con(connection, string);
}

static void gdb_sig_halted(struct connection *connection)
{
	char sig_reply[4];
	snprintf(sig_reply, 4, "T%2.2x", 2);
	gdb_put_packet(connection, sig_reply, 3);
}

//负责解析和响应来自GDB的命令
static int gdb_input_inner(struct connection *connection)
{
	/* Do not allocate this on the stack */
	/* 定义了一个静态缓冲区gdb_packet_buffer，它的大小为GDB_BUFFER_SIZE加上一个额外的字节用于字符串的终止符('\0')
	 * 使用静态变量意味着这个缓冲区在整个程序的生命周期内都存在，不会随着函数调用结束而被销毁
	 */
	static char gdb_packet_buffer[GDB_BUFFER_SIZE + 1]; /* Extra byte for null-termination */

	/* target：指向当前连接对应的目标设备结构体。
	 * packet：指向gdb_packet_buffer，用于后续作为读取到的数据包内容的指针。
	 * packet_size：表示接收到的数据包的大小。
	 * retval：用于存储各个操作的结果，通常用来检查是否发生错误。
	 * gdb_con：从connection结构体的私有数据成员priv中获取，它包含了与GDB连接相关的具体信息。
	 * warn_use_ext：一个静态布尔值，用于跟踪是否已经发出过关于使用扩展协议的警告
	 */
	struct target *target;
	char const *packet = gdb_packet_buffer;
	int packet_size;
	int retval;
	struct gdb_connection *gdb_con = connection->priv;
	static bool warn_use_ext;

	//根据传入的connection参数获取对应的target对象
	target = get_target_from_connection(connection);

	/* drain input buffer. If one of the packets fail, then an error
	 * packet is replied, if applicable.
	 *
	 * This loop will terminate and the error code is returned.
	 *
	 * The calling fn will check if this error is something that
	 * can be recovered from, or if the connection must be closed.
	 *
	 * If the error is recoverable, this fn is called again to
	 * drain the rest of the buffer.
	 */
	 //循环不断尝试读取并处理GDB发送过来的数据包，直到满足某些停止条件(如没有更多数据或发生了不可恢复的错误)
	do {
		//packet_size被初始化为GDB_BUFFER_SIZE，这是缓冲区的最大容量
		//gdb_get_packet函数用于从连接中读取一个完整的GDB数据包，并将其存储在gdb_packet_buffer中
		//该函数还会更新packet_size以反映实际读取到的数据量
		packet_size = GDB_BUFFER_SIZE;
		retval = gdb_get_packet(connection, gdb_packet_buffer, &packet_size);
		if (retval != ERROR_OK)
			return retval;

		/* terminate with zero */
		//在数据包的末尾添加了一个空字符（\0）
		gdb_packet_buffer[packet_size] = '\0';

		//只有当接收到的数据包长度大于0时，才会继续处理
		//如果packet_size为0，意味着没有接收到有效数据，此时不会执行任何操作
		if (packet_size > 0) {
			
			//调用gdb_log_incoming_packet函数记录接收到的数据包内容
			gdb_log_incoming_packet(connection, gdb_packet_buffer);

			retval = ERROR_OK;
			/* 根据数据包的第一个字符（命令类型）来分发不同的处理逻辑
			 * 'T':线程是否存活
			 * 'H':设置当前线程('c'为step/continue，'g'为所有操作)
			 * 'q'或 'Q':查询命令,首先尝试通过gdb_thread_packet处理线程相关的查询，如果未被消耗(GDB_THREAD_PACKET_NOT_CONSUMED)，则调用gdb_query_packet处理其他类型的查询
			 * 'g'、'G'、'p'、'P':依次为获取所有寄存器的值、设置所有寄存器的值、获取单个寄存器的值、设置单个寄存器的值
			 * 'm'、'M':依次为读取内存、写入内存
			 * 'z'或'Z'：删除或插入断点/观察点
			 * '?'：查询最后一次导致目标停下的信号。由gdb_last_signal_packet处理。此外如果检测到用户使用的是非扩展协议，会发出一条警告，建议使用扩展协议
			 * 'c'或's'：分别表示继续运行和单步执行
			 * 'v':处理各种扩展命令，如文件I/O等。由gdb_v_packet处理
			 * 'D':处理GDB断开连接的请求。由gdb_detach处理
			 * 'X':写入二进制数据到内存。由gdb_write_memory_binary_packet处理
			 * 'k':处理GDB关闭连接的请求。如果启用了扩展协议，则只是标记为未附加；否则，直接返回ERROR_SERVER_REMOTE_CLOSED
			 * '!':启用扩展远程协议。由gdb_con->extended_protocol标志控制，并回应OK
			 * 'R':处理重启目标设备的请求。由gdb_restart_inferior处理
			 * 'F':用于处理文件I/O扩展
			 */
			switch (packet[0]) {
				case 'T':	/* Is thread alive? */
					gdb_thread_packet(connection, packet, packet_size);
					break;
				case 'H':	/* Set current thread ( 'c' for step and continue,
							 * 'g' for all other operations ) */
					gdb_thread_packet(connection, packet, packet_size);
					break;
				case 'q':
				case 'Q':
					retval = gdb_thread_packet(connection, packet, packet_size);
					if (retval == GDB_THREAD_PACKET_NOT_CONSUMED)
						retval = gdb_query_packet(connection, packet, packet_size);
					break;
				case 'g':
					retval = gdb_get_registers_packet(connection, packet, packet_size);
					break;
				case 'G':
					retval = gdb_set_registers_packet(connection, packet, packet_size);
					break;
				case 'p':
					retval = gdb_get_register_packet(connection, packet, packet_size);
					break;
				case 'P':
					retval = gdb_set_register_packet(connection, packet, packet_size);
					break;
				case 'm':
					retval = gdb_read_memory_packet(connection, packet, packet_size);
					break;
				case 'M':
					retval = gdb_write_memory_packet(connection, packet, packet_size);
					break;
				case 'z':
				case 'Z':
					retval = gdb_breakpoint_watchpoint_packet(connection, packet, packet_size);
					break;
				case '?':
					gdb_last_signal_packet(connection, packet, packet_size);
					/* '?' is sent after the eventual '!' */
					if (!warn_use_ext && !gdb_con->extended_protocol) {
						warn_use_ext = true;
						LOG_WARNING("Prefer GDB command \"target extended-remote :%s\" instead of \"target remote :%s\"",
									connection->service->port, connection->service->port);
					}
					break;
				case 'c':
				case 's':
				{
					//调用线程相关函数，处理与线程相关的命令，确保当前线程设置正确
					gdb_thread_packet(connection, packet, packet_size);
					//设置输出标志
					gdb_con->output_flag = GDB_OUTPUT_ALL;

					//检查内存写入错误:如果之前有内存写入失败的情况，会记录一个错误，并清除该错误状态,确保后续的操作不会受到之前的错误影响
					if (gdb_con->mem_write_error) {
						LOG_ERROR("Memory write failure!");

						/* now that we have reported the memory write error,
						 * we can clear the condition */
						gdb_con->mem_write_error = false;
					}

					//检查目标设备的当前状态，并根据状态决定如何响应'c'或's'命令
					bool nostep = false;
					bool already_running = false;
					//目标已经在运行:如果目标设备已经在运行，发出警告并设置already_running为true
					if (target->state == TARGET_RUNNING) {
						LOG_WARNING("WARNING! The target is already running. "
								"All changes GDB did to registers will be discarded! "
								"Waiting for target to halt.");
						already_running = true;
					} else if (target->state != TARGET_HALTED) {
					//目标既不在运行也不在暂停状态:如果目标设备既不在运行状态也不在暂停状态，发出警告并设置nostep为true
					//这种情况下，'c'或's'命令将被忽略，因为目标设备可能处于某种不稳定的中间状态
						LOG_WARNING("The target is not in the halted nor running stated, "
								"stepi/continue ignored.");
						nostep = true;
					} else if ((packet[0] == 's') && gdb_con->sync) {
					//单步执行时的同步问题:当用户在GDB中发出continue命令时，GDB可能会先发送一个stepi命令给OpenOCD，从而绕过了单步执行的同步检查。
					//为了避免这种情况，这里直接忽略stepi命令，并允许GDB从目标设备获取最新的寄存器状态
						/* Hmm..... when you issue a continue in GDB, then a "stepi" is
						 * sent by GDB first to OpenOCD, thus defeating the check to
						 * make only the single stepping have the sync feature...
						 */
						nostep = true;
						LOG_DEBUG("stepi ignored. GDB will now fetch the register state "
								"from the target.");
					}
					gdb_con->sync = false;

					//根据状态采取不同行动
					if (!already_running && nostep) {
						/* Either the target isn't in the halted state, then we can't
						 * step/continue. This might be early setup, etc.
						 *
						 * Or we want to allow GDB to pick up a fresh set of
						 * register values without modifying the target state.
						 *
						 */
						 /* 如果目标既不在运行也不在暂停状态：调用gdb_sig_halted函数，模拟目标设备已暂停的状态
						  * 这可能是为了允许GDB获取最新的寄存器状态，而不实际改变目标设备的状态
						  */
						gdb_sig_halted(connection);

						/* stop forwarding log packets! */
						gdb_con->output_flag = GDB_OUTPUT_NO;
					} else {
						/* We're running/stepping, in which case we can
						 * forward log output until the target is halted
						 */
						 /* 如果目标正在运行或可以继续/单步执行：
						  * 将前端状态设置为TARGET_RUNNING，表示目标设备正在运行
						  * 调用target_call_event_callbacks函数，触发与GDB启动相关的事件回调
						  * 如果目标之前不在运行状态，调用gdb_step_continue_packet函数来实际执行继续或单步操作。
						  * 如果这个操作失败，则调用gdb_frontend_halted函数，模拟目标设备已暂停的状态，以避免GDB等待一个永远不会到来的暂停条件
						  */
						gdb_con->frontend_state = TARGET_RUNNING;
						target_call_event_callbacks(target, TARGET_EVENT_GDB_START);

						if (!already_running) {
							/* Here we don't want packet processing to stop even if this fails,
							 * so we use a local variable instead of retval. */
							retval = gdb_step_continue_packet(connection, packet, packet_size);
							if (retval != ERROR_OK) {
								/* we'll never receive a halted
								 * condition... issue a false one..
								 */
								gdb_frontend_halted(target, connection);
							}
						}
					}
				}
				break;
				case 'v':
					retval = gdb_v_packet(connection, packet, packet_size);
					break;
				case 'D':
					retval = gdb_detach(connection);
					break;
				case 'X':
					retval = gdb_write_memory_binary_packet(connection, packet, packet_size);
					if (retval != ERROR_OK)
						return retval;
					break;
				case 'k':
					if (gdb_con->extended_protocol) {
						gdb_con->attached = false;
						break;
					}
					gdb_put_packet(connection, "OK", 2);
					return ERROR_SERVER_REMOTE_CLOSED;
				case '!':
					/* handle extended remote protocol */
					gdb_con->extended_protocol = true;
					gdb_put_packet(connection, "OK", 2);
					break;
				case 'R':
					/* handle extended restart packet */
					gdb_restart_inferior(connection, packet, packet_size);
					break;
				/* 已弃用
				 * 'j':主要用于多核系统(如Cortex-A)，允许GDB知道哪个核心正在被调试;gdb_read_smp_packet用于从目标设备读取当前活动的核心ID，并将该信息发送给GDB
			 	 * 'J':主要用于多核系统，允许GDB指定哪个核心应该在下一步操作中被激活,gdb_write_smp_packet用于设置在下一次恢复执行时应该使用的核心ID
				 */
				case 'j':
					/* DEPRECATED */
					/* packet supported only by smp target i.e cortex_a.c*/
					/* handle smp packet replying coreid played to gbd */
					gdb_read_smp_packet(connection, packet, packet_size);
					break;

				case 'J':
					/* DEPRECATED */
					/* packet supported only by smp target i.e cortex_a.c */
					/* handle smp packet setting coreid to be played at next
					 * resume to gdb */
					gdb_write_smp_packet(connection, packet, packet_size);
					break;

				case 'F':
					/* File-I/O extension */
					/* After gdb uses host-side syscall to complete target file
					 * I/O, gdb sends host-side syscall return value to target
					 * by 'F' packet.
					 * The format of 'F' response packet is
					 * Fretcode,errno,Ctrl-C flag;call-specific attachment
					 */
					 /* 当GDB在主机侧完成了一个系统调用（例如读写文件）后，它会通过'F'包将系统调用的返回值发送回目标设备,
					  * 格式：Fretcode,errno,Ctrl-C flag;call-specific attachment，
					  * 其中：retcode系统调用的返回值、errno错误码、Ctrl-C flag表示是否检测到了Ctrl-C信号
					  * call-specific attachment 是与特定系统调用相关的信息
					  * 处理逻辑：
					  * 	将前端状态设置为TARGET_RUNNING，表示目标设备正在运行
					  * 	设置输出标志为GDB_OUTPUT_ALL，允许所有日志输出
					  * 	调用gdb_fileio_response_packet函数来处理这个文件I/O响应包，通常涉及更新目标设备的状态和发送适当的响应给GDB
					  */
					gdb_con->frontend_state = TARGET_RUNNING;
					gdb_con->output_flag = GDB_OUTPUT_ALL;
					gdb_fileio_response_packet(connection, packet, packet_size);
					break;

				default:
					/* ignore unknown packets */
					LOG_DEBUG("ignoring 0x%2.2x packet", packet[0]);
					gdb_put_packet(connection, "", 0);
					break;
			}

			/* if a packet handler returned an error, exit input loop */
			if (retval != ERROR_OK)
				return retval;
		}

		if (gdb_con->ctrl_c) {
			if (target->state == TARGET_RUNNING) {
				struct target *t = target;
				if (target->rtos)
					target->rtos->gdb_target_for_threadid(connection, target->rtos->current_threadid, &t);
				retval = target_halt(t);
				if (retval == ERROR_OK)
					retval = target_poll(t);
				if (retval != ERROR_OK)
					target_call_event_callbacks(target, TARGET_EVENT_GDB_HALT);
				gdb_con->ctrl_c = false;
			} else {
				LOG_INFO("The target is not running when halt was requested, stopping GDB.");
				target_call_event_callbacks(target, TARGET_EVENT_GDB_HALT);
			}
		}

	} while (gdb_con->buf_cnt > 0);

	return ERROR_OK;
}

static int gdb_input(struct connection *connection)
{
	int retval = gdb_input_inner(connection);
	struct gdb_connection *gdb_con = connection->priv;
	if (retval == ERROR_SERVER_REMOTE_CLOSED)
		return retval;

	/* logging does not propagate the error, yet can set the gdb_con->closed flag */
	if (gdb_con->closed)
		return ERROR_SERVER_REMOTE_CLOSED;

	/* we'll recover from any other errors(e.g. temporary timeouts, etc.) */
	return ERROR_OK;
}

static void gdb_keep_client_alive(struct connection *connection)
{
	struct gdb_connection *gdb_con = connection->priv;

	if (gdb_con->busy) {
		/* do not send packets, retry asap */
		return;
	}

	switch (gdb_con->output_flag) {
	case GDB_OUTPUT_NO:
		/* no need for keep-alive */
		break;
	case GDB_OUTPUT_ALL:
		/* send an empty O packet */
		gdb_output_con(connection, "");
		break;
	default:
		break;
	}
}

static const struct service_driver gdb_service_driver = {
	.name = "gdb",
	.new_connection_during_keep_alive_handler = NULL,
	.new_connection_handler = gdb_new_connection,
	.input_handler = gdb_input,
	.connection_closed_handler = gdb_connection_closed,
	.keep_client_alive_handler = gdb_keep_client_alive,
};

static int gdb_target_start(struct target *target, const char *port)
{
	struct gdb_service *gdb_service;
	int ret;
	gdb_service = malloc(sizeof(struct gdb_service));

	if (!gdb_service)
		return -ENOMEM;

	LOG_INFO("starting gdb server for %s on %s", target_name(target), port);

	gdb_service->target = target;
	gdb_service->core[0] = -1;
	gdb_service->core[1] = -1;
	target->gdb_service = gdb_service;

	ret = add_service(&gdb_service_driver, port, target->gdb_max_connections, gdb_service);
	/* initialize all targets gdb service with the same pointer */
	{
		struct target_list *head;
		foreach_smp_target(head, target->smp_targets) {
			struct target *curr = head->target;
			if (curr != target)
				curr->gdb_service = gdb_service;
		}
	}
	return ret;
}

static int gdb_target_add_one(struct target *target)
{
	/*  one gdb instance per smp list */
	if ((target->smp) && (target->gdb_service))
		return ERROR_OK;

	/* skip targets that cannot handle a gdb connections (e.g. mem_ap) */
	if (!target_supports_gdb_connection(target)) {
		LOG_DEBUG("skip gdb server for target %s", target_name(target));
		return ERROR_OK;
	}

	if (target->gdb_port_override) {
		if (strcmp(target->gdb_port_override, "disabled") == 0) {
			LOG_INFO("gdb port disabled");
			return ERROR_OK;
		}
		return gdb_target_start(target, target->gdb_port_override);
	}

	if (strcmp(gdb_port, "disabled") == 0) {
		LOG_INFO("gdb port disabled");
		return ERROR_OK;
	}

	int retval = gdb_target_start(target, gdb_port_next);
	if (retval == ERROR_OK) {
		/* save the port number so can be queried with
		 * $target_name cget -gdb-port
		 */
		target->gdb_port_override = strdup(gdb_port_next);

		long portnumber;
		/* If we can parse the port number
		 * then we increment the port number for the next target.
		 */
		char *end;
		portnumber = strtol(gdb_port_next, &end, 0);
		if (!*end) {
			if (parse_long(gdb_port_next, &portnumber) == ERROR_OK) {
				free(gdb_port_next);
				if (portnumber) {
					gdb_port_next = alloc_printf("%ld", portnumber+1);
				} else {
					/* Don't increment if gdb_port is 0, since we're just
					 * trying to allocate an unused port. */
					gdb_port_next = strdup("0");
				}
			}
		}
	}
	return retval;
}

int gdb_target_add_all(struct target *target)
{
	if (!target) {
		LOG_WARNING("gdb services need one or more targets defined");
		return ERROR_OK;
	}

	while (target) {
		int retval = gdb_target_add_one(target);
		if (retval != ERROR_OK)
			return retval;

		target = target->next;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_gdb_sync_command)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (!current_gdb_connection) {
		command_print(CMD,
			"gdb_sync command can only be run from within gdb using \"monitor gdb_sync\"");
		return ERROR_FAIL;
	}

	current_gdb_connection->sync = true;

	return ERROR_OK;
}

/* daemon configuration command gdb_port */
COMMAND_HANDLER(handle_gdb_port_command)
{
	int retval = CALL_COMMAND_HANDLER(server_pipe_command, &gdb_port);
	if (retval == ERROR_OK) {
		free(gdb_port_next);
		gdb_port_next = strdup(gdb_port);
	}
	return retval;
}

COMMAND_HANDLER(handle_gdb_memory_map_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_ENABLE(CMD_ARGV[0], gdb_use_memory_map);
	return ERROR_OK;
}

COMMAND_HANDLER(handle_gdb_flash_program_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_ENABLE(CMD_ARGV[0], gdb_flash_program);
	return ERROR_OK;
}

COMMAND_HANDLER(handle_gdb_report_data_abort_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_ENABLE(CMD_ARGV[0], gdb_report_data_abort);
	return ERROR_OK;
}

COMMAND_HANDLER(handle_gdb_report_register_access_error)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_ENABLE(CMD_ARGV[0], gdb_report_register_access_error);
	return ERROR_OK;
}

/* gdb_breakpoint_override */
COMMAND_HANDLER(handle_gdb_breakpoint_override_command)
{
	if (CMD_ARGC == 0) {
		/* nothing */
	} else if (CMD_ARGC == 1) {
		gdb_breakpoint_override = 1;
		if (strcmp(CMD_ARGV[0], "hard") == 0)
			gdb_breakpoint_override_type = BKPT_HARD;
		else if (strcmp(CMD_ARGV[0], "soft") == 0)
			gdb_breakpoint_override_type = BKPT_SOFT;
		else if (strcmp(CMD_ARGV[0], "disable") == 0)
			gdb_breakpoint_override = 0;
	} else
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (gdb_breakpoint_override)
		LOG_USER("force %s breakpoints",
			(gdb_breakpoint_override_type == BKPT_HARD) ? "hard" : "soft");
	else
		LOG_USER("breakpoint type is not overridden");

	return ERROR_OK;
}

COMMAND_HANDLER(handle_gdb_target_description_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_ENABLE(CMD_ARGV[0], gdb_use_target_description);
	return ERROR_OK;
}

COMMAND_HANDLER(handle_gdb_save_tdesc_command)
{
	char *tdesc;
	uint32_t tdesc_length;
	struct target *target = get_current_target(CMD_CTX);

	int retval = gdb_generate_target_description(target, &tdesc);
	if (retval != ERROR_OK) {
		LOG_ERROR("Unable to Generate Target Description");
		return ERROR_FAIL;
	}

	tdesc_length = strlen(tdesc);

	struct fileio *fileio;
	size_t size_written;

	char *tdesc_filename = alloc_printf("%s.xml", target_type_name(target));
	if (!tdesc_filename) {
		retval = ERROR_FAIL;
		goto out;
	}

	retval = fileio_open(&fileio, tdesc_filename, FILEIO_WRITE, FILEIO_TEXT);

	if (retval != ERROR_OK) {
		LOG_ERROR("Can't open %s for writing", tdesc_filename);
		goto out;
	}

	retval = fileio_write(fileio, tdesc_length, tdesc, &size_written);

	fileio_close(fileio);

	if (retval != ERROR_OK)
		LOG_ERROR("Error while writing the tdesc file");

out:
	free(tdesc_filename);
	free(tdesc);

	return retval;
}

static const struct command_registration gdb_command_handlers[] = {
	{
		.name = "gdb_sync",
		.handler = handle_gdb_sync_command,
		.mode = COMMAND_ANY,
		.help = "next stepi will return immediately allowing "
			"GDB to fetch register state without affecting "
			"target state",
		.usage = ""
	},
	{
		.name = "gdb_port",
		.handler = handle_gdb_port_command,
		.mode = COMMAND_CONFIG,
		.help = "Normally gdb listens to a TCP/IP port. Each subsequent GDB "
			"server listens for the next port number after the "
			"base port number specified. "
			"No arguments reports GDB port. \"pipe\" means listen to stdin "
			"output to stdout, an integer is base port number, \"disabled\" disables "
			"port. Any other string is are interpreted as named pipe to listen to. "
			"Output pipe is the same name as input pipe, but with 'o' appended.",
		.usage = "[port_num]",
	},
	{
		.name = "gdb_memory_map",
		.handler = handle_gdb_memory_map_command,
		.mode = COMMAND_CONFIG,
		.help = "enable or disable memory map",
		.usage = "('enable'|'disable')"
	},
	{
		.name = "gdb_flash_program",
		.handler = handle_gdb_flash_program_command,
		.mode = COMMAND_CONFIG,
		.help = "enable or disable flash program",
		.usage = "('enable'|'disable')"
	},
	{
		.name = "gdb_report_data_abort",
		.handler = handle_gdb_report_data_abort_command,
		.mode = COMMAND_CONFIG,
		.help = "enable or disable reporting data aborts",
		.usage = "('enable'|'disable')"
	},
	{
		.name = "gdb_report_register_access_error",
		.handler = handle_gdb_report_register_access_error,
		.mode = COMMAND_CONFIG,
		.help = "enable or disable reporting register access errors",
		.usage = "('enable'|'disable')"
	},
	{
		.name = "gdb_breakpoint_override",
		.handler = handle_gdb_breakpoint_override_command,
		.mode = COMMAND_ANY,
		.help = "Display or specify type of breakpoint "
			"to be used by gdb 'break' commands.",
		.usage = "('hard'|'soft'|'disable')"
	},
	{
		.name = "gdb_target_description",
		.handler = handle_gdb_target_description_command,
		.mode = COMMAND_CONFIG,
		.help = "enable or disable target description",
		.usage = "('enable'|'disable')"
	},
	{
		.name = "gdb_save_tdesc",
		.handler = handle_gdb_save_tdesc_command,
		.mode = COMMAND_EXEC,
		.help = "Save the target description file",
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

int gdb_register_commands(struct command_context *cmd_ctx)
{
	gdb_port = strdup("3333");
	gdb_port_next = strdup("3333");
	return register_commands(cmd_ctx, NULL, gdb_command_handlers);
}

void gdb_service_free(void)
{
	free(gdb_port);
	free(gdb_port_next);
}
