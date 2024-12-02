/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_BREAKPOINTS_H
#define OPENOCD_TARGET_BREAKPOINTS_H

#include <stdint.h>

#include "helper/types.h"

struct target;

enum breakpoint_type {
	BKPT_HARD,
	BKPT_SOFT,
};

enum watchpoint_rw {
	WPT_READ = 0, WPT_WRITE = 1, WPT_ACCESS = 2
};

struct breakpoint {
	target_addr_t address;  //断点地址
	uint32_t asid;   //地址空间标识符，用于区分不同的进程地址空间
	int length;  //断点覆盖的指令长度
	enum breakpoint_type type;  //断点类型
	bool is_set;  //标志断点是否已实际设置到目标设备
	unsigned int number;   //硬件断点编号
	uint8_t *orig_instr;   //保存断点覆盖区域的原始指令内容
	struct breakpoint *next;  //指向下一个断点的指针，用于链表的管理
	uint32_t unique_id;    //唯一标识符，用于区分不同的断点
	int linked_brp;   //链接到的断点寄存器编号，仅适用于硬件断点
};

struct watchpoint {
	target_addr_t address;
	uint32_t length;
	uint32_t mask;
	uint32_t value;
	enum watchpoint_rw rw;
	bool is_set;
	unsigned int number;
	struct watchpoint *next;
	int unique_id;
};

void breakpoint_clear_target(struct target *target);
int breakpoint_add(struct target *target,
		target_addr_t address, uint32_t length, enum breakpoint_type type);
int context_breakpoint_add(struct target *target,
		uint32_t asid, uint32_t length, enum breakpoint_type type);
int hybrid_breakpoint_add(struct target *target,
		target_addr_t address, uint32_t asid, uint32_t length, enum breakpoint_type type);
void breakpoint_remove(struct target *target, target_addr_t address);
void breakpoint_remove_all(struct target *target);

struct breakpoint *breakpoint_find(struct target *target, target_addr_t address);

static inline void breakpoint_hw_set(struct breakpoint *breakpoint, unsigned int hw_number)
{
	breakpoint->is_set = true;
	breakpoint->number = hw_number;
}

void watchpoint_clear_target(struct target *target);
int watchpoint_add(struct target *target,
		target_addr_t address, uint32_t length,
		enum watchpoint_rw rw, uint32_t value, uint32_t mask);
void watchpoint_remove(struct target *target, target_addr_t address);

/* report type and address of just hit watchpoint */
int watchpoint_hit(struct target *target, enum watchpoint_rw *rw,
		target_addr_t *address);

static inline void watchpoint_set(struct watchpoint *watchpoint, unsigned int number)
{
	watchpoint->is_set = true;
	watchpoint->number = number;
}

#endif /* OPENOCD_TARGET_BREAKPOINTS_H */
