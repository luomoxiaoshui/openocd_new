/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_CORTEX_M_H
#define OPENOCD_TARGET_CORTEX_M_H

#include "armv7m.h"
#include "helper/bits.h"

#define CORTEX_M_COMMON_MAGIC 0x1A451A45U

#define SYSTEM_CONTROL_BASE 0x400FE000

#define ITM_TER0	0xE0000E00
#define ITM_TPR		0xE0000E40
#define ITM_TCR		0xE0000E80
#define ITM_TCR_ITMENA_BIT	BIT(0)
#define ITM_TCR_BUSY_BIT	BIT(23)
#define ITM_LAR		0xE0000FB0
#define ITM_LAR_KEY	0xC5ACCE55

#define CPUID		0xE000ED00

#define ARM_CPUID_PARTNO_POS    4
#define ARM_CPUID_PARTNO_MASK	(0xFFF << ARM_CPUID_PARTNO_POS)

//枚举类型cortex_m_partno，用来表示不同的cortex_m核心部件编号，
//这些编号通过读取ARM处理器的CPUID寄存器来识别具体的cortex_m核心型号
enum cortex_m_partno {
	CORTEX_M_PARTNO_INVALID,
	STAR_MC1_PARTNO    = 0x132,
	CORTEX_M0_PARTNO   = 0xC20,
	CORTEX_M1_PARTNO   = 0xC21,
	CORTEX_M3_PARTNO   = 0xC23,
	CORTEX_M4_PARTNO   = 0xC24,
	CORTEX_M7_PARTNO   = 0xC27,
	CORTEX_M0P_PARTNO  = 0xC60,  //M0+
	CORTEX_M23_PARTNO  = 0xD20,
	CORTEX_M33_PARTNO  = 0xD21,
	CORTEX_M35P_PARTNO = 0xD31,
	CORTEX_M55_PARTNO  = 0xD22,
};

/* Relevant Cortex-M flags, used in struct cortex_m_part_info.flags */
#define CORTEX_M_F_HAS_FPV4               BIT(0)
#define CORTEX_M_F_HAS_FPV5               BIT(1)
#define CORTEX_M_F_TAR_AUTOINCR_BLOCK_4K  BIT(2)

struct cortex_m_part_info {
	enum cortex_m_partno partno;
	const char *name;
	enum arm_arch arch;
	uint32_t flags;
};

/* Debug Control Block */
#define DCB_DHCSR	0xE000EDF0
#define DCB_DCRSR	0xE000EDF4
#define DCB_DCRDR	0xE000EDF8
#define DCB_DEMCR	0xE000EDFC
#define DCB_DSCSR	0xE000EE08

#define DAUTHSTATUS	0xE000EFB8
#define DAUTHSTATUS_SID_MASK	0x00000030

#define DCRSR_WNR	BIT(16)

#define DWT_CTRL	0xE0001000
#define DWT_CYCCNT	0xE0001004
#define DWT_PCSR	0xE000101C
#define DWT_COMP0	0xE0001020
#define DWT_MASK0	0xE0001024
#define DWT_FUNCTION0	0xE0001028
#define DWT_DEVARCH		0xE0001FBC

#define DWT_DEVARCH_ARMV8M	0x101A02

#define FP_CTRL		0xE0002000
#define FP_REMAP	0xE0002004
#define FP_COMP0	0xE0002008
#define FP_COMP1	0xE000200C
#define FP_COMP2	0xE0002010
#define FP_COMP3	0xE0002014
#define FP_COMP4	0xE0002018
#define FP_COMP5	0xE000201C
#define FP_COMP6	0xE0002020
#define FP_COMP7	0xE0002024

#define FPU_CPACR	0xE000ED88
#define FPU_FPCCR	0xE000EF34
#define FPU_FPCAR	0xE000EF38
#define FPU_FPDSCR	0xE000EF3C

#define TPIU_SSPSR	0xE0040000
#define TPIU_CSPSR	0xE0040004
#define TPIU_ACPR	0xE0040010
#define TPIU_SPPR	0xE00400F0
#define TPIU_FFSR	0xE0040300
#define TPIU_FFCR	0xE0040304
#define TPIU_FSCR	0xE0040308

/* Maximum SWO prescaler value. */
#define TPIU_ACPR_MAX_SWOSCALER	0x1fff

/* DCB_DHCSR bit and field definitions */
#define DBGKEY		(0xA05Ful << 16)
#define C_DEBUGEN	BIT(0)
#define C_HALT		BIT(1)
#define C_STEP		BIT(2)
#define C_MASKINTS	BIT(3)
#define S_REGRDY	BIT(16)
#define S_HALT		BIT(17)
#define S_SLEEP		BIT(18)
#define S_LOCKUP	BIT(19)
#define S_RETIRE_ST	BIT(24)
#define S_RESET_ST	BIT(25)

/* DCB_DEMCR bit and field definitions */
#define TRCENA			BIT(24)
#define VC_HARDERR		BIT(10)
#define VC_INTERR		BIT(9)
#define VC_BUSERR		BIT(8)
#define VC_STATERR		BIT(7)
#define VC_CHKERR		BIT(6)
#define VC_NOCPERR		BIT(5)
#define VC_MMERR		BIT(4)
#define VC_CORERESET	BIT(0)

/* DCB_DSCSR bit and field definitions */
#define DSCSR_CDS		BIT(16)

/* NVIC registers */
#define NVIC_ICTR		0xE000E004
#define NVIC_ISE0		0xE000E100
#define NVIC_ICSR		0xE000ED04
#define NVIC_AIRCR		0xE000ED0C
#define NVIC_SHCSR		0xE000ED24
#define NVIC_CFSR		0xE000ED28
#define NVIC_MMFSRB		0xE000ED28
#define NVIC_BFSRB		0xE000ED29
#define NVIC_USFSRH		0xE000ED2A
#define NVIC_HFSR		0xE000ED2C
#define NVIC_DFSR		0xE000ED30
#define NVIC_MMFAR		0xE000ED34
#define NVIC_BFAR		0xE000ED38
#define NVIC_SFSR		0xE000EDE4
#define NVIC_SFAR		0xE000EDE8

/* NVIC_AIRCR bits */
#define AIRCR_VECTKEY		(0x5FAul << 16)
#define AIRCR_SYSRESETREQ	BIT(2)
#define AIRCR_VECTCLRACTIVE	BIT(1)
#define AIRCR_VECTRESET		BIT(0)
/* NVIC_SHCSR bits */
#define SHCSR_BUSFAULTENA	BIT(17)
/* NVIC_DFSR bits */
#define DFSR_HALTED			1
#define DFSR_BKPT			2
#define DFSR_DWTTRAP		4
#define DFSR_VCATCH			8
#define DFSR_EXTERNAL		16

#define FPCR_CODE 0
#define FPCR_LITERAL 1
#define FPCR_REPLACE_REMAP  (0ul << 30)
#define FPCR_REPLACE_BKPT_LOW  (1ul << 30)
#define FPCR_REPLACE_BKPT_HIGH  (2ul << 30)
#define FPCR_REPLACE_BKPT_BOTH  (3ul << 30)

struct cortex_m_fp_comparator {
	bool used;
	int type;
	uint32_t fpcr_value;
	uint32_t fpcr_address;
};

struct cortex_m_dwt_comparator {
	bool used;
	uint32_t comp;
	uint32_t mask;
	uint32_t function;
	uint32_t dwt_comparator_address;
};

enum cortex_m_soft_reset_config {
	CORTEX_M_RESET_SYSRESETREQ,
	CORTEX_M_RESET_VECTRESET,
};

enum cortex_m_isrmasking_mode {
	CORTEX_M_ISRMASK_AUTO,
	CORTEX_M_ISRMASK_OFF,
	CORTEX_M_ISRMASK_ON,
	CORTEX_M_ISRMASK_STEPONLY,
};

//用于存储Cortex-M核心调试和状态信息
struct cortex_m_common {
	//用于标识结构体的特定值
	unsigned int common_magic;

	//嵌套了ARMv7-M通用结构体armv7m_common，表示该结构体的基础部分继承了ARMv7通用功能
	struct armv7m_common armv7m;

	/* Context information */
	//dcb_dhcsr：调试控制和状态寄存器，用于获取或设置Cortex-M的调试状态
	//dcb_dhcsr_cumulated_sticky：累积的sticky标志，用于记录特定的调试状态
	uint32_t dcb_dhcsr;
	uint32_t dcb_dhcsr_cumulated_sticky;
	/* DCB DHCSR has been at least once read, so the sticky bits have been reset */
	//dcb_dhcsr_sticky_is_recent：表示sticky位是否在最近读取后被清除
	bool dcb_dhcsr_sticky_is_recent;

	//nvic_dfsr：调试故障状态寄存器，显示调试停止的原因
	//nvic_icsr：中断控制和状态寄存器，显示当前激活或挂起的中断
	uint32_t nvic_dfsr;  /* Debug Fault Status Register - shows reason for debug halt */
	uint32_t nvic_icsr;  /* Interrupt Control State Register - shows active and pending IRQ */

	/* Flash Patch and Breakpoint (FPB) */
	/* fp_num_lit，fp_num_code：支持的字面量断点数量、代码断点数量
	 * fp_rev：FPB模块的版本号，表示Flash Patch和断点单元的修订版本
	 * fpb_enabled：FPB模块是否启用
	 * fp_comparator_list：FPB比较器列表，用于设置断点
	 */
	unsigned int fp_num_lit;
	unsigned int fp_num_code;
	int fp_rev;
	bool fpb_enabled;
	struct cortex_m_fp_comparator *fp_comparator_list;

	/* Data Watchpoint and Trace (DWT) */
	/* dwt_num_comp,dwt_comp_available：支持的数据监视器数量，可用的监视器数量
	 * dwt_devarch：DWT架构寄存器
	 * dwt_comparator_list：DWT比较器列表，用于设置数据监视点
	 * dwt_cache：与DWT相关的寄存器缓存
	 */
	unsigned int dwt_num_comp;
	unsigned int dwt_comp_available;
	uint32_t dwt_devarch;
	struct cortex_m_dwt_comparator *dwt_comparator_list;
	struct reg_cache *dwt_cache;

	//soft_reset_config：软复位配置
	enum cortex_m_soft_reset_config soft_reset_config;
	//vectreset_supported：是否支持矢量表重置
	bool vectreset_supported;
	//isrmasking_mode：中断屏蔽模式
	enum cortex_m_isrmasking_mode isrmasking_mode;

	//存储Cortex-M核心相关的元信息，如架构和功能
	const struct cortex_m_part_info *core_info;

	//slow_register_read标志某些寄存器的读取较慢，需要轮询已确保数据就绪
	bool slow_register_read;	/* A register has not been ready, poll S_REGRDY */

	//AP的选择器，用于标识当前调试的目标
	uint64_t apsel;

	/* Whether this target has the erratum that makes C_MASKINTS not apply to
	 * already pending interrupts */
	//如果目标设备存在硬件缺陷，此标志为true
	bool maskints_erratum;
};

static inline bool is_cortex_m_or_hla(const struct cortex_m_common *cortex_m)
{
	return cortex_m->common_magic == CORTEX_M_COMMON_MAGIC;
}

static inline bool is_cortex_m_with_dap_access(const struct cortex_m_common *cortex_m)
{
	if (!is_cortex_m_or_hla(cortex_m))
		return false;

	return !cortex_m->armv7m.is_hla_target;
}

/**
 * @returns the pointer to the target specific struct
 * without matching a magic number.
 * Use in target specific service routines, where the correct
 * type of arch_info is certain.
 */
 /* target_to_cm：从target中获取包含Cortex-M相关数据的cortex_m_common结构体
  * 使用container_of宏通过arch_info字段找到对应的结构
  */
static inline struct cortex_m_common *target_to_cm(struct target *target)
{
	return (target->arch_info,
			struct cortex_m_common, armv7m.arm);
}

/**
 * @returns the pointer to the target specific struct
 * or NULL if the magic number does not match.
 * Use in a flash driver or any place where mismatch of the arch_info
 * type can happen.
 */
 //将target对象转化为cortex_m_common类型的指针，同时确保目标是一个有效的cortex_m类型
static inline struct cortex_m_common *
target_to_cortex_m_safe(struct target *target)
{
	/* Check the parent types first to prevent peeking memory too far
	 * from arch_info pointer */
	if (!target_to_armv7m_safe(target))
		return NULL;

	struct cortex_m_common *cortex_m = target_to_cm(target);
	if (!is_cortex_m_or_hla(cortex_m))
		return NULL;

	return cortex_m;
}

/**
 * @returns cached value of Cortex-M part number
 * or CORTEX_M_PARTNO_INVALID if the magic number does not match
 * or core_info is not initialised.
 */
static inline enum cortex_m_partno cortex_m_get_partno_safe(struct target *target)
{
	struct cortex_m_common *cortex_m = target_to_cortex_m_safe(target);
	if (!cortex_m)
		return CORTEX_M_PARTNO_INVALID;

	if (!cortex_m->core_info)
		return CORTEX_M_PARTNO_INVALID;

	return cortex_m->core_info->partno;
}

int cortex_m_examine(struct target *target);
int cortex_m_set_breakpoint(struct target *target, struct breakpoint *breakpoint);
int cortex_m_unset_breakpoint(struct target *target, struct breakpoint *breakpoint);
int cortex_m_add_breakpoint(struct target *target, struct breakpoint *breakpoint);
int cortex_m_remove_breakpoint(struct target *target, struct breakpoint *breakpoint);
int cortex_m_add_watchpoint(struct target *target, struct watchpoint *watchpoint);
int cortex_m_remove_watchpoint(struct target *target, struct watchpoint *watchpoint);
void cortex_m_enable_breakpoints(struct target *target);
void cortex_m_enable_watchpoints(struct target *target);
void cortex_m_deinit_target(struct target *target);
int cortex_m_profiling(struct target *target, uint32_t *samples,
	uint32_t max_num_samples, uint32_t *num_samples, uint32_t seconds);

#endif /* OPENOCD_TARGET_CORTEX_M_H */
