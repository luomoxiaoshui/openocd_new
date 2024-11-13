// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include "openocd.h"
#include "helper/system.h"

/* This is the main entry for developer PC hosted OpenOCD.
 *
 * OpenOCD can also be used as a library that is linked with
 * another application(not mainstream yet, but possible), e.g.
 * w/as an embedded application.
 *
 * Those applications will have their own main() implementation
 * and use bits and pieces from openocd.c. */

int main(int argc, char *argv[])
{
	/* disable buffering otherwise piping to logs causes problems work */
	/* 调用setvbuf函数禁用stdout和stderr的缓冲区(即设置为无缓冲模式 _IONBF),
         * 目的是避免在将输出重定向到日志文件时出现延迟或其他问题，确保日志能及时输出 */
	setvbuf(stdout, NULL, _IONBF, 0);
	setvbuf(stderr, NULL, _IONBF, 0);

	return openocd_main(argc, argv);//实际执行openocd_main，并将返回值作为main的输出
}
