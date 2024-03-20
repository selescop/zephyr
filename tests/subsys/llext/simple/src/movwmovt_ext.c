/*
 * Copyright (c) 2023 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * This very simple hello world C code can be used as a test case for building
 * probably the simplest loadable extension. It requires a single symbol be
 * linked, section relocation support, and the ability to export and call out to
 * a function.
 */

#include <stdint.h>
#include <zephyr/llext/symbol.h>
#include <zephyr/sys/printk.h>


static void test_func()
{
	printk("test_func\n");
}

void test_entry(void)
{
	test_func();
#if defined(CONFIG_ARM) && !defined(CONFIG_CPU_CORTEX_M0)
	printk("test movwmovt\n");
	__asm volatile ("movw r0, #:lower16:test_func");
	__asm volatile ("movt r0, #:upper16:test_func");
	__asm volatile ("blx r0");
#endif
}
LL_EXTENSION_SYMBOL(test_entry);
