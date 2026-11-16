/* Minimal autoconf.h for the host-side test harness (shadows the
   generated file - do not use for firmware builds). */
#ifndef AUTOCONF_H
#define AUTOCONF_H

#define CONFIG_BUFFER_COUNT        4
#define CONFIG_MAX_PARTITIONS      2
#define CONFIG_ERROR_BUFFER_SIZE   100
#define CONFIG_COMMAND_BUFFER_SIZE 120

/* the harness tests the disk image REL implementation */
#define CONFIG_D64_REL

/* Single-LED hardware simplifies the LED macros (host harness) */
#define SINGLE_LED

#endif
