// SPDX-FileCopyrightText: (c) 2024 Shawn Silverman <shawn@pobox.com>
// SPDX-License-Identifier: AGPL-3.0-or-later

// qnethernet_opts.h defines configuration options for the QNEthernet library.
// Users can modify this file instead of defining project-wide macros.
// This file is part of the QNEthernet library.

#pragma once

// Enables the 'altcp_tls_adapter' functions for easier TLS library integration.
// It's set, by default here, to be enabled if MbedTLS is enabled.
#ifndef QNETHERNET_ALTCP_TLS_ADAPTER
#define QNETHERNET_ALTCP_TLS_ADAPTER LWIP_ALTCP_TLS_MBEDTLS
#endif

// Put the RX and TX buffers into RAM1. (Teensy 4)
#ifndef QNETHERNET_BUFFERS_IN_RAM1
#define QNETHERNET_BUFFERS_IN_RAM1 0
#endif

// Changes 'stdio' output to use expanded behaviour.
#ifndef QNETHERNET_CUSTOM_WRITE
#define QNETHERNET_CUSTOM_WRITE 0
#endif

// The default DHCP client timeout, in milliseconds.
#ifndef QNETHERNET_DEFAULT_DHCP_CLIENT_TIMEOUT
#define QNETHERNET_DEFAULT_DHCP_CLIENT_TIMEOUT 60000
#endif

// The default DNS lookup timeout, in milliseconds. It's set, by default here,
// to a value based on the DNS timer interval and maximum number of retries.
#ifndef QNETHERNET_DEFAULT_DNS_LOOKUP_TIMEOUT
#define QNETHERNET_DEFAULT_DNS_LOOKUP_TIMEOUT (((DNS_MAX_RETRIES) + 1)*(DNS_TMR_INTERVAL))
#endif

// The default MAC address if one isn't specified, or if a default one is needed.
// Local, non-multicast: Lower two bits of the top byte must be 0b10.
//
// This is specified as a list of comma-separated bytes.
#ifndef QNETHERNET_DEFAULT_MAC_ADDRESS
#define QNETHERNET_DEFAULT_MAC_ADDRESS 0x02, 0, 0, 0, 0, 0
#endif

// The default hostname, used if the LWIP_NETIF_HOSTNAME option is set. Note
// that the hostname can also be set programmatically; this value is just the
// initial default.
#ifndef QNETHERNET_DEFAULT_HOSTNAME
#define QNETHERNET_DEFAULT_HOSTNAME "qnethernet-lwip"
#endif

// Builds with the W5500 driver.
// #define QNETHERNET_DRIVER_W5500

// Enables default implementations of the altcp interface functions.
#ifndef QNETHERNET_ENABLE_ALTCP_DEFAULT_FUNCTIONS
#define QNETHERNET_ENABLE_ALTCP_DEFAULT_FUNCTIONS 0
#endif

// Enables promiscuous mode.
#ifndef QNETHERNET_ENABLE_PROMISCUOUS_MODE
#define QNETHERNET_ENABLE_PROMISCUOUS_MODE 0
#endif

// Enables raw frame loopback when the destination MAC address matches the local
// MAC address.
#ifndef QNETHERNET_ENABLE_RAW_FRAME_LOOPBACK
#define QNETHERNET_ENABLE_RAW_FRAME_LOOPBACK 0
#endif

// Enables raw frame support.
#ifndef QNETHERNET_ENABLE_RAW_FRAME_SUPPORT
#define QNETHERNET_ENABLE_RAW_FRAME_SUPPORT 0
#endif

// Follows every call to 'EthernetClient::write()` with a flush. This may reduce
// TCP efficency. This option is for use with hard-to-modify code or libraries
// that assume data will get sent immediately. The preferred approach is to call
// flush() in the code or library.
#ifndef QNETHERNET_FLUSH_AFTER_WRITE
#define QNETHERNET_FLUSH_AFTER_WRITE 0
#endif

// Put lwIP-declared memory into RAM1. (Teensy 4)
#ifndef QNETHERNET_LWIP_MEMORY_IN_RAM1
#define QNETHERNET_LWIP_MEMORY_IN_RAM1 0
#endif

// Use the Entropy library instead of internal functions. (Teensy 4)
#ifndef QNETHERNET_USE_ENTROPY_LIB
#define QNETHERNET_USE_ENTROPY_LIB 0
#endif
