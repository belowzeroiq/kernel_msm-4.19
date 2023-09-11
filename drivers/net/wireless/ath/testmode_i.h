/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * Copyright (c) 2018-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* "API" level of the ath testmode interface. Bump it after every
 * incompatible interface change.
 */
#define ATH_TESTMODE_VERSION_MAJOR 1

/* Bump this after every _compatible_ interface change, for example
 * addition of a new command or an attribute.
 */
#define ATH_TESTMODE_VERSION_MINOR 0

#define ATH_TM_DATA_MAX_LEN		5000
#define ATH_FTM_EVENT_MAX_BUF_LENGTH 	2048

enum ath_tm_attr {
	__ATH_TM_ATTR_INVALID		= 0,
	ATH_TM_ATTR_CMD			= 1,
	ATH_TM_ATTR_DATA		= 2,
	ATH_TM_ATTR_WMI_CMDID		= 3,
	ATH_TM_ATTR_VERSION_MAJOR	= 4,
	ATH_TM_ATTR_VERSION_MINOR	= 5,
	ATH_TM_ATTR_WMI_OP_VERSION	= 6,

	/* keep last */
	__ATH_TM_ATTR_AFTER_LAST,
	ATH_TM_ATTR_MAX			= __ATH_TM_ATTR_AFTER_LAST - 1,
};

/* All ath testmode interface commands specified in
 * ATH_TM_ATTR_CMD
 */
enum ath_tm_cmd {
	/* Returns the supported ath testmode interface version in
	 * ATH_TM_ATTR_VERSION. Always guaranteed to work. User space
	 * uses this to verify it's using the correct version of the
	 * testmode interface
	 */
	ATH_TM_CMD_GET_VERSION = 0,

	/* Set ar state to test mode. */
	ATH_TM_CMD_TESTMODE_START = 1,

	/* Set ar state back into OFF state. */
	ATH_TM_CMD_TESTMODE_STOP = 2,

	/* The command used to transmit a WMI command to the firmware and
	 * the event to receive WMI events from the firmware. Without
	 * struct wmi_cmd_hdr header, only the WMI payload. Command id is
	 * provided with ATH_TM_ATTR_WMI_CMDID and payload in
	 * ATH_TM_ATTR_DATA.
	 */
	ATH_TM_CMD_WMI = 3,

	/* The command used to transmit a FTM WMI command to the firmware
	 * and the event to receive WMI events from the firmware. The data
	 * received only contain the payload. Need to add the tlv
	 * header and send the cmd to fw with commandid WMI_PDEV_UTF_CMDID.
	 */
	ATH_TM_CMD_WMI_FTM = 4,
};
