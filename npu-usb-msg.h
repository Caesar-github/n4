
/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Rockchip CIF Driver
 *
 * Copyright (C) 2018 Rockchip Electronics Co., Ltd.
 */

#ifndef _RKCOPROC_MSG_H
#define _RKCOPROC_MSG_H

#ifdef __cplusplus
extern "C" {
#endif // #ifdef __cplusplus

#include <linux/types.h>

/* user definition msg type*/
enum { ID_MSG_INIT = 0x001 };

typedef struct {
  __u32 type;       /* msg identification */
  __u8 cam_id;      /* camera identification */
  void *msg_entity; /* msg entity */
} msg_s;

typedef struct {
  __u32 msg_size; /* unit 4 bytes */
  __u32 mipi_clk; /* bps */
  __u8 mipi_lane;
  __u8 cam_num;
} msg_init_s;

#define MSG(TYPE, ID, CMD, var)                                                \
  TYPE var;                                                                    \
  var.type = CMD;                                                              \
  var.cam_id = ID

#define BASE_MSG_SIZE(m) (sizeof((m).type) + sizeof((m).cam_id))

typedef msg_s msg_init_t;
typedef msg_s msg_s_fmt_t;
typedef msg_s msg_stream_t;

#ifdef __cplusplus
}
#endif // #ifdef __cplusplus

#endif
