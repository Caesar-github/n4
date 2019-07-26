/*
 * Copyright (C) 2019 Hertz Wang 1989wanghang@163.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see http://www.gnu.org/licenses
 *
 * Any non-GPL usage of this software or parts of this software is strictly
 * forbidden.
 *
 */

#include <assert.h>
#include <signal.h>

extern "C" {
#include <linux/rk-npu-usb-msg.h>
}

#include "easymedia/buffer.h"
#include "easymedia/control.h"
#include "easymedia/flow.h"
#include "easymedia/key_string.h"
#include "easymedia/utils.h"

static bool quit = false;

#ifdef DEBUG
static void sigterm_handler(int sig) {
  fprintf(stderr, "signal %d\n", sig);
  quit = true;
}
#endif

static void OpenSensor() {
  const int w = 1280, h = 720;
  static char media_ctl_str[128] = {0};
  if (media_ctl_str[0] == 0) {
    snprintf(media_ctl_str, sizeof(media_ctl_str),
             "media-ctl -vvv -d /dev/media0 --set-v4l2 \'\"jaguar1 "
             "3-0030\":0[fmt:UYVY8_2X8/%dx%d]\'",
             w, h);
    system(media_ctl_str);
  }
}

class N4DataFlow {
public:
  N4DataFlow();
  ~N4DataFlow();
  int IoCtrl(__u8 cam_id, __u32 type, void *param);

private:
  std::vector<std::shared_ptr<easymedia::Flow>> n4;
  std::shared_ptr<easymedia::Flow> rga;
  std::shared_ptr<easymedia::Flow> drm;

  static const std::string drm_dev_path;
  static const int drm_fps = 25;
  int drm_w;
  int drm_h;
  static const std::string n4_data_type;
  std::vector<int> n4_dev_indexs;
  std::vector<std::string> n4_dev_paths;
  std::vector<ImageRect> n4_src_rects;
  std::vector<ImageRect> n4_dst_rects;

  bool InitCapture(int index, __u8 cam_id);
  int StreamOn(int index);
};

const std::string N4DataFlow::drm_dev_path("/dev/dri/card0");
const std::string N4DataFlow::n4_data_type(IMAGE_YUYV422);

N4DataFlow::N4DataFlow()
    : n4_dev_paths(
          {"/dev/video0", "/dev/video1", "/dev/video2", "/dev/video3"}),
      n4_src_rects({{0, 0, 1280, 720},
                    {0, 0, 1280, 720},
                    {0, 0, 1280, 720},
                    {0, 0, 1280, 720}}),
      n4_dst_rects({{0, 0, 1280, 720},
                    {1280, 0, 1280, 720},
                    {0, 720, 1280, 720},
                    {1280, 720, 1280, 720}}) {
  n4.resize(4);
  drm_w = 2560;
  drm_h = 1440;
  n4_dev_indexs.resize(4, -1);
}

int N4DataFlow::StreamOn(int index) {
  assert(index >= 0 && index < 4);
  std::string v4l2_data_type;
  std::string rga_in_data_type;
  std::string rga_out_data_type;
  std::string drm_data_type;
  size_t w_factor = 1;

  // must be yuyv
  v4l2_data_type = n4_data_type;
  rga_in_data_type = IMAGE_RGB565;
  rga_out_data_type = IMAGE_RGB565; // assert(src w/h == dst w/h);
  drm_data_type = IMAGE_RGB332;
  w_factor = 2;

  std::string flow_name("source_stream");
  std::string stream_name("v4l2_capture_stream");
  do {
    std::string param;
    PARAM_STRING_APPEND(param, KEY_NAME, stream_name);
    param.append(" ");
    std::string v4l2_param;
    PARAM_STRING_APPEND_TO(v4l2_param, KEY_USE_LIBV4L2, 1);
    PARAM_STRING_APPEND(v4l2_param, KEY_DEVICE, n4_dev_paths[index]);
    // PARAM_STRING_APPEND(v4l2_param, KEY_SUB_DEVICE, sub_input_path);
    PARAM_STRING_APPEND(v4l2_param, KEY_V4L2_CAP_TYPE,
                        KEY_V4L2_C_TYPE(VIDEO_CAPTURE));
    PARAM_STRING_APPEND(v4l2_param, KEY_V4L2_MEM_TYPE,
                        KEY_V4L2_M_TYPE(MEMORY_MMAP));
    PARAM_STRING_APPEND_TO(v4l2_param, KEY_FRAMES, 8);
    PARAM_STRING_APPEND(v4l2_param, KEY_OUTPUTDATATYPE, v4l2_data_type);
    PARAM_STRING_APPEND_TO(v4l2_param, KEY_BUFFER_WIDTH, n4_src_rects[index].w);
    // (n4_src_rects[index].w * w_factor));
    PARAM_STRING_APPEND_TO(v4l2_param, KEY_BUFFER_HEIGHT,
                           n4_src_rects[index].h);
    param.append(v4l2_param);
    auto n = easymedia::REFLECTOR(Flow)::Create<easymedia::Flow>(
        flow_name.c_str(), param.c_str());
    if (!n) {
      fprintf(stderr, "[camid=%d]Create flow %s failed\n", n4_dev_indexs[index],
              flow_name.c_str());
      return -1;
    }
    n4[index] = n;
  } while (0);
  fprintf(stderr, "init camera successfully, index=%d<%s>\n", index, n4_dev_paths[index].c_str());
  for (auto n : n4) {
    if (!n)
      return 0;
  }
  fprintf(stderr, "init n4 camera successfully\n");
  // the final camera, calculate the draw final dst rects
  ImageRect &dst_rect = n4_dst_rects[0];
  dst_rect.x = 0;
  dst_rect.y = 0;
  dst_rect.w = n4_src_rects[0].w;
  dst_rect.h = n4_src_rects[0].h;

  dst_rect = n4_dst_rects[1];
  dst_rect.x = n4_dst_rects[0].w;
  dst_rect.y = 0;
  dst_rect.w = n4_src_rects[1].w;
  dst_rect.h = n4_src_rects[1].h;

  dst_rect = n4_dst_rects[2];
  dst_rect.x = 0;
  dst_rect.y = std::max<int>(n4_dst_rects[0].h, n4_dst_rects[1].h);
  dst_rect.w = n4_src_rects[2].w;
  dst_rect.h = n4_src_rects[2].h;

  dst_rect = n4_dst_rects[3];
  dst_rect.x = n4_dst_rects[2].w;
  dst_rect.y = n4_dst_rects[2].y;
  dst_rect.w = n4_src_rects[3].w;
  dst_rect.h = n4_src_rects[3].h;

  drm_w = std::max<int>(n4_dst_rects[0].w + n4_dst_rects[1].w,
                        n4_dst_rects[2].w + n4_dst_rects[3].w);
  drm_h = std::max<int>(n4_dst_rects[0].h, n4_dst_rects[2].h) +
          std::max<int>(n4_dst_rects[1].h, n4_dst_rects[3].h);

  do {
    flow_name = "filter";
    std::string filter_name("rkrga");
    std::string param;
    PARAM_STRING_APPEND(param, KEY_NAME, filter_name);
    // PARAM_STRING_APPEND(param, KEK_THREAD_SYNC_MODEL, KEY_ASYNCCOMMON);
    PARAM_STRING_APPEND_TO(param, KEY_FPS, drm_fps);
    PixelFormat rga_out_pix_fmt = GetPixFmtByString(rga_out_data_type.c_str());
    ImageInfo out_img_info = {rga_out_pix_fmt, drm_w, drm_h, drm_w, drm_h};
    PARAM_STRING_APPEND(param, KEY_INPUTDATATYPE, rga_in_data_type);
    param.append(easymedia::to_param_string(out_img_info, false));
    param.append(" ");
    std::string rga_param;
    for (int i = 0; i < n4.size(); i++) {
      std::vector<ImageRect> v = {n4_src_rects[i], n4_dst_rects[i]};
      PARAM_STRING_APPEND(rga_param, KEY_BUFFER_RECT,
                          easymedia::TwoImageRectToString(v));
    }
    param.append(rga_param);
    rga = easymedia::REFLECTOR(Flow)::Create<easymedia::Flow>(flow_name.c_str(),
                                                              param.c_str());
    if (!rga) {
      fprintf(stderr, "Create flow %s failed\n", flow_name.c_str());
      return -1;
    }
  } while (0);

  fprintf(stderr, "init rga successfully\n");

  do {
    flow_name = "output_stream";
    stream_name = "drm_output_stream";
    std::string param;
    PARAM_STRING_APPEND(param, KEY_NAME, stream_name);
    // PARAM_STRING_APPEND(param, KEK_THREAD_SYNC_MODEL, KEY_SYNC);
    param.append(" ");
    std::string drm_param;
    PARAM_STRING_APPEND(drm_param, KEY_DEVICE, drm_dev_path);
    PARAM_STRING_APPEND(drm_param, KEY_OUTPUTDATATYPE, drm_data_type);
    PARAM_STRING_APPEND_TO(drm_param, KEY_BUFFER_WIDTH, (drm_w * w_factor));
    PARAM_STRING_APPEND_TO(drm_param, KEY_BUFFER_HEIGHT, drm_h);
    param.append(drm_param);
    drm = easymedia::REFLECTOR(Flow)::Create<easymedia::Flow>(flow_name.c_str(),
                                                              param.c_str());
    if (!drm) {
      fprintf(stderr, "Create flow %s failed\n", flow_name.c_str());
      return -1;
    }
    if (drm_data_type == IMAGE_RGB332) {
      easymedia::DRMPropertyArg arg = {"WORK_MODE", 1};
      if (drm->Control(easymedia::S_CRTC_PROPERTY, &arg)) {
        errno = -EINVAL;
        return -1;
      }
      arg.name = "PDAF_TYPE";
      arg.value = 2;
      if (drm->Control(easymedia::S_CRTC_PROPERTY, &arg)) {
        errno = -EINVAL;
        return -1;
      }
      arg.name = "CSI-TX-PATH";
      arg.value = 0;
      if (drm->Control(easymedia::S_CONNECTOR_PROPERTY, &arg)) {
        errno = -EINVAL;
        return -1;
      }
    }
    fprintf(stderr, "init drm successfully\n");
    rga->AddDownFlow(drm, 0, 0);
  } while (0);

  for (int i = 0; i < n4.size(); i++)
    n4[i]->AddDownFlow(rga, 0, i);

  return 0;
}

N4DataFlow::~N4DataFlow() {
  if (rga) {
    for (int i = 0; i < n4.size(); i++)
      n4[i]->RemoveDownFlow(rga);
    n4.clear();
    if (drm)
      rga->RemoveDownFlow(drm);
  }
  rga.reset();
  drm.reset();
}

bool N4DataFlow::InitCapture(int index, __u8 cam_id) {
  n4_dev_indexs[index] = cam_id;
  n4_dev_paths[index] =
      std::string("/dev/video").append(std::to_string(cam_id));
  return true;
}

int N4DataFlow::IoCtrl(__u8 cam_id, __u32 type, void *param) {
  int first_slot = -1;
  int found_slot = -1;
  int index = 0;
  fprintf(stderr, "IoCtrl cam_id=%d, type=0x%08x\n", (int)cam_id, (int)type);
  for (int i : n4_dev_indexs) {
    if (i == cam_id) {
      first_slot = -1;
      found_slot = index++;
      break;
    }
    if (i < 0 && first_slot < 0)
      first_slot = index;
    index++;
  }
  if (first_slot >= 0) {
    fprintf(stderr, "first_slot=%d\n", first_slot);
    if (!InitCapture(first_slot, cam_id))
      return -1;
    found_slot = first_slot;
  } else if (found_slot < 0) {
    // more than 4
    return -EINVAL;
  }
  switch (type) {
  case VIDIOC_S_FMT: {
    auto fmt = (struct v4l2_format *)param;
    if (fmt->fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV)
      return -EINVAL;
    ImageRect &rect = n4_src_rects[found_slot];
    rect.w = fmt->fmt.pix.width;
    rect.h = fmt->fmt.pix.height;
  } break;
  case VIDIOC_STREAMON: {
    fprintf(stderr, "found_slot=%d\n", found_slot);
    return StreamOn(found_slot);
  }
  default:
    fprintf(stderr, "TODO: %d\n", (int)type);
    break;
  }
  return 0;
}

#include <libserialport.h>

static struct cam_msg_s *ReadMsg(struct sp_port *port) {
  __u32 type = 0;
  size_t read_size = 0;
  // must read the type at least
  while (read_size < sizeof(__u32) && !quit) {
    __u8 *str_type = ((__u8 *)&type) + read_size;
    // timeout 10ms
    int rsize = sp_blocking_read(port, str_type, sizeof(__u32) - read_size, 10);
    if (rsize <= 0) {
      // fprintf(stderr, "something error happened when reading, %m\n");
      return nullptr;
    }
    fprintf(stderr, "rsize: %d; read_size: %d\n", rsize, read_size);
    read_size += rsize;
  }
  __u8 *tttt = (__u8 *)&type;
  fprintf(stderr, "type: 0x%08x; %02x %02x %02x %02x\n", type, tttt[0], tttt[1],
          tttt[2], tttt[3]);
  size_t msg_struct_size = sizeof(cam_msg_s) + _IOC_SIZE(type);
  struct cam_msg_s *ms = (struct cam_msg_s *)calloc(1, msg_struct_size);
  if (!ms) {
    errno = ENOMEM;
    return nullptr;
  }
  ms->type = type;
  // this must be fine, else means the send data or path is broken
  read_size = sp_blocking_read(port, ((__u8 *)ms) + sizeof(__u32),
                               msg_struct_size - sizeof(__u32), 50000);
  if (read_size != msg_struct_size - sizeof(__u32)) {
    __u8 *dump_str = (__u8 *)ms;
    fprintf(stderr, "error: read size [%d] != expect size [%d-%d]...\n",
            (int)read_size, (int)msg_struct_size, (int)sizeof(__u32));
    for (int i = 0; i < read_size; i++) {
      fprintf(stderr, "%02x ", dump_str[i]);
      if ((i + 1) % 8 == 0)
        fprintf(stderr, "\n");
    }
    fprintf(stderr, "\n");
    sp_drain(port);
    free(ms);
    return nullptr;
  }
  __u16 tab_newline;
  read_size = sp_blocking_read(port, &tab_newline, sizeof(__u16), 50000);
  if (read_size != sizeof(__u16))
    fprintf(stderr, "not end of \\r\\n?\n");
  return ms;
}

static void WriteMsg(struct sp_port *port, struct cam_msg_s *ms) {
  size_t size = sizeof(*ms) + _IOC_SIZE(ms->type);
  size_t write_size = 0;
  while (write_size < size && !quit) {
    int wsize = sp_nonblocking_write(port, ((__u8 *)ms) + write_size,
                                     size - write_size);
    if (wsize < 0) {
      if (errno == EAGAIN)
        continue;
      fprintf(stderr, "something error happened when writing, %m\n");
      return;
    }
    write_size += wsize;
  }
  static const __u8 tab_newline[2] = {'\r', '\n'};
  write_size = sp_blocking_write(port, tab_newline, sizeof(__u8) * 2, 50000);
  if (write_size != sizeof(__u8) * 2)
    fprintf(stderr, "Fail to write end \\r\\n\n");
}

static std::shared_ptr<N4DataFlow> CreateN4() {
  auto n4 = std::make_shared<N4DataFlow>();
  if (!n4) {
    fprintf(stderr, "Fail to create n4 data flow\n");
    return nullptr;
  }
  return n4;
}

int main() {
  // OpenSensor();
  // signal(SIGINT, sigterm_handler);

  const char *tty_path = "/dev/ttyGS0";
  int i = 0;
  std::shared_ptr<N4DataFlow> n4;

  bool port_opened = false;
  struct sp_port *port = nullptr;
  sp_return sr = sp_get_port_by_name(tty_path, &port);
  if (sr != SP_OK) {
    fprintf(stderr, "Fail to get port of %s\n", tty_path);
    goto error;
  }
  sr = sp_open(port, SP_MODE_READ_WRITE);
  if (sr != SP_OK) {
    fprintf(stderr, "Fail to open %s\n", tty_path);
    goto error;
  }
  port_opened = true;
  sr = sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE);
  if (sr != SP_OK) {
    fprintf(stderr, "Fail to set flowcontrol to %s\n", tty_path);
    goto error;
  }
  sr = sp_set_baudrate(port, 1500000);
  if (sr != SP_OK) {
    fprintf(stderr, "Fail to set baudrate to %s\n", tty_path);
    goto error;
  }
  sr = sp_set_bits(port, 8);
  if (sr != SP_OK) {
    fprintf(stderr, "Fail to set bits to %s\n", tty_path);
    goto error;
  }
  while (!quit) {
#if 0 // rk's acm tty do not support return waiting bytes
    int bytes_waiting = sp_input_waiting(port);
    if (bytes_waiting < sizeof(__u32)) {
      easymedia::msleep(5);
      continue;
    }
#endif
    auto ms = ReadMsg(port);
    if (!ms) {
      easymedia::msleep(5);
      continue;
    }
    // parse msg
    switch (ms->type) {
    case ID_MSG_INIT: {
      msg_init_s *msg = (msg_init_s *)ms->msg_entity;
      fprintf(stderr,
              "TODO, msg_size: %d, mipi_clk: %d, mipi_lane: %d, cam_num: %d\n",
              (int)msg->msg_size, (int)msg->mipi_clk, (int)msg->mipi_lane,
              (int)msg->cam_num);
    } break;
    case VIDIOC_S_FMT:
    case VIDIOC_STREAMON: {
      if (!n4)
        n4 = CreateN4();
      __s8 ret = n4 ? 0 : -1;
      if (n4)
        ret = n4->IoCtrl(ms->cam_id, ms->type, ms->msg_entity);
      ms->ret_value = ret;
      WriteMsg(port, ms);
    } break;
    case VIDIOC_STREAMOFF: {
      n4.reset();
      ms->ret_value = 0;
      WriteMsg(port, ms);
    } break;
    }
    free(ms);
  }

error:
  n4.reset();
  if (port) {
    if (port_opened)
      sp_close(port);
    sp_free_port(port);
  }

  return sr == SP_OK ? 0 : -1;
}
