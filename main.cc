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

#include <signal.h>

#include "easymedia/buffer.h"
#include "easymedia/control.h"
#include "easymedia/flow.h"
#include "easymedia/key_string.h"
#include "easymedia/utils.h"

#include "npu-usb-msg.h"

static bool quit = false;

static void sigterm_handler(int sig) {
  fprintf(stderr, "signal %d\n", sig);
  quit = true;
}

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

private:
  std::vector<std::shared_ptr<easymedia::Flow>> n4;
  std::shared_ptr<easymedia::Flow> rga;
  std::shared_ptr<easymedia::Flow> drm;

  static const std::string drm_dev_path;
  static const int drm_fps = 25;
  static const int drm_w = 2560;
  static const int drm_h = 1440;
  static const std::string n4_data_type;
  static std::vector<std::string> n4_dev_paths;
  static std::vector<ImageRect> n4_src_rects;
  static std::vector<ImageRect> n4_dst_rects;
};

const std::string N4DataFlow::drm_dev_path("/dev/dri/card0");
const std::string N4DataFlow::n4_data_type(IMAGE_YUYV422);
std::vector<std::string> N4DataFlow::n4_dev_paths = {
    "/dev/video0", "/dev/video1", "/dev/video2", "/dev/video3"};
std::vector<ImageRect> N4DataFlow::n4_src_rects = {
    {0, 0, 1280, 720}, {0, 0, 1280, 720}, {0, 0, 1280, 720}, {0, 0, 1280, 720}};
std::vector<ImageRect> N4DataFlow::n4_dst_rects = {{0, 0, 1280, 720},
                                                   {1280, 0, 1280, 720},
                                                   {0, 720, 1280, 720},
                                                   {1280, 720, 1280, 720}};

N4DataFlow::N4DataFlow() {
  std::string v4l2_data_type;
  std::string rga_in_data_type;
  std::string rga_out_data_type;
  std::string drm_data_type;
  size_t w_factor = 1;
  if (n4_data_type == IMAGE_NV16) {
    v4l2_data_type = IMAGE_NV16;
    rga_in_data_type = IMAGE_NV16;
    rga_out_data_type = IMAGE_RGB888;
    drm_data_type = IMAGE_RGB888;
  } else if (n4_data_type == IMAGE_YUYV422) {
    v4l2_data_type = IMAGE_RGB332;
    rga_in_data_type = IMAGE_RGB565;
    rga_out_data_type = IMAGE_RGB565; // assert(src w/h == dst w/h);
    drm_data_type = IMAGE_RGB332;
    w_factor = 2;
  } else {
    abort();
  }
  std::string flow_name("source_stream");
  std::string stream_name("v4l2_capture_stream");
  for (int i = 0; i < n4_dev_paths.size(); i++) {
    std::string param;
    PARAM_STRING_APPEND(param, KEY_NAME, stream_name);
    param.append(" ");
    std::string v4l2_param;
    PARAM_STRING_APPEND_TO(v4l2_param, KEY_USE_LIBV4L2, 1);
    PARAM_STRING_APPEND(v4l2_param, KEY_DEVICE, n4_dev_paths[i]);
    // PARAM_STRING_APPEND(v4l2_param, KEY_SUB_DEVICE, sub_input_path);
    PARAM_STRING_APPEND(v4l2_param, KEY_V4L2_CAP_TYPE,
                        KEY_V4L2_C_TYPE(VIDEO_CAPTURE));
    PARAM_STRING_APPEND(v4l2_param, KEY_V4L2_MEM_TYPE,
                        KEY_V4L2_M_TYPE(MEMORY_MMAP));
    PARAM_STRING_APPEND_TO(v4l2_param, KEY_FRAMES, 4);
    PARAM_STRING_APPEND(v4l2_param, KEY_OUTPUTDATATYPE, v4l2_data_type);
    PARAM_STRING_APPEND_TO(v4l2_param, KEY_BUFFER_WIDTH,
                           (n4_src_rects[i].w * w_factor));
    PARAM_STRING_APPEND_TO(v4l2_param, KEY_BUFFER_HEIGHT, n4_src_rects[i].h);
    param.append(v4l2_param);
    auto n = easymedia::REFLECTOR(Flow)::Create<easymedia::Flow>(
        flow_name.c_str(), param.c_str());
    if (!n || errno) {
      fprintf(stderr, "Create flow %s failed\n", flow_name.c_str());
      return;
    }
    n4.push_back(n);
  }

  fprintf(stderr, "init n4 camera successfully\n");

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
    if (!rga || errno) {
      fprintf(stderr, "Create flow %s failed\n", flow_name.c_str());
      return;
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
    if (!drm || errno) {
      fprintf(stderr, "Create flow %s failed\n", flow_name.c_str());
      return;
    }
    if (drm_data_type == IMAGE_RGB332) {
      easymedia::DRMPropertyArg arg = {"WORK_MODE", 1};
      if (drm->Control(easymedia::S_CRTC_PROPERTY, &arg)) {
        errno = -EINVAL;
        return;
      }
      arg.name = "PDAF_TYPE";
      arg.value = 2;
      if (drm->Control(easymedia::S_CRTC_PROPERTY, &arg)) {
        errno = -EINVAL;
        return;
      }
      arg.name = "CSI-TX-PATH";
      arg.value = 0;
      if (drm->Control(easymedia::S_CONNECTOR_PROPERTY, &arg)) {
        errno = -EINVAL;
        return;
      }
    }
    fprintf(stderr, "init drm successfully\n");
    rga->AddDownFlow(drm, 0, 0);
  } while (0);

  for (int i = 0; i < n4.size(); i++)
    n4[i]->AddDownFlow(rga, 0, i);
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

int main() {
  OpenSensor();
  signal(SIGINT, sigterm_handler);

  // const char *tty_path = "/dev/ttyGS0";
  int i = 0;
  N4DataFlow *n4 = nullptr;
  while (!quit) {
    if (!n4) {
      n4 = new N4DataFlow();
      if (!n4 || errno) {
        if (n4) {
          delete n4;
          n4 = nullptr;
        }
        fprintf(stderr, "Fail to init n4 data flow\n");
        break;
      }
    }
    if (i++ > 10000)
      break;
    easymedia::msleep(16);
  }

  if (n4)
    delete n4;

  return 0;
}
