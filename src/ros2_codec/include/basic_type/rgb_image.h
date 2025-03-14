/******************************************************************************
 * Copyright 2017 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#ifndef HYPER_VISION_COMMON_BASIC_TYPE_RGB_IMAGE_H_
#define HYPER_VISION_COMMON_BASIC_TYPE_RGB_IMAGE_H_

#include "basic_type/DataType.h"
#include "basic_type/header.h"

namespace robosense {
namespace common {

struct RgbImage {
  Header header;

  unsigned int width;
  unsigned int height;
  std::vector<unsigned char> data_vec;

  ::RgbImage ToOutRgbImage() {
    ::RgbImage out_rgb_image;
    unsigned int size = height * width * 3;
    if (size != data_vec.size()) {
      std::cout << "RgbImage: size is not equal the data size! size is " << size << ", data size is " << data_vec.size();
    }
    out_rgb_image.width = width;
    out_rgb_image.height = height;
    out_rgb_image.timeStamp = header.timestamp;
    out_rgb_image.data = data_vec.data();
    return out_rgb_image;
  }
};

}  // namespace common
}  // namespace robosense

#endif  // HYPER_VISION_COMMON_BASIC_TYPE_RGB_IMAGE_H_
