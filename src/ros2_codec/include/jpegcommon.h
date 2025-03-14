#ifndef JPEGCOMMON_H
#define JPEGCOMMON_H

#include <memory.h>

#include "common.h"
#include <memory>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

#define ENABLE_GPU_JPEG_ENCODE_DEBUG (1)
#if ENABLE_GPU_JPEG_ENCODE_DEBUG
#include <iostream>
#endif // ENABLE_GPU_JPEG_ENCODE_DEBUG

namespace robosense {
namespace jpeg {

enum class RS_GPU_JPEG_ENCODE_SUPPORT_TYPE {
  RS_GPU_JPEG_SUPPORT_YUV420P = 0,
  RS_GPU_JPEG_SUPPORT_NV12,
  RS_GPU_JPEG_SUPPORT_RGB,
  RS_GPU_JPEG_SUPPORT_YUYV422,
};

enum class RS_GPU_JPEG_ENCODE_SAMPLE_TYPE {
  RS_GPU_JPEG_ENCODE_SAMPLE_FULL = 0,
  RS_GPU_JPEG_ENCODE_SAMPLE_HALF,
  RS_GPU_JPEG_ENCODE_SAMPLE_ONE_THIRD,
};

enum class JPEG_CODER_TYPE {
  RS_JPEG_CODER_ENCODE = 0, // 编码
  RS_JPEG_CODER_DECODE,     // 解码
};

class JpegCodesConfig {
public:
  JPEG_CODER_TYPE coderType;
  robosense::common::ImageFrameFormat imageFrameFormat;
  int imageWidth;
  int imageHeight;
  int jpegQuality;
  int sampleFactor{1};
  int gpuDeviceId{0};

public:
  RS_GPU_JPEG_ENCODE_SAMPLE_TYPE encodeSampleTypHelper() {
    if (sampleFactor == 2) {
      return RS_GPU_JPEG_ENCODE_SAMPLE_TYPE::RS_GPU_JPEG_ENCODE_SAMPLE_HALF;
    } else if (sampleFactor == 3) {
      return RS_GPU_JPEG_ENCODE_SAMPLE_TYPE::
          RS_GPU_JPEG_ENCODE_SAMPLE_ONE_THIRD;
    } else {
      return RS_GPU_JPEG_ENCODE_SAMPLE_TYPE::RS_GPU_JPEG_ENCODE_SAMPLE_FULL;
    }
  }

  RS_GPU_JPEG_ENCODE_SUPPORT_TYPE codecColorSpaceTypeHelper() {
    switch (imageFrameFormat) {
    case robosense::common::FRAME_FORMAT_RGB24: {
      return RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_RGB;
    }
    case robosense::common::FRAME_FORMAT_NV12: {
      return RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_NV12;
    }
    case robosense::common::FRAME_FORMAT_YUYV422: {
      return RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_YUYV422;
    }
    case robosense::common::FRAME_FORMAT_YUV420P: {
      return RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::
          RS_GPU_JPEG_SUPPORT_YUV420P;
    }
    default: {
      return RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_NV12;
    }
    }
  }
};

} // namespace jpeg
} // namespace robosense

#endif // JPEGCOMMON_H
