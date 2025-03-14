#include "jpegcoder.h"

namespace robosense {
namespace jpeg {

JpegCoder::JpegCoder() {}
JpegCoder::~JpegCoder() {}

int JpegCoder::init(const JpegCodesConfig &jpegCodesConfig) {
  jpeg_codes_config_ = jpegCodesConfig;

  int ret = init();
  if (ret != 0) {
    return -1;
  }
  return 0;
}

int JpegCoder::encode(unsigned char *yuv420Buffer, int yuv420BufferLen,
                      unsigned char *jpegBuffer, size_t &jpegBufferLen) {
  if (opencv_jpeg_encoder_ptr_ != nullptr) {
    int ret = opencv_jpeg_encoder_ptr_->encode(yuv420Buffer, yuv420BufferLen,
                                               jpegBuffer, jpegBufferLen);
    if (ret != 0) {
      return -4;
    }
  }
  return 0;
}

int JpegCoder::decode(unsigned char *yuv420Buffer, int yuv420BufferLen,
                      unsigned char *jpegBuffer, size_t &jpegBufferLen) {
  if (opencv_jpeg_decoder_ptr_ != nullptr) {
    int ret = opencv_jpeg_decoder_ptr_->decode(yuv420Buffer, yuv420BufferLen,
                                               jpegBuffer, jpegBufferLen);
    if (ret != 0) {
      std::cout << "jpeg cpu decoder(disable gpu) failed: ret = " << ret
                << std::endl;
      return -3;
    }
  }
  return 0;
}

int JpegCoder::init() {
  int ret = 0;
  switch (jpeg_codes_config_.coderType) {
  case JPEG_CODER_TYPE::RS_JPEG_CODER_ENCODE: {
    ret = initEncoder();
    break;
  }
  case JPEG_CODER_TYPE::RS_JPEG_CODER_DECODE: {
    ret = initDecoder();
    break;
  }
  }

  return ret;
}

int JpegCoder::initEncoder() {
  try {
    opencv_jpeg_encoder_ptr_.reset(new OpencvJpegEncoder());
  } catch (...) {
    return -1;
  }
  int ret = opencv_jpeg_encoder_ptr_->init(
      jpeg_codes_config_.imageWidth, jpeg_codes_config_.imageHeight,
      jpeg_codes_config_.jpegQuality,
      jpeg_codes_config_.codecColorSpaceTypeHelper(),
      jpeg_codes_config_.encodeSampleTypHelper());

  if (ret != 0) {
    return -1;
  }
  return 0;
}

int JpegCoder::initDecoder() {
  try {
    opencv_jpeg_decoder_ptr_.reset(new OpencvJpegDecoder());
  } catch (...) {
    return -4;
  }

  int ret = opencv_jpeg_decoder_ptr_->init(
      jpeg_codes_config_.imageWidth, jpeg_codes_config_.imageHeight,
      jpeg_codes_config_.codecColorSpaceTypeHelper());
  if (ret != 0) {
    return -5;
  }
  return 0;
}

} // namespace jpeg
} // namespace robosense
