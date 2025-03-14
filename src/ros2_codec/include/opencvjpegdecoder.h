#ifndef OPENCVJPEGDECODER_H
#define OPENCVJPEGDECODER_H

#include "jpegcommon.h"

namespace robosense {
namespace jpeg {

class OpencvJpegDecoder {
public:
  using Ptr = std::shared_ptr<OpencvJpegDecoder>;
  using ConstPtr = std::shared_ptr<const OpencvJpegDecoder>;

public:
  OpencvJpegDecoder();
  ~OpencvJpegDecoder();

public:
  int init(const int imageWidth, const int imageHeight,
           const RS_GPU_JPEG_ENCODE_SUPPORT_TYPE output_type =
               RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_RGB);

  int decode(unsigned char *jpegBuffer, int jpegBufferLen,
             unsigned char *rawBuffer, size_t &rawBufferLen);

private:
  int init();

private:
  bool is_initial_;
  int image_width_;
  int image_height_;
  std::vector<unsigned char> buffer_;
  RS_GPU_JPEG_ENCODE_SUPPORT_TYPE output_type_;
};

} // namespace jpeg
} // namespace robosense

#endif // OPENCVJPEGDECODER_H
