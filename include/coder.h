#ifndef _VOLUMETRIC_VIDEO_CODER_H_
#define _VOLUMETRIC_VIDEO_CODER_H_

#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <zstd.h>

namespace pco {
namespace coder {
    extern void MortonIndex2XY(const std::vector<uint8_t>&, std::vector<std::vector<uint8_t>>&);
    extern void MortonXY2Index(const std::vector<std::vector<uint8_t>>&, std::vector<uint8_t>&);
    extern void TurboJpegEncoder();
    extern void TurboJpegDecoder();
    extern int  ZstdEncoder();
    extern int  ZstdDecoder();
}  // namespace coder
}  // namespace pco
#endif