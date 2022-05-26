/***
 * @Author: ChenRP07
 * @Date: 2022-05-22 16:59:30
 * @LastEditTime: 2022-05-25 21:13:54
 * @LastEditors: ChenRP07
 * @Description:
 */
#ifndef _VOLUMETRIC_VIDEO_CODER_H_
#define _VOLUMETRIC_VIDEO_CODER_H_

#include "turbojpeg.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <sys/types.h>
#include <zstd.h>

namespace pco {
namespace coder {
    extern size_t MortonIndex2XY(const std::vector<uint8_t>&, std::vector<std::vector<uint8_t>>&);
    extern void   MortonXY2Index(const std::vector<std::vector<uint8_t>>&, std::vector<uint8_t>&, const size_t);
    extern void   TurboJpegEncoder(const std::vector<std::vector<uint8_t>>&, const std::string&, const int);
    extern void   TurboJpegDecoder(std::vector<std::vector<uint8_t>>&, const std::string&);
    extern void   ZstdEncoder(const std::string&, FILE*, const int);
    extern void   ZstdDecoder(std::string&, FILE*);
}  // namespace coder
}  // namespace pco
#endif