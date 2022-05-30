/***
 * @Author: ChenRP07
 * @Date: 2022-05-22 16:59:30
 * @LastEditTime: 2022-05-27 14:41:10
 * @LastEditors: ChenRP07
 * @Description: This namespace pco::coder contain Morton code 1D-2D converter, TurbojpegCoder and ZstdCoder.
 */
#ifndef _VOLUMETRIC_VIDEO_CODER_H_
#define _VOLUMETRIC_VIDEO_CODER_H_

#include "turbojpeg.h"
#include <Eigen/Dense>
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
    // convert a color sequence to a 2D Morton ordered image, return size of color sequence
    extern size_t MortonIndex2XY(const std::vector<uint8_t>&, std::vector<std::vector<uint8_t>>&);
    // convert a 2D Morton ordered image to a color sequence
    extern void MortonXY2Index(const std::vector<std::vector<uint8_t>>&, std::vector<uint8_t>&, const size_t);
    // save a 2D image to a *.jpg file, using a SIMD accelerated turbojpeg library
    extern void TurboJpegEncoder(const std::vector<std::vector<uint8_t>>&, const std::string&, const int);
    // read a 2D image from a *.jpg file, using a SIMD accelerated turbojpeg library
    extern void TurboJpegDecoder(std::vector<std::vector<uint8_t>>&, const std::string&);
    // compress a string using entropy coder ZStandard, write the Size + Data to file
    extern void ZstdEncoder(const std::string&, FILE*, const int);
    // read a compressed string from file and decompress it
    extern void ZstdDecoder(std::string&, FILE*);
}  // namespace coder
}  // namespace pco
#endif