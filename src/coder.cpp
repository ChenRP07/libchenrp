/***
 * @Author: ChenRP07
 * @Date: 2022-05-19 15:33:02
 * @LastEditTime: 2022-05-19 17:02:21
 * @LastEditors: ChenRP07
 * @Description:
 */

#include "coder.h"
using namespace pco;

void coder::MortonIndex2XY(const std::vector<uint8_t>& __sequence, std::vector<std::vector<uint8_t>>& __image) {
    try {
        if (__sequence.size() % 3 != 0) {
            throw "Color sequence is not mathing RGB space.";
        }
    }
    catch (const char* error_message) {
        std::cerr << "Fatal error in function MortonIndex2XY() : " << error_message << std::endl;
        std::exit(1);
    }

    size_t __color_cnt = __sequence.size() / 3;
    size_t k           = std::ceil(std::log2(__color_cnt) / 2);

    size_t __max_x = 0;
    size_t __max_y = 0;

    __image.resize((size_t)std::pow(2, k), std::vector<uint8_t>((size_t)std::pow(2, k) * 3, 0x00));

    for (size_t i = 0; i < __color_cnt; i++) {
        size_t __temp_x = 0, __temp_y = 0;
        size_t temp = i;
        for (size_t t = 0; t < k; t++) {
            __temp_x <<= 1, __temp_y <<= 1;
            __temp_x |= (temp & 0x1);
            temp >>= 1;
            __temp_y |= (temp & 0x1);
            temp >>= 1;
        }

        size_t __pos_x = 0, __pos_y = 0;
        for (size_t t = 0; t < k; t++) {
            __pos_x <<= 1, __pos_y <<= 1;
            __pos_x |= (__temp_x & 0x1);
            __pos_y |= (__temp_y & 0x1);
            __temp_x >>= 1, __temp_y >>= 1;
        }

        __max_x = std::max(__max_x, __pos_x + 1);
        __max_y = std::max(__max_y, __pos_y + 1);

        __image[__pos_y][__pos_x * 3]     = __sequence[i * 3];
        __image[__pos_y][__pos_x * 3 + 1] = __sequence[i * 3 + 1];
        __image[__pos_y][__pos_x * 3 + 2] = __sequence[i * 3 + 2];
    }

    __image.resize(__max_y);
    for (size_t i = 0; i < __max_y; i++) {
        __image[i].resize(__max_x * 3);
    }
}

void coder::MortonXY2Index(const std::vector<std::vector<uint8_t>>& __image, std::vector<uint8_t>& __sequence) {
    try {
        for (auto& i : __image) {
            if (i.size() % 3 != 0) {
                throw "Color sequence is not mathing RGB space.";
                break;
            }
        }
    }
    catch (const char* error_message) {
        std::cerr << "Fatal error in function MortonIndex2XY() : " << error_message << std::endl;
        std::exit(1);
    }

    __sequence.resize(__image.size() * __image[0].size(), 0x00);
    size_t k = std::ceil(std::log2(std::max(__image.size(), __image[0].size())));

    for (size_t i = 0; i < __image.size(); i++) {
        for (size_t j = 0; j < __image[i].size() / 3; j++) {
            size_t __temp_index = 0, __temp_x = j, __temp_y = i;
            for (size_t t = 0; t < k; t++) {
                __temp_index <<= 1;
                __temp_index |= (__temp_x & 0x1);
                __temp_x >>= 1;
                __temp_index <<= 1;
                __temp_index |= (__temp_y & 0x1);
                __temp_y >>= 1;
            }

            size_t __index = 0;
            for (size_t t = 0; t < 2 * k; t++) {
                __index <<= 1;
                __index |= (__temp_index & 0x1);
                __temp_index >>= 1;
            }

            __sequence[__index * 3]     = __image[i][j * 3];
            __sequence[__index * 3 + 1] = __image[i][j * 3 + 1];
            __sequence[__index * 3 + 2] = __image[i][j * 3 + 2];
        }
    }
}
