/***
 * @Author: ChenRP07
 * @Date: 2022-05-19 15:33:02
 * @LastEditTime: 2022-05-25 21:33:05
 * @LastEditors: ChenRP07
 * @Description:
 */

#include "coder.h"
using namespace pco;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
/***
 * @description: map a 1-D RGB __sequence to a 2-D Morton ordered __image
 * @param {vector<uint8_t>} __sequence
 * @param {vector<vector<uint8_t>>} __image
 * @return {*}
 */
size_t coder::MortonIndex2XY(const std::vector<uint8_t>& __sequence, std::vector<std::vector<uint8_t>>& __image) {
    try {
        // __sequence size must be multiples of three
        if (__sequence.size() % 3 != 0) {
            throw "Color sequence is not mathing RGB space.";
        }
    }
    catch (const char* error_message) {
        std::cerr << "Fatal error in function MortonIndex2XY() : " << error_message << std::endl;
        std::exit(1);
    }

    // number of colors
    size_t __color_cnt = __sequence.size() / 3;
    // those colors will be map to a 2^k * 2^k image
    size_t k = std::ceil(std::log2(__color_cnt) / 2);
    // actual image size
    size_t __max_x = 0;
    size_t __max_y = 0;

    __image.resize((size_t)std::pow(2, k), std::vector<uint8_t>((size_t)std::pow(2, k) * 3, 0x00));

    // calculate each point's position
    for (size_t i = 0; i < __color_cnt; i++) {
        // k-bit number cross, e.g. 11: 0000 1011 -> x: 0001, y: 0011
        // __temp_x and __temp_y is reverse, e.g. 1000, 1100
        size_t __temp_x = 0, __temp_y = 0;
        size_t temp = i;
        for (size_t t = 0; t < k; t++) {
            __temp_x <<= 1, __temp_y <<= 1;
            __temp_x |= (temp & 0x1);
            temp >>= 1;
            __temp_y |= (temp & 0x1);
            temp >>= 1;
        }

        // reverse __temp_x and __temp_y
        size_t __pos_x = 0, __pos_y = 0;
        for (size_t t = 0; t < k; t++) {
            __pos_x <<= 1, __pos_y <<= 1;
            __pos_x |= (__temp_x & 0x1);
            __pos_y |= (__temp_y & 0x1);
            __temp_x >>= 1, __temp_y >>= 1;
        }

        // get the max position
        __max_x = std::max(__max_x, __pos_x + 1);
        __max_y = std::max(__max_y, __pos_y + 1);

        // order the RGB color
        __image[__pos_y][__pos_x * 3]     = __sequence[i * 3];
        __image[__pos_y][__pos_x * 3 + 1] = __sequence[i * 3 + 1];
        __image[__pos_y][__pos_x * 3 + 2] = __sequence[i * 3 + 2];
    }

    // delete the free row
    __image.resize(__max_y);
    for (size_t i = 0; i < __max_y; i++) {
        // delete the free column
        __image[i].resize(__max_x * 3);
    }

    return __color_cnt;
}

/***
 * @description: map a 2-D Morton ordered image to a 1-D RGB sequence
 * @param {vector<vector<uint8_t>>} __image
 * @param {vector<uint8_t>} __sequence
 * @return {*}
 */
void coder::MortonXY2Index(const std::vector<std::vector<uint8_t>>& __image, std::vector<uint8_t>& __sequence, const size_t __count) {
    // each row must be multiples of three
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

    // ceil size is 2^k
    __sequence.resize(__count * 3, 0x00);
    size_t k = std::ceil(std::log2(std::max(__image.size(), __image[0].size() / 3)));
    // for each point

    for (size_t i = 0; i < __count / 3; i++) {
        // k-bit number cross, e.g. 11: 0000 1011 -> x: 0001, y: 0011
        // __temp_x and __temp_y is reverse, e.g. 1000, 1100
        size_t __temp_x = 0, __temp_y = 0;
        size_t temp = i;
        for (size_t t = 0; t < k; t++) {
            __temp_x <<= 1, __temp_y <<= 1;
            __temp_x |= (temp & 0x1);
            temp >>= 1;
            __temp_y |= (temp & 0x1);
            temp >>= 1;
        }

        // reverse __temp_x and __temp_y
        size_t __pos_x = 0, __pos_y = 0;
        for (size_t t = 0; t < k; t++) {
            __pos_x <<= 1, __pos_y <<= 1;
            __pos_x |= (__temp_x & 0x1);
            __pos_y |= (__temp_y & 0x1);
            __temp_x >>= 1, __temp_y >>= 1;
        }

        // RGB
        __sequence[i * 3]     = __image[__pos_y][__pos_x * 3];
        __sequence[i * 3 + 1] = __image[__pos_y][__pos_x * 3 + 1];
        __sequence[i * 3 + 2] = __image[__pos_y][__pos_x * 3 + 2];
    }
}

/***
 * @description: use turbojpeg compress __image and write the compressed image
 * into __file_name, jpeg compress quality __quality
 * @param {vector<vector<uint8_t>>&} __image
 * @param {string&} __file_name
 * @param {int} __quality
 * @return {*}
 */
void coder::TurboJpegEncoder(const std::vector<std::vector<uint8_t>>& __image, const std::string& __file_name, const int __quality) {
    // image size
    size_t __image_width  = __image[0].size() / 3;
    size_t __image_height = __image.size();

    // each row must have same size
    try {
        for (auto& i : __image) {
            if (i.size() != __image_width * 3) {
                throw "Unmatching size of rows.";
            }
        }
    }
    catch (const char* error_message) {
        std::cerr << "Fatal error in function TurboJpegEncoder() : " << error_message << std::endl;
        std::exit(1);
    }

    // scan image into __buffer
    const int      kColorComponents = 3;
    unsigned char  __buffer[__image_width * __image_height * kColorComponents];
    unsigned char* __compressed_image = nullptr;

    size_t __index = 0;
    for (size_t i = 0; i < __image.size(); i++) {
        for (size_t j = 0; j < __image[i].size() / 3; j++) {
            __buffer[__index]     = __image[i][j * 3];
            __buffer[__index + 1] = __image[i][j * 3 + 1];
            __buffer[__index + 2] = __image[i][j * 3 + 2];
            __index += 3;
        }
    }

    long unsigned int __jpeg_size = 0;

    // init jpeg
    tjhandle __jpeg_compressor = tjInitCompress();

    // compressing, use YUV420
    tjCompress2(__jpeg_compressor, __buffer, __image_width, 0, __image_height, TJPF_RGB, &__compressed_image, &__jpeg_size, TJSAMP_420, __quality, TJFLAG_FASTDCT);

    // write to file
    FILE* __file_pointer = nullptr;
    try {
        __file_pointer = fopen(__file_name.c_str(), "w");
        if (__file_pointer == nullptr)
            throw strerror(errno);
    }
    // open failed
    catch (const char* error_message) {
        std::cerr << "File " << __file_name << " open failed : " << error_message << "." << std::endl;
        fclose(__file_pointer);
        std::exit(1);
    }
    std::cout << "Jpeg size : " << __jpeg_size << std::endl;
    // compressed size, data
    fwrite(&__jpeg_size, sizeof(long unsigned int), 1, __file_pointer);
    fwrite(__compressed_image, sizeof(unsigned char), __jpeg_size, __file_pointer);
    fclose(__file_pointer);

    // free memory
    tjDestroy(__jpeg_compressor);
    tjFree(__compressed_image);
}

/***
 * @description: read a jpeg image from __file_name, decompress it into __image
 * @param {vector<vector<uint8_t>>&} __image
 * @param {string&} __file_name
 * @return {*}
 */
void coder::TurboJpegDecoder(std::vector<std::vector<uint8_t>>& __image, const std::string& __file_name) {
    // read from file
    FILE* __file_pointer = nullptr;
    __file_pointer       = fopen(__file_name.c_str(), "r");
    try {
        __file_pointer = fopen(__file_name.c_str(), "r");
        if (__file_pointer == nullptr)
            throw strerror(errno);
    }
    // open failed
    catch (const char* error_message) {
        std::cerr << "File " << __file_name << " open failed : " << error_message << "." << std::endl;
        fclose(__file_pointer);
        std::exit(1);
    }

    // read compressed size
    long unsigned int __jpeg_size = 0;
    fread(&__jpeg_size, sizeof(long unsigned int), 1, __file_pointer);
    std::cout << "JPEG size : " << __jpeg_size << std::endl;

    // read data
    unsigned char __compressed_image[__jpeg_size];
    fread(__compressed_image, sizeof(unsigned char), __jpeg_size, __file_pointer);

    fclose(__file_pointer);

    // decompressing
    tjhandle __jpeg_decompressor = tjInitDecompress();
    // read header information, width and height
    int __image_width, __image_height, __jpeg_subsamp;
    tjDecompressHeader2(__jpeg_decompressor, __compressed_image, __jpeg_size, &__image_width, &__image_height, &__jpeg_subsamp);

    try {
        if (__image_width * __image_height == 0) {
            throw "Image size is 0.";
        }
    }
    catch (const char* error_message) {
        std::cerr << "Fatal error in function TurboJpegDecoder : " << error_message << std::endl;
        std::exit(1);
    }

    // __jpeg_decompressor
    const int     kColorComponents = 3;
    unsigned char __buffer[__image_width * __image_height * kColorComponents];
    tjDecompress2(__jpeg_decompressor, __compressed_image, __jpeg_size, __buffer, __image_width, 0, __image_height, TJPF_RGB, TJFLAG_FASTDCT);

    __image.resize(__image_height, std::vector<uint8_t>(__image_width * 3, 0x00));

    // scan to a image
    size_t __index = 0;
    for (size_t i = 0; i < __image_height; i++) {
        for (size_t j = 0; j < __image_width; j++) {
            __image[i][j * 3]     = __buffer[__index];
            __image[i][j * 3 + 1] = __buffer[__index + 1];
            __image[i][j * 3 + 2] = __buffer[__index + 2];
            __index += 3;
        }
    }
}

void coder::ZstdEncoder(const std::string& __source, FILE* __file, const int kCompressionLevel) {
    try {
        if (__source.empty()) {
            throw "No data in source string.";
        }

        if (kCompressionLevel < ZSTD_minCLevel() || kCompressionLevel > ZSTD_maxCLevel()) {
            throw("Wrong compression level, must be from " + std::to_string(ZSTD_minCLevel()) + " to " + std::to_string(ZSTD_maxCLevel()) + ".").c_str();
        }

        // compressed string in __destination
        std::string __destination;

        // malloc some space
        const size_t kBufferSize = ZSTD_compressBound(__source.size());
        __destination.resize(kBufferSize);

        // compression
        const size_t kCompressedSize = ZSTD_compress(const_cast<char*>(__destination.c_str()), kBufferSize, __source.c_str(), __source.size(), kCompressionLevel);

        // if error?
        const size_t __error_code = ZSTD_isError(kCompressedSize);

        if (__error_code != 0) {
            throw "Wrong compressed string size.";
        }
        // delete excess space
        __destination.resize(kCompressedSize);

        if (__file == nullptr) {
            throw "Null file pointer.";
        }

        // write compressed string size to file
        fwrite(&kCompressedSize, sizeof(size_t), 1, __file);
        // write compressed string to file
        fwrite(__destination.c_str(), sizeof(char), __destination.size(), __file);

        if (ferror(__file)) {
            throw "Cannot write compressed string to file.";
        }
    }
    catch (const char* error_message) {
        std::cerr << "Fatal error in Zstd compression : " << error_message << std::endl;
        std::exit(1);
    }
}

void coder::ZstdDecoder(std::string& __source, FILE* __file) {
    try {
        if (__file == nullptr) {
            throw "Null file pointer.";
        }

        // read compressed string size
        size_t kCompressedSize;
        fread(&kCompressedSize, sizeof(size_t), 1, __file);

        // read compressed string
        std::string __destination;
        __destination.resize(kCompressedSize);

        for (size_t i = 0; i < kCompressedSize; i++) {
            fread(&__destination[i], sizeof(char), 1, __file);
        }

        if (ferror(__file)) {
            throw "Cannot read compressed string from file.";
        }

        // malloc some space
        const size_t kBufferSize = ZSTD_getFrameContentSize(__destination.c_str(), __destination.size());
        if (kBufferSize == 0 || kBufferSize == ZSTD_CONTENTSIZE_UNKNOWN || kBufferSize == ZSTD_CONTENTSIZE_ERROR) {
            throw "Wrong buffer size.";
        }

        __source.resize(kBufferSize);

        // decompression
        const size_t kDecompressedSize = ZSTD_decompress(const_cast<char*>(__source.c_str()), kBufferSize, __destination.c_str(), __destination.size());

        // if error?
        const size_t __error_code = ZSTD_isError(kDecompressedSize);
        if (__error_code != 0) {
            throw "Wrong decompressed string size.";
        }

        // free excess space
        __source.resize(kDecompressedSize);
    }
    catch (const char* error_message) {
        std::cerr << "Fatal error in Zstd compression : " << error_message << std::endl;
        std::exit(1);
    }
}
#pragma GCC diagnostic pop