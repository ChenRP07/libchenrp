/***
 * @Author: ChenRP07
 * @Date: 2022-05-16 16:41:28
 * @LastEditTime: 2022-05-31 16:31:43
 * @LastEditors: ChenRP07
 * @Description:
 */

#include "octree.h"
using namespace pco;

/***
 * @description: return if all elements in __group_of_frames are all empty
 * @param {vector<vector<size_t>>} __group_of_frames
 * @return {bool}
 */
bool operation::AreFramesEmpty(const std::vector<std::vector<size_t>>& __group_of_frames) {
	for (size_t i = 0; i < __group_of_frames.size(); i++) {
		if (!__group_of_frames[i].empty()) {
			return false;
		}
	}
	return true;
}

/***
 * @description: return a subnode center according to position and resolution
 * @param {PointXYZ} __old_center
 * @param {int} __position
 * @param {float} __resolution
 * @return {PointXYZ} __new_center
 */
pcl::PointXYZ operation::SubnodeCenter(const pcl::PointXYZ __old_center, const int __position, const float __resolution) {
	pcl::PointXYZ __new_center;
	/*
	    for a cube, eight subnodes are
	      6————4
	     /|   /|
	    2—+——0 |
	    | 7——+-5
	    |/   |/
	    3————1
	    xyz : 000 ? 111 (x/y/z > center ? 0 : 1)
	*/
	switch (__position) {
		case 0: __new_center = pcl::PointXYZ(__old_center.x + __resolution / 2, __old_center.y + __resolution / 2, __old_center.z + __resolution / 2); break;
		case 1: __new_center = pcl::PointXYZ(__old_center.x + __resolution / 2, __old_center.y + __resolution / 2, __old_center.z - __resolution / 2); break;
		case 2: __new_center = pcl::PointXYZ(__old_center.x + __resolution / 2, __old_center.y - __resolution / 2, __old_center.z + __resolution / 2); break;
		case 3: __new_center = pcl::PointXYZ(__old_center.x + __resolution / 2, __old_center.y - __resolution / 2, __old_center.z - __resolution / 2); break;
		case 4: __new_center = pcl::PointXYZ(__old_center.x - __resolution / 2, __old_center.y + __resolution / 2, __old_center.z + __resolution / 2); break;
		case 5: __new_center = pcl::PointXYZ(__old_center.x - __resolution / 2, __old_center.y + __resolution / 2, __old_center.z - __resolution / 2); break;
		case 6: __new_center = pcl::PointXYZ(__old_center.x - __resolution / 2, __old_center.y - __resolution / 2, __old_center.z + __resolution / 2); break;
		case 7: __new_center = pcl::PointXYZ(__old_center.x - __resolution / 2, __old_center.y - __resolution / 2, __old_center.z - __resolution / 2); break;
	}
	return __new_center;
}

/***
 * @description: convert a 8bit bool vector to uchar, from vector[0] to vector[7] is higher bit to lower bit
 * @param {vector<bool>} __vector
 * @return {uint8_t} __result
 */
uint8_t operation::BoolSetToUChar(const std::vector<bool>& __vector) {
	try {
		uint8_t __result = 0x00;
		if (__vector.size() == 8) {
			for (size_t i = 0; i < 8; i++) {
				__result <<= 1;
				__result |= __vector[i];
			}
			return __result;
		}
		else {
			throw "Size of bool set is not 8.";
		}
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error in function BoolSetToUChar() : " << error_message << std::endl;
		std::exit(1);
	}
}

/***
 * @description: count how many 1 in __node_value
 * @param {uint8_t} __node_value
 * @return {size_t} __count;
 */
size_t operation::NodePointCount(uint8_t __node_value) {
	size_t __count = 0;
	for (size_t i = 0; i < 8; i++) {
		__count += (__node_value & 0x01);
		__node_value >>= 1;
	}
	return __count;
}

/***
 * @description: add occupation bit to __bit_map accoring to the __node_value and __merge_node_value
 * @param {uint8_t} __node_value
 * @param {uint8_t} __merge_node_value
 * @param {string&} __bit_map
 * @return {*}
 */
void operation::SetBitMap(uint8_t __node_value, uint8_t __merge_node_value, std::string& __bit_map) {
	try {
		// __merge_node_value cannot be 0000 0000
		if (__merge_node_value == 0x00) {
			throw "Empty node.";
		}
		else {
			for (size_t i = 0; i < 8; i++) {
				// extract the highest bit
				bool __merge_bit = (__merge_node_value & 0x80) == 0x00 ? false : true;
				bool __node_bit  = (__node_value & 0x80) == 0x00 ? false : true;
				// merge bit must be 1 to indicate there is a point
				if (__merge_bit) {
					// this point is from this node
					if (__node_bit) {
						__bit_map += '1';
					}
					// this point is from other nodes
					else {
						__bit_map += '0';
					}
				}
				// left move 1 bit to extract next bit
				__merge_node_value <<= 1, __node_value <<= 1;
			}
		}
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error : " << error_message << std::endl;
		std::exit(1);
	}
}