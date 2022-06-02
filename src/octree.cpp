/***
 * @Author: ChenRP07
 * @Date: 2022-05-03 19:12:40
 * @LastEditTime: 2022-06-02 15:44:25
 * @LastEditors: ChenRP07
 * @Description: C++ implement for class octree
 */

#include "octree.h"
using namespace pco::octree;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"

Octree::Octree(const size_t __gof, const float __resolution) : kGroupOfFrames{__gof}, kMinResolution{__resolution} {
	try {
		if (this->kGroupOfFrames < 1) {
			throw "GOF size is smaller than 1.";
		}
	}
	catch (const char* error_message) {
		std::cerr << "Octree constructing failed : " << error_message << std::endl;
		std::exit(1);
	}
}

/***
 * @description: from a GOF object set octree
 * @param {GOF&} __gof
 * @return {*}
 */
void Octree::SetFrames(const GOF& __gof) {
	try {
		// __gof size must be equal to the constructor set.
		if (__gof.size() == this->kGroupOfFrames) {
			// tree height, e.g. 32->16->8->4, so 4
			this->tree_nodes_.resize((size_t)std::log2(__gof.GetResolutionBinary()) - 1);

			// GOF size
			this->tree_leave_.resize(this->kGroupOfFrames);
			this->tree_colors_.resize(this->kGroupOfFrames);
			this->bit_map_.resize(this->kGroupOfFrames);
			this->motion_vectors_.resize(this->kGroupOfFrames);

			// tree_residuals_ is be activated when kIsVoxelized is false, i.e. the unvoxelized point cloud
			// this->tree_residuals_.resize(this->kGroupOfFrames);

			// center, tree height and resolution
			this->tree_center_     = __gof.GetCenter();
			this->tree_height_     = this->tree_nodes_.size();
			this->tree_resolution_ = __gof.GetResolutionBinary();
		}
		else {
			throw "Unmatching GOF size between constructor and function SetFrames.";
		}
	}
	catch (const char* error_message) {
		std::cerr << "Octree constructing failed : " << error_message << std::endl;
		std::exit(1);
	}

	// use a index vector to indicate if there is point in a octree node.
	std::vector<std::vector<size_t>> __octree_points(__gof.size());
	for (size_t i = 0; i < __octree_points.size(); i++) {
		//__octree_points[i].resize(__gof.size(i));
		for (size_t j = 0; j < __gof.size(i); j++) {
			__octree_points[i].emplace_back(j);
		}
	}

	// recursively add tree nodes
	this->AddTreeNode(__gof, __octree_points, 0, this->tree_resolution_, this->tree_center_);
	// set transformation matrices
	__gof.GetMatrices(this->motion_vectors_);
	// calculate bit_map_
	// count the leaf number and point number
	size_t __leave_number = this->tree_leave_[0].size();
	size_t __point_number = 0;

	try {
		// leaf number in each frame must be consistent
		for (size_t i = 0; i < this->tree_leave_.size(); i++) {
			if (this->tree_leave_[i].size() != __leave_number) {
				throw "Unmatching leave number of frames";
			}
			this->bit_map_[i].clear();
		}

		// allocate memory for tree_merge_leave_
		this->tree_merge_leave_.resize(__leave_number);

		// using OR to generate merged leave
		for (size_t i = 0; i < __leave_number; i++) {
			uint8_t __merge_leaf_value = 0x00;
			for (size_t j = 0; j < this->tree_leave_.size(); j++) {
				__merge_leaf_value |= this->tree_leave_[j][i];
			}

			// count point number
			__point_number += pco::operation::NodePointCount(__merge_leaf_value);
			// add merged leaf
			this->tree_merge_leave_[i] = __merge_leaf_value;
		}
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error in octree constructing : " << error_message << std::endl;
		std::exit(1);
	}
	// for (size_t i = 0; i < __leave_number; i++) {
	// 	printf("%02x ", this->tree_merge_leave_[i]);
	// }

	// calculate bit_map_
	for (size_t i = 0; i < __leave_number; i++) {
		for (size_t j = 0; j < this->kGroupOfFrames; j++) {
			pco::operation::SetBitMap(this->tree_leave_[j][i], this->tree_merge_leave_[i], this->bit_map_[j]);
		}
	}
}

/***
 * @description: recursively add tree node into octree
 * @param {vector<vector<size_t>>} __node_points
 * @param {size_t} __height
 * @param {float} __resolution
 * @param {PointXYZ} __center
 * @return {bool}
 */
bool Octree::AddTreeNode(const GOF& __gof, std::vector<std::vector<size_t>>& __node_points, size_t __height, float __resolution, pcl::PointXYZ __center) {
	// init value for a node 0000 0000
	// uint8_t __node_value = 0x00;

	// if this node is empty, return a 0
	if (pco::operation::AreFramesEmpty(__node_points)) {
		return false;
	}
	// if height smaller than the tree height, this is a branch node
	else if (__height < this->tree_height_) {
		// split points into 8 subnodes, then clear the father node vector.
		std::vector<std::vector<std::vector<size_t>>> __subnodes_value(8, std::vector<std::vector<size_t>>(this->kGroupOfFrames));
		// for each frame
		for (size_t i = 0; i < __node_points.size(); i++) {
			// for each point
			for (size_t j = 0; j < __node_points[i].size(); j++) {
				// subnode index point belong to
				int pos = 0;

				// get the real point
				pcl::PointXYZRGB __point = __gof(i, __node_points[i][j]);
				// point position, z-order
				pos |= __point.x > __center.x ? 0 : 1;
				pos <<= 1;
				pos |= __point.y > __center.y ? 0 : 1;
				pos <<= 1;
				pos |= __point.z > __center.z ? 0 : 1;
				// pos must in {0, 1, 2, 3, 4, 5, 6, 7}
				try {
					if (pos <= 7 && pos >= 0) {
						// std::cout << pos << " " << i << " " << j << std::endl;
						__subnodes_value[pos][i].push_back(__node_points[i][j]);
					}
					else {
						throw "Point not belongs to any subnode.";
					}
				}
				catch (const char* error_message) {
					std::cerr << "Adding tree node to octree failed : " << error_message << std::endl;
					std::exit(1);
				}
			}
		}
		// clear father node
		__node_points.clear();

		// subnodes value
		std::vector<bool> __node_value_bits(8, false);

		// values are got from AddTreeNode itself
		for (size_t i = 0; i < 8; i++) {
			// a new center
			pcl::PointXYZ __new_center = pco::operation::SubnodeCenter(__center, i, __resolution / 2);
			// recursively add process
			__node_value_bits[i] = this->AddTreeNode(__gof, __subnodes_value[i], __height + 1, __resolution / 2, __new_center);
		}

		// add this node to tree
		this->tree_nodes_[__height].push_back(pco::operation::BoolSetToUChar(__node_value_bits));

		// this node has a least one point
		return true;
	}
	// else this is a leaf node, then add leave
	else {
		// split points into 8 subnodes, then clear the father node vector.
		std::vector<std::vector<std::vector<size_t>>> __subnodes_value(8, std::vector<std::vector<size_t>>(this->kGroupOfFrames));
		// for each frame
		for (size_t i = 0; i < __node_points.size(); i++) {
			// for each point
			for (size_t j = 0; j < __node_points[i].size(); j++) {
				// subnode index point belong to
				int pos = 0;

				// get the real point
				pcl::PointXYZRGB __point = __gof(i, __node_points[i][j]);
				// point position, z-order
				pos |= __point.x > __center.x ? 0 : 1;
				pos <<= 1;
				pos |= __point.y > __center.y ? 0 : 1;
				pos <<= 1;
				pos |= __point.z > __center.z ? 0 : 1;
				// pos must in {0, 1, 2, 3, 4, 5, 6, 7}
				try {
					if (pos <= 7 && pos >= 0) {
						__subnodes_value[pos][i].push_back(__node_points[i][j]);
					}
					else {
						throw "Point not belongs to any subnode.";
					}
				}
				catch (const char* error_message) {
					std::cerr << "Adding tree node to octree failed : " << error_message << std::endl;
					std::exit(1);
				}
			}
		}
		// clear father node
		__node_points.clear();

		// for each frame
		for (size_t i = 0; i < this->kGroupOfFrames; i++) {
			// init leaf value to 0000 0000
			uint8_t __leaf_value = 0x00;

			for (size_t j = 0; j < 8; j++) {
				__leaf_value <<= 1;
				// if this subnode is empty
				if (__subnodes_value[j][i].empty()) {
					__leaf_value |= 0;
				}
				else {
					// this section is designed for an unvoxelized point cloud
					/*
					 * if (kIsVoxelized == false) {
					 *     code for add tree_residuals_ and various colors in a leaf node and anything used...
					 * }
					 * this section will be activated if necessary.
					 */

					// this subnode is not empty
					__leaf_value |= 1;

					// do color voxel downsampling
					pco::type::Color __leaf_color;
					for (size_t k = 0; k < __subnodes_value[j][i].size(); k++) {
						__leaf_color += __gof(i, __subnodes_value[j][i][k]);
					}
					__leaf_color.r_ /= __subnodes_value[j][i].size();
					__leaf_color.g_ /= __subnodes_value[j][i].size();
					__leaf_color.b_ /= __subnodes_value[j][i].size();

					// add the sampled color into tree_colors_
					this->tree_colors_[i].emplace_back(__leaf_color);
				}
			}

			// add this leaf into tree_leave_
			this->tree_leave_[i].emplace_back(__leaf_value);
		}

		// this node has at least one point
		return true;
	}
}

/***
 * @description: do color motion compensation
 * @param {*}
 * @return {*}
 */
void Octree::ColorCompensation() {
	// for each p-frame
	for (size_t i = 1; i < this->kGroupOfFrames; i++) {
		// init __last_color 0x00 0x00 0x00
		type::Color __last_color;

		// i-frame pointer and p-frame pointer
		size_t __index_i = 0, __index_p = 0;

		// traverse the i-frame and p-frame bit_map_
		for (size_t k = 0; k < this->bit_map_[0].size(); k++) {
			if (this->bit_map_[i][k] == '1') {
				// this position has no i-frame point, so this color do mv with last point in p-frame
				if (this->bit_map_[0][k] == '0') {
					type::Color __temp = this->tree_colors_[i][__index_p];
					this->tree_colors_[i][__index_p] -= __last_color;
					__last_color = __temp;
				}
				else if (this->bit_map_[0][k] == '1') {
					__last_color = this->tree_colors_[i][__index_p];
					this->tree_colors_[i][__index_p] -= this->tree_colors_[0][__index_i];
					__index_i++;
				}
				__index_p++;
			}
			else if (this->bit_map_[0][k] == '1') {
				__index_i++;
			}
		}
	}
}

void Octree::OutputTree(std::string& __tree_data) {
	try {
		for (auto& i : this->tree_nodes_) {
			for (auto& j : i) {
				__tree_data += static_cast<char>(j);
			}
		}

		for (auto& i : this->tree_merge_leave_) {
			__tree_data += static_cast<char>(i);
		}

		if (__tree_data.empty()) {
			throw "No data in octree.";
		}
	}
	// open failed
	catch (const char* error_message) {
		std::cerr << "Fatal error : " << error_message << std::endl;
		std::exit(1);
	}
}

void Octree::OutputPatches(std::vector<std::string>& __bit_maps, std::vector<std::vector<uint8_t>>& __colors) {
	try {
		__bit_maps.clear();
		__bit_maps.resize(this->kGroupOfFrames, "");
		__colors.clear();
		__colors.resize(this->kGroupOfFrames, std::vector<uint8_t>());
		// for each patch
		for (size_t i = 0; i < this->kGroupOfFrames; i++) {
			// bit-map
			std::string temp = this->bit_map_[i];
			while (temp.size() % 8 != 0)
				temp += '0';
			for (size_t j = 0; j < temp.size() / 8; j++) {
				char temp_c = 0x00;
				for (size_t k = 0; k < 8; k++) {
					temp_c <<= 1;
					if (j * 8 + k >= temp.size()) {
						throw "Out of bit_map_ range.";
					}
					if (temp[j * 8 + k] != '1' && temp[j * 8 + k] != '0') {
						throw "Wrong bit_map_ value.";
					}
					temp_c |= (temp[j * 8 + k] - '0');
				}
				__bit_maps[i] += temp_c;
			}

			// colors
			for (size_t j = 0; j < this->tree_colors_[i].size(); j++) {
				__colors[i].emplace_back(this->tree_colors_[i][j].r_);
				__colors[i].emplace_back(this->tree_colors_[i][j].g_);
				__colors[i].emplace_back(this->tree_colors_[i][j].b_);
			}
		}
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error : " << error_message << std::endl;
		std::exit(1);
	}
}

pcl::PointXYZ Octree::GetCenter() const {
	return this->tree_center_;
}

size_t Octree::GetHeight() const {
	return this->tree_height_;
}
#pragma GCC diagnostic pop