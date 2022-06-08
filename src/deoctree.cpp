/***
 * @Author: ChenRP07
 * @Date: 2022-06-06 15:54:35
 * @LastEditTime: 2022-06-08 19:55:44
 * @LastEditors: ChenRP07
 * @Description:
 */
#include "octree.h"
using namespace pco;

octree::DeOctree::DeOctree(const size_t __gof, const float __resolution) : kGroupOfFrames{__gof}, kMinResolution{__resolution} {}

void octree::DeOctree::SetCenter(pcl::PointXYZ& __center) {
	this->tree_center_ = __center;
}

void octree::DeOctree::SetHieght(size_t __height) {
	this->tree_height_ = __height;
}

/***
 * @description: from __nodes input all branch nodes and calculate these nodes' center
 * @param {string&} __nodes
 * @return {*}
 */
void octree::DeOctree::SetTreeNodes(std::string& __nodes) {
	try {
		this->tree_nodes_.resize(this->tree_height_, std::vector<uint8_t>());

		// Input tree node
		size_t __layer_node_count   = 1;
		size_t __nodes_string_index = 0;

		// for each layer
		for (size_t i = 0; i < this->tree_height_; i++) {
			// count next layer should have how many nodes
			size_t new_node_cnt = 0;
			// get nodes according to the last layer count
			for (size_t j = 0; j < __layer_node_count; j++) {
				if (__nodes_string_index >= __nodes.size()) {
					throw "Out of nodes string range.";
				}
				uint8_t data = static_cast<uint8_t>(__nodes[__nodes_string_index]);
				// count how many 1-bits this node has
				new_node_cnt += operation::NodePointCount(data);
				this->tree_nodes_[i].emplace_back(data);
				__nodes_string_index++;
			}
			__layer_node_count = new_node_cnt;
		}

		// __nodes should be scanned to the end.
		if (__nodes_string_index != __nodes.size()) {
			throw "String is too long.";
		}
		// Calculate node center
		// node center should be 1 more than tree height, i.e. tree leave layer
		this->node_centers_.resize(this->tree_height_ + 1, std::vector<pcl::PointXYZ>());
		// add tree center to the first layer
		this->node_centers_[0].emplace_back(this->tree_center_);
		// max resolution
		this->tree_resolution_ = std::pow(this->kMinResolution, this->tree_height_ + 1);
		// res to calculate centers
		float Res = this->tree_resolution_;
		// each layer
		for (size_t i = 0; i < this->tree_height_; i++) {
			// each node
			for (size_t j = 0; j < this->node_centers_[i].size(); j++) {
				// 1 bit position
				std::vector<size_t> pos;
				operation::NodePointPosition(this->tree_nodes_[i][j], pos);
				// for each 1 bit
				for (size_t k = 0; k < pos.size(); k++) {
					// calculate the subnode center and add it to next layer
					pcl::PointXYZ subnode_center = operation::SubnodeCenter(this->node_centers_[i][j], pos[i], Res / 2);
					this->node_centers_[i + 1].emplace_back(subnode_center);
				}
			}
			// next layer res should be half of now
			Res /= 2;
		}
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error in octree reconstruction : " << error_message << std::endl;
		std::exit(1);
	}
}

void octree::DeOctree::GetPatch(std::string& __nodes, std::vector<pcl::PointXYZ>& __points) {
	// resize enough space
	__points.resize(__nodes.size() * 8);
	// count points
	size_t point_cnt = 0;

	// for each leaf
	for (size_t i = 0; i < __nodes.size(); i++) {
		// 1 bit position
		std::vector<size_t> pos;
		operation::NodePointPosition(static_cast<uint8_t>(__nodes[i]), pos);
		printf("%lu %.2f %.2f %.2f \n", i, this->node_centers_[this->tree_height_][i].x, this->node_centers_[this->tree_height_][i].y, this->node_centers_[this->tree_height_][i].z);
		// for each 1 bit
		for (size_t j = 0; j < pos.size(); j++) {
			// calculate leaf point and add it to __points
			if (this->kMinResolution == 2.0f) {
				operation::SubnodePoint(this->node_centers_[this->tree_height_][i], pos[j], __points[point_cnt]);
			}
			else if (this->kMinResolution > 2.0f) {
				__points[point_cnt] = operation::SubnodeCenter(this->node_centers_[this->tree_height_][i], pos[j], this->kMinResolution / 2);
			}
			point_cnt++;
		}
	}

	// delete empty space
	__points.resize(point_cnt);
}