/***
 * @Author: ChenRP07
 * @Date: 2022-05-13 10:58:55
 * @LastEditTime: 2022-05-30 14:58:19
 * @LastEditors: ChenRP07
 * @Description:
 */

#include "octree.h"

using namespace pco::octree;

/***
 * @description: constructor, input kGroupOfFrames
 * @param {int} group_of_frames
 * @return {*}
 */
GOF::GOF(const size_t group_of_frames)
    : kGroupOfFrames{group_of_frames}, frame_patches_{std::vector<pcl::PointCloud<pcl::PointXYZRGB>>(group_of_frames)}, motion_vectors_{std::vector<Eigen::Matrix4f>(group_of_frames)} {
	try {
		if (this->kGroupOfFrames < 1) {
			throw "GOF size is smaller than 1.";
		}
	}
	catch (const char* error_message) {
		std::cerr << "GOF constructing failed : " << error_message << std::endl;
		std::exit(1);
	}
	this->min_x_        = FLT_MAX;
	this->max_x_        = FLT_MIN;
	this->min_y_        = FLT_MAX;
	this->max_y_        = FLT_MIN;
	this->min_z_        = FLT_MAX;
	this->max_z_        = FLT_MIN;
	this->frame_number_ = 0;
}

/***
 * @description: get size of frame_patches_
 * @param {*}
 * @return {size_t} frame_patches_.size()
 */
size_t GOF::size() const {
	try {
		if (this->frame_patches_.size() == this->motion_vectors_.size()) {
			return this->frame_patches_.size();
		}
		else {
			throw "Unmatching size between frames and matrices.";
		}
	}
	catch (const char* error_message) {
		std::cerr << "GOF is broken : " << error_message << std::endl;
		std::exit(1);
	}
}

/***
 * @description: get size of frame_patches_[index]
 * @param {size_t} index
 * @return {size_t} frame_patches_[index].size()
 */
size_t GOF::size(const size_t __index) const {
	try {
		if (__index < this->frame_patches_.size()) {
			return this->frame_patches_[__index].size();
		}
		else {
			throw "Out of range.";
		}
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error from function GOF::size(__index) : " << error_message << std::endl;
		std::exit(1);
	}
}

/***
 * @description: get i-th frame_patches_'s j-th element
 * @param {size_t} __x
 * @param {size_t} __y
 * @return {PointXYZRGB&} frame_patches_[__x][__y]
 */
pcl::PointXYZRGB& GOF::operator()(const size_t __x, const size_t __y) {
	try {
		if (__x < this->frame_patches_.size()) {
			if (__y < this->frame_patches_[__x].size()) {
				return this->frame_patches_[__x][__y];
			}
			else {
				throw "Second index out of range.";
			}
		}
		else {
			throw "First index out of range.";
		}
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error from access GOF point : " << error_message << std::endl;
		std::exit(1);
	}
}

/***
 * @description: get i-th frame_patches_'s j-th element
 * @param {size_t} __x
 * @param {size_t} __y
 * @return {PointXYZRGB&} frame_patches_[__x][__y]
 */
const pcl::PointXYZRGB& GOF::operator()(const size_t __x, const size_t __y) const {
	try {
		if (__x < this->frame_patches_.size()) {
			if (__y < this->frame_patches_[__x].size()) {
				return this->frame_patches_[__x][__y];
			}
			else {
				throw "Second index out of range.";
			}
		}
		else {
			throw "First index out of range.";
		}
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error from access GOF point : " << error_message << std::endl;
		std::exit(1);
	}
}

/***
 * @description: add a patch into gof
 * @param {PointCloud} __patch
 * @param {Matrix4f} __matrix
 * @return {*}
 */
void GOF::AddPatch(const pcl::PointCloud<pcl::PointXYZRGB>& __patch, const Eigen::Matrix4f& __matrix) {
	try {
		// frame_patches_'s size must be less than kGroupOfFrames
		if (this->frame_patches_.size() < this->kGroupOfFrames) {
			// add a patch
			this->frame_patches_.emplace_back(pcl::PointCloud<pcl::PointXYZRGB>());
			this->frame_patches_[this->frame_patches_.size() - 1].resize(__patch.size());
			for (size_t i = 0; i < __patch.size(); i++) {
				this->frame_patches_[this->frame_patches_.size() - 1][i] = __patch[i];
				// update range of coordinates
				this->min_x_ = this->min_x_ < __patch[i].x ? this->min_x_ : __patch[i].x;
				this->max_x_ = this->max_x_ > __patch[i].x ? this->max_x_ : __patch[i].x;
				this->min_y_ = this->min_y_ < __patch[i].y ? this->min_y_ : __patch[i].y;
				this->max_y_ = this->max_y_ > __patch[i].y ? this->max_y_ : __patch[i].y;
				this->min_z_ = this->min_z_ < __patch[i].z ? this->min_z_ : __patch[i].z;
				this->max_z_ = this->max_z_ > __patch[i].z ? this->max_z_ : __patch[i].z;
			}
			// add transformation matrix
			if (this->frame_patches_.size() == 1) {
				this->motion_vectors_.emplace_back(Eigen::Matrix4f::Identity());
			}
			else {
				this->motion_vectors_.emplace_back(__matrix);
			}
		}
		else {
			throw "Out of GOF size limitation.";
		}
	}
	catch (const char* error_message) {
		std::cerr << "Adding patch into GOF failed : " << error_message << std::endl;
	}
}

/***
 * @description: get __index-th patch
 * @param {PointCloud} __patch
 * @param {Matrix4f} __matrix
 * @param {int} __index
 * @return {*}
 */
void GOF::GetPatch(pcl::PointCloud<pcl::PointXYZRGB>& __patch, Eigen::Matrix4f& __matrix, const size_t __index) const {
	try {
		if (__index < this->frame_patches_.size()) {
			__patch.resize(this->frame_patches_[__index].size());
			for (size_t i = 0; i < this->frame_patches_[__index].size(); i++) {
				__patch[i] = this->frame_patches_[__index][i];
			}
			__matrix = this->motion_vectors_[__index];
		}
		else {
			throw "Out of patch range.";
		}
	}
	catch (const char* error_message) {
		std::cerr << "Fatal error from function GetPatch() : " << error_message << std::endl;
		std::exit(1);
	}
}

/***
 * @description: get all patches
 * @param {vector<PointCloud>} __patches
 * @param {vector<Matrix4f>} __matrices
 * @return {*}
 */
void GOF::GetPatches(std::vector<pcl::PointCloud<pcl::PointXYZRGB>>& __patches, std::vector<Eigen::Matrix4f>& __matrices) const {
	try {
		if (this->frame_patches_.size() == this->motion_vectors_.size()) {
			__patches.resize(this->frame_patches_.size());
			__matrices.resize(this->motion_vectors_.size());
			for (size_t i = 0; i < this->frame_patches_.size(); i++) {
				__patches[i].resize(this->frame_patches_[i].size());
				for (size_t j = 0; j < this->frame_patches_[i].size(); j++) {
					__patches[i][j] = this->frame_patches_[i][j];
				}
				__matrices[i] = this->motion_vectors_[i];
			}
		}
		else {
			throw "Unmatching size between frames and matrices.";
		}
	}
	catch (const char* error_message) {
		std::cerr << "GOF is broken : " << error_message << std::endl;
		std::exit(1);
	}
}

/***
 * @description: return maximum span of 3D coordinates
 * @param {*}
 * @return {float}
 */
float GOF::GetResolution() const {
	return std::max(std::max(this->max_x_ - this->min_x_, this->max_y_ - this->min_y_), this->max_z_ - this->min_z_);
}

/***
 * @description: return maximum span of 3D coordinates which is rounded up as a power of 2;
 * @param {*}
 * @return {float}
 */
float GOF::GetResolutionBinary() const {
	return std::pow(2.0f, std::ceil(std::log2(this->GetResolution())));
}

/***
 * @description: return the center of 3D coordinates of patches
 * @param {*}
 * @return {PointXYZ}
 */
pcl::PointXYZ GOF::GetCenter() const {
	return pcl::PointXYZ((this->min_x_ + this->max_x_) / 2, (this->min_y_ + this->max_y_) / 2, (this->min_z_ + this->max_z_) / 2);
}

/***
 * @description: get motion vectors
 * @param {vector<Matrix4f>} __matrices
 * @return {*}
 */
void GOF::GetMatrices(std::vector<Eigen::Matrix4f>& __matrices) const {
	__matrices.resize(this->motion_vectors_.size());
	for (size_t i = 0; i < this->motion_vectors_.size(); i++) {
		__matrices[i] = motion_vectors_[i];
	}
}