/***
 * @Author: ChenRP07
 * @Date: 2022-05-03 19:12:35
 * @LastEditTime: 2022-06-08 20:13:39
 * @LastEditors: ChenRP07
 * @Description: C++ header for class GOF and class octree.
 */

#ifndef LIB_OCTREE_H
#define LIB_OCTREE_H

#include <Eigen/Dense>
#include <float.h>
#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <string>
#include <vector>

namespace pco {
namespace type {
	struct Residual {
		float x_;
		float y_;
		float z_;
		Residual(float __x, float __y, float __z) {
			this->x_ = __x, this->y_ = __y, this->z_ = __z;
		}
	};
	struct Color {
		uint8_t r_;
		uint8_t g_;
		uint8_t b_;
		Color() {
			this->r_ = this->g_ = this->b_ = 0x00;
		}
		Color(pcl::PointXYZRGB& __x) {
			this->r_ = __x.r, this->g_ = __x.g, this->b_ = __x.b;
		}
		Color& operator=(const Color& __x) {
			this->r_ = __x.r_, this->g_ = __x.g_, this->b_ = __x.b_;
			return *this;
		}
		Color& operator+=(const pcl::PointXYZRGB& __x) {
			this->r_ += __x.r, this->g_ += __x.g, this->b_ += __x.b;
			return *this;
		}
		Color& operator-=(const Color& __x) {
			this->r_ -= __x.r_, this->g_ -= __x.g_, this->b_ -= __x.b_;
			return *this;
		}
	};
}  // namespace type

namespace operation {
	extern bool          AreFramesEmpty(const std::vector<std::vector<size_t>>&);
	extern pcl::PointXYZ SubnodeCenter(const pcl::PointXYZ, const int, const float);
	extern uint8_t       BoolSetToUChar(const std::vector<bool>&);
	extern size_t        NodePointCount(uint8_t);
	extern void          SetBitMap(uint8_t, uint8_t, std::string&);
	extern void          NodePointPosition(uint8_t __node, std::vector<size_t>& __pos);
	extern void          SubnodePoint(const pcl::PointXYZ& __center, const size_t __pos, pcl::PointXYZ& __point);
	extern float         PSNRGeo(const pcl::PointCloud<pcl::PointXYZ>& __x, const pcl::PointCloud<pcl::PointXYZ>& __y);
}  // namespace operation

namespace octree {

	class GOF {
	  private:
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>> frame_patches_;
		std::vector<Eigen::Matrix4f>                   motion_vectors_;

		const size_t kGroupOfFrames;
		size_t       frame_number_;
		float        min_x_;
		float        max_x_;
		float        min_y_;
		float        max_y_;
		float        min_z_;
		float        max_z_;

	  public:
		GOF(const size_t);
		size_t                  size() const;
		size_t                  size(const size_t) const;
		pcl::PointXYZRGB&       operator()(const size_t, const size_t);
		const pcl::PointXYZRGB& operator()(const size_t, const size_t) const;
		void                    AddPatch(const pcl::PointCloud<pcl::PointXYZRGB>&, const Eigen::Matrix4f&);
		void                    GetPatch(pcl::PointCloud<pcl::PointXYZRGB>&, Eigen::Matrix4f&, const size_t) const;
		void                    GetPatches(std::vector<pcl::PointCloud<pcl::PointXYZRGB>>&, std::vector<Eigen::Matrix4f>&) const;
		float                   GetResolution() const;
		float                   GetResolutionBinary() const;
		pcl::PointXYZ           GetCenter() const;
		void                    GetMatrices(std::vector<Eigen::Matrix4f>&) const;
	};

	class Octree {
	  private:
		std::vector<std::vector<uint8_t>>     tree_nodes_;
		std::vector<std::vector<uint8_t>>     tree_leave_;
		std::vector<std::vector<type::Color>> tree_colors_;
		std::vector<std::string>              bit_map_;
		std::vector<Eigen::Matrix4f>          motion_vectors_;
		std::vector<uint8_t>                  tree_merge_leave_;
		// tree_residuals_ is be activated when kIsVoxelized is false, i.e. the unvoxelized point cloud
		// std::vector<std::vector<type::Residual>> tree_residuals_;

		const size_t  kGroupOfFrames;
		const float   kMinResolution;
		pcl::PointXYZ tree_center_;
		size_t        tree_height_;
		float         tree_resolution_;

		// this controlling parameter is activated if necessary
		// const bool kIsVoxelized;

	  public:
		Octree(const size_t, const float = 2.0f);
		bool AddTreeNode(const GOF&, std::vector<std::vector<size_t>>&, size_t, float, pcl::PointXYZ);
		void SetFrames(const GOF&);
		void ColorCompensation();

		pcl::PointXYZ   GetCenter() const;
		Eigen::Matrix4f GetMotionVector(const size_t __index) const;
		size_t          GetHeight() const;
		void            OutputTree(std::string& __tree_data);
		void            OutputPatches(std::vector<std::string>& __bit_maps, std::vector<std::vector<uint8_t>>& __colors);
		size_t          GetMapSize(size_t __index) const;
	};

	class DeOctree {
	  private:
		std::vector<std::vector<uint8_t>>       tree_nodes_;
		std::vector<std::vector<pcl::PointXYZ>> node_centers_;

		const size_t  kGroupOfFrames;
		const float   kMinResolution;
		pcl::PointXYZ tree_center_;
		size_t        tree_height_;
		float         tree_resolution_;

	  public:
		DeOctree(const size_t __gof, const float __resolution = 2.0f);
		void SetCenter(pcl::PointXYZ& __center);
		void SetHieght(size_t __height);
		void SetTreeNodes(std::string& __nodes);
		void GetPatch(std::string& __nodes, std::vector<pcl::PointXYZ>& __points);
	};

}  // namespace octree
}  // namespace pco
#endif