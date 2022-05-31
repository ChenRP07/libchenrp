/***
 * @Author: ChenRP07
 * @Date: 2022-04-28 17:00:32
 * @LastEditTime: 2022-05-31 15:34:56
 * @LastEditors: ChenRP07
 * @Description:
 */
#include "registration.h"
#include <pcl/io/ply_io.h>
using namespace pco::registration;

/***
 * @description: constructor
 * @param {float} max_correspondence
 * @param {int} max_iteration
 * @param {float} max_mse_difference
 * @param {float} max_tranformation_diference
 * @return {*}
 */
ICP::ICP(const float max_correspondence, const int max_iteration, const float max_mse_difference, const float max_tranformation_diference)
    : kCorrespondenceThreshold{max_correspondence}, kIterationThreshold{max_iteration}, kMSEThreshold{max_mse_difference}, kTransformationThreshold{max_tranformation_diference}, regist_base() {}

/***
 * @description: do Iterative Closest Point Algorithm, using centroid alignment
 * @param {int} centroid_alignment_type, 0 none, 1 global, 2 local.
 * @return {bool} Algorithm success ?
 */
bool ICP::align(const int centroid_alignment_type) {
	// cloud is empty or wrong centroid alignment type, throw an error
	try {
		if (this->source_point_cloud_.empty())
			throw "Source point cloud is empty.";
		else if (this->target_point_cloud_.empty())
			throw "Target point cloud is empty.";

		if (centroid_alignment_type != 0 && centroid_alignment_type != 1 && centroid_alignment_type != 2)
			throw "Wrong centroid alignment type";
	}
	catch (const char* error_message) {
		std::cerr << "Point cloud alignment failed. " << error_message << std::endl;
		return 0;
	}

	if (centroid_alignment_type == 1) {
		this->GolbalCentroidAlignment();
	}
	else if (centroid_alignment_type == 2) {
		this->LocalCentroidAlignment();
	}

	// Iterative Closest Point Algorithm
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

	// set point cloud
	icp.setInputSource(this->result_point_cloud_.makeShared());
	icp.setInputTarget(this->target_point_cloud_.makeShared());

	// set manaully parameter
	icp.setMaxCorrespondenceDistance(this->kCorrespondenceThreshold);
	icp.setTransformationEpsilon(this->kTransformationThreshold);
	icp.setEuclideanFitnessEpsilon(this->kMSEThreshold);
	icp.setMaximumIterations(this->kIterationThreshold);

	pcl::PointCloud<pcl::PointXYZRGB> temp_point_cloud;

	// do alignment
	icp.align(temp_point_cloud);

	// check whether converged
	try {
		if (icp.hasConverged() == false) {
			throw "Cannot converge.";
		}
		else
			this->result_point_cloud_.swap(temp_point_cloud);
	}
	catch (const char* error_message) {
		std::cerr << "Alignment failed. " << error_message << " Source size " << this->GetSourcePointCloudSize() << ". Target size " << this->GetTargetPointCloudSize() << "." << std::endl;
		return 0;
	}

	// record result
	this->mean_squred_error_    = icp.getFitnessScore();
	this->tranformation_matrix_ = icp.getFinalTransformation() * this->tranformation_matrix_;

	return 1;
}