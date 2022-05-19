/***
 * @Author: ChenRP07
 * @Date: 2022-05-03 11:12:39
 * @LastEditTime: 2022-05-03 11:12:52
 * @LastEditors: ChenRP07
 * @Description:
 */
#include "registration.h"

using namespace pco::registration;

/***
 * @description: constructor
 * @param {float} max_correspondence
 * @param {int} max_iteration
 * @param {float} normal_search_radius
 * @param {float} max_mse_difference
 * @param {float} max_tranformation_diference
 * @return {*}
 */
NICP::NICP(const float max_correspondence, const int max_iteration, const float normal_search_radius, const float max_mse_difference, const float max_tranformation_diference)
    : kCorrespondenceThreshold{ max_correspondence }, kIterationThreshold{ max_iteration },
      kNormalRadius(normal_search_radius), kMSEThreshold{ max_mse_difference }, kTransformationThreshold{ max_tranformation_diference }, regist_base() {
    this->normaled_result_point_cloud_.reserve(10);
    this->normaled_target_point_cloud_.reserve(10);
}

/***
 * @description: compute normals
 * @param {*}
 * @return {*}
 */
void NICP::ComputeNormals() {
    // cloud is empty or wrong centroid alignment type, throw an error
    try {
        if (this->source_point_cloud_.empty())
            throw "Source point cloud is empty.";
        else if (this->target_point_cloud_.empty())
            throw "Target point cloud is empty.";
    }
    catch (const char* error_message) {
        std::cerr << "Computing point cloud normals failed. " << error_message << std::endl;
        return;
    }

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> result_estimator, target_estimator;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr           result_kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>), target_kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>                         result_normals, target_normals;

    result_estimator.setInputCloud(this->result_point_cloud_.makeShared());
    result_estimator.setRadiusSearch(this->kNormalRadius);
    result_estimator.setSearchMethod(result_kdtree);
    result_estimator.compute(result_normals);

    target_estimator.setInputCloud(this->target_point_cloud_.makeShared());
    target_estimator.setRadiusSearch(this->kNormalRadius);
    target_estimator.setSearchMethod(target_kdtree);
    target_estimator.compute(target_normals);

    this->normaled_result_point_cloud_.resize(this->result_point_cloud_.size());
    this->normaled_target_point_cloud_.resize(this->target_point_cloud_.size());

    for (size_t i = 0; i < this->result_point_cloud_.size(); i++) {
        this->normaled_result_point_cloud_[i].x         = this->result_point_cloud_[i].x;
        this->normaled_result_point_cloud_[i].y         = this->result_point_cloud_[i].y;
        this->normaled_result_point_cloud_[i].z         = this->result_point_cloud_[i].z;
        this->normaled_result_point_cloud_[i].rgb       = this->result_point_cloud_[i].rgb;
        this->normaled_result_point_cloud_[i].normal_x  = result_normals[i].normal_x;
        this->normaled_result_point_cloud_[i].normal_y  = result_normals[i].normal_y;
        this->normaled_result_point_cloud_[i].normal_z  = result_normals[i].normal_z;
        this->normaled_target_point_cloud_[i].curvature = result_normals[i].curvature;
    }

    for (size_t i = 0; i < this->target_point_cloud_.size(); i++) {
        this->normaled_target_point_cloud_[i].x         = this->target_point_cloud_[i].x;
        this->normaled_target_point_cloud_[i].y         = this->target_point_cloud_[i].y;
        this->normaled_target_point_cloud_[i].z         = this->target_point_cloud_[i].z;
        this->normaled_target_point_cloud_[i].rgb       = this->target_point_cloud_[i].rgb;
        this->normaled_target_point_cloud_[i].normal_x  = target_normals[i].normal_x;
        this->normaled_target_point_cloud_[i].normal_y  = target_normals[i].normal_y;
        this->normaled_target_point_cloud_[i].normal_z  = target_normals[i].normal_z;
        this->normaled_target_point_cloud_[i].curvature = target_normals[i].curvature;
    }
}

/***
 * @description: do normal icp, using centroid alignment
 * @param {int} centroid_alignment_type, 0 none, 1 global, 2 local
 * @return {*}
 */
bool NICP::align(const int centroid_alignment_type) {
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

    // do centroid alignment
    if (centroid_alignment_type == 1) {
        this->GolbalCentroidAlignment();
    }
    else if (centroid_alignment_type == 2) {
        this->LocalCentroidAlignment();
    }

    // compute normals
    this->ComputeNormals();

    // do nicp
    pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> nicp;
    nicp.setInputSource(this->normaled_result_point_cloud_.makeShared());
    nicp.setInputTarget(this->normaled_target_point_cloud_.makeShared());

    nicp.setMaxCorrespondenceDistance(this->kCorrespondenceThreshold);
    nicp.setEuclideanFitnessEpsilon(this->kMSEThreshold);
    nicp.setTransformationEpsilon(this->kTransformationThreshold);
    nicp.setMaximumIterations(this->kIterationThreshold);

    pcl::PointCloud<pcl::PointXYZRGBNormal> temp_point_cloud;
    nicp.align(temp_point_cloud);

    // check whether converged
    try {
        // not, throw an error
        if (nicp.hasConverged() == false) {
            throw "Cannot converge.";
        }
        // converged, save result
        else {
            for (size_t i = 0; i < temp_point_cloud.size(); i++) {
                this->result_point_cloud_[i].x   = temp_point_cloud[i].x;
                this->result_point_cloud_[i].y   = temp_point_cloud[i].y;
                this->result_point_cloud_[i].z   = temp_point_cloud[i].z;
                this->result_point_cloud_[i].rgb = temp_point_cloud[i].rgb;
            }
        }
    }
    catch (const char* error_message) {
        std::cerr << "Alignment failed. " << error_message << " Source size " << this->GetSourcePointCloudSize() << ". Target size " << this->GetTargetPointCloudSize() << "." << std::endl;
        return 0;
    }

    // save tranformation_matrix_
    this->mean_squred_error_    = nicp.getFitnessScore();
    this->tranformation_matrix_ = nicp.getFinalTransformation() * this->tranformation_matrix_;
    return 1;
}