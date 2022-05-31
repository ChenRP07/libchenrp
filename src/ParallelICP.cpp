/***
 * @Author: ChenRP07
 * @Date: 2022-05-07 15:04:27
 * @LastEditTime: 2022-05-31 16:41:33
 * @LastEditors: ChenRP07
 * @Description:
 */
#include "registration.h"

using namespace pco::registration;

/***
 * @description: constructor
 * @param {int} thread_num
 * @param {float} max_correspondence
 * @param {int} max_iteration
 * @param {float} max_mse_difference
 * @param {float} max_tranformation_diference
 * @return {*}
 */
ParallelICP::ParallelICP(const int thread_num, const float max_correspondence, const int max_iteration, const float max_mse_difference, const float max_tranformation_diference)
    : kThreads(thread_num), kCorrespondenceThreshold{max_correspondence}, kIterationThreshold{max_iteration}, kMSEThreshold{max_mse_difference}, kTransformationThreshold{max_tranformation_diference} {
}

/***
 * @description: set source point cloud patches, using swap
 * @param {vector<PointCloud>&} patches
 * @return {*}
 */
void ParallelICP::SetSourcePatchesSwap(std::vector<pcl::PointCloud<pcl::PointXYZRGB>>& patches) {
	// resize containers
	this->source_patches_.resize(patches.size());
	this->motion_vectors_.resize(patches.size());
	this->mean_squred_errors_.resize(patches.size());

	// swap patches to source_patches_
	for (size_t i = 0; i < patches.size(); i++) {
		this->source_patches_[i].swap(patches[i]);
	}
}

/***
 * @description: set source point cloud patch copy
 * @param {vector<PointCloud>&} patches
 * @return {*}
 */
void ParallelICP::SetSourcePatchesCopy(std::vector<pcl::PointCloud<pcl::PointXYZRGB>>& patches) {
	// resize containers
	this->source_patches_.resize(patches.size());
	this->motion_vectors_.resize(patches.size());
	this->mean_squred_errors_.resize(patches.size());

	// copy patches to source_patches_
	for (size_t i = 0; i < patches.size(); i++) {
		for (size_t j = 0; j < patches[i].size(); j++) {
			this->source_patches_[i].push_back(patches[i][j]);
		}
	}
}

/***
 * @description: set target point cloud, using copy
 * @param {PointCloud&} point_cloud
 * @return {*}
 */
void ParallelICP::SetTargetPointCloudCopy(pcl::PointCloud<pcl::PointXYZRGB>& point_cloud) {
	// resize containers
	this->target_point_cloud_.resize(point_cloud.size());
	// copy point cloud
	for (size_t i = 0; i < point_cloud.size(); i++) {
		this->target_point_cloud_[i] = point_cloud[i];
	}
}

/***
 * @description: set target point cloud, using swap
 * @param {PointCloud&} point_cloud
 * @return {*}
 */
void ParallelICP::SetTargetPointCloudSwap(pcl::PointCloud<pcl::PointXYZRGB>& point_cloud) {
	// swap point cloud
	this->target_point_cloud_.swap(point_cloud);
}

/***
 * @description: thread proccess function
 * @param {*}
 * @return {*}
 */
void ParallelICP::ThreadProcess() {
	while (true) {
		// get task from pool
		int task_index = -1;
		this->task_pool_mutex_.lock();
		if (!this->task_pool_.empty()) {
			task_index = this->task_pool_.front();
			this->task_pool_.pop();
		}
		this->task_pool_mutex_.unlock();
		// all task has done
		if (task_index == -1)
			return;
		else {
			// do icp
			try {
				ICP task(this->kCorrespondenceThreshold, this->kIterationThreshold, this->kMSEThreshold, this->kTransformationThreshold);
				task.SetSourcePointCloudCopy(this->source_patches_[task_index]);
				task.SetTargetPointCloudCopy(this->target_point_cloud_);
				bool converge                         = task.align(2);
				this->motion_vectors_[task_index]     = task.GetMotionVector();
				this->mean_squred_errors_[task_index] = task.GetMSE();
				if (converge == false)
					throw "ICP is not converged";
			}
			catch (const char* error_message) {
				IO_mutex_.lock();
				std::cerr << "Patch #" << task_index << " registration error : " << error_message << std::endl;
				IO_mutex_.unlock();
			}
		}
	}
}

/***
 * @description: parallel local icp registration
 * @param {*}
 * @return {*}
 */
void ParallelICP::ParallelAlign() {
	// global icp
	// collect whole i-frame
	// pcl::PointCloud<pcl::PointXYZRGB> __i_frame;
	// for (size_t i = 0; i < this->source_patches_.size(); i++) {
	// 	for (size_t j = 0; j < this->source_patches_[i].size(); j++) {
	// 		__i_frame.emplace_back(this->source_patches_[i][j]);
	// 	}
	// }
	// size_t i_size = __i_frame.size();

	// // do icp
	// ICP __global(this->kCorrespondenceThreshold, this->kIterationThreshold, this->kMSEThreshold, this->kTransformationThreshold);
	// __global.SetTargetPointCloudSwap(__i_frame);
	// __global.SetSourcePointCloudSwap(this->target_point_cloud_);

	// bool converge        = __global.align();
	// this->global_vector_ = __global.GetMotionVector();
	// float error          = __global.GetMSE();

	// // get result
	// __global.GetResultPointCloudSwap(this->target_point_cloud_);
	// std::cout << "Global alignment";
	// if (!converge) {
	// 	std::cout << " not";
	// }
	// std::cout << " converged, MSE is " << error << ", Target size is " << i_size << ", Source size is " << this->target_point_cloud_.size() << std::endl;

	// add task to pool
	for (size_t i = 0; i < this->source_patches_.size(); i++) {
		this->task_pool_.push(i);
	}

	// kThreads thread
	std::thread ICP_threads[kThreads];
	// create threads
	for (int i = 0; i < kThreads; i++) {
		ICP_threads[i] = std::thread(&ParallelICP::ThreadProcess, this);
	}
	// wait for all threads completing
	for (auto& th : ICP_threads) {
		th.join();
	}

	// information of mse, avg-dev-min-max
	float MSE_sum   = std::accumulate(this->mean_squred_errors_.begin(), this->mean_squred_errors_.end(), 0);
	float MSE_avg   = MSE_sum / this->mean_squred_errors_.size();
	float MSE_accum = 0.0;
	std::for_each(this->mean_squred_errors_.begin(), this->mean_squred_errors_.end(), [&](const float d) { MSE_accum += std::pow(d - MSE_avg, 2); });
	float MSE_dev = std::sqrt(MSE_accum / (this->mean_squred_errors_.size() - 1));

	std::cout << "Local alignment complete, using " << this->kThreads << " threads." << std::endl;
	std::cout << "MSE of " << this->mean_squred_errors_.size() << " patches : ";
	std::cout << "avg is " << MSE_avg << ", dev is " << MSE_dev << ", min is " << *std::min_element(this->mean_squred_errors_.begin(), this->mean_squred_errors_.end()) << ", max is "
	          << *std::max_element(this->mean_squred_errors_.begin(), this->mean_squred_errors_.end()) << std::endl;
}

/***
 * @description: split targe point cloud into patches according to the nearest neighbor in source patches,
                 transform the split patches and record matrices and patches.
 * @param {vector<PointCloud>&} patches
 * @param {vector<Matrix4f>&} matrices
 * @return {*}
 */
void ParallelICP::GetTargetPatches(std::vector<pcl::PointCloud<pcl::PointXYZRGB>>& patches, std::vector<Eigen::Matrix4f>& matrices) {
	try {
		// target patches are split already, output directly
		if (indexed && (this->target_patch_index_.size() == this->target_point_cloud_.size())) {
			patches.resize(this->source_patches_.size());
			matrices.resize(this->motion_vectors_.size());
			for (size_t i = 0; i < this->target_patch_index_.size(); i++) {
				patches[this->target_patch_index_[i]].push_back(this->target_point_cloud_[i]);
			}

			// tranform the patch and record the matrix, make the target patch align to the related source patch
			for (size_t i = 0; i < this->motion_vectors_.size(); i++) {
				pco::operation::PointCloudMul(patches[i], this->motion_vectors_[i].inverse());
				matrices[i] = this->motion_vectors_[i].inverse() * this->global_vector_;
			}
		}
		// target patches are not split already, split them
		else if (!indexed) {
			// transform the source patches
			pcl::PointCloud<pcl::PointXYZRGB> transformed_source;
			std::vector<size_t>               source_index;
			if (this->source_patches_.size() != this->motion_vectors_.size()) {
				throw "Patch size and Matrix size don't match.";
			}
			for (size_t i = 0; i < this->source_patches_.size(); i++) {
				pco::operation::PointCloudMulAdd(transformed_source, this->source_patches_[i], this->motion_vectors_[i]);
				for (size_t j = 0; j < this->source_patches_[i].size(); j++) {
					source_index.emplace_back(i);
				}
			}
			// form a kdtree and search nearest neighbor for target points
			pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
			kdtree.setInputCloud(transformed_source.makeShared());

			this->target_patch_index_.resize(this->target_point_cloud_.size());

			for (size_t i = 0; i < this->target_point_cloud_.size(); i++) {
				// do 1-nn search
				std::vector<int>   __index(1);
				std::vector<float> __distance(1);
				kdtree.nearestKSearch(this->target_point_cloud_[i], 1, __index, __distance);

				// add it into related patch
				this->target_patch_index_[i] = source_index[__index[0]];
			}

			// split
			patches.resize(this->source_patches_.size());
			matrices.resize(this->motion_vectors_.size());

			for (size_t i = 0; i < this->target_patch_index_.size(); i++) {
				patches[this->target_patch_index_[i]].push_back(this->target_point_cloud_[i]);
			}

			// tranform the patch and record the matrix, make the target patch align to the related source patch
			for (size_t i = 0; i < this->motion_vectors_.size(); i++) {
				pco::operation::PointCloudMul(patches[i], this->motion_vectors_[i].inverse());
				matrices[i] = this->motion_vectors_[i].inverse() * this->global_vector_;
			}

			this->indexed = true;
		}
		else {
			throw "Target patches split error.";
		}
	}
	catch (const char* error_message) {
		std::cerr << "ParallelICP failed : " << error_message << std::endl;
	}
}

/***
 * @description: get patches mses
 * @param {vector<float>&} mses
 * @return {*}
 */
void ParallelICP::GetTargetMSEs(std::vector<float>& mses) const {
	try {
		if (this->mean_squred_errors_.size() != this->source_patches_.size())
			throw "Target patches mse not match.";
		else {
			mses.resize(this->mean_squred_errors_.size());
			for (size_t i = 0; i < this->mean_squred_errors_.size(); i++) {
				mses[i] = this->mean_squred_errors_[i];
			}
		}
	}
	catch (const char* error_message) {
		std::cerr << "ParallelICP failed : " << error_message << std::endl;
	}
}