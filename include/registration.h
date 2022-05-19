/***
 * @Author: ChenRP07
 * @Date: 2022-04-28 15:31:32
 * @LastEditTime: 2022-04-28 16:01:58
 * @LastEditors: ChenRP07
 * @Description:
 */

#ifndef LIB_REGISTRATION_H
#define LIB_REGISTRATION_H

#include <mutex>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <thread>

namespace pco {
namespace operation {
    // return a + b
    extern pcl::PointXYZ    PointAddAssign(const pcl::PointXYZ&, const pcl::PointXYZ&);
    extern pcl::PointXYZ    PointAddAssign(const pcl::PointXYZ&, const pcl::PointXYZRGB&);
    extern pcl::PointXYZRGB PointAddAssign(const pcl::PointXYZRGB&, const pcl::PointXYZ&);
    extern pcl::PointXYZRGB PointAddAssign(const pcl::PointXYZRGB&, const pcl::PointXYZRGB&);

    // a += b
    extern void PointAddCopy(pcl::PointXYZ&, const pcl::PointXYZ&);
    extern void PointAddCopy(pcl::PointXYZ&, const pcl::PointXYZRGB&);
    extern void PointAddCopy(pcl::PointXYZRGB&, const pcl::PointXYZ&);
    extern void PointAddCopy(pcl::PointXYZRGB&, const pcl::PointXYZRGB&);

    // return a - b
    extern pcl::PointXYZ    PointSubAssign(const pcl::PointXYZ&, const pcl::PointXYZ&);
    extern pcl::PointXYZ    PointSubAssign(const pcl::PointXYZ&, const pcl::PointXYZRGB&);
    extern pcl::PointXYZRGB PointSubAssign(const pcl::PointXYZRGB&, const pcl::PointXYZ&);
    extern pcl::PointXYZRGB PointSubAssign(const pcl::PointXYZRGB&, const pcl::PointXYZRGB&);

    // a -= b
    extern void PointSubCopy(pcl::PointXYZ&, const pcl::PointXYZ&);
    extern void PointSubCopy(pcl::PointXYZ&, const pcl::PointXYZRGB&);
    extern void PointSubCopy(pcl::PointXYZRGB&, const pcl::PointXYZ&);
    extern void PointSubCopy(pcl::PointXYZRGB&, const pcl::PointXYZRGB&);

    // return a(:3) + b
    extern Eigen::Matrix4f MatrixAddAssign(const Eigen::Matrix4f&, const pcl::PointXYZ&);
    extern Eigen::Matrix4f MatrixAddAssign(const Eigen::Matrix4f&, const pcl::PointXYZRGB&);

    // a(:3) += b
    extern void MatrixAddCopy(Eigen::Matrix4f&, const pcl::PointXYZ&);
    extern void MatrixAddCopy(Eigen::Matrix4f&, const pcl::PointXYZRGB&);

    // a = b * a
    extern void PointCloudMul(pcl::PointCloud<pcl::PointXYZRGB>&, const Eigen::Matrix4f&);
    extern void PointCloudMulAdd(pcl::PointCloud<pcl::PointXYZRGB>&, const pcl::PointCloud<pcl::PointXYZRGB>&, const Eigen::Matrix4f&);

    // a += b
    extern void PointCloudAdd(pcl::PointCloud<pcl::PointXYZRGB>&, const pcl::PointXYZ&);
    extern void PointCloudAdd(pcl::PointCloud<pcl::PointXYZRGB>&, const pcl::PointXYZRGB&);

}  // namespace operation

namespace registration {
    // base class
    class regist_base {
    protected:
        pcl::PointCloud<pcl::PointXYZRGB> source_point_cloud_;    // point cloud to be transformed
        pcl::PointCloud<pcl::PointXYZRGB> target_point_cloud_;    // point cloud to be aligned with
        pcl::PointCloud<pcl::PointXYZRGB> result_point_cloud_;    // align result
        Eigen::Matrix4f                   tranformation_matrix_;  // transformation matrix
        float                             mean_squred_error_;     // align score

    public:
        regist_base();
        virtual bool align(const int = 0) = 0;  // virtual function
        float        GetMSE() const;            // get mse
        // set source point cloud
        void SetSourcePointCloudSwap(pcl::PointCloud<pcl::PointXYZRGB>&);
        void SetSourcePointCloudCopy(const pcl::PointCloud<pcl::PointXYZRGB>&);
        // set target point cloud
        void            SetTargetPointCloudSwap(pcl::PointCloud<pcl::PointXYZRGB>&);
        void            SetTargetPointCloudCopy(const pcl::PointCloud<pcl::PointXYZRGB>&);
        Eigen::Matrix4f GetMotionVector() const;  // get tranformation matrix
        // get source point cloud
        void GetSourcePointCloudSwap(pcl::PointCloud<pcl::PointXYZRGB>&);
        void GetSourcePointCloudCopy(pcl::PointCloud<pcl::PointXYZRGB>&) const;
        // get target point cloud
        void GetTargetPointCloudSwap(pcl::PointCloud<pcl::PointXYZRGB>&);
        void GetTargetPointCloudCopy(pcl::PointCloud<pcl::PointXYZRGB>&) const;
        // get point cloud size
        size_t GetSourcePointCloudSize() const;
        size_t GetTargetPointCloudSize() const;
        // centroid alignment
        void GolbalCentroidAlignment();
        void LocalCentroidAlignment();
    };

    // icp
    class ICP : public regist_base {
    private:
        const float kCorrespondenceThreshold;  // max correspondence distance
        const float kMSEThreshold;             // max mse difference between two iteration
        const float kTransformationThreshold;  // max matrix difference between two iteration
        const int   kIterationThreshold;       // max iteration

    public:
        ICP(const float, const int, const float = 0.01f, const float = 1e-6);
        bool align(const int = 0);
    };

    class NICP : public regist_base {
    private:
        const float                             kCorrespondenceThreshold;      // max correspondence distance
        const float                             kMSEThreshold;                 // max mse difference between two iteration
        const float                             kTransformationThreshold;      // max matrix difference between two iteration
        const int                               kIterationThreshold;           // max matrix difference between two iteration
        const float                             kNormalRadius;                 // max iteration
        pcl::PointCloud<pcl::PointXYZRGBNormal> normaled_result_point_cloud_;  // result point cloud with normals
        pcl::PointCloud<pcl::PointXYZRGBNormal> normaled_target_point_cloud_;  // target point cloud with normals

    protected:
        void ComputeNormals();

    public:
        NICP(const float, const int, const float = 10.0f, const float = 0.01f, const float = 1e-6);
        bool align(const int = 0);
    };

    class ParallelICP {
    private:
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>> source_patches_;
        pcl::PointCloud<pcl::PointXYZRGB>              target_point_cloud_;
        const float                                    kCorrespondenceThreshold;  // max correspondence distance
        const float                                    kMSEThreshold;             // max mse difference between two iteration
        const float                                    kTransformationThreshold;  // max matrix difference between two iteration
        const int                                      kIterationThreshold;       // max iteration
        const int                                      kThreads;                  // thread number
        std::vector<int>                               target_patch_index_;
        std::vector<Eigen::Matrix4f>                   motion_vectors_;
        std::vector<float>                             mean_squred_errors_;
        std::queue<size_t>                             task_pool_;
        std::mutex                                     task_pool_mutex_;
        std::mutex                                     IO_mutex_;
        bool                                           indexed = false;

    public:
        ParallelICP(const int, const float, const int, const float = 0.01f, const float = 1e-6);
        void ThreadProcess();
        void ParallelAlign();
        void SetSourcePatchesCopy(std::vector<pcl::PointCloud<pcl::PointXYZRGB>>&);
        void SetSourcePatchesSwap(std::vector<pcl::PointCloud<pcl::PointXYZRGB>>&);
        void SetTargetPointCloudCopy(pcl::PointCloud<pcl::PointXYZRGB>&);
        void SetTargetPointCloudSwap(pcl::PointCloud<pcl::PointXYZRGB>&);
        void GetTargetPatches(std::vector<pcl::PointCloud<pcl::PointXYZRGB>>&, std::vector<Eigen::Matrix4f>&);
        void GetTargetMSEs(std::vector<float>&) const;
    };
}  // namespace registration
}  // namespace pco
#endif
