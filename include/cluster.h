/***
 * @Author: ChenRP07
 * @Date: 2022-04-24 21:33:23
 * @LastEditTime: 2022-04-25 10:32:06
 * @LastEditors: ChenRP07
 * @Description: C++ Header for point cloud patch generating -- some clustering
 * methods.
 */

#ifndef LIB_CLUSTER_H
#define LIB_CLUSTER_H

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <fstream>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <queue>
#include <regex>
#include <string>
#include <sys/types.h>
#include <vector>

namespace pco {

namespace io {
    extern int LoadColorPlyFile(const std::string&, pcl::PointCloud<pcl::PointXYZRGB>&);  // load colored point cloud from *.ply file

    extern int SaveColorPlyFile(const std::string&, const pcl::PointCloud<pcl::PointXYZRGB>&, const bool = false);  // save colored point cloud to *.ply file

    extern int SaveUniqueColorPlyFile(const std::string&, const pcl::PointCloud<pcl::PointXYZRGB>&, unsigned int = 0x000000,
                                      const bool = false);  // save colored point cloud to *.ply file using unique color
}  // namespace io

/*
 * class Patch : base for the different clustering method.
 */
class Patch {
protected:
    pcl::PointCloud<pcl::PointXYZRGB> point_cloud_;  // unsplit point cloud
    std::vector<int>                  point_index_;  // patch index for each point
public:
    Patch();                                                                   // constructor
    void         SetPointCloudCopy(const pcl::PointCloud<pcl::PointXYZRGB>&);  // copy point cloud
    void         SetPointCloudSwap(pcl::PointCloud<pcl::PointXYZRGB>&);        // swap point cloud
    void         GetPointCloud(pcl::PointCloud<pcl::PointXYZRGB>&);            // output point cloud
    int          GetPointIndex(std::vector<int>&);                             // output point index
    int          GetPatches(std::vector<pcl::PointCloud<pcl::PointXYZRGB>>&);  // output patches
    virtual void Clustering() = 0;
};

/* simple method -- constant clustering */
class SimplePatch : public Patch {
private:
    int patch_number_;

public:
    SimplePatch();
    SimplePatch(int);
    void Clustering();
};

}  // namespace pco
#endif