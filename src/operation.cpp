/***
 * @Author: ChenRP07
 * @Date: 2022-04-29 10:42:41
 * @LastEditTime: 2022-05-31 16:07:09
 * @LastEditors: ChenRP07
 * @Description:
 */

#include "registration.h"

using namespace pco;

/***
 * @description: __result = __a + __b
 * @param {PointXYZ&} __a
 * @param {PointXYZ&} __b
 * @return {__result}
 */
pcl::PointXYZ operation::PointAddAssign(const pcl::PointXYZ& __a, const pcl::PointXYZ& __b) {
	pcl::PointXYZ __result{__a.x + __b.x, __a.y + __b.y, __a.z + __b.z};
	return __result;
}

/***
 * @description: __a += __b
 * @param {PointXYZ&} __a
 * @param {PointXYZ&} __b
 * @return {*}
 */
void operation::PointAddCopy(pcl::PointXYZ& __a, const pcl::PointXYZ& __b) {
	__a.x += __b.x, __a.y += __b.y, __a.z += __b.z;
}

/***
 * @description: __result = __a + __b.geometry
 * @param {PointXYZ&} __a
 * @param {PointXYZRGB&} __b
 * @return {__result}
 */
pcl::PointXYZ operation::PointAddAssign(const pcl::PointXYZ& __a, const pcl::PointXYZRGB& __b) {
	pcl::PointXYZ __result{__a.x + __b.x, __a.y + __b.y, __a.z + __b.z};
	return __result;
}

/***
 * @description: __a += __b.geometry
 * @param {PointXYZ&} __a
 * @param {PointXYZRGB&} __b
 * @return {*}
 */
void operation::PointAddCopy(pcl::PointXYZ& __a, const pcl::PointXYZRGB& __b) {
	__a.x += __b.x, __a.y += __b.y, __a.z += __b.z;
}

/***
 * @description: __result.geometry = __a.geometry + __b, __result.color = _a.color
 * @param {PointXYZRGB&} __a
 * @param {PointXYZ&} __b
 * @return {__result}
 */
pcl::PointXYZRGB operation::PointAddAssign(const pcl::PointXYZRGB& __a, const pcl::PointXYZ& __b) {
	pcl::PointXYZRGB __result{__a};
	__result.x += __b.x, __result.y += __b.y, __result.z += __b.z;
	return __result;
}

/***
 * @description: __a.geometry += _b
 * @param {PointXYZRGB&} __a
 * @param {PointXYZ&} __b
 * @return {*}
 */
void operation::PointAddCopy(pcl::PointXYZRGB& __a, const pcl::PointXYZ& __b) {
	__a.x += __b.x, __a.y += __b.y, __a.z += __b.z;
}

/***
 * @description: __result.geometry = __a.geometry + __b.geometry, __result.color = __a.color
 * @param {PointXYZRGB&} __a
 * @param {PointXYZRGB&} __b
 * @return {__result}
 */
pcl::PointXYZRGB operation::PointAddAssign(const pcl::PointXYZRGB& __a, const pcl::PointXYZRGB& __b) {
	pcl::PointXYZRGB __result{__a};
	__result.x += __b.x, __result.y += __b.y, __result.z += __b.z;
	return __result;
}

/***
 * @description: __a.geometry += __b.geometry
 * @param {PointXYZRGB&} __a
 * @param {PointXYZRGB&} __b
 * @return {*}
 */
void operation::PointAddCopy(pcl::PointXYZRGB& __a, const pcl::PointXYZRGB& __b) {
	__a.x += __b.x, __a.y += __b.y, __a.z += __b.z;
}

/***
 * @description: __result = __a - __b
 * @param {PointXYZ&} __a
 * @param {PointXYZ&} __b
 * @return {__result}
 */
pcl::PointXYZ operation::PointSubAssign(const pcl::PointXYZ& __a, const pcl::PointXYZ& __b) {
	pcl::PointXYZ __result{__a.x - __b.x, __a.y - __b.y, __a.z - __b.z};
	return __result;
}

/***
 * @description: __a -= __b
 * @param {PointXYZ&} __a
 * @param {PointXYZ&} __b
 * @return {*}
 */
void operation::PointSubCopy(pcl::PointXYZ& __a, const pcl::PointXYZ& __b) {
	__a.x -= __b.x, __a.y -= __b.y, __a.z -= __b.z;
}

/***
 * @description: __result = __a - __b.geometry
 * @param {PointXYZ&} __a
 * @param {PointXYZRGB&} __b
 * @return {__result}
 */
pcl::PointXYZ operation::PointSubAssign(const pcl::PointXYZ& __a, const pcl::PointXYZRGB& __b) {
	pcl::PointXYZ __result{__a.x - __b.x, __a.y - __b.y, __a.z - __b.z};
	return __result;
}

/***
 * @description: __a -= __b.geometry
 * @param {PointXYZ&} __a
 * @param {PointXYZRGB&} __b
 * @return {*}
 */
void operation::PointSubCopy(pcl::PointXYZ& __a, const pcl::PointXYZRGB& __b) {
	__a.x -= __b.x, __a.y -= __b.y, __a.z -= __b.z;
}

/***
 * @description: __result.geometry = __a.geometry - __b, __result.color = _a.color
 * @param {PointXYZRGB&} __a
 * @param {PointXYZ&} __b
 * @return {__result}
 */
pcl::PointXYZRGB operation::PointSubAssign(const pcl::PointXYZRGB& __a, const pcl::PointXYZ& __b) {
	pcl::PointXYZRGB __result{__a};
	__result.x -= __b.x, __result.y -= __b.y, __result.z -= __b.z;
	return __result;
}

/***
 * @description: __a.geometry -= _b
 * @param {PointXYZRGB&} __a
 * @param {PointXYZ&} __b
 * @return {*}
 */
void operation::PointSubCopy(pcl::PointXYZRGB& __a, const pcl::PointXYZ& __b) {
	__a.x -= __b.x, __a.y -= __b.y, __a.z -= __b.z;
}

/***
 * @description: __result.geometry = __a.geometry - __b.geometry, __result.color = __a.color
 * @param {PointXYZRGB&} __a
 * @param {PointXYZRGB&} __b
 * @return {__result}
 */
pcl::PointXYZRGB operation::PointSubAssign(const pcl::PointXYZRGB& __a, const pcl::PointXYZRGB& __b) {
	pcl::PointXYZRGB __result{__a};
	__result.x -= __b.x, __result.y -= __b.y, __result.z -= __b.z;
	return __result;
}

/***
 * @description: __a.geometry -= __b.geometry
 * @param {PointXYZRGB&} __a
 * @param {PointXYZRGB&} __b
 * @return {*}
 */
void operation::PointSubCopy(pcl::PointXYZRGB& __a, const pcl::PointXYZRGB& __b) {
	__a.x -= __b.x, __a.y -= __b.y, __a.z -= __b.z;
}

void operation::PointDivCopy(pcl::PointXYZ& __a, const size_t __b) {
	__a.x /= __b, __a.y /= __b, __a.z /= __b;
}

void operation::PointDivCopy(pcl::PointXYZRGB& __a, const size_t __b) {
	__a.x /= __b, __a.y /= __b, __a.z /= __b;
}

/***
 * @description: __result = __matrix , __result(:3) += __vector
 * @param {Matrix4f&} __matrix
 * @param {PointXYZ&} __vector
 * @return {__result}
 */
Eigen::Matrix4f operation::MatrixAddAssign(const Eigen::Matrix4f& __matrix, const pcl::PointXYZ& __vector) {
	Eigen::Matrix4f __result{__matrix};
	__result(0, 3) += __vector.x;
	__result(1, 3) += __vector.y;
	__result(2, 3) += __vector.z;
	return __result;
}

/***
 * @description: __result = __matrix , __result(:3) += __vector
 * @param {Matrix4f&} __matrix
 * @param {PointXYZRGB&} __vector
 * @return {*}
 */
Eigen::Matrix4f operation::MatrixAddAssign(const Eigen::Matrix4f& __matrix, const pcl::PointXYZRGB& __vector) {
	Eigen::Matrix4f __result{__matrix};
	__result(0, 3) += __vector.x;
	__result(1, 3) += __vector.y;
	__result(2, 3) += __vector.z;
	return __result;
}

/***
 * @description: __matrix(:3) += __vector
 * @param {Matrix4f&} __matrix
 * @param {PointXYZ&} __vector
 * @return {*}
 */
void operation::MatrixAddCopy(Eigen::Matrix4f& __matrix, const pcl::PointXYZ& __vector) {
	__matrix(0, 3) += __vector.x;
	__matrix(1, 3) += __vector.y;
	__matrix(2, 3) += __vector.z;
}

/***
 * @description: __matrix(:3) += __vector
 * @param {Matrix4f&} __matrix
 * @param {PointXYZRGB&} __vector
 * @return {*}
 */
void operation::MatrixAddCopy(Eigen::Matrix4f& __matrix, const pcl::PointXYZRGB& __vector) {
	__matrix(0, 3) += __vector.x;
	__matrix(1, 3) += __vector.y;
	__matrix(2, 3) += __vector.z;
}

/***
 * @description: __point_cloud = __matrix * __point_cloud
 * @param {PointCloud<PointXYZRGB>} __point_cloud
 * @param {Matrix4f&} __matrix
 * @return {*}
 */
void operation::PointCloudMul(pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud, const Eigen::Matrix4f& __matrix) {
	for (size_t i = 0; i < __point_cloud.size(); i++) {
		pcl::PointXYZRGB temp{__point_cloud[i]};
		temp.x           = __point_cloud[i].x * __matrix(0, 0) + __point_cloud[i].y * __matrix(0, 1) + __point_cloud[i].z * __matrix(0, 2) + __matrix(0, 3);
		temp.y           = __point_cloud[i].x * __matrix(1, 0) + __point_cloud[i].y * __matrix(1, 1) + __point_cloud[i].z * __matrix(1, 2) + __matrix(1, 3);
		temp.z           = __point_cloud[i].x * __matrix(2, 0) + __point_cloud[i].y * __matrix(2, 1) + __point_cloud[i].z * __matrix(2, 2) + __matrix(2, 3);
		__point_cloud[i] = temp;
	}
}

/***
 * @description: __point_cloud_x += __matrix * __point_cloud_y
 * @param {PointCloud} __point_cloud_x
 * @param {PointCloud} __point_cloud_y
 * @param {Matrix4f&} __matrix
 * @return {*}
 */
void operation::PointCloudMulAdd(pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud_x, const pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud_y, const Eigen::Matrix4f& __matrix) {
	for (size_t i = 0; i < __point_cloud_y.size(); i++) {
		pcl::PointXYZRGB temp{__point_cloud_y[i]};
		temp.x = __point_cloud_y[i].x * __matrix(0, 0) + __point_cloud_y[i].y * __matrix(0, 1) + __point_cloud_y[i].z * __matrix(0, 2) + __matrix(0, 3);
		temp.y = __point_cloud_y[i].x * __matrix(1, 0) + __point_cloud_y[i].y * __matrix(1, 1) + __point_cloud_y[i].z * __matrix(1, 2) + __matrix(1, 3);
		temp.z = __point_cloud_y[i].x * __matrix(2, 0) + __point_cloud_y[i].y * __matrix(2, 1) + __point_cloud_y[i].z * __matrix(2, 2) + __matrix(2, 3);
		__point_cloud_x.push_back(temp);
	}
}

/***
 * @description: __point_cloud[i] += __center
 * @param {PointCloud<PointXYZRGB>} __point_cloud
 * @param {PointXYZ&} __center
 * @return {*}
 */
void operation::PointCloudAdd(pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud, const pcl::PointXYZ& __center) {
	for (size_t i = 0; i < __point_cloud.size(); i++) {
		operation::PointAddCopy(__point_cloud[i], __center);
	}
}

/***
 * @description: __point_cloud[i] += __center
 * @param {PointCloud<PointXYZRGB>} __point_cloud
 * @param {PointXYZRGB&} __center
 * @return {*}
 */
void operation::PointCloudAdd(pcl::PointCloud<pcl::PointXYZRGB>& __point_cloud, const pcl::PointXYZRGB& __center) {
	for (size_t i = 0; i < __point_cloud.size(); i++) {
		operation::PointAddCopy(__point_cloud[i], __center);
	}
}
