// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/range_image/range_image.h>
#include <pcl/point_types.h>
#include "pcd_custom_types.h"

// C++ includes 
#include <iostream>
#include <iterator>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include <unistd.h>


void func() {

  // Generate the data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
  for (float y=-0.5f; y<=0.5f; y+=0.01f) {
    for (float z=-0.5f; z<=0.5f; z+=0.01f) {
      pcl::PointXYZ point;
      //velodyne_pointcloud::PointXYZIR point;
      point.x = 2.0f - y;
      point.y = y;
      point.z = z;
      cloud->push_back(point);
    }
  }

  std::cout << "cloud size: " << cloud->size() << std::endl;

  float noise_level = 0.0; // noise already added on the gazebo simulation.
  float min_range = 0;
  int border_size = 0;

  Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
  Eigen::Quaternion<float> testOrientation(1, 0, 0, 0);
  scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(cloud->sensor_origin_[0],
          cloud->sensor_origin_[1],
          cloud->sensor_origin_[2])) *
      Eigen::Affine3f(testOrientation);

  pcl::RangeImage::Ptr range_image_ptr(new pcl::RangeImage);
  // range_image_ptr.reset();

  pcl::RangeImage &range_image = *range_image_ptr;
  float angular_resolution_x = 0.00174533;
  float angular_resolution_y = 0.00174533;
  range_image.createFromPointCloud(*cloud, angular_resolution_x, angular_resolution_y,
      pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
      scene_sensor_pose, pcl::RangeImage::LASER_FRAME, noise_level, min_range, border_size);

  int height = range_image.height;
  int width = range_image.width;
  std::cout << "2D Image (H, W) = " << height << ", " << width << std::endl;

  std::cout << "END FUNC" << std::endl;
  // std::cout << noise_level << std::endl;
  // std::cout << min_range << std::endl;
  // std::cout << border_size << std::endl;
  //  std::cout << scene_sensor_pose.matrix() << std::endl;
  // std::cout << testOrientation.matrix() << std::endl;
  // std::cout << cloud->sensor_origin_ << std::endl;
  // std::cout << angular_resolution_x << std::endl;
  // std::cout << angular_resolution_y << std::endl;

  // std::cout << "END FUNC" << std::endl;
}


int main(int argc, char ** argv) {

  func();

  std::cout << "END MAIN" << std::endl;
}

// ISSUES:
/* When gtsam is not linked to executable, range_image performs as expected and creates (height, width)=353x300 image.
 * When gtsam is lined to the executable, range_image not created due to "double free or corruption (out) Aborted (core dumped)" error. Initialized range_image of (height, width)=(0, 0) size is retained
 * Issue occurs when range_image goes out of scope. That is, either when program exits func() and range_image gets deleted OR when range_image is manually deleted, there is a "double free or corruption (out) Aborted (core dumped)". 
 * Again, all these issues only happen if gtsam is linked to the given library. Else range_image and entire code works perfectly fine. I'm not sure how double deletion of range_image or memory overwriting of range_image happens just because gtsam is linked with the executable.
 * Also, same issue occur if pcl::RangeImage is used directly rather than using its  pcl::RangeImage::Ptr version. So the issue has nothing to do with how the range_image pointer is being created.
 */
