#include <iostream>
#include <fstream>
#include <stdio.h>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>

#include <pcl/common/common_headers.h>
#include <pcl/conversions.h>
#include <pcl/point_types_conversion.h>

#include <pcl/filters/passthrough.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/crop_box.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/features/vfh.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <pcl_ros/point_cloud.h>

using namespace std;
using namespace cv;

#define PI 3.14

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

tf::StampedTransform model_tf;
pcl::PCLPointCloud2 sceneCloud;
pcl::PCLPointCloud2 modelCloud;

void getAffineTransform(tf::StampedTransform& TFtransform, Eigen::Affine3f& Afftransform)
{
    Afftransform.translation() << (float)TFtransform.getOrigin()[0], 
                                  (float)TFtransform.getOrigin()[1], 
                                  (float)TFtransform.getOrigin()[2];

    Afftransform.rotate (Eigen::Quaternionf((float)TFtransform.getRotation().getW(),
                        (float)TFtransform.getRotation()[0], 
                        (float)TFtransform.getRotation()[1],
                        (float)TFtransform.getRotation()[2]));
}

void changeRPY(tf::StampedTransform& transform, std::string mode, double change)
{
  double roll, pitch, yaw;
  tf::Matrix3x3 m_rotation(transform.getRotation());
  m_rotation.getRPY(roll,pitch,yaw);
  tf::Quaternion q_rotation;
  change = change*PI/180.0;
  std::cout<<"Changing "<<mode<<" by "<<change<<std::endl;
  std::cout<<"roll : "<<roll<<" pitch : "<<pitch<<" yaw: "<<yaw<<std::endl;
  if(strcmp(mode.c_str(),"roll") == 0)
  {
    roll += change;
    m_rotation.setRPY(roll, pitch, yaw);
    m_rotation.getRotation(q_rotation);

    transform.setRotation(q_rotation);
  }

  else if(strcmp(mode.c_str(),"pitch") == 0)
  {
    pitch += change;
    m_rotation.setRPY(roll, pitch, yaw);
    m_rotation.getRotation(q_rotation);

    transform.setRotation(q_rotation);
  }

  else if(strcmp(mode.c_str(),"yaw") == 0)
  {
    yaw += change;
    m_rotation.setRPY(roll, pitch, yaw);
    m_rotation.getRotation(q_rotation);

    transform.setRotation(q_rotation);
  }
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getKeySym () == "r" && event.keyDown ())
    model_tf.getOrigin()[0] -= 0.01;
  else if (event.getKeySym () == "R" && event.keyDown ())
    model_tf.getOrigin()[0] += 0.01;
  else if (event.getKeySym () == "t" && event.keyDown ())
    model_tf.getOrigin()[1] -= 0.01;
  else if (event.getKeySym () == "T" && event.keyDown ())
    model_tf.getOrigin()[1] += 0.01;
  else if (event.getKeySym () == "y" && event.keyDown ())
    model_tf.getOrigin()[2] -= 0.01;
  else if (event.getKeySym () == "Y" && event.keyDown ())
    model_tf.getOrigin()[2] += 0.01;
  else if (event.getKeySym () == "h" && event.keyDown ())
    changeRPY(model_tf,"roll",2);
  else if (event.getKeySym () == "H" && event.keyDown ())
    changeRPY(model_tf,"roll",-2);
  else if (event.getKeySym () == "j" && event.keyDown ())
    changeRPY(model_tf,"pitch",2);
  else if (event.getKeySym () == "J" && event.keyDown ())
    changeRPY(model_tf,"pitch",-2);
  else if (event.getKeySym () == "k" && event.keyDown ())
    changeRPY(model_tf,"yaw",2);
  else if (event.getKeySym () == "K" && event.keyDown ())
    changeRPY(model_tf,"yaw",-2);

  PointCloudRGB::Ptr sceneCloudPtr(new PointCloudRGB);
  PointCloudRGB::Ptr modelCloudPtr(new PointCloudRGB);
  pcl::fromPCLPointCloud2(sceneCloud,*sceneCloudPtr);
  pcl::fromPCLPointCloud2(modelCloud,*modelCloudPtr);

  PointCloudRGB::Ptr combinedCloud(new PointCloudRGB);
  *combinedCloud = *sceneCloudPtr;

  // Transform model point cloud and combine
  Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
  getAffineTransform(model_tf, transform_1);
  PointCloudRGB::Ptr transformed_cloud (new PointCloudRGB);
  pcl::transformPointCloud (*modelCloudPtr, *transformed_cloud, transform_1);
  *combinedCloud += *transformed_cloud;

  viewer->removePointCloud("CombinedCloud");
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(combinedCloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (combinedCloud, rgb, "CombinedCloud"); 
}

int main( int ac, char* av[] ) {
    std::string node_name;

    ros::init(ac, av, node_name);
    ros::NodeHandle main_node_handle;

    if(ac > 2){
      pcl::io::loadPLYFile (std::string(av[1]), sceneCloud);
      pcl::io::loadPLYFile (std::string(av[2]), modelCloud);
    }
    else{
      std::cout<<"Enter the scene and model point clouds as input "<<std::endl;
      exit(-1);
    }

    model_tf.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    model_tf.setRotation( tf::Quaternion(0, 0, 0, 1) );

    std::cout<<"Received PointClouds !"<<std::endl;

    PointCloudRGB::Ptr sceneCloudPtr(new PointCloudRGB);
    pcl::fromPCLPointCloud2(sceneCloud,*sceneCloudPtr);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "CombinedCloud");
    viewer->initCameraParameters ();

    // Add the first cloud as dummy
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(sceneCloudPtr);
    viewer->addPointCloud<pcl::PointXYZRGB> (sceneCloudPtr, rgb, "CombinedCloud");
    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());

    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    FILE* fp = fopen("cam.info.txt", "w");
    for (int i = 0; i < 3; ++i)
        fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t %15.8e\t\n", (float)(model_tf.getBasis()[i][0]), 
                                                              (float)(model_tf.getBasis()[i][1]), 
                                                              (float)(model_tf.getBasis()[i][2]), 
                                                              (float)(model_tf.getOrigin()[i]));
    fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t %15.8e\t\n", 0.0f, 0.0f, 0.0f, 1.0f);
    fclose(fp);

    ros::shutdown();
    return 0;
}
