#include <iostream>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <stdlib.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>

#include <ros/package.h>

#define PI 3.14

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

tf::StampedTransform model_tf;
PointCloudRGB::Ptr sceneCloud;
PointCloudRGB::Ptr modelCloud;
double rIncr;
double tIncr;

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
    model_tf.getOrigin()[0] -= tIncr;
  else if (event.getKeySym () == "R" && event.keyDown ())
    model_tf.getOrigin()[0] += tIncr;
  else if (event.getKeySym () == "t" && event.keyDown ())
    model_tf.getOrigin()[1] -= tIncr;
  else if (event.getKeySym () == "T" && event.keyDown ())
    model_tf.getOrigin()[1] += tIncr;
  else if (event.getKeySym () == "y" && event.keyDown ())
    model_tf.getOrigin()[2] -= tIncr;
  else if (event.getKeySym () == "Y" && event.keyDown ())
    model_tf.getOrigin()[2] += tIncr;
  else if (event.getKeySym () == "b" && event.keyDown ())
    changeRPY(model_tf,"roll",rIncr);
  else if (event.getKeySym () == "B" && event.keyDown ())
    changeRPY(model_tf,"roll",-rIncr);
  else if (event.getKeySym () == "n" && event.keyDown ())
    changeRPY(model_tf,"pitch",rIncr);
  else if (event.getKeySym () == "N" && event.keyDown ())
    changeRPY(model_tf,"pitch",-rIncr);
  else if (event.getKeySym () == "m" && event.keyDown ())
    changeRPY(model_tf,"yaw",rIncr);
  else if (event.getKeySym () == "M" && event.keyDown ())
    changeRPY(model_tf,"yaw",-rIncr);
  else if (event.getKeySym () == "a" && event.keyDown ())
    tIncr -= 0.005;
  else if (event.getKeySym () == "A" && event.keyDown ())
    tIncr += 0.005;
  else if (event.getKeySym () == "z" && event.keyDown ())
    rIncr -= 1;
  else if (event.getKeySym () == "Z" && event.keyDown ())
    rIncr += 1;

  viewer->removeShape ("abc");
  char str[512];
  sprintf (str, "tIncr:%f, rIncr:%f",tIncr, rIncr);
  viewer->addText (str, 100, 0, "abc");

  PointCloudRGB::Ptr combinedCloud(new PointCloudRGB);
  *combinedCloud = *sceneCloud;

  // Transform model point cloud and combine
  Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
  getAffineTransform(model_tf, transform_1);
  PointCloudRGB::Ptr transformed_cloud (new PointCloudRGB);
  pcl::transformPointCloud (*modelCloud, *transformed_cloud, transform_1);
  *combinedCloud += *transformed_cloud;

  viewer->removePointCloud("CombinedCloud");
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(combinedCloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (combinedCloud, rgb, "CombinedCloud"); 
}

void removeTable(){    
    // Creating the filtering object: downsample the dataset using a leaf size of 0.5cm
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (sceneCloud);
    sor.setLeafSize (0.002f, 0.002f, 0.002f);
    sor.filter (*sceneCloud);

    // Plane Fitting
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_MSAC);
    seg.setDistanceThreshold (0.005);
    seg.setMaxIterations (1000);
    seg.setInputCloud (sceneCloud);
    seg.segment (*inliers, *coefficients);

    PointCloudRGB::Ptr objectPoints(new PointCloudRGB);
    for(int i=0;i<sceneCloud->points.size();i++){
      pcl::PointXYZRGB pt = sceneCloud->points[i];
      double dist = pcl::pointToPlaneDistance(pt, coefficients->values[0], coefficients->values[1],
                             coefficients->values[2], coefficients->values[3]);

      if(dist>0.005 && sceneCloud->points[i].z)
        objectPoints->points.push_back(pt);
    }
    sceneCloud = objectPoints;
}

void convert3dUnOrganized(cv::Mat &objDepth, cv::Mat &objColor, Eigen::Matrix3f &camIntrinsic, PointCloudRGB::Ptr objCloud){
    int imgWidth = objDepth.cols;
    int imgHeight = objDepth.rows;
    
    for(int u=0; u<imgHeight; u++)
      for(int v=0; v<imgWidth; v++){
        float depth = objDepth.at<float>(u,v);
        cv::Vec3b col = objColor.at<cv::Vec3b>(u,v);
        if(depth > 0.1 && depth < 1.0){
          pcl::PointXYZRGB pt;
          pt.x = (float)((v - camIntrinsic(0,2)) * depth / camIntrinsic(0,0));
          pt.y = (float)((u - camIntrinsic(1,2)) * depth / camIntrinsic(1,1));
          pt.z = depth;
          pt.r = col[0];
          pt.g = col[1];
          pt.b = col[2];
          objCloud->points.push_back(pt);
        }
    }
  }

void readDepthImage(cv::Mat &depthImg, std::string path){
    cv::Mat depthImgRaw = cv::imread(path, CV_16UC1);
    depthImg = cv::Mat::zeros(depthImgRaw.rows, depthImgRaw.cols, CV_32FC1);
    for(int u=0; u<depthImgRaw.rows; u++)
      for(int v=0; v<depthImgRaw.cols; v++){
        unsigned short depthShort = depthImgRaw.at<unsigned short>(u,v);
        depthShort = (depthShort << 13 | depthShort >> 3);
        float depth = (float)depthShort/10000;
        depthImg.at<float>(u, v) = depth;
      }
  }

int main( int ac, char* av[] ) {
    ros::init(ac, av, "label_pose");

    std::string repo_path = ros::package::getPath("label_poses");

    tIncr = 0.0;
    rIncr = 0;

    std::string scenePath;
    std::string modelName;
    
    if(ac == 3){
      scenePath = std::string(av[1]);
      modelName = std::string(av[2]);
      cv::Mat colorImage = cv::imread(scenePath + "/frame-000000.color.png", CV_LOAD_IMAGE_COLOR);

      cv::Mat depthImage;
      readDepthImage(depthImage, scenePath + "/frame-000000.depth.png");

      std::cout<<scenePath<<std::endl;
      Eigen::Matrix3f camIntrinsic = Eigen::Matrix3f::Zero(3,3);
      std::ifstream filein;
      filein.open ((scenePath + "/cameraIntinsic.txt").c_str(), std::ifstream::in);
      for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
          filein >> camIntrinsic(i,j);
      filein.close();

      sceneCloud = PointCloudRGB::Ptr(new PointCloudRGB);
      modelCloud = PointCloudRGB::Ptr(new PointCloudRGB);
      convert3dUnOrganized(depthImage, colorImage, camIntrinsic, sceneCloud);

      Eigen::Matrix4f camPose = Eigen::Matrix4f::Zero(4,4);
      filein.open ((scenePath + "/cameraExtrinsic.txt").c_str(), std::ifstream::in);
      for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
          filein >> camPose(i,j);
      filein.close();
      pcl::transformPointCloud(*sceneCloud, *sceneCloud, camPose);

      //removeTable();
      pcl::io::loadPCDFile (repo_path + "/models/" + modelName + ".pcd", *modelCloud);
    }
    else{
      std::cout<<"Enter the scenePath and modelName clouds as input "<<std::endl;
      exit(-1);
    }

    //pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    //outrem.setInputCloud(sceneCloud);
    //outrem.setRadiusSearch(0.01);
    //outrem.setMinNeighborsInRadius (4);
    //outrem.filter (*sceneCloud);

    for (int ii=0;ii<modelCloud->points.size();ii++){
      modelCloud->points[ii].r = 255;
      modelCloud->points[ii].g = 0;
      modelCloud->points[ii].b = 0;
    }

    //pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    //sor.setInputCloud (modelCloud);
    //sor.setLeafSize (0.002f, 0.002f, 0.002f);
    //sor.filter (*modelCloud);

    double cent_x, cent_y, cent_z;
    for (int ii=0;ii<sceneCloud->points.size();ii++){
      cent_x += sceneCloud->points[ii].x;
      cent_y += sceneCloud->points[ii].y;
      cent_z += sceneCloud->points[ii].z;
    }
    cent_x = cent_x/sceneCloud->points.size();
    cent_y = cent_y/sceneCloud->points.size();
    cent_z = cent_z/sceneCloud->points.size();

    model_tf.setOrigin( tf::Vector3(cent_x, cent_y, cent_z) );
    model_tf.setRotation( tf::Quaternion(0, 0, 0, 1) );

    std::cout<<"Received PointClouds !"<<std::endl;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "CombinedCloud");
    viewer->initCameraParameters ();

    // Add the first cloud as dummy
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(sceneCloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (sceneCloud, rgb, "CombinedCloud");
    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());

    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    ofstream fileout;
    fileout.open ((repo_path + "/objects.txt").c_str(), std::ofstream::out | std::ofstream::app);
    fileout << modelName << std::endl;
    fileout.close();

    fileout.open ((repo_path + "/scenes.txt").c_str(), std::ofstream::out | std::ofstream::app);
    fileout << scenePath << std::endl;
    fileout.close();

    fileout.open ((repo_path + "/poses.txt").c_str(), std::ofstream::out | std::ofstream::app);
    fileout << (float)(model_tf.getBasis()[0][0])<< " " << (float)(model_tf.getBasis()[0][1])<< " " << float(model_tf.getBasis()[0][2])<< " " << (float)(model_tf.getOrigin()[0])
           << " " << (float)(model_tf.getBasis()[1][0])<< " " << (float)(model_tf.getBasis()[1][1])<< " " << float(model_tf.getBasis()[1][2])<< " " << (float)(model_tf.getOrigin()[1])
           << " " << (float)(model_tf.getBasis()[2][0])<< " " << (float)(model_tf.getBasis()[2][1])<< " " << float(model_tf.getBasis()[2][2])<< " " << (float)(model_tf.getOrigin()[2]) << std::endl;
    fileout.close();

    ros::shutdown();
    return 0;
}
