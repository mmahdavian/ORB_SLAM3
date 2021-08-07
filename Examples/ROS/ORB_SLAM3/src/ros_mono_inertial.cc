/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../include/ImuTypes.h"

#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"
#include "../../../include/Converter.h"

#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>


using namespace std;
ofstream file;
geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped pose2;
geometry_msgs::PoseStamped keyframe_pose;
sensor_msgs::PointCloud cloud;
std_msgs::String str;

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, const bool bClahe): mpSLAM(pSLAM), mpImuGb(pImuGb), mbClahe(bClahe){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();

    queue<sensor_msgs::ImageConstPtr> img0Buf;
    std::mutex mBufMutex;
   
    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "Mono_Inertial");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  bool bEqual = false;
  if(argc < 3 || argc > 4)
  {
    cerr << endl << "Usage: rosrun ORB_SLAM3 Mono_Inertial path_to_vocabulary path_to_settings [do_equalize]" << endl;
    ros::shutdown();
    return 1;
  }


  if(argc==4)
  {
    std::string sbEqual(argv[3]);
    if(sbEqual == "true")
      bEqual = true;
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR,true);

  ImuGrabber imugb;
  ImageGrabber igb(&SLAM,&imugb,bEqual); // TODO
  
  // Maximum delay, 5 seconds
  ros::Subscriber sub_imu = n.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb); 
//  ros::Subscriber sub_img0 = n.subscribe("/camera/image_raw", 100, &ImageGrabber::GrabImage,&igb);
  ros::Subscriber sub_img0 = n.subscribe("/camera/image_raw", 100, &ImageGrabber::GrabImage,&igb);
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("orb_pose", 1);
  ros::Publisher keyframe_pub = n.advertise<geometry_msgs::PoseStamped>("keyframe_pose",1);
  ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("point_cloud",1);
  ros::Rate loop_rate(30);


  std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);

    while (ros::ok())
    {
        pose_pub.publish(pose);
        keyframe_pub.publish(keyframe_pose);
        cloud_pub.publish(cloud);
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("/home/mohammad/Mohammad_ws/vtr/cube_prepare/KeyFrameTrajectory.txt");
    ros::shutdown();

//  ros::spin();

  return 0;
}

tf::Transform TransformFromMat (cv::Mat position_mat) {
  cv::Mat rotation(3,3,CV_32F);
  cv::Mat translation(3,1,CV_32F);

  rotation = position_mat.rowRange(0,3).colRange(0,3);
  translation = position_mat.rowRange(0,3).col(3);

  tf::Matrix3x3 tf_camera_rotation (rotation.at<float> (0,0), rotation.at<float> (0,1), rotation.at<float> (0,2),
                                    rotation.at<float> (1,0), rotation.at<float> (1,1), rotation.at<float> (1,2),
                                    rotation.at<float> (2,0), rotation.at<float> (2,1), rotation.at<float> (2,2));

  tf::Vector3 tf_camera_translation (translation.at<float> (0), translation.at<float> (1), translation.at<float> (2));

  //Coordinate transformation matrix from orb coordinate system to ros coordinate system
  const tf::Matrix3x3 tf_orb_to_ros (0, 0, 1,
                                    -1, 0, 0,
                                     0,-1, 0);

  //Transform from orb coordinate system to ros coordinate system on camera coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  //Inverse matrix
  tf_camera_rotation = tf_camera_rotation.transpose();
  tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);

  //Transform from orb coordinate system to ros coordinate system on map coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  return tf::Transform (tf_camera_rotation, tf_camera_translation);
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutex.lock();
  if (!img0Buf.empty())
    img0Buf.pop();
  img0Buf.push(img_msg);
  mBufMutex.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  
  if(cv_ptr->image.type()==0)
  {
    return cv_ptr->image.clone();
  }
  else
  {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

void ImageGrabber::SyncWithImu()
{
  while(1)
  {
    cv::Mat im;
    double tIm = 0;
    if (!img0Buf.empty()&&!mpImuGb->imuBuf.empty())
    {
      tIm = img0Buf.front()->header.stamp.toSec();
      if(tIm>mpImuGb->imuBuf.back()->header.stamp.toSec())
          continue;
      {
      this->mBufMutex.lock();
      im = GetImage(img0Buf.front());
      img0Buf.pop();
      this->mBufMutex.unlock();
      }

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mpImuGb->mBufMutex.lock();
      if(!mpImuGb->imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tIm)
        {
          double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
          cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
          mpImuGb->imuBuf.pop();
        }
      }
      mpImuGb->mBufMutex.unlock();
      if(mbClahe)
        mClahe->apply(im,im);

    cv::Mat Tcw =  mpSLAM->TrackMonocular(im,tIm,vImuMeas);
    if (!Tcw.empty()) {
      tf::Transform transform = TransformFromMat (Tcw);
    }

   if (!Tcw.empty()){

   pose.header.stamp = ros::Time::now();
   pose.header.frame_id ="map";
   pose2.header.stamp = ros::Time::now();
   pose2.header.frame_id ="map";


   cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t(); // Rotation information
   cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3); // translation information
   vector<float> q = ORB_SLAM3::Converter::toQuaternion(Rwc);

   tf::Transform new_transform;
   new_transform.setOrigin(tf::Vector3(twc.at<float>(0, 0), twc.at<float>(0, 1), twc.at<float>(0, 2)));

   tf::Quaternion quaternion(q[0], q[1], q[2], q[3]);
   new_transform.setRotation(quaternion);

   tf::poseTFToMsg(new_transform, pose.pose);

   //////////////////////////////
   cv::Mat key= mpSLAM->keyframes();
   keyframe_pose.header.stamp = ros::Time::now();
   keyframe_pose.header.frame_id = "map";

   cv::Mat Rk = key.rowRange(0,3).colRange(0,3).t(); // Rotation information
   cv::Mat tk = -Rk*key.rowRange(0,3).col(3); // translation information

   vector<float> qk = ORB_SLAM3::Converter::toQuaternion(Rk);

   tf::Transform new_transform2;
   new_transform2.setOrigin(tf::Vector3(tk.at<float>(0, 0), tk.at<float>(0, 1), tk.at<float>(0, 2)));

   tf::Quaternion quaternion2(qk[0], qk[1], qk[2], qk[3]);
   new_transform2.setRotation(quaternion2);
   tf::poseTFToMsg(new_transform2, keyframe_pose.pose);



cloud.header.frame_id = "camera_link";
cloud.header.stamp = ros::Time::now();

std::vector<geometry_msgs::Point32> geo_points;
std::vector<ORB_SLAM3::MapPoint*> points = mpSLAM->GetTrackedMapPoints();

for (size_t i = 0; i < points.size(); i=i+1) {

    if (points[i]) {
        cv::Mat coords = points[i]->GetWorldPos();
        geometry_msgs::Point32 pt;
        pt.x = coords.at<float>(0);
        pt.y = coords.at<float>(1);
        pt.z = coords.at<float>(2);
        geo_points.push_back(pt);

    } else {
    }
}  
cloud.points = geo_points;
geo_points.clear();

        }


    }

    std::chrono::milliseconds tSleep(1);
    std::this_thread::sleep_for(tSleep);
  }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();
  return;
}


