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

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>

#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"

#include "../../../include/Converter.h"

#include"../../../include/System.h"

#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

using namespace std;

//    geometry_msgs::PoseStamped pose_msg;
//    geometry_msgs::PoseStamped key_msg;
    ofstream file;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped pose2;
    geometry_msgs::PoseStamped keyframe_pose;
    sensor_msgs::PointCloud cloud;
    std_msgs::String str;


//    sensor_msgs::PointCloud features;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM3::System* mpSLAM;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();
//    rosbag::Bag bag;
//    bag.open("/home/mohammad/Desktop/pose.bag", rosbag::bagmode::Write);


    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    ros::Publisher pose_pub = nodeHandler.advertise<geometry_msgs::PoseStamped>("orb_pose", 1);
//    ros::Publisher pose_pub2 = nodeHandler.advertise<geometry_msgs::PoseStamped>("orb_pose2",1);
    ros::Publisher keyframe_pub = nodeHandler.advertise<geometry_msgs::PoseStamped>("keyframe_pose",1);
    ros::Publisher cloud_pub = nodeHandler.advertise<sensor_msgs::PointCloud>("point_cloud",1);   
//    ros::Publisher feature_pub = nodeHandler.advertise<sensor_msgs::PointCloud>("orb_features",1);
    ros::Rate loop_rate(30);   

    int count = 0;
    while (ros::ok())
    {
  //      bag.write("chatter", ros::Time::now(), str);
        pose_pub.publish(pose);
  //      pose_pub2.publish(pose2);
        keyframe_pub.publish(keyframe_pose);

//        pose_pub.publish(pose_msg);
//        keyframe_pub.publish(key_msg);
        cloud_pub.publish(cloud);
        ros::spinOnce();
        loop_rate.sleep();
    }


    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("/home/mohammad/Mohammad_ws/vtr/cube_prepare/KeyFrameTrajectory.txt");
//    bag.close();
    ros::shutdown();
    file.close();
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

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

/*
   cv::Mat position =mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());

   ros::Time current_frame_time_ = ros::Time::now();

    if (!position.empty()) {
      tf::Transform transform = TransformFromMat (position);
    
      static tf::TransformBroadcaster tf_broadcaster;
      tf_broadcaster.sendTransform(tf::StampedTransform(transform, current_frame_time_, "map", "camera_link"));

      tf::Transform grasp_tf = TransformFromMat (position);
      tf::Stamped<tf::Pose> grasp_tf_pose(grasp_tf, current_frame_time_, "map");
      tf::poseStampedTFToMsg (grasp_tf_pose, pose_msg);
    
//////////////////////////////////////////////////

      cv::Mat key= mpSLAM->keyframes();
      tf::Transform transform2 = TransformFromMat (key);

      static tf::TransformBroadcaster tf_broadcaster2;
      tf_broadcaster2.sendTransform(tf::StampedTransform(transform2, current_frame_time_, "map", "camera_link"));

      tf::Transform grasp_tf2 = TransformFromMat (key);
      tf::Stamped<tf::Pose> grasp_tf_pose2(grasp_tf2, current_frame_time_, "map");
      tf::poseStampedTFToMsg (grasp_tf_pose2, key_msg);
      }
*/
   cv::Mat Tcw =mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());

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
//   cv::Mat twc2 = Tcw.rowRange(0,3).col(3); // translation information
 //       cout<<"twc is: "<<twc<<endl;
//   cout<<"twc2 is: "<<twc2<<endl;
//   cout<<"twc2 is: "<<twc2.at<float>(0, 0)<<" "<<twc2.at<float>(1,0)<<" "<<twc2.at<float>(2,0)<<endl;
   vector<float> q = ORB_SLAM3::Converter::toQuaternion(Rwc);

//   cout<<"ROS tcw is: "<<-twc<<endl;
//   cout<<"ros Rcw is: "<<Rwc.t()<<endl;

   tf::Transform new_transform;
   new_transform.setOrigin(tf::Vector3(twc.at<float>(0, 0), twc.at<float>(0, 1), twc.at<float>(0, 2)));
//   tf::Transform new_transform2;
 //  new_transform2.setOrigin(tf::Vector3(twc2.at<float>(0, 0), twc2.at<float>(1,0), twc2.at<float>(2,0)));


   tf::Quaternion quaternion(q[0], q[1], q[2], q[3]);
   new_transform.setRotation(quaternion);
  // new_transform2.setRotation(quaternion);


   tf::poseTFToMsg(new_transform, pose.pose);
  // tf::poseTFToMsg(new_transform2, pose2.pose);

  // str.data = twc.at<float>(0, 0);

//   file.open("/home/mohammad/Desktop/pose.txt",ios::out | ios::app);
//   cout<<ros::Time::now()<<" "<<twc2.at<float>(0, 0)<<" "<<twc2.at<float>(1,0)<<" "<<twc2.at<float>(2, 0)<<" "<<q[0]<<" "<<q[1]<<" "<<q[2]<<" "<<q[3]<<endl;
 //  cout<<ros::Time::now()<<" "<<twc.at<float>(0, 0)<<" "<<twc.at<float>(0, 1)<<" "<<twc.at<float>(0, 2)<<" "<<q[0]<<" "<<q[1]<<" "<<q[2]<<" "<<q[3]<<endl;

   //////////////////////////////
   cv::Mat key= mpSLAM->keyframes();
   keyframe_pose.header.stamp = ros::Time::now();
   keyframe_pose.header.frame_id = "map";
  
   cv::Mat Rk = key.rowRange(0,3).colRange(0,3).t(); // Rotation information
   cv::Mat tk = -Rk*key.rowRange(0,3).col(3); // translation information
//   cv::Mat tk = key.rowRange(0,3).col(3); // translation information

   vector<float> qk = ORB_SLAM3::Converter::toQuaternion(Rk);

//   cout<<"slam Rot is: "<<Rk<<endl;

   tf::Transform new_transform2;
   new_transform2.setOrigin(tf::Vector3(tk.at<float>(0, 0), tk.at<float>(0, 1), tk.at<float>(0, 2)));

   tf::Quaternion quaternion2(qk[0], qk[1], qk[2], qk[3]);
   new_transform2.setRotation(quaternion2);
  // cout<<tk.at<float>(0, 0)<<" "<<tk.at<float>(0, 1)<<" "<<tk.at<float>(0, 2)<<" "<<qk[0]<<" "<<qk[1]<<" "<<qk[2]<<" "<<qk[3]<<endl;
   tf::poseTFToMsg(new_transform2, keyframe_pose.pose);


 //  cout<<"ros Tcw is : "<<Tcw<<endl;
//   cout<<"Keyframe is : "<<key<<endl;

cloud.header.frame_id = "camera_link";
//features.header.frame_id = "camera_link";
cloud.header.stamp = ros::Time::now();
//features.header.stamp = ros::Time::now(); 

std::vector<geometry_msgs::Point32> geo_points;
//std::vector<geometry_msgs::Point32> feature_points;
std::vector<ORB_SLAM3::MapPoint*> points = mpSLAM->GetTrackedMapPoints();
//cout << points.size() << endl;
//cout << "Points size: " << points.size() << endl;

//float k[3][3]={{528.96002, 0, 620.22998},{0, 528.66498, 368.64499},{0, 0, 1}};
//cv::Mat K = cv::Mat(3, 3, CV_32FC1, &k);
//cout<<Tcw.size()<<endl;
//cv::Mat proj = K*Tcw.rowRange(0,3).colRange(0,4);
//cout<<"proj is: "<<proj<<endl;

for (size_t i = 0; i < points.size(); i=i+1) {

    if (points[i]) {
        cv::Mat coords = points[i]->GetWorldPos();
//	cout<<"coords is: "<<coords<<endl;
        geometry_msgs::Point32 pt;
//        pt.x = coords.at<float>(2);
//        pt.y = -coords.at<float>(0);
//        pt.z = -coords.at<float>(1);
        pt.x = coords.at<float>(0);
        pt.y = coords.at<float>(1);
        pt.z = coords.at<float>(2);
        geo_points.push_back(pt);

/*	float pt3d[4][1] = {{coords.at<float>(0)},{coords.at<float>(1)},{coords.at<float>(2)},1};
	cv::Mat PT3D = cv::Mat(4, 1, CV_32FC1, &pt3d);
	cv::Mat pt2d = proj*PT3D;
        pt2d.at<float>(0) = pt2d.at<float>(0)/pt2d.at<float>(2);
        pt2d.at<float>(1) = pt2d.at<float>(1)/pt2d.at<float>(2);	
        pt2d.at<float>(2) = pt2d.at<float>(2)/pt2d.at<float>(2);	
	geometry_msgs::Point32 pt_image;
	pt_image.x = pt2d.at<float>(0);
	pt_image.y = pt2d.at<float>(1);
	pt_image.z = pt2d.at<float>(2);
	feature_points.push_back(pt_image);
//	cout<<pt2d<<endl;   
*/
    } else {
    }
}
//cout << geo_points.size() << endl;
cloud.points = geo_points;
//cout<<"number of points: "<<geo_points.size()<<endl;
geo_points.clear();

//features.points = feature_points;
	}

//vector<cv::KeyPoint> vKeys = mpSLAM->GetTrackedKeyPointsUn();
//std::vector<cv::Point2f> point2f;

//cv::KeyPoint::convert(vKeys,point2f);
    //descriptor->compute ( cv_ptr->image, keypoints, descriptors);

//    float points[point2f.size()*2]; 
//    float descs[point2f.size()*32];

//std::vector<double> points{};

//for(size_t i=0; i<point2f.size(); i++){
//        points.push_back(point2f[i].x);
//        points.push_back(point2f[i].y);
//	cout<<"x is: "<<point2f[i].x<<" y is: "<<point2f[i].y<<endl;
//}



//for(size_t i=0;i<vKeys.size();i++){
//cout<<"vKeys are: "<<vKeys[i]<<endl;
//}
   
//   cv::Mat newTcw = mpSLAM->GetTracker()->mCurrentFrame.mTcw;
//   cout<<"new Tcw is : "<<newTcw<<endl;

}
