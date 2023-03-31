#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/UInt16.h>

#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <typeinfo>
#include <fstream>
#include <string>
#include <vector>

#include <thread>
#include "nav_msgs/Odometry.h"

using namespace std;
// typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
//  nav_msgs::Odometry, geometry_msgs::PoseStamped, nav_msgs::Odometry>  mySyncPolicy;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
  geometry_msgs::PoseStamped, nav_msgs::Odometry>  mySyncPolicy;

int N = 0;
Eigen::VectorXd vins_pose(8), optitrack_pose(8), t265_pose(8);
Eigen::VectorXd all_pose(16);
vector<Eigen::VectorXd> pose_vec;

ofstream f_out("data/jiahao_test1.txt",ios::app);
string basedir;
int frame_n, n;
cv::Mat rgb_img, depth_img;


Eigen::Matrix3d Quaternion2RotationMatrix(const double x, const double y, const double z,const double w){

  Eigen::Quaterniond q;
  q.x() = x;
  q.y() = y;
  q.z() = z;
  q.w() = w;
  Eigen::Matrix3d R = q.normalized().toRotationMatrix();
  return R;

}




class talker{
    public:
      talker();
      void registerNodeHandle(ros::NodeHandle& _nh);
      void registerPubSub();
      // void cd_callback(const sensor_msgs::ImageConstPtr rgb, const sensor_msgs::ImageConstPtr depth,
      //  const nav_msgs::Odometry::ConstPtr vins,  const geometry_msgs::PoseStampedConstPtr optitrack,  const nav_msgs::Odometry::ConstPtr t265);
      void cd_callback(const sensor_msgs::ImageConstPtr rgb, const sensor_msgs::ImageConstPtr depth,
        const geometry_msgs::PoseStampedConstPtr optitrack,  const nav_msgs::Odometry::ConstPtr t265);
      
    private:
      // ros::Subscriber  vins_sub, optitrack_sub, t265_sub;  
      message_filters::Subscriber<sensor_msgs::Image>* rgb_sub;             // topic1 input
      message_filters::Subscriber<sensor_msgs::Image>* depth_sub;           // topic2 input
      // message_filters::Subscriber<nav_msgs::Odometry>* vins_sub;
      message_filters::Subscriber<geometry_msgs::PoseStamped>* optitrack_sub;
      message_filters::Subscriber<nav_msgs::Odometry>* t265_sub;
      message_filters::Synchronizer<mySyncPolicy>* sync; 
      ros::Time t0 = ros::Time::now();
      ros::NodeHandle nh; 
};

talker::talker(){
};

void talker::registerNodeHandle(ros::NodeHandle& _nh){
   nh = _nh;
}
void talker::registerPubSub(){
 
  basedir = "data/jiahao_test1/";

  // vins_sub = new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/vins_fusion/odometry", 10); 
  optitrack_sub = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh, "/vrpn_client_node/jiahao_sun/pose", 10); 
  t265_sub = new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/cam_2/odom/sample", 10); 
  rgb_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, "/camera/color/image_raw", 10);
  depth_sub  = new message_filters::Subscriber<sensor_msgs::Image>(nh, "/camera/aligned_depth_to_color/image_raw", 10);
  // sync = new  message_filters::Synchronizer<mySyncPolicy>(mySyncPolicy(10), *rgb_sub, *depth_sub, *vins_sub, *optitrack_sub, *t265_sub);
  // sync->registerCallback(boost::bind(&talker::cd_callback,this, _1, _2, _3, _4, _5));
  sync = new  message_filters::Synchronizer<mySyncPolicy>(mySyncPolicy(10), *rgb_sub, *depth_sub, *optitrack_sub, *t265_sub);
  sync->registerCallback(boost::bind(&talker::cd_callback,this, _1, _2, _3, _4));
}


// void talker::cd_callback(const sensor_msgs::ImageConstPtr rgb, const sensor_msgs::ImageConstPtr depth, 
//                         const nav_msgs::Odometry::ConstPtr vins,  const geometry_msgs::PoseStampedConstPtr optitrack,  const nav_msgs::Odometry::ConstPtr t265)

void talker::cd_callback(const sensor_msgs::ImageConstPtr rgb, const sensor_msgs::ImageConstPtr depth, const geometry_msgs::PoseStampedConstPtr optitrack,  const nav_msgs::Odometry::ConstPtr t265) 
{
  cout<<"come in"<<endl;
  //vins
  // double px = vins->pose.pose.position.x;
  // double py = vins->pose.pose.position.y;
  // double pz = vins->pose.pose.position.z;
  // double x = vins->pose.pose.orientation.x;
  // double y = vins->pose.pose.orientation.y;
  // double z = vins->pose.pose.orientation.z;
  // double w = vins->pose.pose.orientation.w;

  // double t = (vins-> header.stamp-t0).toSec();
  // cout << "t = " << t << endl;

  // vins_pose << t,px,py,pz,x,y,z,w;

  
  //optitrack
  // px = optitrack->pose.position.x;
  // py = optitrack->pose.position.y;
  // pz = optitrack->pose.position.z;
  // x = optitrack->pose.orientation.x;
  // y = optitrack->pose.orientation.y;
  // z = optitrack->pose.orientation.z;
  // w = optitrack->pose.orientation.w;

  // t = (optitrack-> header.stamp-t0).toSec();

  double px = optitrack->pose.position.x;
  double py = optitrack->pose.position.y;
  double pz = optitrack->pose.position.z;
  double x = optitrack->pose.orientation.x;
  double y = optitrack->pose.orientation.y;
  double z = optitrack->pose.orientation.z;
  double w = optitrack->pose.orientation.w;

  double t = (optitrack-> header.stamp-t0).toSec();

  optitrack_pose << t,px,py,pz,x,y,z,w;

  //t265
  px = t265->pose.pose.position.x;
  py = t265->pose.pose.position.y;
  pz = t265->pose.pose.position.z;
  x = t265->pose.pose.orientation.x;
  y = t265->pose.pose.orientation.y;
  z = t265->pose.pose.orientation.z;
  w = t265->pose.pose.orientation.w;

  t = (t265-> header.stamp-t0 ).toSec();
  cout << "t = " << t << endl;

  t265_pose << t,px,py,pz,x,y,z,w;


  if (optitrack_pose.size() != 0){
    //  all_pose << vins_pose, optitrack_pose, t265_pose;
     all_pose << optitrack_pose, t265_pose;
     cout << "all_pose size = " << all_pose.size() << endl;
     pose_vec.push_back(all_pose);
     cout << "pose_vec size = " << pose_vec.size() << endl;
     for (int i=0; i<all_pose.size()-1; i++ )
        f_out << all_pose(i) << " ";
     f_out << all_pose(15) << endl;
  }
  //rgb,depth
  cv::imwrite(basedir + to_string(frame_n) + "_main.png",cv_bridge::toCvShare(rgb, "bgr8")->image);
  cv::imwrite(basedir + to_string(frame_n) + "_depth.png",cv_bridge::toCvShare(depth, "16UC1")->image);
  frame_n += 1;
}



int main(int argc, char** argv){

  ros::init(argc, argv, "vins_publisher");
  ros::NodeHandle nh;

  talker mytalker;
  mytalker.registerNodeHandle(nh);
  mytalker.registerPubSub();
  cout << "init_sy" << endl;
  ros::spin();

}