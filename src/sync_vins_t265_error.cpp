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

int N = 0;
Eigen::VectorXd vins_pose(8), optitrack_pose(8), t265_pose(8);
Eigen::VectorXd all_pose(24);
vector<Eigen::VectorXd> pose_vec;

ofstream f_out("data/jiahao_test1.txt",ios::app);
ros::Time t0;
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

void vinsCallback(const nav_msgs::Odometry::ConstPtr& vins)//指针
{
  double px = vins->pose.pose.position.x;
  double py = vins->pose.pose.position.y;
  double pz = vins->pose.pose.position.z;
  double x = vins->pose.pose.orientation.x;
  double y = vins->pose.pose.orientation.y;
  double z = vins->pose.pose.orientation.z;
  double w = vins->pose.pose.orientation.w;

  double t = (ros::Time::now()-t0 ).toSec();
  cout << "t = " << t << endl;

  vins_pose << t,px,py,pz,x,y,z,w;

  if (optitrack_pose.size() != 0)
     all_pose << vins_pose, optitrack_pose, t265_pose;
     cout << "all_pose size = " << all_pose.size() << endl;
     pose_vec.push_back(all_pose);
     cout << "pose_vec size = " << pose_vec.size() << endl;
     for (int i=0; i<all_pose.size()-1; i++ )
        f_out << all_pose(i) << " ";
     f_out << all_pose(23) << endl;
  

}


void optitrackCallback(const geometry_msgs::PoseStamped::ConstPtr& optitrack)//指针
{

  double px = optitrack->pose.position.x;
  double py = optitrack->pose.position.y;
  double pz = optitrack->pose.position.z;
  double x = optitrack->pose.orientation.x;
  double y = optitrack->pose.orientation.y;
  double z = optitrack->pose.orientation.z;
  double w = optitrack->pose.orientation.w;

  double t = (ros::Time::now()-t0).toSec();

  optitrack_pose << t,px,py,pz,x,y,z,w;


}



void t265Callback(const nav_msgs::Odometry::ConstPtr& t265)//指针
{
  double px = t265->pose.pose.position.x;
  double py = t265->pose.pose.position.y;
  double pz = t265->pose.pose.position.z;
  double x = t265->pose.pose.orientation.x;
  double y = t265->pose.pose.orientation.y;
  double z = t265->pose.pose.orientation.z;
  double w = t265->pose.pose.orientation.w;

  double t = (ros::Time::now()-t0 ).toSec();
  cout << "t = " << t << endl;

  t265_pose << t,px,py,pz,x,y,z,w;

  

}

void rgbCallback(const sensor_msgs::ImageConstPtr rgb)
{
  rgb_img = cv_bridge::toCvShare(rgb, "bgr8")->image;
  if (n >0 && n%10==0){
    cv::imwrite(basedir + to_string(frame_n) + "_main.png",rgb_img);
    cv::imwrite(basedir + to_string(frame_n) + "_depth.png",depth_img);
    frame_n ++;
  }

  
  n ++;


}


void depthCallback(const sensor_msgs::ImageConstPtr depth)
{
  depth_img = cv_bridge::toCvShare(depth, "16UC1")->image;



  
}



class talker{
    public:
      talker();
      void registerNodeHandle(ros::NodeHandle& _nh);
      void registerPubSub();
      
    private:
      ros::Subscriber  vins_sub, optitrack_sub, t265_sub, rgb_sub, depth_sub;  
      ros::NodeHandle nh; 
};

talker::talker(){
};

void talker::registerNodeHandle(ros::NodeHandle& _nh){
   nh = _nh;
}
void talker::registerPubSub(){
 
  basedir = "data/jiahao_test1/";
  vins_sub  = nh.subscribe("/vins_fusion/odometry",10,vinsCallback); 
  optitrack_sub  = nh.subscribe("/vrpn_client_node/jiahao_sun/pose",10,optitrackCallback);
  t265_sub  = nh.subscribe("/cam_2/odom/sample",10,t265Callback); 
  rgb_sub  = nh.subscribe("/camera/color/image_raw",10,rgbCallback);
  depth_sub  = nh.subscribe("/camera/aligned_depth_to_color/image_raw",10,depthCallback); 


}



int main(int argc, char** argv){

  ros::init(argc, argv, "vins_publisher");
  ros::NodeHandle nh;

  talker mytalker;
  mytalker.registerNodeHandle(nh);
  mytalker.registerPubSub();
  cout << "init_sy" << endl;
  t0 = ros::Time::now();
  ros::spin();

}