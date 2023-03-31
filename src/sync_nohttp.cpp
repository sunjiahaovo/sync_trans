#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

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


#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <boost/format.hpp>

using namespace std;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
        geometry_msgs::PoseStamped>  mySyncPolicy;

string basedir ;
int frame_n = 0;

bool saveImage = false;

void show_rgb(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("rgb_view", cv_bridge::toCvShare(msg, "bgr8")->image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void show_depth(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::Mat depth_image = cv_bridge::toCvShare(msg, "16UC1")->image;
    cv::imshow("depth_view", depth_image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

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
      void vrpn_callback(const geometry_msgs::PoseStampedConstPtr odom);
      void pcd_callback(const sensor_msgs::ImageConstPtr rgb, const sensor_msgs::ImageConstPtr depth, 
      const geometry_msgs::PoseStampedConstPtr pose);
      void get_aligned_vrpn_to_rgbd(double rgbd_stamp);
    private:
      message_filters::Subscriber<sensor_msgs::Image>* rgb_sub;             // topic1 input
      message_filters::Subscriber<sensor_msgs::Image>* depth_sub;           // topic2 input
      message_filters::Subscriber<geometry_msgs::PoseStamped>* pose_sub;    // topic3 input
      message_filters::Synchronizer<mySyncPolicy>* sync; 
      ros::NodeHandle nh; 
      bool isSave;
};

talker::talker() : isSave(false){
};
void talker::registerNodeHandle(ros::NodeHandle& _nh){
   nh = _nh;
}
void talker::registerPubSub(){
  image_transport::ImageTransport it(nh);           
  pose_sub = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh, "/vrpn_client_node/danzhe/pose", 10);
  rgb_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, "/camera/color/image_raw", 10);
  depth_sub  = new message_filters::Subscriber<sensor_msgs::Image>(nh, "/camera/aligned_depth_to_color/image_raw", 10);
  sync = new  message_filters::Synchronizer<mySyncPolicy>(mySyncPolicy(10), *rgb_sub, *depth_sub, *pose_sub);
  sync->registerCallback(boost::bind(&talker::pcd_callback,this, _1, _2, _3));
}

void talker::pcd_callback(const sensor_msgs::ImageConstPtr rgb, const sensor_msgs::ImageConstPtr depth, 
const geometry_msgs::PoseStampedConstPtr pose) 
{
  show_rgb(rgb);
  //show_depth(depth);
  // cout << "rgb stamp = " << rgb -> header.stamp <<endl;
  // cout << "depth stamp = " << depth -> header.stamp <<endl;
  // cout << "pose stamp = " << pose->header.stamp << endl;  
  double px = pose->pose.position.x;
  double py = pose->pose.position.y;
  double pz = pose->pose.position.z;
  double x = pose->pose.orientation.x;
  double y = pose->pose.orientation.y;
  double z = pose->pose.orientation.z;
  double w = pose->pose.orientation.w;
  // cout << "x,y,z,w = " << x << ", "<< y<< "," << z << "," << w << endl;
  Eigen::Matrix3d R = Quaternion2RotationMatrix(x,y,z,w);
  // cout << "R: \n" << R << endl;
  Eigen::Matrix4d T ;
  T.setZero();
  T.block(0,0,3,3) = R;
  T(0,3) = px;
  T(1,3) = py;
  T(2,3) = pz;
  T(3,3) = 1;
  // cout << "T: \n" << T << endl;

  if (saveImage){
    cout << "save success" << endl;
    cv::Mat rgb_img = cv_bridge::toCvShare(rgb, "bgr8")->image;
    cv::Mat depth_img = cv_bridge::toCvShare(depth, "16UC1")->image;
    // cv::Mat dst_rgb_img;
    // cv::Mat dst_depth_img;
    // cv::resize(rgb_img, dst_rgb_img, cv::Size(640,360), 0, 0, CV_INTER_AREA);
    // cv::resize(depth_img, dst_depth_img, cv::Size(640,360), 0, 0, CV_INTER_AREA);
    // cv::imwrite(basedir + to_string(frame_n) + "_main.png",dst_rgb_img);
    // cv::imwrite(basedir + to_string(frame_n) + "_depth.png",dst_depth_img);
    cv::imwrite(basedir + to_string(frame_n) + "_main.png",rgb_img);
    cv::imwrite(basedir + to_string(frame_n) + "_depth.png",depth_img);
    
    ofstream fout;
    fout.open(basedir + to_string(frame_n) +  "_pose.txt");
    
    for (int i =0; i< 4;i++){
        for (int j = 0; j < 4; j++){
          if (j== 3)
             fout << T(i,j);
          else
             fout << T(i,j) << ",";
        }
        fout << endl; 
    }
    fout.flush();
    fout.close();
   
    frame_n ++;
    saveImage = false;
  }
}


void kb_callback(std_msgs::UInt16ConstPtr msg){
  if(msg->data == 1){
    saveImage = true;
  }
  else{
    saveImage = false;
  }
}

using bfmt = boost::format;

string obj_dir = "/home/nvidia/NERF-demo/src/sync_trans/obj_data/";
string calib_dir = "/home/nvidia/NERF-demo/src/sync_trans/calib_data/";
string test_dir;

int maxnum = 0;

string check_dir(char* testType){
    if(!strcmp(testType, "obj")){
        test_dir = obj_dir;
    }
    else if(!strcmp(testType, "calib")){
        test_dir = calib_dir;
    }
    DIR* fp = opendir(test_dir.c_str());
    if(!fp){
        cout << "the parent dir not exists" << endl;
        return "";
    }

    struct dirent* dirItem = nullptr;
    
    while(dirItem = readdir(fp)){
        if(dirItem->d_type == DT_DIR){
            char* fname = dirItem->d_name;
            if(isdigit(fname[strlen(fname)-1])){
                int num = fname[strlen(fname)-1] - '0';
                if(num > maxnum){
                    maxnum = num;
                }
            }
        }
    }
    maxnum += 1;
    string newFolder;
    if(test_dir == obj_dir){
        newFolder = test_dir + boost::str(bfmt("obj_v%d/")%maxnum);
    }
    else{
        newFolder = test_dir + boost::str(bfmt("calib_v%d/")%maxnum);
    }
    
    mkdir(newFolder.c_str(), S_IRWXU);
    return newFolder;
}



int main(int argc, char** argv){
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  cv::namedWindow("rgb_view");
  // cv::namedWindow("depth_view");
  cv::startWindowThread();
  talker mytalker;
  mytalker.registerNodeHandle(nh);
  mytalker.registerPubSub();
  ros::Subscriber kb_sub = nh.subscribe<std_msgs::UInt16>("/key_command", 10, kb_callback);
  basedir = check_dir(argv[1]);
  ros::spin();
  cv::destroyWindow("rgb_view");
  // cv::destroyWindow("depth_view");
}
