#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


#include "april_detection.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

ros::Publisher pose_pub;
ros::Subscriber image_sub;
AprilDetection det;


double distortion_coeff[] = {0.036156, 
                             -0.014115, 
                            -0.0007291, 
                            0.01183, 
                            0.000000};
double intrinsics[] = {717.01615,    0.     ,  995.51,
                       0.     ,  739.10114,  498.77467,
                       0.     ,    0.     ,    1.};

const cv::Mat d(cv::Size(1, 5), CV_64FC1, distortion_coeff);
const cv::Mat K(cv::Size(3, 3), CV_64FC1, intrinsics);

cv::Mat rectify(const cv::Mat image){
  cv::Mat image_rect = image.clone();
  const cv::Mat new_K = cv::getOptimalNewCameraMatrix(K, d, image.size(), 1.0); 
  cv::undistort(image, image_rect, K, d, new_K); 

  return image_rect;
}

void publishTransforms(vector<apriltag_pose_t> poses, vector<int> ids, std_msgs::Header header){
  tf::Quaternion q;
  tf::Matrix3x3 so3_mat;
  tf::Transform tf;
  static tf::TransformBroadcaster br;
  geometry_msgs::PoseArray pose_array_msg;
  pose_array_msg.header = header;

  for (int i=0; i<poses.size(); i++){

    // translation
    tf.setOrigin(tf::Vector3(poses[i].t->data[0],
                             poses[i].t->data[1],
                             poses[i].t->data[2]));
    // orientation - SO(3)
    so3_mat.setValue(poses[i].R->data[0], poses[i].R->data[1], poses[i].R->data[2],
                     poses[i].R->data[3], poses[i].R->data[4], poses[i].R->data[5], 
                     poses[i].R->data[6], poses[i].R->data[7], poses[i].R->data[8]);

    double roll, pitch, yaw; 

    // orientation - q
    so3_mat.getRPY(roll, pitch, yaw); // so3 to RPY
    q.setRPY(roll, pitch, yaw);

    tf.setRotation(q);
    string marker_name = "marker_" + to_string(ids[i]);
    br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "camera", marker_name));
    ROS_INFO("Transformation published for marker.");
    
    geometry_msgs::Pose pose;
    pose.position.x = poses[i].t->data[0];
    pose.position.y = poses[i].t->data[1];
    pose.position.z = poses[i].t->data[2];
    
    tf::quaternionTFToMsg(q, pose.orientation);
    pose_array_msg.poses.push_back(pose);
  }

  pose_pub.publish(pose_array_msg);
}


void imageCallback(const sensor_msgs::Image::ConstPtr& msg){
  
  cv_bridge::CvImagePtr img_cv = cv_bridge::toCvCopy(msg);
  std_msgs::Header header = msg->header;

  // rectify and run detection (pair<vector<apriltag_pose_t>, cv::Mat>)
  auto april_obj =  det.processImage(rectify(img_cv->image));

  publishTransforms(get<0>(april_obj), get<1>(april_obj), header);

}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "april_detection");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  pose_pub = n.advertise<geometry_msgs::PoseArray>("/april_poses", 5); 
  image_sub = n.subscribe("/camera_0", 1, imageCallback);
  
  ros::spin();
  return 0;
  
}
