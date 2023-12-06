#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "usb_camera_pub_node");
  ros::NodeHandle nh;
  ROS_INFO("usb_camera_pub_node start");

  int video_device = 0;
  cv::VideoCapture cap(video_device, cv::CAP_V4L2);

  image_transport::ImageTransport it(nh);
  image_transport::Publisher image_pub = it.advertise("usb_camera/image_raw", 10);

  ros::Rate rate(100);

  while (ros::ok())
  {
    cv::Mat frame;
    cap >> frame;

    if (!frame.empty())
    {
      sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      image_pub.publish(image_msg);
    }

    ros::spinOnce();
    rate.sleep();
  }

  cap.release();
  return 0;
}