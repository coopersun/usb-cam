#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <std_srvs/Empty.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace usb_cam {

static void reportMessageAge(const std::string& description, double age) {
    std::cout << description << ": " << std::fixed << std::setprecision(3) << age * 1000 << " ms" << std::endl;
}

static double getMessageAge(ros::Time t) {
    ros::Time tnow = ros::Time::now();
    if (tnow < t) {
        std::cout  << "ros::Time::now() earlier than message timestamp " << std::fixed
                     << std::setprecision(5) << tnow.toSec() << " " << t.toSec() << std::endl;
        return tnow.toSec() - t.toSec();
    } else {
        return (tnow - t).toSec();
    }
}

void handleCompressedImageMsg(const sensor_msgs::CompressedImage::ConstPtr &msg)
  {
      double age = getMessageAge(msg->header.stamp);
      reportMessageAge("Compressed Image Message age: ", age);
      cv::Mat image = cv::imdecode(cv::Mat(msg->data), 1);//convert compressed image data to cv::Mat
//      cv::imshow("view", image);
//      cv::waitKey(1);
  }

  void handleImageMsg(const sensor_msgs::ImageConstPtr &msg)
  {
      double age = getMessageAge(msg->header.stamp);
      reportMessageAge("Image Message Age: ", age);
//      cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
//      cv::waitKey(1);
  }


class ReceiverNode
{
public:
  // private ROS node handle
  ros::NodeHandle node_;
  std::string camera_topic_;
  ros::Subscriber sub_image_;
  int queue_length_ = 10;

  ReceiverNode() :
      node_("~")
  {
    node_.param("camera_topic", camera_topic_, std::string("/usb_cam/image_raw"));
    node_.param("queue_length", queue_length_, 10);
    ROS_INFO("camera_topic to subscribe is: %s\n", camera_topic_.c_str());
    bool is_compressed_image = (camera_topic_.find("compressed") != std::string::npos);
    if (!camera_topic_.empty()) {
        if (is_compressed_image) {
            sub_image_ =
                node_.subscribe(camera_topic_, queue_length_, handleCompressedImageMsg);
        } else {
            sub_image_ =
                node_.subscribe(camera_topic_, queue_length_, handleImageMsg);
        }
    }
  }

  virtual ~ReceiverNode() {}

  bool spin()
  {
    ros::Rate loop_rate(1000);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return true;
  }

};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "usb_cam_receiver");
//  cv::namedWindow("view");
//  cv::startWindowThread();
  usb_cam::ReceiverNode a;
  a.spin();
 // cv::destroyWindow("view");
  return EXIT_SUCCESS;
}
