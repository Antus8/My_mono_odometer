#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace sensor_msgs;
using namespace cv;
using namespace cv_bridge;

int main(int argc, char** argv)
{
ros::init(argc, argv, "my_camera_publisher");
ros::NodeHandle nh;
image_transport::ImageTransport it(nh);
image_transport::Publisher pub = it.advertise("camera/image", 1);
std::cout << "Starting.." << std::endl;

cv::VideoCapture cap("/home/antonello/catkin_ws/src/ros_vio/src/test_4.avi");

if(!cap.isOpened()){
     std::cout << "Camera Errata!" << std::endl;
     return 1;
}

cv::Mat frame;
sensor_msgs::ImagePtr msg;

ros::Rate loop_rate(5);
while (nh.ok()) {
cap >> frame;

if(!frame.empty()) {
msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
pub.publish(msg);
std::cout << "I published something!" << std::endl;
cv::waitKey(1);
}

ros::spinOnce();
loop_rate.sleep();
}
}