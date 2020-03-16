#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include "opencv2/calib3d/calib3d_c.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <queue>

using namespace cv;
using namespace std;

class SubscribeAndPublish
{
    public:

        cv::Mat previous, previous_gray, current, current_gray;
        bool isPreviousAlreadyStored = false;
        bool isCurrentAlreadyStored = false;

        vector<Point2F> initial_corners, some_tracked_corners;
        vector<uchar> status;
        vector<float> err;

        Mat R_f,t_f, mask;
        bool R_f_t_f_initialized = false;

        Mat trajectory = Mat::zeros(600, 600, CV_8UC3);
        SubscribeAndPublish(){
            ros::NodeHandle n_;
            image_transport::ImageTransport it(n_);
            sub_ =it.subscribe("camera/image", 1, &SubscribeAndPublish::imageCallback, this);
            pub_ = it.advertise("odometry", 1);
        
        }

        void imageCallback(const sensor_msgs::ImageConstPtr& msg)
        {
            try
            {
                cv::Mat myImage = cv_bridge::toCvShare(msg, "bgr8")->image;

                if(!isPreviousAlreadyStored){
                    previous = myImage.clone();
                    cvtColor(previous, previous_gray, COLOR_BGR2GRAY);
                    // initial_corners = my_goodFeaturesToTrack(0,0, previous_gray);
                    feature_detection(previous_gray, initial_corners);
                    mask = Mat::zeros(previous.size(),previous.type());
                    isPreviousAlreadyStored = true;
                } 
                else{
                    current = myImage.clone();
                    isCurrentAlreadyStored = true;
                }

                if(isPreviousAlreadyStored && isCurrentAlreadyStored){
                    cvtColor(current, current_gray, COLOR_BGR2GRAY);
                    featureDetection(current_gray, some_tracked_corners);
                    //ASK
                    /*
                    cv::calcOpticalFlowPyrLK( previous_gray, current_gray, initial_corners, some_tracked_corners, status, err );
                       
                    // Get good matches
                    vector<Point2f> good_new;
                    for(uint i = 0; i < initial_corners.size(); i++)
                    {
                        // Select good points
                        if(status[i] == 1) {
                            good_new.push_back(initial_corners[i]);
                            // Draw the tracks
                            line(mask, some_tracked_corners[i], initial_corners[i], Scalar(0,255,255), 2);
                            circle(current, some_tracked_corners[i], 5, Scalar(255,0,0), -1);
                        }
                    }
                    */
                    featureTracking(previous_gray, current_gray, initial_corners, some_tracked_corners, status);
                    //Now I'm ready to estimate R and t

                    double focal = 646.31701214;
                    Point2d pp = Point2d (324.41027914, 243.22871802); 
                    Mat E, R, t, myMask;
                    E=findEssentialMat(some_tracked_corners, initial_corners, focal, pp, RANSAC, 0.999, 1.0, myMask);
                    recoverPose(E, some_tracked_corners, initial_corners, R, t, focal, pp, myMask);

                    if (!R_f_t_f_initialized) {
                        std::cout << "Initializing Rt real quick" << std::endl;
                        R_f = R.clone();
                        t_f = t.clone();
                        R_f_t_f_initialized = true; 
                        // This means that this is operated only once
                    }

                    t_f = t_f + 1*(R_f*t);
                    R_f = R*R_f;

                    //values of x and y on the window
                    int x = int(t_f.at<double>(0)) + 300;
                    int y = int(t_f.at<double>(2)) + 100;

                    circle(trajectory, Point(x,y), 5, Scalar(255,0,0), -1);

                    // Draw and display
                    Mat img;
                    add(current, mask, img);
                    imshow("Test", img);
                    imshow("Trajectory", trajectory);

                    char key = cvWaitKey(10);        
                    if (key == 27) // ESC
                        std::cout << "Hello" << std::endl;    

                    previous_gray = current_gray.clone();
                    initial_corners = some_tracked_corners;
                }

                sensor_msgs::ImagePtr outMsg;
                outMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", trajectory).toImageMsg();
                pub_.publish(outMsg);

                }
            catch(cv_bridge::Exception& e)
            {
                ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
            }
            
        }
    void featureDetection(Mat img_1, vector<Point2f>& points1){
        vector<KeyPoint> keypoints_1;
        int fast_threshold = 20;
        bool nonmaxSuppression = true;
        FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
        KeyPoint::convert(keypoints_1, points1, vector<int>());
    }

    void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status)	{ 

        //In this function, after calOpticalFlow, I erase the points for which tracking fails

        vector<float> err;					
        Size winSize=Size(21,21);																								
        TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);

        calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);

        //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
        int indexCorrection = 0;
        for( int i=0; i<status.size(); i++)
            {  Point2f pt = points2.at(i- indexCorrection);
                if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	{
                    if((pt.x<0)||(pt.y<0))	{
                        status.at(i) = 0;
                    }
                    points1.erase (points1.begin() + (i - indexCorrection));
                    points2.erase (points2.begin() + (i - indexCorrection));
                    indexCorrection++;
                }

            }

        }

    private:

    image_transport::Subscriber sub_;
    image_transport::Publisher pub_;  
};

int main(int argc, char **argv){
    ros::init(argc, argv, "subscribe_and_publish");

    SubscribeAndPublish SAPObject;
    ros::spin();
    return 0;
}