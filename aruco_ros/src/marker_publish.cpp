/*****************************
 Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are
 permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of
 conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list
 of conditions and the following disclaimer in the documentation and/or other materials
 provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those of the
 authors and should not be interpreted as representing official policies, either expressed
 or implied, of Rafael Muñoz Salinas.
 ********************************/
/**
 * @file marker_publish.cpp
 * @author Bence Magyar
 * @date June 2014
 * @brief Modified copy of simple_single.cpp to publish all markers visible
 * (modified by Josh Langsfeld, 2014)
 */

#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <exprob_ass3/ID.h>

class ArucoMarkerPublisher
{
private:
  // ArUco stuff
  // camera1
  aruco::MarkerDetector mDetector_1;
  std::vector<aruco::Marker> markers_1;
  aruco::CameraParameters camParam_1;
  
  //camera2
  aruco::MarkerDetector mDetector_2;
  std::vector<aruco::Marker> markers_2;
  aruco::CameraParameters camParam_2;

  // node params
  double marker_size_1;
  bool useCamInfo_1;
  double marker_size_2;
  bool useCamInfo_2;

  // ROS pub-sub
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_1;
  image_transport::Subscriber image_sub_1;
  image_transport::ImageTransport it_2;
  image_transport::Subscriber image_sub_2;

  image_transport::Publisher image_pub_1;
  image_transport::Publisher debug_pub_1;
  image_transport::Publisher image_pub_2;
  image_transport::Publisher debug_pub_2;
  ros::Publisher ID_pub_;

  cv::Mat inImage_1;
  cv::Mat inImage_2;
  
public:
  ArucoMarkerPublisher() :
      nh_("~"), it_1(nh_), useCamInfo_1(true), it_2(nh_), useCamInfo_2(true)
  {
    image_sub_1 = it_1.subscribe("/robot/camera1/image_raw", 1, &ArucoMarkerPublisher::image_callback1, this);
    image_sub_2 = it_2.subscribe("/robot/camera2/image_raw", 1, &ArucoMarkerPublisher::image_callback2, this);
    image_pub_1 = it_1.advertise("result1", 1);
    image_pub_2 = it_2.advertise("result2", 1);
    debug_pub_1 = it_1.advertise("debug1", 1);
    debug_pub_2 = it_2.advertise("debug2", 1);
    ID_pub_ = nh_.advertise<exprob_ass3::ID>("ID", 1000);
    
    nh_.param<bool>("use_camera_info1", useCamInfo_1, false);
    nh_.param<bool>("use_camera_info2", useCamInfo_2, false);
    camParam_1 = aruco::CameraParameters();
    camParam_2 = aruco::CameraParameters();
  }

  void image_callback1(const sensor_msgs::ImageConstPtr& msg)
  {
    bool publishImage1 = image_pub_1.getNumSubscribers() > 0;
    bool publishDebug1 = debug_pub_1.getNumSubscribers() > 0;

    ros::Time curr_stamp = msg->header.stamp;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      inImage_1 = cv_ptr->image;
   
      // clear out previous detection results
      markers_1.clear();

      // ok, let's detect
      mDetector_1.detect(inImage_1, markers_1, camParam_1, marker_size_1, false);
		exprob_ass3::ID msg;
		std::cout << "Camera1 detects: ";
        for (std::size_t i = 0; i < markers_1.size(); ++i)
        {
          std::cout << markers_1.at(i).id << " ";
          // publish the ID on a rostopic
          msg.current_id = markers_1.at(i).id;
          ID_pub_.publish(msg);
        }
        std::cout << std::endl;

      // draw detected markers on the image for visualization
      for (std::size_t i = 0; i < markers_1.size(); ++i)
      {
        markers_1[i].draw(inImage_1, cv::Scalar(0, 0, 255), 2);
      }
      // publish input image with markers drawn on it
      if (publishImage1)
      {
        // show input with augmented information
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = curr_stamp;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = inImage_1;
        image_pub_1.publish(out_msg.toImageMsg());
      }

      // publish image after internal image processing
      if (publishDebug1)
      {
        // show also the internal image resulting from the threshold operation
        cv_bridge::CvImage debug_msg;
        debug_msg.header.stamp = curr_stamp;
        debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
        debug_msg.image = mDetector_1.getThresholdedImage();
        debug_pub_1.publish(debug_msg.toImageMsg());
      }

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }
  
  
  void image_callback2(const sensor_msgs::ImageConstPtr& msg)
  {
    bool publishImage2 = image_pub_2.getNumSubscribers() > 0;
    bool publishDebug2 = debug_pub_2.getNumSubscribers() > 0;

    ros::Time curr_stamp = msg->header.stamp;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      inImage_2 = cv_ptr->image;
   
      // clear out previous detection results
      markers_2.clear();

      // ok, let's detect
      mDetector_2.detect(inImage_2, markers_2, camParam_2, marker_size_2, false);
		exprob_ass3::ID msg;
		std::cout << "Camera2 detects: ";
        for (std::size_t i = 0; i < markers_2.size(); ++i)
        {
          std::cout << markers_2.at(i).id << " ";
          // publish the ID on a rostopic
          msg.current_id = markers_2.at(i).id;
          ID_pub_.publish(msg);
        }
        std::cout << std::endl;

      // draw detected markers on the image for visualization
      for (std::size_t i = 0; i < markers_2.size(); ++i)
      {
        markers_2[i].draw(inImage_2, cv::Scalar(0, 0, 255), 2);
      }
      // publish input image with markers drawn on it
      if (publishImage2)
      {
        // show input with augmented information
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = curr_stamp;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = inImage_2;
        image_pub_2.publish(out_msg.toImageMsg());
      }

      // publish image after internal image processing
      if (publishDebug2)
      {
        // show also the internal image resulting from the threshold operation
        cv_bridge::CvImage debug_msg;
        debug_msg.header.stamp = curr_stamp;
        debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
        debug_msg.image = mDetector_2.getThresholdedImage();
        debug_pub_2.publish(debug_msg.toImageMsg());
      }

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_marker_publisher");

  ArucoMarkerPublisher node;

  ros::spin();
}
