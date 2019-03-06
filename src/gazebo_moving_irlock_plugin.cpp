/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include "gazebo/sensors/DepthCameraSensor.hh"
#include "gazebo_moving_irlock_plugin.h"

#include <highgui.h>
#include <math.h>
#include <string>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include <ignition/math.hh>

#include "opencv2/highgui.hpp"
#include <opencv2/features2d.hpp>

using namespace cv;
using namespace std;

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(MovingIRLockPlugin)

MovingIRLockPlugin::MovingIRLockPlugin() : SensorPlugin()
{

}

MovingIRLockPlugin::~MovingIRLockPlugin()
{
  this->camera.reset();
}

void MovingIRLockPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  if (!_sensor)
    gzerr << "Invalid sensor pointer.\n";

  this->camera = std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);

  if (!this->camera) {
    gzerr << "MovingIRLockPlugin requires a CameraSensor.\n";
  }

  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzwarn << "Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  const string scopedName = _sensor->ParentName();

  string topicName = "~/" + scopedName + "/irlock";
  boost::replace_all(topicName, "::", "/");

  irlock_pub_ = node_handle_->Advertise<sensor_msgs::msgs::IRLock>(topicName, 10);

  this->camera->SetActive(true);
  this->rcamera = this->camera->Camera();

  this->width = this->rcamera->ImageWidth();
  this->height = this->rcamera->ImageHeight();
  this->depth = this->rcamera->ImageDepth();
  this->format = this->rcamera->ImageFormat();
  this->rate = this->rcamera->RenderRate();

  this->newFrameConnection = this->rcamera->ConnectNewImageFrame(
      boost::bind(&MovingIRLockPlugin::OnNewFrame, this, _1, this->width, this->height, this->depth, this->format));
}

void MovingIRLockPlugin::OnNewFrame(const unsigned char *image,
                              unsigned int width, unsigned int height,
                              unsigned int depth, const std::string &format)
{



    cv::Mat frame = cv::Mat(height, width, CV_8UC3);
    frame.data = (uchar *)image;

    cv::Mat threshf;
    cv::inRange(frame,cv::Scalar(0, 200, 0), cv::Scalar(180, 255, 180),threshf);
    cv::SimpleBlobDetector::Params params;
    // Change thresholds
    params.minThreshold = 10;
    params.maxThreshold = 100;

    // Filter by Area.
    params.filterByArea = 1;
    params.minArea = 1;
    params.maxArea = 100000;
    params.filterByCircularity = false;
    params.filterByColor = false;
    params.filterByConvexity = false;
    params.filterByInertia = false;

    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    std::vector<KeyPoint> keypoints;
    detector->detect( threshf, keypoints);

    static int cnt = 0;
    if (!(cnt++ % 50)) {
//        printf("shizzle: %s, %d...  %d x  %d\n", format.c_str(), depth,width,height);
//        cv::imwrite("testlalala.png",frame);
//        cv::imwrite("testlalala2.png",threshf);
    }

    if (keypoints.size() > 0) {
        KeyPoint k = keypoints.at(0);
        float x = (k.pt.x - IRLOCK_CENTER_X) * IRLOCK_TAN_ANG_PER_PIXEL_X;
        float y = (k.pt.y - IRLOCK_CENTER_Y) * IRLOCK_TAN_ANG_PER_PIXEL_Y;
        if (!(cnt++ % 50)) {
            printf("Angle coordinates beacon: %f, %f\n",x,y);
        }



        // prepare irlock message
        irlock_message.set_time_usec(0); // will be filled in simulator_mavlink.cpp
        irlock_message.set_signature(0); // unused by beacon estimator
        irlock_message.set_pos_x(x);
        irlock_message.set_pos_y(y);
        irlock_message.set_size_x(0); // unused by beacon estimator
        irlock_message.set_size_y(0); // unused by beacon estimator

        irlock_pub_->Publish(irlock_message);
    } else{
        //printf("nothing\n");
    }



}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=2 ts=2 : */
