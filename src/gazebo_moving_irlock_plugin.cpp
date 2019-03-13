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

#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/features2d.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"



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

  dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
}

void MovingIRLockPlugin::OnNewFrame(const unsigned char *image,
                              unsigned int width, unsigned int height,
                              unsigned int depth, const std::string &format)
{

    static int cnt = 0;
    cnt++;

    cv::Mat frame = cv::Mat(height, width, CV_8UC3);
    frame.data = (uchar *)image;


    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;

    cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();

    cv::aruco::detectMarkers(frame, dictionary, corners, ids, params);

    int marker_10_id = -1 ;
    int marker_17_id = -1;

    for (uint i = 0; i < ids.size(); i++)
    {
        printf("%d, ",ids.at(i));
        if (ids.at(i) == 0)
        {
            marker_17_id = i;
        }
        if (ids.at(i) == 1)
        {
            marker_10_id = i;
        }
    }
    if(ids.size()>0)
        printf("\n");

    if (marker_17_id >= 0 || marker_10_id >= 0)
    {
        int marker_id = marker_10_id; // prefer the marker in the red area, as that is probably better visible
        if (marker_id <0)
            marker_id = marker_17_id;
        cv::Point2f p = {0};
        for (uint i = 0; i < corners.at(marker_id).size(); i++)
        {
            p += corners.at(marker_id).at(i);
        }
        p.x /= static_cast<float>(corners.at(marker_id).size());
        p.y /= static_cast<float>(corners.at(marker_id).size());


        //marker_angle = transformPixelToTanAngle_rgb(p*resizef);

        float x = (p.x - IRLOCK_CENTER_X) * IRLOCK_TAN_ANG_PER_PIXEL_X;
        float y = (p.y - IRLOCK_CENTER_Y) * IRLOCK_TAN_ANG_PER_PIXEL_Y;
        if (!(cnt % 50)) {
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

    }
    if (!(cnt % 50)) {
        cv::imwrite("testlalala.png",frame);
    }
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=2 ts=2 : */
