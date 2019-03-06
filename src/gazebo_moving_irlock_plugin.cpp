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

    static int cnt = 0;
    if (!(cnt++ % 50)) {
        printf("shizzle: %s, %d...  %d x  %d\n", format.c_str(), depth,width,height);
        cv::imwrite("testlalala.png",frame);
    }



//  gazebo::msgs::CameraImage img = this->camera->Image();

//  for (int idx = 0; idx < img.model_size(); idx++) {

//    gazebo::msgs::LogicalCameraImage_Model model = img.model(idx);

//    if (model.has_name())
//        printf("Model name: %s\n",model.name().c_str());

//    if (model.has_name() && model.name() == "irlock_beacon") {

//      if (model.has_pose()) {

//        // position of the beacon in camera frame
//        ignition::math::Vector3d pos;
//        pos.X() = model.pose().position().x();
//        pos.Y() = model.pose().position().y();
//        pos.Z() = model.pose().position().z();

//        // the default orientation of the IRLock sensor reports beacon in front of vehicle as -y values, beacon right of vehicle as x values
//        // rotate the measurement accordingly
//        ignition::math::Vector3d meas(-pos.Y()/pos.X(), -pos.Z()/pos.X(), 1.0);
//        printf("Gazebo beacon: %f, %f, %f\n",meas.X(),meas.Y(),meas.Z());

//        // prepare irlock message
//        irlock_message.set_time_usec(0); // will be filled in simulator_mavlink.cpp
//        irlock_message.set_signature(idx); // unused by beacon estimator
//        irlock_message.set_pos_x(meas.X());
//        irlock_message.set_pos_y(meas.Y());
//        irlock_message.set_size_x(0); // unused by beacon estimator
//        irlock_message.set_size_y(0); // unused by beacon estimator

//        // send message
//        irlock_pub_->Publish(irlock_message);

//      }
//    }
//  }

}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=2 ts=2 : */
