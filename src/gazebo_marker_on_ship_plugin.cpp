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
#include "gazebo_marker_on_ship_plugin.h"

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

GZ_REGISTER_SENSOR_PLUGIN(MarkerOnShipPlugin)

MarkerOnShipPlugin::MarkerOnShipPlugin() : SensorPlugin()
{

}

MarkerOnShipPlugin::~MarkerOnShipPlugin()
{
    this->camera.reset();
}

void MarkerOnShipPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
    if (!_sensor)
        gzerr << "Invalid sensor pointer.\n";

    this->camera = std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);

    if (!this->camera) {
        gzerr << "MarkerOnShipPlugin requires a CameraSensor.\n";
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
                boost::bind(&MarkerOnShipPlugin::OnNewFrame, this, _1, this->width, this->height, this->depth, this->format));

    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
}

cv::Point2f getCenterOfMarker(std::vector<std::vector<cv::Point2f>> corners){
    cv::Point2f p = {0};
    int cnt = 0;
    for (uint j = 0; j < corners.size(); j++) {
        std::vector<cv::Point2f> c = corners.at(j);
        cnt+=c.size();
        for (uint i = 0; i < c.size(); i++) {
            p += c.at(i);
        }
    }
    p.x /= static_cast<float>(cnt);
    p.y /= static_cast<float>(cnt);
    return p;
}

void MarkerOnShipPlugin::OnNewFrame(const unsigned char *image,
                                    unsigned int width, unsigned int height,
                                    unsigned int depth, const std::string &format)
{


}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=2 ts=2 : */
