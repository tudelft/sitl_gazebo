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

    static int cnt = 0;
    cnt++;

    cv::Mat tframe = cv::Mat(height, width, CV_8UC3);
    tframe.data = (uchar *)image;

    // data is rgb, we need bgr. Convert it:
    cv::Mat rgb[3];
    cv::split(tframe,rgb);
    cv::Mat bgr[3];
    bgr[0] = rgb[2];
    bgr[1] = rgb[1];
    bgr[2] = rgb[0];
    cv::Mat frame;
    cv::merge(bgr, 3, frame);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;

    cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();

    cv::aruco::detectMarkers(frame, dictionary, corners, ids, params);

    int markers[20] = {-1};
    for (uint i = 0; i < ids.size(); i++)
    {
        if (ids.at(i) >= 0 && ids.at(i) < 20)
            markers[ids.at(i)] = i;
    }

    //            std::cout  << getCenterOfMarker({corners.at(markers[10]),corners.at(markers[18])}) << std::endl;
    //            std::cout  << getCenterOfMarker({corners.at(markers[12]),corners.at(markers[16])}) << std::endl;
    //            std::cout  << getCenterOfMarker({corners.at(markers[11]),corners.at(markers[17])}) << std::endl;
    //            std::cout  << getCenterOfMarker({corners.at(markers[13]),corners.at(markers[15])}) << std::endl;
    //            std::cout  << getCenterOfMarker({corners.at(markers[1]),corners.at(markers[3])}) << std::endl;
    //            std::cout  << getCenterOfMarker({corners.at(markers[0]),corners.at(markers[4])}) << std::endl;

    //            std::cout  << getCenterOfMarker({corners.at(markers[14])}) << std::endl;

    cv::Point2f p = {0};
    bool found = false;
    if (markers[14] >0) {
        p = getCenterOfMarker({corners.at(markers[14])});
        //                std::cout  << p << std::endl;
        found = true;
    } else if (markers[10] >0 && markers[18]){
        p = getCenterOfMarker({corners.at(markers[10]),corners.at(markers[18])});
        //                std::cout  << p << std::endl;
        found = true;
    } else if (markers[12] >0 && markers[16]){
        p = getCenterOfMarker({corners.at(markers[12]),corners.at(markers[16])});
        //                std::cout  << p << std::endl;
        found = true;
    } else if (markers[11] >0 && markers[17]){
        p = getCenterOfMarker({corners.at(markers[11]),corners.at(markers[17])});
        //                std::cout  << p << std::endl;
        found = true;
    } else if (markers[13] >0 && markers[15]){
        p = getCenterOfMarker({corners.at(markers[13]),corners.at(markers[15])});
        //                std::cout  << p << std::endl;
        found = true;
    } else if (markers[1] >0 && markers[3]){
        p = getCenterOfMarker({corners.at(markers[1]),corners.at(markers[3])});
        //                std::cout  << p << std::endl;
        found = true;
    } else if (markers[0] >0 && markers[4]){
        p = getCenterOfMarker({corners.at(markers[0]),corners.at(markers[4])});
        //                std::cout  << p << std::endl;
        found = true;
    }



    if (found)
    {
        float x = (p.x - CAM_CENTER_X) * CAM_TAN_ANG_PER_PIXEL_X;
        float y = (p.y - CAM_CENTER_Y) * CAM_TAN_ANG_PER_PIXEL_Y;


        //calc movvar guido:
        mahist.push_back(cv::Point2f(x,y));

        uint window_size = 30;
        double mx = 0,my=0;
        if (mahist.size() > window_size) {
            for (uint i=0; i<window_size; i++) {
                uint ii = (i + window_size-1) % window_size; //TODO: FIXED BUG HERE CHECK IN OBC_VISION!!!
                //std::cout << "ii " << ii << " i " << i << std::endl;
                mx += sqrtf(powf(mahist.at(i).x - mahist.at(ii).x,2));
                my += sqrtf(powf(mahist.at(i).y - mahist.at(ii).y,2));
            }
        }
        //std::cout << "movvar x: " << mx << "  y: " << my << std::endl;

        if (mahist.size() > window_size+1)
            mahist.erase(mahist.begin());


        // prepare irlock message
        irlock_message.set_time_usec(0); // will be filled in simulator_mavlink.cpp
        irlock_message.set_signature(0); // unused by beacon estimator
        irlock_message.set_pos_x(x);
        irlock_message.set_pos_y(y);
        irlock_message.set_size_x(mx);
        irlock_message.set_size_y(my);

        irlock_pub_->Publish(irlock_message);

    } else { //TODO: add below to obc_vision!!!
        irlock_message.set_time_usec(0); // will be filled in simulator_mavlink.cpp
        irlock_message.set_signature(0); // unused by beacon estimator
        irlock_message.set_pos_x(0);
        irlock_message.set_pos_y(0);
        irlock_message.set_size_x(-1);
        irlock_message.set_size_y(-1);
        irlock_pub_->Publish(irlock_message);
    }
    if (!(cnt % 3)) {
        cv::Mat fs;
        cv::resize(frame,fs,cv::Size(frame.cols/2,frame.rows/2));
        cv::imwrite("testlalala.png",fs);
    }
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=2 ts=2 : */
