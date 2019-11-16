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

    string topicName = "~/" + scopedName + "/moving_marker";
    boost::replace_all(topicName, "::", "/");

    moving_marker_pub_ = node_handle_->Advertise<sensor_msgs::msgs::MovingMarker>(topicName, 10);

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
std::tuple<float,float> getDistanceAndSizeFrom2Markers(std::vector<cv::Point2f> corners1,std::vector<cv::Point2f> corners2, float distane_between_markers_mm, float f, float fx, float fy){
    cv::Point2f p1 = getCenterOfMarker({corners1});
    cv::Point2f p2 = getCenterOfMarker({corners2});
    float size = static_cast<float>(cv::norm(p1-p2));
    cv::Point2f p(p1.x-p2.x, p1.y-p2.y);
    float dist_px = sqrtf(p.x*p.x+p.y*p.y);

    float mx = fx / f; // px/mm
    float my = fy / f;
    float m = (mx+my)*0.5f;

    float object_image_sensor_mm = dist_px / m;
    float distance_x_mm = (distane_between_markers_mm * f) / (object_image_sensor_mm);

    return std::make_tuple( distance_x_mm/1000.f,size);
}

std::tuple<float,float> update_distance(int markers[20], std::vector<std::vector<cv::Point2f>> corners){
    float fx,fy,f;
    fx = 0.01;
    fy = 0.01;
    f = 1.143;

    const uint marker_interdist_big = 1850;
    const uint marker_interdist_small = 540;
    const float marker_interdist_small2big = static_cast<float>(marker_interdist_big) / static_cast<float>(marker_interdist_small);


    int cnt = 0;
    float d = 0;
    float size = 0;
    if (markers[1] > 0 && markers[3] > 0 ){
        float part_dist, part_size;
        std::tie(part_dist, part_size) = getDistanceAndSizeFrom2Markers(corners.at(markers[1]),corners.at(markers[3]),marker_interdist_big,f,fx, fy);
        d+=part_dist;
        size+=part_size;
        cnt++;
    }
    if (markers[0] > 0 && markers[4] > 0 ){
        float part_dist, part_size;
        std::tie(part_dist, part_size) = getDistanceAndSizeFrom2Markers(corners.at(markers[0]),corners.at(markers[4]),marker_interdist_big,f,fx, fy);
        d+=part_dist;
        size+=part_size;
        cnt++;
    }
    if (cnt == 0){ // only use the smaller markers if the big ones weren't seen
        if (markers[10] > 0 && markers[12] > 0 ){
            float part_dist, part_size;
            std::tie(part_dist, part_size) = getDistanceAndSizeFrom2Markers(corners.at(markers[10]),corners.at(markers[12]),marker_interdist_small,f,fx, fy);
            d+=part_dist;
            size+=part_size*marker_interdist_small2big;
            cnt++;
        }
        if (markers[13] > 0 && markers[15] > 0 ){
            float part_dist, part_size;
            std::tie(part_dist, part_size) = getDistanceAndSizeFrom2Markers(corners.at(markers[13]),corners.at(markers[15]),marker_interdist_small,f,fx, fy);
            d+=part_dist;
            size+=part_size*marker_interdist_small2big;
            cnt++;
        }
        if (markers[16] > 0 && markers[18] > 0 ){
            float part_dist, part_size;
            std::tie(part_dist, part_size) = getDistanceAndSizeFrom2Markers(corners.at(markers[16]),corners.at(markers[18]),marker_interdist_small,f,fx, fy);
            d+=part_dist;
            size+=part_size*marker_interdist_small2big;
            cnt++;
        }

        if (markers[10] > 0 && markers[16] > 0 ){
            float part_dist, part_size;
            std::tie(part_dist, part_size) = getDistanceAndSizeFrom2Markers(corners.at(markers[10]),corners.at(markers[16]),marker_interdist_small,f,fx, fy);
            d+=part_dist;
            size+=part_size*marker_interdist_small2big;
            cnt++;
        }
        if (markers[11] > 0 && markers[17] > 0 ){
            float part_dist, part_size;
            std::tie(part_dist, part_size) = getDistanceAndSizeFrom2Markers(corners.at(markers[11]),corners.at(markers[17]),marker_interdist_small,f,fx, fy);
            d+=part_dist;
            size+=part_size*marker_interdist_small2big;
            cnt++;
        }
        if (markers[12] > 0 && markers[18] > 0 ){
            float part_dist, part_size;
            std::tie(part_dist, part_size) = getDistanceAndSizeFrom2Markers(corners.at(markers[12]),corners.at(markers[18]),marker_interdist_small,f,fx, fy);
            d+=part_dist;
            size+=part_size*marker_interdist_small2big;
            cnt++;
        }
    }
    float marker_size,marker_distance ;
    if (cnt > 0) {
        marker_distance = d / cnt;
        marker_size = size / cnt;
    } else {
        marker_distance = -1;
        marker_size = -1;
    }
    return std::make_tuple(marker_size,marker_distance);
}



void MarkerOnShipPlugin::OnNewFrame(const unsigned char *image,
                                    unsigned int width, unsigned int height,
                                    unsigned int depth [[maybe_unused]], const std::string &format [[maybe_unused]])
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

    float marker_size,marker_distance;
    std::tie(marker_size,marker_distance) =  update_distance(markers,corners);

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
                uint ii = (i + window_size-1) % window_size;
                mx += sqrtf(powf(mahist.at(i).x - mahist.at(ii).x,2));
                my += sqrtf(powf(mahist.at(i).y - mahist.at(ii).y,2));
            }
        }
        //std::cout << "movvar x: " << mx << "  y: " << my << std::endl;

        if (mahist.size() > window_size+1)
            mahist.erase(mahist.begin());


        // prepare message
        moving_marker_message.set_time_usec(0); // will be filled in simulator_mavlink.cpp
        moving_marker_message.set_size(marker_size);
        moving_marker_message.set_distance(marker_distance);
        moving_marker_message.set_angle_x(x);
        moving_marker_message.set_angle_y(y);
        moving_marker_message.set_movvar_x(mx);
        moving_marker_message.set_movvar_y(my);
        moving_marker_pub_->Publish(moving_marker_message);

    } else {
        moving_marker_message.set_time_usec(0); // will be filled in simulator_mavlink.cpp
        moving_marker_message.set_size(-1);
        moving_marker_message.set_distance(-1);
        moving_marker_message.set_angle_x(0);
        moving_marker_message.set_angle_y(0);
        moving_marker_message.set_movvar_x(-1);
        moving_marker_message.set_movvar_y(-1);
        moving_marker_pub_->Publish(moving_marker_message);
    }
    if (!(cnt % 3)) {
        cv::Mat fs;
        cv::resize(frame,fs,cv::Size(frame.cols/2,frame.rows/2));
        cv::imwrite("testlalala.png",fs);
    }
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=2 ts=2 : */
