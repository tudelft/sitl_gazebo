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

#include "gazebo_moving_beacon_plugin.h"

#include <highgui.h>
#include <math.h>
#include <string>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include <ignition/math.hh>

using namespace cv;
using namespace std;

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(MovingBeaconPlugin)

MovingBeaconPlugin::MovingBeaconPlugin()
{

}

MovingBeaconPlugin::~MovingBeaconPlugin()
{
  this->actor.reset();
}

void MovingBeaconPlugin::Load(physics::ModelPtr _model,sdf::ElementPtr _sdf)
{
  if (!_model)
    gzerr << "Invalid model pointer.\n";

  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);

  if (!this->actor) {
    gzerr << "MovingBeaconPlugin requires a MovingBeacon.\n";
  }

  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzwarn << "Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);


  this->updateConnection_actor = (event::Events::ConnectWorldUpdateBegin(
      std::bind(&MovingBeaconPlugin::OnUpdated, this, std::placeholders::_1)));


}

void MovingBeaconPlugin::OnUpdated(const common::UpdateInfo &_info)
{

//    printf("Actor name: %s\n",this->actor->GetName().c_str());

    ignition::math::Pose3d pose = this->actor->WorldPose();

//printf("Bla: %f, %f, %f\n",pose.Pos().X(),pose.Pos().Y(),pose.Pos().Z());

    pose.Pos().Z(5);

    this->actor->SetWorldPose(pose, true, true);

}

void MovingBeaconPlugin::Reset()
{

}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=2 ts=2 : */
