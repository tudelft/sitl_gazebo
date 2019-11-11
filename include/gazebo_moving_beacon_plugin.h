#ifndef _GAZEBO_MOVING_BEACON_PLUGIN_HH_
#define _GAZEBO_MOVING_BEACON_PLUGIN_HH_

#include <string>

#include "gazebo/common/Plugin.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/util/system.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"

#include "IRLock.pb.h"

#include <iostream>

using namespace std;

namespace gazebo
{
class GAZEBO_VISIBLE MovingBeaconPlugin : public ModelPlugin
{
public:
	MovingBeaconPlugin();
	virtual ~MovingBeaconPlugin();
	virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
	virtual void OnUpdated(const common::UpdateInfo &_info);
	virtual void Reset();

protected:
	physics::ModelPtr model;


private:
	event::ConnectionPtr updateConnection;
	event::ConnectionPtr updateConnection_model;
	transport::PublisherPtr irlock_pub_;
	transport::NodePtr node_handle_;
	physics::WorldPtr world_;

	std::string namespace_;

};
}
#endif
