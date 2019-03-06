#ifndef _GAZEBO_IRLOCK_PLUGIN_HH_
#define _GAZEBO_IRLOCK_PLUGIN_HH_

#include <string>

#include "gazebo/common/Plugin.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/util/system.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"

#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/rendering/Camera.hh"


#include "IRLock.pb.h"

#include <iostream>

using namespace std;

namespace gazebo
{
  class GAZEBO_VISIBLE MovingIRLockPlugin : public SensorPlugin
  {
    public:
      MovingIRLockPlugin();
      virtual ~MovingIRLockPlugin();
      virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
      virtual void OnNewFrame(const unsigned char *image,
                              unsigned int width, unsigned int height,
                              unsigned int depth, const std::string &format);

    protected:
      sensors::CameraSensorPtr camera;
      rendering::CameraPtr rcamera;
      unsigned int width, height, depth;
      float rate;
      std::string format;

    private:
      event::ConnectionPtr newFrameConnection;
      transport::PublisherPtr irlock_pub_;
      transport::NodePtr node_handle_;
      sensor_msgs::msgs::IRLock irlock_message;
      std::string namespace_;

  };
}
#endif
