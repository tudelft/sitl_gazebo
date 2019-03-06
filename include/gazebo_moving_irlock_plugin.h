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



#define IRLOCK_RES_X 320
#define IRLOCK_RES_Y 200

#define IRLOCK_CENTER_X				(IRLOCK_RES_X/2)			// the x-axis center pixel position
#define IRLOCK_CENTER_Y				(IRLOCK_RES_Y/2)			// the y-axis center pixel position

#define IRLOCK_FOV_X (60.0f*M_PI_F/180.0f)
#define IRLOCK_FOV_Y (35.0f*M_PI_F/180.0f)

#define IRLOCK_TAN_HALF_FOV_X 0.57735026919f // tan(0.5 * 60 * pi/180)
#define IRLOCK_TAN_HALF_FOV_Y 0.31529878887f // tan(0.5 * 35 * pi/180)

#define IRLOCK_TAN_ANG_PER_PIXEL_X	(2*IRLOCK_TAN_HALF_FOV_X/IRLOCK_RES_X)
#define IRLOCK_TAN_ANG_PER_PIXEL_Y	(2*IRLOCK_TAN_HALF_FOV_Y/IRLOCK_RES_Y)

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
