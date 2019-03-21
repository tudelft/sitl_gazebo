#ifndef _GAZEBO_MARKER_ON_SHIP_PLUGIN_HH_
#define _GAZEBO_MARKER_ON_SHIP_PLUGIN_HH_

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

#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/features2d.hpp>
#include "opencv2/aruco.hpp"

//IRLOCK specs:
//#define CAM_RES_X 320
//#define CAM_RES_Y 200
//#define CAM_FOV_X 60.f
//#define CAM_FOV_Y 35.f
//#define CAM_TAN_HALF_FOV_X 0.57735026919f // tan(0.5 * 60 * pi/180)
//#define CAM_TAN_HALF_FOV_Y 0.31529878887f // tan(0.5 * 35 * pi/180)

//REALSENSE RGB SPECS:
//#define CAM_RES_X 1920
//#define CAM_RES_Y 1080
//#define CAM_FOV_X 69.4f
//#define CAM_FOV_Y 42.5f
//#define CAM_TAN_HALF_FOV_X 0.69243282809f // tan(0.5 * 69.4 * pi/180)
//#define CAM_TAN_HALF_FOV_Y 0.38887873185f // tan(0.5 * 42.5 * pi/180)

//REALSENSE IR SPECS:
#define CAM_RES_X 848
#define CAM_RES_Y 480
#define CAM_FOV_X 91.2f
#define CAM_FOV_Y 65.5f
#define CAM_TAN_HALF_FOV_X 0.69243282809f // tan(0.5 * 69.4 * pi/180)
#define CAM_TAN_HALF_FOV_Y 0.38887873185f // tan(0.5 * 42.5 * pi/180)


#define CAM_CENTER_X				(CAM_RES_X/2)			// the x-axis center pixel position
#define CAM_CENTER_Y				(CAM_RES_Y/2)			// the y-axis center pixel position
#define CAM_TAN_ANG_PER_PIXEL_X	(2*CAM_TAN_HALF_FOV_X/CAM_RES_X)
#define CAM_TAN_ANG_PER_PIXEL_Y	(2*CAM_TAN_HALF_FOV_Y/CAM_RES_Y)

using namespace std;

namespace gazebo
{
  class GAZEBO_VISIBLE MarkerOnShipPlugin : public SensorPlugin
  {
	public:
	  MarkerOnShipPlugin();
	  virtual ~MarkerOnShipPlugin();
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

	  cv::Ptr<cv::aruco::Dictionary> dictionary;

  };
}
#endif
