#ifndef DEPTHVISTA_WRAPPER_NODELET_H
#define DEPTHVISTA_WRAPPER_NODELET_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud2.h>

#include <depthvista/camera.hpp>

#include <sensor_msgs/point_cloud2_iterator.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/distortion_models.h>
#include <image_transport/image_transport.h>

#include <depthvista_camera/DepthVistaConfig.h>

#include <memory>
#include <mutex>
#include <iostream>
#include <fstream>

namespace depthvista_camera {

class DepthVistaNodelet : public nodelet::Nodelet {

public:
    DepthVistaNodelet();
    virtual ~DepthVistaNodelet();

protected:
    virtual void onInit();

private:
  	depthvista::Camera camera;

	int mDevId = 0;
	bool mEnableDepth = 0;
	bool mEnableRgb = 0;
	bool mEnableIr = 0;

	depthvista::CameraStaticParams mParams;
  	depthvista::CameraDynamicParams mDynamicParams;
  	depthvista::CalibrationParams mCalibParams;

	ros::NodeHandle mNhNs;

	// Publishers
    image_transport::CameraPublisher mPubRgb;
    image_transport::CameraPublisher mPubDepthRawConv;
    image_transport::CameraPublisher mPubDepthConv;

    ros::Publisher mPubPointCloud;
	ros::Subscriber mSubPointCloud;

	sensor_msgs::CameraInfoPtr mRgbCamInfoMsg;
	sensor_msgs::CameraInfoPtr mDepthRawConvCamInfoMsg;

	sensor_msgs::ImagePtr imgMsgPtrRGB;
	sensor_msgs::ImagePtr imgMsgPtrDepth;

	// Timers
    ros::Timer mVideoDepthTimer;
	double mVideoDepthFreq = 30.;

	// Dynamic reconfigure
    boost::recursive_mutex mDynServerMutex; // To avoid Dynamic Reconfigure Server warning
    boost::shared_ptr<dynamic_reconfigure::Server<depthvista_camera::DepthVistaConfig>> mDynRecServer;

	std::mutex mDynParMutex;
	void updateDynamicReconfigure();	
	void callback_dynamicReconf(depthvista_camera::DepthVistaConfig& config, uint32_t level);

	void callback_pubVideoDepth(const ros::TimerEvent& e);

	void publish_points(const sensor_msgs::ImageConstPtr& msg);
}; // class DepthVistaNodelet

} // namespace depthvista_camera

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(depthvista_camera::DepthVistaNodelet, nodelet::Nodelet);

#endif // DEPTHVISTA_WRAPPER_NODELET_H
