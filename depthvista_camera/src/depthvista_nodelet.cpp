#include <csignal>
#include <sstream>

#include <sensor_msgs/image_encodings.h>

#include "depthvista_nodelet.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#ifndef NDEBUG
#include <ros/console.h>
#endif

namespace depthvista_camera {

uint8_t Div256(int32_t val) {
  return static_cast<uint8_t>(val >> 8);
}

float clamp(const float val, const float minVal, const float maxVal)
{
	assert(minVal <= maxVal);
	return std::min(maxVal, std::max(minVal, val));
}

void YCbCrToRgb(uint8_t y, uint8_t cb, uint8_t cr, uint8_t* out) {
  const int32_t y_shifted = y - 16;
  const int32_t cr_shifted = cr - 128;
  const int32_t cb_shifted = cb - 128;

  out[0] = Div256(clamp(298 * y_shifted + 409 * cr_shifted, 0, 0xffff));
  out[1] = Div256(clamp(298 * y_shifted - 208 * cr_shifted - 100 * cb_shifted, 0, 0xffff));
  out[2] = Div256(clamp(298 * y_shifted + 516 * cb_shifted, 0, 0xffff));
}

void ConvertUyvyToRgb(uint8_t *uyvy, size_t size, uint8_t *rgb) {
	size_t i;
	for (i = 0; i < size; i+=4) {
		const uint32_t uyvy_block = *(uint32_t *) &uyvy[i];
		const uint8_t y0 = (uyvy_block >> 8) & 0xff;
		const uint8_t cb = (uyvy_block >> 0) & 0xff;
		const uint8_t y1 = (uyvy_block >> 24) & 0xff;
		const uint8_t cr = (uyvy_block >> 16) & 0xff;

		YCbCrToRgb(y0, cb, cr, rgb);
		rgb += 3;
		YCbCrToRgb(y1, cb, cr, rgb);
		rgb += 3;
	}
}

DepthVistaNodelet::DepthVistaNodelet()
    : Nodelet()
{
	std::cerr << "DepthVista Nodelet created" << std::endl;

	// ----> Dynamic Reconfigure parameters
	mDynRecServer = boost::make_shared<dynamic_reconfigure::Server<depthvista_camera::DepthVistaConfig>>(mDynServerMutex);
	dynamic_reconfigure::Server<depthvista_camera::DepthVistaConfig>::CallbackType f;
	f = boost::bind(&DepthVistaNodelet::callback_dynamicReconf, this, _1, _2);
	mDynRecServer->setCallback(f);
	// <---- Dynamic Reconfigure parameters

	imgMsgPtrRGB = boost::make_shared<sensor_msgs::Image>();
	imgMsgPtrDepth = boost::make_shared<sensor_msgs::Image>();
}

DepthVistaNodelet::~DepthVistaNodelet()
{
    std::cerr << "DepthVista Nodelet destroyed" << std::endl;

	camera.stop();
}

void DepthVistaNodelet::onInit()
{
    std::cerr << "DepthVista Nodelet onInit" << std::endl;
	
	mParams.res.w = 640;
	mParams.res.h = 480;
	mParams.dev_id = mDevId;
	mParams.mode = 0;
	if (mEnableDepth)
		mParams.mode |= depthvista::MODE::DEPTH;
	if (mEnableRgb)
		mParams.mode |= depthvista::MODE::RGB;
	if (mEnableIr)
		mParams.mode |= depthvista::MODE::IR;
	mParams.verbose = depthvista::VERBOSITY::NONE;

	camera.init(mParams);

	camera.update_config(mDynamicParams);

	camera.start();

	mCalibParams = camera.get_calibration_details();

	// Create camera info
    mRgbCamInfoMsg.reset(new sensor_msgs::CameraInfo());
    mDepthRawConvCamInfoMsg.reset(new sensor_msgs::CameraInfo());

	// depth calibration data
	mDepthRawConvCamInfoMsg->header.frame_id = "depthvista";
	mDepthRawConvCamInfoMsg->width = mCalibParams.depth_intrinsics.w;
	mDepthRawConvCamInfoMsg->height = mCalibParams.depth_intrinsics.h;
	mDepthRawConvCamInfoMsg->K[0] = mCalibParams.depth_intrinsics.fx;
	mDepthRawConvCamInfoMsg->K[1] = 0.0;
	mDepthRawConvCamInfoMsg->K[2] = mCalibParams.depth_intrinsics.cx;
	mDepthRawConvCamInfoMsg->K[3] = 0.0;
	mDepthRawConvCamInfoMsg->K[4] = mCalibParams.depth_intrinsics.fy;
	mDepthRawConvCamInfoMsg->K[5] = mCalibParams.depth_intrinsics.cy;
	mDepthRawConvCamInfoMsg->K[6] = 0.0;
	mDepthRawConvCamInfoMsg->K[7] = 0.0;
	mDepthRawConvCamInfoMsg->K[8] = 1.0;
	mDepthRawConvCamInfoMsg->P[0] = mCalibParams.depth_intrinsics.fx;
	mDepthRawConvCamInfoMsg->P[1] = 0.0;
	mDepthRawConvCamInfoMsg->P[2] = mCalibParams.depth_intrinsics.cx;
	mDepthRawConvCamInfoMsg->P[3] = 0.0;
	mDepthRawConvCamInfoMsg->P[4] = 0.0;
	mDepthRawConvCamInfoMsg->P[5] = mCalibParams.depth_intrinsics.fy;
	mDepthRawConvCamInfoMsg->P[6] = mCalibParams.depth_intrinsics.cy;
	mDepthRawConvCamInfoMsg->P[7] = 0.0;
	mDepthRawConvCamInfoMsg->P[8] = 0.0;
	mDepthRawConvCamInfoMsg->P[9] = 0.0;
	mDepthRawConvCamInfoMsg->P[10] = 1.0;
	mDepthRawConvCamInfoMsg->P[11] = 0.0;

	// rgb calibration data
	mRgbCamInfoMsg->header.frame_id = "depthvista";
	mRgbCamInfoMsg->width = mCalibParams.rgb_intrinsics.w;
	mRgbCamInfoMsg->height = mCalibParams.rgb_intrinsics.h;
	mRgbCamInfoMsg->K[0] = mCalibParams.rgb_intrinsics.fx;
	mRgbCamInfoMsg->K[1] = 0.0;
	mRgbCamInfoMsg->K[2] = mCalibParams.rgb_intrinsics.cx;
	mRgbCamInfoMsg->K[3] = 0.0;
	mRgbCamInfoMsg->K[4] = mCalibParams.rgb_intrinsics.fy;
	mRgbCamInfoMsg->K[5] = mCalibParams.rgb_intrinsics.cy;
	mRgbCamInfoMsg->K[6] = 0.0;
	mRgbCamInfoMsg->K[7] = 0.0;
	mRgbCamInfoMsg->K[8] = 1.0;
	mRgbCamInfoMsg->P[0] = mCalibParams.rgb_intrinsics.fx;
	mRgbCamInfoMsg->P[1] = 0.0;
	mRgbCamInfoMsg->P[2] = mCalibParams.rgb_intrinsics.cx;
	mRgbCamInfoMsg->P[3] = 0.0;
	mRgbCamInfoMsg->P[4] = 0.0;
	mRgbCamInfoMsg->P[5] = mCalibParams.rgb_intrinsics.fy;
	mRgbCamInfoMsg->P[6] = mCalibParams.rgb_intrinsics.cy;
	mRgbCamInfoMsg->P[7] = 0.0;
	mRgbCamInfoMsg->P[8] = 0.0;
	mRgbCamInfoMsg->P[9] = 0.0;
	mRgbCamInfoMsg->P[10] = 1.0;
	mRgbCamInfoMsg->P[11] = 0.0;

	// Set the video topic names
	std::string rgb_topic = "camera/color/image_rect_raw";
	std::string depth_conv = "camera/depth/image_raw";
	std::string pointcloud_topic = "camera/depth/points";

	// Create all the publishers
    image_transport::ImageTransport it_depthvista(mNhNs);

	mPubRgb = it_depthvista.advertiseCamera(rgb_topic, 1); // rgb
    NODELET_INFO_STREAM("Advertised on topic " << mPubRgb.getTopic());
    NODELET_INFO_STREAM("Advertised on topic " << mPubRgb.getInfoTopic());

	mPubDepthRawConv = it_depthvista.advertiseCamera(depth_conv, 1); // depth raw conv
    NODELET_INFO_STREAM("Advertised on topic " << mPubDepthRawConv.getTopic());
    NODELET_INFO_STREAM("Advertised on topic " << mPubDepthRawConv.getInfoTopic());

	mPubPointCloud = mNhNs.advertise<sensor_msgs::PointCloud2>(pointcloud_topic, 1); // pointcloudxyzrgb
    NODELET_INFO_STREAM("Advertised on topic " << mPubPointCloud.getTopic());

	// Start data publishing timer
    mVideoDepthTimer = mNhNs.createTimer(ros::Duration(1.0 / mVideoDepthFreq), &DepthVistaNodelet::callback_pubVideoDepth, this);
}

void DepthVistaNodelet::updateDynamicReconfigure()
{
    NODELET_INFO_STREAM(" * [DYN] updateDynamicReconfigure");
	depthvista_camera::DepthVistaConfig config;

	mDynParMutex.lock();
	config.range = mDynamicParams.range;
	config.video_dev_id = mDevId;
	config.enable_depth = mEnableDepth;
	config.enable_rgb = mEnableRgb;
	config.enable_ir = mEnableIr;
	mDynParMutex.unlock();

    mDynServerMutex.lock();
    mDynRecServer->updateConfig(config);
    mDynServerMutex.unlock();
}

void DepthVistaNodelet::callback_dynamicReconf(depthvista_camera::DepthVistaConfig& config, uint32_t level)
{
	NODELET_INFO_STREAM(" * [DYN] dynamicReconf VideoDeviceId " 
								<< std::to_string(config.video_dev_id).c_str() << 
								" Depth range " << std::to_string(config.range).c_str());
	mDynParMutex.lock();
	mDevId = config.video_dev_id;
	mEnableDepth = config.enable_depth;
	mEnableRgb = config.enable_rgb;
	mEnableIr = config.enable_ir;
	if (config.range == 0 || config.range == 1) {
		NODELET_INFO_STREAM(" * [DYN] depth range");
		mDynamicParams.range = config.range;
		camera.update_config(mDynamicParams);
	} else NODELET_INFO_STREAM(" * [DYN] invalid depth range");
	mDynParMutex.unlock();
}

void DepthVistaNodelet::publish_points(const sensor_msgs::ImageConstPtr& msg)
{
	float cx = mDepthRawConvCamInfoMsg->K[2], cy = mDepthRawConvCamInfoMsg->K[5];
	float fx = mDepthRawConvCamInfoMsg->K[0], fy = mDepthRawConvCamInfoMsg->K[4];

	sensor_msgs::PointCloud2 pointcloudMsg;
	pointcloudMsg.header = msg->header;
	pointcloudMsg.is_bigendian = msg->is_bigendian;
	pointcloudMsg.is_dense = false;
	pointcloudMsg.width = msg->width;
	pointcloudMsg.height = msg->height;

	if (mParams.mode == depthvista::MODE::DEPTH) {
		sensor_msgs::PointCloud2Modifier modifier(pointcloudMsg);
		modifier.setPointCloud2FieldsByString(1, "xyz");

		sensor_msgs::PointCloud2Iterator<float>iter_x(pointcloudMsg, "x");
		sensor_msgs::PointCloud2Iterator<float>iter_y(pointcloudMsg, "y");
		sensor_msgs::PointCloud2Iterator<float>iter_z(pointcloudMsg, "z");

		uint16_t *depth_data_raw = (uint16_t*)(&msg->data[0]);
		cv::Mat depth_16 = cv::Mat(480, 640, CV_16UC1);
		memcpy(depth_16.data, (char*)&msg->data[0], 640*480*2);
		cv::Mat depthF;

		depth_16.convertTo(depthF, CV_32F, 0.001f);

		for (int v = 0; v < msg->height; v++) {
			for (int u = 0; u < msg->width; u++) {
				int idx = v * msg->width + u;
				uint16_t raw_depth = depth_16.at<uint16_t>(v, u);
				float Z = depthF.at<float>(v, u);
				//float Z = (float)raw_depth * 0.001f;

				if (((mDynamicParams.range == 0) 
							&& (raw_depth < DEPTHVISTA_RANGE_NEAR_MINLIMIT 
								|| raw_depth > DEPTHVISTA_RANGE_NEAR_MAXLIMIT))
						|| ((mDynamicParams.range == 1) 
							&& (raw_depth < DEPTHVISTA_RANGE_FAR_MINLIMIT 
								|| raw_depth > DEPTHVISTA_RANGE_FAR_MAXLIMIT))) {
					*iter_x = *iter_y = *iter_z = 0;
					++iter_x; ++iter_y; ++iter_z;
				} else {
					*iter_x = (u - cx) * Z / fx;
					*iter_y = -((v - cy) * Z / fy);
					*iter_z = -Z;
					++iter_x; ++iter_y; ++iter_z;
				}
			}
		}
	} else {
		sensor_msgs::PointCloud2Modifier modifier(pointcloudMsg);
		modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
		//pointcloudMsg.row_step = pointcloudMsg.width * pointcloudMsg.point_step;
		//pointcloudMsg.data.resize(pointcloudMsg.height * pointcloudMsg.row_step);

		sensor_msgs::PointCloud2Iterator<float>iter_x(pointcloudMsg, "x");
		sensor_msgs::PointCloud2Iterator<float>iter_y(pointcloudMsg, "y");
		sensor_msgs::PointCloud2Iterator<float>iter_z(pointcloudMsg, "z");
		sensor_msgs::PointCloud2Iterator<uint8_t>iter_color(pointcloudMsg, "rgb");

		uint16_t *depth_data_raw = (uint16_t*)(&msg->data[0]);
		uint8_t *rgb_data = (uint8_t*)&imgMsgPtrRGB->data[0];
		float bad_point = std::numeric_limits<float>::quiet_NaN();

		for (int v = 0; v < msg->height; v++) {
			for (int u = 0; u < msg->width; u++) {
				int idx = v * msg->width + u;
				float Z = (depth_data_raw[idx] == 0) ? bad_point : (float)depth_data_raw[idx] * 0.001f;

				if (((mDynamicParams.range == 0) 
							&& (depth_data_raw[idx] < DEPTHVISTA_RANGE_NEAR_MINLIMIT 
								|| depth_data_raw[idx] > DEPTHVISTA_RANGE_NEAR_MAXLIMIT))
						|| ((mDynamicParams.range == 1) 
							&& (depth_data_raw[idx] < DEPTHVISTA_RANGE_FAR_MINLIMIT 
								|| depth_data_raw[idx] > DEPTHVISTA_RANGE_FAR_MAXLIMIT))) {
					*iter_x = *iter_y = *iter_z = 0;
					++iter_x; ++iter_y; ++iter_z;

					iter_color[0] = iter_color[1] = iter_color[2] = 0;
					++iter_color;
					rgb_data += 3;
				} else {
					*iter_x = (u - cx) * Z / fx;
					*iter_y = ((v - cy) * Z / fy);
					*iter_z = Z;
					++iter_x; ++iter_y; ++iter_z;

					// order of rgb is bgr
					iter_color[0] = rgb_data[2];
					iter_color[1] = rgb_data[1];
					iter_color[2] = rgb_data[0];
					++iter_color;
					rgb_data += 3;
				}
			}
		}
	}

	mPubPointCloud.publish(pointcloudMsg);
}

void DepthVistaNodelet::callback_pubVideoDepth(const ros::TimerEvent& e)
{
	uint32_t rgbSubnumber = mPubRgb.getNumSubscribers();
	uint32_t depthSubnumber = mPubDepthRawConv.getNumSubscribers();
	uint32_t pointsSubnumber = mPubPointCloud.getNumSubscribers();

	if (camera.wait_for_frames()) {
		ros::Time t = ros::Time::now();
    	
		if (rgbSubnumber || (pointsSubnumber && (mParams.mode != depthvista::MODE::DEPTH))) { //rgb frame
			depthvista::Frame frame = camera.get_rgb_frame();
			sensor_msgs::ImagePtr imgMsgPtrUVYV = boost::make_shared<sensor_msgs::Image>();

			imgMsgPtrUVYV->header.stamp = t;
  			imgMsgPtrUVYV->header.frame_id = "depthvista";
  			imgMsgPtrUVYV->height = frame.height;
  			imgMsgPtrUVYV->width = frame.width;
			imgMsgPtrRGB->header.stamp = imgMsgPtrUVYV->header.stamp;
  			imgMsgPtrRGB->header.frame_id = imgMsgPtrUVYV->header.frame_id;
  			imgMsgPtrRGB->height = frame.height;
  			imgMsgPtrRGB->width = frame.width;

			size_t sizeUYVY = frame.width * frame.height * 2;
			size_t sizeRGB = frame.width * frame.height * 3;

			imgMsgPtrUVYV->data.resize(sizeUYVY);
			imgMsgPtrRGB->data.resize(sizeRGB);

			memcpy((char*)(&imgMsgPtrUVYV->data[0]), (char*)(&frame.data[0]), sizeUYVY);
			ConvertUyvyToRgb(&imgMsgPtrUVYV->data[0], sizeUYVY, &imgMsgPtrRGB->data[0]);

			imgMsgPtrRGB->is_bigendian = 0;
			imgMsgPtrRGB->step = imgMsgPtrRGB->width * 3;
			imgMsgPtrRGB->encoding = sensor_msgs::image_encodings::RGB8;

			mRgbCamInfoMsg->header.stamp = t;

			if (rgbSubnumber) mPubRgb.publish(imgMsgPtrRGB, mRgbCamInfoMsg);
		}

		if (pointsSubnumber || depthSubnumber) { //depth frame
			depthvista::Frame frame = camera.get_depth_frame();

			imgMsgPtrDepth->header.stamp = t;
  			imgMsgPtrDepth->header.frame_id = "depthvista";
  			imgMsgPtrDepth->height = frame.height;
  			imgMsgPtrDepth->width = frame.width;

			size_t sizeDepth = frame.width * frame.height * 2;
			imgMsgPtrDepth->data.resize(sizeDepth);

			memcpy((char*)(&imgMsgPtrDepth->data[0]), (char*)(&frame.data[0]), sizeDepth);

			imgMsgPtrDepth->is_bigendian = 0;
			imgMsgPtrDepth->step = imgMsgPtrDepth->width * 2;
			imgMsgPtrDepth->encoding = sensor_msgs::image_encodings::TYPE_16UC1;

			mDepthRawConvCamInfoMsg->header.stamp = t;

			if (depthSubnumber) mPubDepthRawConv.publish(imgMsgPtrDepth, mDepthRawConvCamInfoMsg);
			if (pointsSubnumber) publish_points(imgMsgPtrDepth);
		}
	}
}

} // namespace depthvista_camera
