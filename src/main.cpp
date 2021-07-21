#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <string>
#include "detect_potholes.hpp"

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
	ros::NodeHandle m_nh;
	image_transport::ImageTransport m_it;
	image_transport::CameraSubscriber m_image_sub;
	image_geometry::PinholeCameraModel m_cam_model;
	tf2_ros::Buffer m_tf_buffer;
	tf2_ros::TransformListener m_tf_listener;

public:
	ImageConverter() : m_it(m_nh),
					   m_tf_listener(m_tf_buffer)
	{
		m_image_sub = m_it.subscribeCamera("/virat/camera_top/image_raw", 1, &ImageConverter::image_cvb, this);
	}

	~ImageConverter()
	{
	}

	void image_cvb(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &info_msg)
	{
		cv_bridge::CvImagePtr cv_ptr;

		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception &cv_bridge_exc)
		{
			ROS_ERROR("cv_bridge exception: ", cv_bridge_exc.what());
			return;
		}

		std::vector<cv::Point> cam_result = highlight_potholes(cv_ptr);

		m_cam_model.fromCameraInfo(info_msg);

		geometry_msgs::TransformStamped cam2odom;

		try
		{
			cam2odom = m_tf_buffer.lookupTransform("odom", m_cam_model.tfFrame(), ros::Time(0));
		}
		catch (tf2::TransformException &exc)
		{
			ROS_WARN("%s", exc.what());
			ros::Duration(0.1).sleep();
			return;
		}

		cv::Point3d direction_vec;
		geometry_msgs::Point pos_cam, pos_odom;
		geometry_msgs::Vector3 vec_cam, vec_odom;

		pos_cam.x = 0;
		pos_cam.y = 0;
		pos_cam.z = 0;

		int count = 1;

		for (auto &pixel : cam_result)
		{
			direction_vec = m_cam_model.projectPixelTo3dRay(pixel);

			vec_cam.x = direction_vec.z;
			vec_cam.y = -direction_vec.x;
			vec_cam.z = -direction_vec.y;

			double norm = sqrt(vec_cam.x * vec_cam.x + vec_cam.y * vec_cam.y + vec_cam.z * vec_cam.z);

			vec_cam.x /= norm;
			vec_cam.y /= norm;
			vec_cam.z /= norm;

			tf2::doTransform<geometry_msgs::Point>(pos_cam, pos_odom, cam2odom);
			tf2::doTransform<geometry_msgs::Vector3>(vec_cam, vec_odom, cam2odom);

			norm = sqrt(vec_odom.x * vec_odom.x + vec_odom.y * vec_odom.y + vec_odom.z * vec_odom.z);

			vec_odom.x /= norm;
			vec_odom.y /= norm;
			vec_odom.z /= norm;

			double lambda = -pos_odom.z / vec_odom.z;

			double x = pos_odom.x + lambda * vec_odom.x;
			double y = pos_odom.y + lambda * vec_odom.y;

			ROS_INFO("Pothole %d: (%lf, %lf)", count, x, y);
			// ROS_INFO("Pothole %d: (%lf, %lf) | vec_odom: (%lf, %lf, %lf)", count, x, y, vec_odom.x, vec_odom.y, vec_odom.z);

			ros::Duration(0.1).sleep();

			count++;
		}
	}
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "virat_image_processor");

	ImageConverter img_cvt;

	ros::spin();
}