#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
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
	std::string m_frame_id;

public:
	ImageConverter(std::string frame_id) : m_it(m_nh),
										   m_frame_id(frame_id),
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

		geometry_msgs::TransformStamped cam2ground;

		try
		{
			cam2ground = m_tf_buffer.lookupTransform("base_link", m_cam_model.tfFrame(), ros::Time(0));
		}
		catch (tf2::TransformException &exc)
		{
			ROS_WARN("%s", exc.what());
			ros::Duration(0.1).sleep();
			return;
		}

		cv::Point3d unit_vec;

		int count = 1;

		for (auto &pixel : cam_result)
		{
			unit_vec = m_cam_model.projectPixelTo3dRay(pixel);

			double norm_x = unit_vec.x, norm_y = unit_vec.y, norm_z = unit_vec.z, norm;

			norm = sqrt(norm_x*norm_x + norm_y*norm_y + norm_z*norm_z);

			norm_x /= norm;
			norm_y /= norm;
			norm_z /= norm;

			tf2::Quaternion vec_q, tf_q, conj_tf_q;

			vec_q.setValue(norm_x, norm_y, norm_z, 0.0);

			auto temp = cam2ground.transform.rotation;

			tf_q.setValue(temp.x, temp.y, temp.z, temp.w);
			tf_q = tf_q.inverse();

			conj_tf_q.setValue(tf_q.x(), -tf_q.y(), -tf_q.z(), -tf_q.w());

			vec_q = tf_q*vec_q*conj_tf_q;

			double lambda = -cam2ground.transform.translation.z/vec_q.z();

			double x = cam2ground.transform.translation.x + lambda*vec_q.x();
			double y = cam2ground.transform.translation.y + lambda*vec_q.y();

			ROS_INFO("Pothole %d: (%lf, %lf) | Vector: %lf %lf %lf", count, x, y, vec_q.x(), vec_q.y(), vec_q.z());

			ros::Duration(0.1).sleep();

			count++;
		}
	}
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "virat_image_processor");

	ImageConverter img_cvt("camera_link");

	ros::spin();
}