#include "detect_potholes.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <vector>

std::vector<cv::Point> highlight_potholes(const cv_bridge::CvImagePtr &input)
{
	cv::Mat highlighted_image = input->image;

	cv::cvtColor(highlighted_image, highlighted_image, cv::COLOR_BGR2HSV);

	cv::Mat mask;

	cv::inRange(highlighted_image, cv::Scalar(0, 0, 200), cv::Scalar(0, 0, 255), mask);
	cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);

	cv::bitwise_and(highlighted_image, mask, highlighted_image);

	cv::cvtColor(highlighted_image, highlighted_image, cv::COLOR_BGR2GRAY);
	cv::threshold(highlighted_image, highlighted_image, 0, 255, cv::ThresholdTypes::THRESH_BINARY);

	std::vector<std::vector<cv::Point>> contours;
	cv::Mat contour_img = highlighted_image.clone();

	cv::findContours(contour_img, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

	std::vector<cv::Point> centers;

	for (unsigned int ind = 0; ind < contours.size(); ind++)
	{
		try
		{
			cv::RotatedRect ellipse = cv::fitEllipse(contours[ind]);
			centers.push_back(ellipse.center);
		}
		catch (cv::Exception &exc)
		{
			continue;
		}
	}

	return centers;
}