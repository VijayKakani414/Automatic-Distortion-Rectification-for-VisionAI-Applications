#include <iostream>
#include <string>
#include <libdistrect.hpp>
#include <MatlabDataArray.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;

//const std::string INPUT_IMAGE = "../../images/Image11_fixed.bmp";
const std::string INPUT_IMAGE = "../../images/000931.jpg";


int main(int argc, char **argv)
{
	distrect::DistortionRectifier dr;
	dr.setImage(INPUT_IMAGE);
	double t1 = (double)getTickCount();
	distrect::LineSegmentList segments = dr.getLineSegments();
	std::cout << "found line segments size: " << segments.size() << std::endl;

	distrect::LineSegmentList filteredSegments = dr.filterLineSegments(segments);
	std::cout << "after filtering: " << filteredSegments.size() << std::endl;

	distrect::LineSegmentList groupSegments = dr.groupLineSegments(filteredSegments);
	std::cout << "after grouping: " << groupSegments.size() << std::endl;

	distrect::LineSegmentList finalSegments = dr.selectLineSegmentGroups(groupSegments);
	std::cout << "final size: " << finalSegments.size() << std::endl;

	distrect::camera_props props = dr.getCameraParams(finalSegments);
	t1 = ((double)getTickCount() - t1) / getTickFrequency();
	std::cout << "Time taken for calibration & distortion estimation in seconds: " << t1 << std::endl;
	std::cout << props.intrinsic_matrix << std::endl;
	std::cout << props.distortion_params << std::endl;
	double t2 = (double)getTickCount();
	cv::Mat img = dr.undistort(props,0);
	t2 = ((double)getTickCount() - t2) / getTickFrequency();
	std::cout << "Time taken for undistortion in seconds: " << t2 << std::endl;
	std::cout << "Undistortion (FPS): " << 1/t2 << std::endl;
	cv::imshow("UNDISTORTED", img);
	cv::waitKey(0);
	return 0;
}







//#include <iostream>
//#include <string>
//#include <libdistrect.hpp>
//#include <MatlabDataArray.hpp>
//#include <opencv2/opencv.hpp>
//
////const std::string INPUT_IMAGE = "../../images/Image8_fixed.bmp";
//const std::string INPUT_IMAGE = "../../images/wide_00001.jpg";
////const std::string INPUT_IMAGE = "../../images/000106.jpg";
//
//int main(int argc, char **argv)
//{
//	distrect::DistortionRectifier dr;
//	dr.setImage(INPUT_IMAGE);
//	distrect::LineSegmentList segments = dr.getLineSegments();
//	std::cout << "found line segments size: " << segments.size() << std::endl;
//
//	distrect::LineSegmentList filteredSegments = dr.filterLineSegments(segments);
//	std::cout << "after filtering: " << filteredSegments.size() << std::endl;
//
//	distrect::LineSegmentList groupSegments = dr.groupLineSegments(filteredSegments);
//	std::cout << "after grouping: " << groupSegments.size() << std::endl;
//
//	distrect::LineSegmentList finalSegments = dr.selectLineSegmentGroups(groupSegments);
//	std::cout << "final size: " << finalSegments.size() << std::endl;
//
//	distrect::camera_props props = dr.getCameraParams(finalSegments);
//
//	std::cout << props.intrinsic_matrix << std::endl;
//	std::cout << props.distortion_params << std::endl;
//
//	cv::VideoCapture cap("wide.mp4");
//	while (1)
//	{
//		cv::Mat frame;
//		cap >> frame;
//		if (frame.empty())
//			break;
//		frame = dr.undistort(props, 0);
//		cv::imshow("UNDISTORTED", frame);
//		char c = (char)cv::waitKey(25);
//		if (c == 27)
//			break;
//	}
//	cap.release();
//	cv::destroyAllWindows();
//	return 0;
//}