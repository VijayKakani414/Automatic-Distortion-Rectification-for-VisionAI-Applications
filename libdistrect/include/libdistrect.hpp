#ifndef LIBDISTRECT_HPP
#define LIBDISTRECT_HPP

#include <iostream>
#include <string>
#include <vector>
#include <MatlabEngine.hpp>
#include <MatlabDataArray.hpp>
#include <opencv2/opencv.hpp>

namespace distrect
{

	const double RAD_TO_DEG_MULT = 180.0 / 3.141592653589793238463;
	const double FILTER_LENGTH_THRES = 25.0;
	const double FILTER_RAD_DIST_THRES = 50.0;
	const double GROUP_DIST_THRES = 800.0;
	const double GROUP_ANGLE_THRES = 30.0;
	const int MIN_NUM_OF_SELECTED_LINE_GROUPS = 5;
	const double UNDIST_FULL = 1.0;
	const double UNDIST_VALID = 0.0;

	class ILineSegment
	{
	public:
		ILineSegment();
		ILineSegment(const ILineSegment &);
		virtual ~ILineSegment();

		double a, b, sx, sy, ex, ey;
		int segmentNo;
		bool invert;

	protected:
		friend std::ostream &operator<<(std::ostream &os, const ILineSegment &seg)
		{
			os << "("
				<< "SEG_NO: " << seg.segmentNo << ", "
				<< "A: " << seg.a << ", "
				<< "B: " << seg.b << ", "
				<< "SX: " << seg.sx << ", "
				<< "SY: " << seg.sy << ", "
				<< "EX: " << seg.ex << ", "
				<< "EY: " << seg.ey << ", "
				<< "INVERT: " << seg.invert
				<< ")"
				<< std::endl;

			return os;
		}
	};

	typedef struct camera_props_t
	{
		cv::Mat intrinsic_matrix, distortion_params;
	} camera_props;

	typedef std::vector<std::vector<ILineSegment>> LineSegmentList;

	template <class _T>
	_T scalar_mod(_T x, _T y)
	{
		if (y == 0)
		{
			return x;
		}
		return _T(x - floor(x / y) * y);
	};

	template <class _T>
	cv::Mat mod(cv::Mat x, _T y)
	{
		cv::Mat rv(x.size(), x.depth(), Scalar(0));
		rv.forEach<_T>([rv, x, y](_T &val, const int pos[]) {
			val = scalar_mod<_T>(x.at<_T>(pos[0], pos[1]), y);
		});
		return rv;
	}

	template <class _T>
	cv::Mat mod(cv::Mat x, cv::Mat y)
	{
		cv::Mat rv(x.size(), x.depth(), Scalar(0));
		rv.forEach<_T>([rv, x, y](_T &val, const int pos[]) {
			val = scalar_mod<_T>(x.at<_T>(pos[0], pos[1]), y.at<_T>(pos[0], pos[1]));
		});
		return rv;
	}

	inline double radtodegree(double rad)
	{
		return rad * RAD_TO_DEG_MULT;
	}

	class DistortionRectifier
	{
	public:
		DistortionRectifier();
		virtual ~DistortionRectifier();

		/**
		* setImage
		*
		* Function to set the current image.
		*
		* Args:
		*  image(cv::Mat): color image to be set as current image.
		*/
		void setImage(const cv::Mat);

		/**
		* setImage
		*
		* Function to set the current image.
		*
		* Args:
		*  filePath(std::string): file path/name of the image.
		*/
		void setImage(const std::string);

		/**
		* getCurImage
		*
		* Function to get/retreive the current color image.
		*
		* Ret:
		*  image(cv::Mat): current color image. this image can b
		*  manupulated externally/from caller function, the internal
		*  image won't be affected by it.
		*/
		cv::Mat getCurImage();

		/**
		* getCurGrayImage
		*
		* Function to get/retreive the current grayscale image.
		*
		* Ret:
		*  image(cv::Mat): single channel grayscale image. this image can b
		*  manupulated externally/from caller function, the internal
		*  image won't be affected by it.
		*/
		cv::Mat getCurGrayImage();

		/**
		* getMatlabImage
		*
		* Function to convert a `cv::Mat` into `matlab::data::Array`
		* The input image must be grayscale single channel image.
		*
		* Args:
		*  image(cv::Mat): image to be converted into matlab array
		*
		* Ret:
		*  matrix(matlab::data::Array): converted array
		*/
		matlab::data::Array getMatlabImage(const cv::Mat &);

		/**
		* getLineSegments
		*
		* Funtion to get a vector of `vector<ILineSegment>` from the
		* current gray scale image.
		*
		* Ret:
		*  LineSegments(LineSegmentList)
		*/
		LineSegmentList getLineSegments();

		/**
		* filterLineSegments
		*
		* Funtion to filter out unnecessary line segments.
		*
		*
		* Args:
		*  segments(LineSegmentList): input list of segments to be filtered
		*  lengthThres(double): threshold for line length. default 25.0.
		*  radDistThres(double): radius threshold of line segment. default 50.0.
		*
		* Ret:
		*  LineSegments(LineSegmentList)
		*
		*/
		LineSegmentList filterLineSegments(LineSegmentList segments, double lengthThres = FILTER_LENGTH_THRES, double radDistThres = FILTER_RAD_DIST_THRES);

		/**
		* groupLineSegments
		*
		* Funtion to group line segments.
		*
		* Args:
		*  segments(LineSegmentList): segments to be grouped.
		*  distThres(double): distance threshold. default 800.0.
		*  angleThres(double): angle threshold. default 30.0.
		*
		* Ret:
		*  outSegments(LineSegmentList)
		*/
		LineSegmentList groupLineSegments(LineSegmentList segments, double distThres = GROUP_DIST_THRES, double angleThres = GROUP_ANGLE_THRES);

		/**
		* selectLineSegmentGroups
		*
		* Funtion to filter and select fewer groups for calculating k parameters.
		*
		* Args:
		*  segments(LineSegmentList): segments group to be reduced.
		*/
		LineSegmentList selectLineSegmentGroups(LineSegmentList);

		/**
		* getDistortionParams
		*
		* Funtion to get/retreive the camera parameters
		* out of the given line segment list.
		*
		* Args:
		*  segments(LineSegmentList): segmets after filtering and grouping.
		*
		* Ret:
		*  props(camera_props): camera properties structure.
		*/
		camera_props getCameraParams(LineSegmentList);

		/**
		* undistort
		*
		* Convenient function to undistort an image with given camera
		* properties.
		*
		* Args:
		*  props(camera_props)
		*
		* Ret:
		*  img(cv::Mat): undistorted image copy.
		*/
		cv::Mat undistort(camera_props props, double alpha = UNDIST_VALID);

		/**
		* undistort
		*
		* Helper function to perform all operations sequentially
		* and return undistorted image. This function doesn't return
		* or save the camera parameters.
		*
		* Ret:
		*  img(cv::Mat): undistorted image copy.
		*/
		cv::Mat undistort();

	private:
		cv::Mat m_curImage, m_curGrayImage;
		matlab::data::ArrayFactory m_arrayFactory;
		std::unique_ptr<matlab::engine::MATLABEngine> m_matlabEngine;

		void mSetImage(cv::Mat);
		matlab::data::CellArray mGetLineSegments(LineSegmentList);
		inline double mGetLineError(ILineSegment, ILineSegment);
		inline double mGetDifferenceOfAngles(double, double);
		inline double mGetLineSegmentAngle(ILineSegment);

		template <class _T>
		inline matlab::data::Array mCvToMatlabCopy(const cv::Mat &image)
		{
			std::vector<_T> data;
			for (int col = 0; col < image.cols; col++)
			{
				for (int row = 0; row < image.rows; row++)
				{
					data.push_back(image.at<_T>(row, col));
				}
			}
			matlab::data::ArrayDimensions dims;
			dims.push_back(image.rows);
			dims.push_back(image.cols);
			return m_arrayFactory.createArray<_T>(dims, std::initializer_list<_T>(data.data(), data.data() + data.size()));
		}
	};

} // namespace distrect

#endif //LIBDISTRECT_HPP