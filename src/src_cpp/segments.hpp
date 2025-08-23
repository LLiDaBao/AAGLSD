#ifndef __SEGMENTS_HPP__
#define __SEGMENTS_HPP__

#include <algorithm>
#include <opencv2/opencv.hpp>


// Image Pixel
class Pixel
{
public:
	Pixel() = default;
	Pixel(const Pixel& _px) : x(_px.x), y(_px.y), val(_px.val) { }
	Pixel(const cv::Point& _pt) : x(_pt.x), y(_pt.y) { }
	Pixel(float _x, float _y) : x(_x), y(_y) { }
	Pixel(float _x, float _y, float _val) : x(_x), y(_y), val(_val) { }


	//overload operator
	//
	bool operator==(const Pixel& _px) const;

	bool operator<(const Pixel& _px) const;

	Pixel& operator=(const Pixel& _px);

	Pixel operator+(const Pixel& _px) const;

	Pixel operator-(const Pixel& _px) const;

	Pixel& operator-=(const Pixel& _px);

	Pixel& operator+=(const Pixel& _px);

	// dot
	float operator*(const Pixel& _px) const;

	// scale vector
	Pixel operator*(float _v) const;

	/* @brief Check index if is valid. */
	inline
	bool isInMatrix(const cv::Mat& src) const
	{
		return int(x) >= 0 && int(y) >= 0 &&
			int(x) < src.cols && int(y) < src.rows;
	}

	/* @brief Convert to cv::Point. */
	inline
	cv::Point point() const
	{
		return cv::Point(this->x, this->y);
	}

	/* @brief Round pixel to nearset. */
	inline
	Pixel round() const
	{
		return Pixel(roundf(x), roundf(y));
	}

public:
	// sub-pixel
	float x = FLT_MIN;
	float y = FLT_MIN;
	float val = FLT_MIN;
};

typedef std::vector<Pixel> PixelList;
typedef std::list<Pixel> PixelLinkList;


// Line Segment
class LineSegment
{
public:
	LineSegment() = default;
	LineSegment(float _x0, float _y0, float _x1, float _y1) : begPx(_x0, _y0), endPx(_x1, _y1) { }
	LineSegment(const Pixel& _beg, const Pixel& _end) : begPx(_beg), endPx(_end) { }
	LineSegment(const cv::Point& _beg, const cv::Point& _end) : begPx(_beg), endPx(_end) { }
	LineSegment(const LineSegment& _seg) : begPx(_seg.begPx), endPx(_seg.endPx) { }
	
	/* @brief Constructor. */
	LineSegment(const cv::Vec4f& _line);

	~LineSegment() {}

	bool operator==(const LineSegment& _seg) const;

	bool operator!=(const LineSegment& _seg) const;

	/* @brief Project given pixel to the line. */
	Pixel project2line(const Pixel& _px) const;

	/* @brief Distance from given pixel to the line. */
	double point2lineDist(const Pixel& _px) const;

	/* @brief Get line angle, its range is in [-PI/2, PI/2] by degree. */
	double angleRad() const
	{
		Pixel px(endPx - begPx);
		return std::atan(px.y / px.x);
	}

	/* @brief Get line angle, its range is in [-90.0, 90.0] by degree. */
	double angleDeg() const
	{
		return angleRad() * 180.0 / CV_PI;
	}

	/* @brief Return segment's length. */
	double length()
	{
		return std::sqrt((endPx - begPx) * (endPx - begPx));
	}

	/* @brief Convert to OpenCV's cv::Vec4f. */
	cv::Vec4f convert() const
	{
		Pixel px(endPx - begPx);
		return cv::Vec4f(px.x, px.y, begPx.x, begPx.y);
	}

public:
	Pixel begPx;
	Pixel endPx;
};

typedef std::vector<LineSegment> LineSegList;


/* @brief Input the y-coordinate value and a line, return the x-coordinate value.
* @param _y: input y-coordinate value.
* @param line: a line with (v_x, v_y, x0, y0).
* @return _x: output x-coordinate value. */
inline
static float retX(double _y, const cv::Vec4f& line)
{
	return (line[0] + 1e-5) / line[1] * (_y - line[3]) + line[2];
}


/* @brief Input the x-coordinate value and a line, return the y-coordinate value.
* @param _x: input x-coordinate value.
* @param line: a line with (v_x, v_y, x0, y0).
* @return _y: output y-coordinate value. */
inline
static float retY(double _x, const cv::Vec4f& line)
{
	return line[1] / (line[0] + 1e-5) * (_x - line[2]) + line[3];
}


/* @brief Perpendicular distance of a segment's mid pixel to another line segment. */
extern
double perpendDist(
	const LineSegment& seg,
	const LineSegment& targetSeg
);


/* @brief Test a prjected pixel if is in a segment. */
extern
bool isInLineSegment(
	const Pixel&       px, 
	const LineSegment& seg
);


#endif // !__SEGMENTS_HPP__

