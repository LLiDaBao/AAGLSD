#ifndef __DRAW_UTILS__
#define __DRAW_UTILS__


#include <opencv2/opencv.hpp>
#include "lsd.hpp"
#include <fstream>
#include <unordered_set>


/* @brief Generate random bgr. */
inline cv::Vec3b&& randBGR()
{
	return std::move(cv::Vec3b(rand() % 256, rand() % 256, rand() % 256));
}


/* @brief Draw Rectangle. */
extern
void drawRect(
	cv::Mat&          canvas,
	const cv::Rect&   rect,
	const cv::Scalar& color = cv::Scalar(0, 0, 255)
);


extern void drawPixelList(
	cv::Mat&         canvas, 
	PixelList&       pxList, 
	int              groupNum = 1,
	bool             useRand = false,
	const cv::Vec3b& color = cv::Vec3b()
);


/* @brief Draw line segment. */
extern cv::Mat drawLineSegments(
	const LineSegList& lineSegments,
	const cv::Size&    imgSize = cv::Size(640, 480),
	const cv::Vec3b& color = cv::Vec3b()
);


/* @brief Draw line segment. */
extern void drawLineSegments(
	cv::Mat& canvas,
	const LineSegList& lineSegments,
	bool               useRand = true,
	const cv::Vec3b&   color = cv::Vec3b()
);


/* @brief Draw the YorkUrban dataset. */
cv::Mat drawYUK(
	const std::string& txtname,
	const char         splitText = ' ',
	const cv::Size&    imgSize = cv::Size(640, 480)
);

extern void extractYUK(
	const std::string& txtname,
	LineSegList&       segments,
	const char         splitText = '\t'
);


#endif // !__DRAW_UTILS__

