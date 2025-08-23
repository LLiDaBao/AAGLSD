#ifndef __UTILITY_HPP__
#define __UTILITY_HPP__


#include <opencv2/opencv.hpp>
#include <memory>
#include <limits.h>
#include <math.h>
#include "segments.hpp"


constexpr double MIN_GRAD_THRESH = 5.22;	// According to LSD, we choose angle-tolerance = 22.5 degree, and p = 1/8.
constexpr double DIST_TOLERANCE = 1.5;
constexpr double ANG_TOLERANCE = 22.5;

// Kernel type for calculating gradient operation.
enum KernelType
{
	MASK2x2 = 0,
	SOBEL
};


struct GradientInfo
{
	cv::Mat gradx;
	cv::Mat grady;
	cv::Mat mag;
	cv::Mat ori;
};

typedef std::shared_ptr<GradientInfo> GradientInfoPtr;


/* @brief Return cv::Vec4f line angle, its range is in [-pi/2, pi/2] or [-90, 90]. */
inline
float lineAngle(const cv::Vec4f& _line, bool isDegree = true)
{
	float ang = std::atanf(_line[1] / _line[0]);
	if (isDegree)
		ang *= (180.f / CV_PI);
	return ang;
}


/* @brief Return the difference between 2 angle. It's range is in [0, 90]. */
inline
float angleDiff(float lhs, float rhs)
{
	// input angle is in range [0, 180] or [-90, 90] degree.
	float diff = std::abs(lhs - rhs);

	if (diff > 90.0f)
		diff = 180.0 - diff;

	return diff;
}


/* @brief Check cv::Rect if is in matrix. */
inline
bool checkRect(
	const cv::Size& size,
	const cv::Rect& rect
)
{
	int _y = rect.y + rect.height;
	int _x = rect.x + rect.width;
	
	return rect.x >= 0 && rect.y >= 0 && _x < size.width && _y < size.height;
}


/* @brief Check input type and matrix type if is match. */
template<typename T>
bool checkDataType(const cv::Mat& src)
{
	if (src.empty())
		return false;

	std::string matType;
	if (src.type() == CV_8UC1)
	{
		return std::is_same<uchar, T>::value;
	}
	else if (src.type() == CV_32S)
	{
		return std::is_same<int, T>::value;
	}
	else if (src.type() == CV_32F)
	{
		return std::is_same<float, T>::value;
	}
	else if (src.type() == CV_64F)
	{
		return std::is_same<double, T>::value;
	}

	// unsupported type
	return false;
}


/* @brief Visit the given pixel in matrix. */
template <typename T = float>
T atPixel(const cv::Mat& src, const Pixel& px)
{
	const T* ptr = src.ptr<T>();
	return ptr[int(px.x) + int(px.y) * src.cols];
}

template <typename T = float>
T& atPixel(cv::Mat& src, const Pixel& px)
{
	T* ptr = (T*)src.ptr<T>();
	return ptr[int(px.x) + int(px.y) * src.cols];
}


/* @brief Visit the given point in matrix. */
template <typename T = float>
T atPixel(const cv::Mat& src, const cv::Point& pt)
{
	const T* ptr = src.ptr<T>();
	return ptr[pt.x + pt.y * src.cols];
}

template <typename T = float>
T& atPixel(cv::Mat& src, const cv::Point& pt)
{
	T* ptr = src.ptr<T>();
	return ptr[pt.x + pt.y * src.cols];
}


/* @brief Calculate magnitude-map from input gradient-map. */
template <typename T = float>
bool calcMagnitude(
	const cv::Mat& gradx,
	const cv::Mat& grady,
	cv::Mat&       mag,
	bool           useL1 = true
)
{
	// check type
	if (!checkDataType<T>(gradx) || !checkDataType<T>(grady))
		return false;

	// check input map size
	if (gradx.size != grady.size)
		return false;
	
	mag = cv::Mat_<T>::zeros(gradx.rows, gradx.cols);
	
	size_t sz = gradx.rows * gradx.cols;
	const T* ptrX = (const T*)gradx.ptr();
	const T* ptrY = (const T*)grady.ptr();

	for (size_t i = 0; i != sz; ++i)
	{
		mag.ptr<T>()[i] = useL1 ? std::abs(ptrX[i]) + std::abs(ptrY[i]) :
			std::sqrt(ptrX[i] * ptrX[i] + ptrY[i] * ptrY[i]);

		if (mag.ptr<T>()[i] < MIN_GRAD_THRESH)
			mag.ptr<T>()[i] = 0;
	}

	return true;
}


/* @brief Calculate orientation-map from input gradient-map. 
The range is 0 to 180 degree. */
template <typename T = float>
bool calcOrientation(
	const cv::Mat& gradx,
	const cv::Mat& grady,
	cv::Mat&       ori
)
{
	// check type
	if (!checkDataType<T>(gradx) || !checkDataType<T>(grady))
		return false;

	// check input map size
	if (gradx.size != grady.size)
		return false;
	
	ori = cv::Mat_<T>::zeros(gradx.rows, gradx.cols);

	size_t sz = gradx.rows * gradx.cols;
	const T* ptrX = (const T*)gradx.ptr();
	const T* ptrY = (const T*)grady.ptr();

	for (size_t i = 0; i != sz; ++i)
	{
		auto& ref = ori.ptr<T>()[i];
		if (ptrX[i] < 0 && ptrY[i] < 0 || ptrX[i] > 0 && ptrY[i] < 0)
			ref = std::atan2(-ptrY[i], -ptrX[i]) * 180.0 / CV_PI;
		else
			ref = std::atan2(ptrY[i], ptrX[i]) * 180.0 / CV_PI;
	}
	
	return true;
}


/* @brief Calculate mean and std in input region. */
template <typename T>
bool getMeanStd(
	const cv::Mat&  src,
	const cv::Rect& rect,
	double&         mean,
	double&         std
)
{
	// check type
	if (!checkDataType<T>(src))
		return false;

	if (!checkRect(src.size(), rect))
		return false;

	cv::Mat roi = src(rect).clone();

	size_t sz = rect.width * rect.height;
	T* ptr = (T*)roi.ptr<T>();

	mean = 0.0;
	for (size_t i = 0; i != sz; ++i)
	{
		mean += ptr[i];
	}
	mean /= sz;
	
	std = 0.0;
	for (size_t i = 0; i != sz; ++i)
	{
		std += 1.0 * (ptr[i] - mean) * (ptr[i] - mean);
	}
	std = std::sqrt(std / sz);

	return true;
}


/* @brief Calculate Kurtosis. */
template <typename T>
double kurtosis(
	const std::vector<T>& data,
	double                mean,
	double                std
)
{
	assert(!data.empty());
	assert(std > 0);

	double kurt = 0.0;
	for (auto it = data.cbegin(); it != data.cend(); ++it)
	{
		kurt += std::pow((*it - mean) / std, 4);
	}

	return kurt / data.size();
}


/* @brief Calculate Kurtosis. */
template <typename T>
double skewness(
	const std::vector<T>& data,
	double                mean,
	double                std
)
{
	assert(!data.empty() && std > 0);

	double skew = 0.0;
	for (auto it = data.cbegin(); it != data.cend(); ++it)
	{
		skew += std::pow((*it - mean) / std, 3);
	}

	return skew / data.size();
}


/* @brief Calculate gradient information. */
extern 
bool calcGradInfo(
	const cv::Mat& src,
	GradientInfo*  pGradInfo,
	int            kernelType = 0
);


/* @brief Canny non-maximal suppress. */
extern 
void NMS(
	const GradientInfo* pGradInfo,
	PixelList&          anchorPixels
);


/* @brief Returns the line pixels using the Bresenham Algorithm:
 * https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm */
extern 
void bresenham(
	const Pixel& px0,
	const Pixel& px1,
	PixelList& pixels
);


/* @brief Return a bounding rectangle of the point-set. */
extern
cv::Rect pointsBoundingRect(
	const cv::Size&               size,
	const std::vector<cv::Point>& points
);


/* @brief Return a bounding rectangle of the line segment. */
extern
cv::Rect segmentBoundingRect(
	const cv::Size&    size,
	const LineSegment& segment
);


/* @brief Number of False Alarm(NFA). */
extern
bool NFA(
	const cv::Mat&     src,
	const LineSegment& seg,
	int                N,
	float              p
);


/*---------------------- Histogram ----------------------*/

struct HistInfo
{
	std::vector<double> hist;

	double lowerB = 0.0;
	double upperB = 0.0;

	int bins = 0;

	int numElements = 0;

	void normalize()
	{
		if (numElements > 0)
			for (auto& num : hist)
				num /= numElements;
	}
};

typedef std::shared_ptr<HistInfo> HistInfoPtr;


/* @brief Create histogram information pointer. */
HistInfoPtr createHistInfo(
	int    bins,
	double lowerB,
	double upperB
);


/* @brief Draw histogram. */
void drawHistogram(
	cv::Mat& histImg,
	const HistInfoPtr& histPtr,
	int                binWidth,
	double             maxVal = 1.0,
	double             interval = 1.0,
	const cv::Scalar& color = cv::Scalar(0, 255, 0)
);


template <typename T = float>
HistInfoPtr segmentHist(
	const cv::Mat& src,
	const LineSegment& segment,
	int                bins,
	double             lowerB,
	double             upperB
)
{
	HistInfoPtr pHist = createHistInfo(bins, lowerB, upperB);

	double binSize = (upperB - lowerB) / bins;

	PixelList pixels;
	bresenham(segment.begPx, segment.endPx, pixels);

	for (const auto& px : pixels)
	{
		double val = atPixel<T>(src, px);

		int binInd = (val + lowerB) / binSize;
		if (binInd >= bins)
			binInd = bins - 1;
		pHist->hist[binInd] += 1.0;
		++pHist->numElements;
	}

	return pHist;
}


template <typename T = float>
HistInfoPtr imgHist(
	const cv::Mat& src,
	int                bins,
	double             lowerB,
	double             upperB
)
{
	HistInfoPtr pHist = createHistInfo(bins, lowerB, upperB);

	double binSize = (upperB - lowerB) / bins;

	T* ptr = (T*)src.ptr<T>();
	size_t sz = src.rows * src.cols;
	for (size_t i = 0; i != sz; ++i)
	{
		double val = ptr[i];

		int binInd = (val + lowerB) / binSize;
		if (binInd >= bins)
			binInd = bins - 1;
		pHist->hist[binInd] += 1.0;
		++pHist->numElements;
	}

	return pHist;
}


#endif
