#include "utilities.hpp"


/* @brief Calculate gradient information. */
bool calcGradInfo(
	const cv::Mat& src,
	GradientInfo*  gradInfo,
	int            kernelType
)
{
	if (src.empty() || gradInfo == nullptr)
		return false;

	cv::Mat kx, ky;
	cv::Point anchor(-1, -1);
	switch (kernelType)
	{
	case MASK2x2:
		kx = (cv::Mat_<float>(2, 2) << -1.f, 1.f, -1.f, 1.f);
		ky = (cv::Mat_<float>(2, 2) << -1.f, -1.f, 1.f, 1.f);
		//anchor.x = 0, anchor.y = 0;
		break;

	case SOBEL:
		kx = (cv::Mat_<float>(3, 3) << -1.f, 0.f, 1.f, -2.f, 0.f, 2.f, -1.f, 0.f, 1.f);
		ky = (cv::Mat_<float>(3, 3) << -1.f, -2.f, -1.f, 0.f, 0.f, 0.f, 1.f, 2.f, 1.f);
	default:
		break;
	}

	// calculate gradient
	cv::filter2D(src, gradInfo->gradx, CV_32F, kx, anchor, 0.0, cv::BORDER_REPLICATE);
	cv::filter2D(src, gradInfo->grady, CV_32F, ky, anchor, 0.0, cv::BORDER_REPLICATE);

	// calculate magnitude and orientation
	bool ret = calcMagnitude(gradInfo->gradx, gradInfo->grady, gradInfo->mag, true);

	if (!ret) return false;

	ret = calcOrientation(gradInfo->gradx, gradInfo->grady, gradInfo->ori);

	return ret;
}


/* @brief Canny Non-maximal suppress. */
void NMS(
	const GradientInfo* pGradInfo,
	PixelList&          anchorPixels
)
{
	anchorPixels.clear();

	auto& gradx = pGradInfo->gradx;
	auto& grady = pGradInfo->grady;
	auto& mag = pGradInfo->mag;
	auto& ori = pGradInfo->ori;
	
	if (gradx.size != grady.size)
		return;

	Pixel px1, px2, px3, px4;	// for NMS interpolate

	for (int row = 1; row < gradx.rows - 1; ++row)
	{
		for (int col = 1; col < gradx.cols - 1; ++col)
		{
			const auto& currGx  = gradx.ptr<float>(row)[col];
			const auto& currGy  = grady.ptr<float>(row)[col];
			const auto& currMag = mag.ptr<float>(row)[col];
			const auto& currOri = ori.ptr<float>(row)[col];

			if (currOri < 45.0)
			{
				px1.y = row - 1, px1.x = col - 1;
				px3.y = row + 1, px3.x = col + 1;
				px2.y = row, px2.x = col - 1;
				px4.y = row, px4.x = col + 1;
			}
			else if (currOri >= 45.0 && currOri < 90.0)
			{
				px1.y = row - 1, px1.x = col - 1;
				px3.y = row + 1, px3.x = col + 1;
				px2.y = row - 1, px2.x = col;
				px4.y = row + 1, px4.x = col;
			}
			else if (currOri >= 90.0 && currOri < 135.0)
			{
				px1.y = row - 1, px1.x = col + 1;
				px3.y = row + 1, px3.x = col - 1;
				px2.y = row - 1, px2.x = col;
				px4.y = row + 1, px4.x = col;
			}
			else
			{
				px1.y = row + 1, px1.x = col - 1;
				px3.y = row - 1, px3.x = col + 1;
				px2.y = row, px2.x = col - 1;
				px4.y = row, px4.x = col + 1;
			}

			float w = (currOri >= 45.0 && currOri < 135.0) ? 
				std::abs(currGx / currGy) : std::abs(currGy / currGx);

			float temp1 = w * mag.ptr<float>(px1.y)[int(px1.x)] + 
				(1 - w) * mag.ptr<float>(px2.y)[int(px2.x)];

			float temp2 = w * mag.ptr<float>(px3.y)[int(px3.x)] +
				(1 - w) * mag.ptr<float>(px4.y)[int(px4.x)];

			if (currMag > temp1 && currMag > temp2)
			{
				anchorPixels.emplace_back(col, row, currMag);
			}
		}
	}
	return;
}


/*@brief Returns the line pixels using the Bresenham Algorithm:
 * https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm */
void bresenham(
	const Pixel& px0, 
	const Pixel& px1, 
	PixelList& pixels
) 
{
	pixels.clear();

	int x0 = px0.round().x;
	int y0 = px0.round().y;

	int x1 = px1.round().x;
	int y1 = px1.round().y;

	int dx = x1 - x0, dy = y1 - y0;
	int p = 0, x = x0, y = y0; 

	// Determine the line direction
	int xIncrement = dx < 0 ? -1 : +1;
	int yIncrement = dy < 0 ? -1 : +1;

	dx = std::abs(dx);
	dy = std::abs(dy);

	if (dx >= dy) {
		// Horizontal like line
		p = 2 * dy - dx;
		while (x != x1) {
			pixels.emplace_back(x, y);
			if (p >= 0) {
				y += yIncrement;
				p += 2 * dy - 2 * dx;
			}
			else {
				p += 2 * dy;
			}
			// Increment the axis in which we are moving
			x += xIncrement;
		}  // End of while
	}
	else {
		// Vertical like line
		p = 2 * dx - dy;
		while (y != y1) {
			pixels.emplace_back(x, y);
			if (p >= 0) {
				x += xIncrement;
				p += +2 * dx - 2 * dy;
			}
			else {
				p += 2 * dx;
			}
			// Increment the axis in which we are moving
			y += yIncrement;
		}  // End of while
	}

	pixels.emplace_back(x1, y1);
	return;
}



/* @brief Number of False Alarm(NFA). */
bool NFA(
	const cv::Mat&     src,
	const LineSegment& seg,
	int                N,
	float              p
)
{
	PixelList pixels;
	bresenham(seg.begPx, seg.endPx, pixels);

	int k = 0;
	double numFalse = 0.0;

	float lineAng = std::atanf(
		(seg.begPx - seg.endPx).y / (seg.begPx - seg.endPx).x) * 180.0 / CV_PI;

	for (const auto& px : pixels)
	{
		const auto& ang = atPixel<float>(src, px) - 90.0f;
		if (angleDiff(ang, lineAng) <= ANG_TOLERANCE)
			++k;
	}

	for (int i = k; i <= pixels.size(); ++i)
	{
		int A_ni = 1, A_ii = 1;
		for (int j = 0; j < i; ++j)
		{
			A_ni *= (pixels.size() - j);
			A_ii *= (i - j);
		}

		numFalse += (1.0 * A_ni / A_ii) * std::pow(p, i) *
			std::pow(1 - p, pixels.size() - i);
	}

	return std::pow(N, 4) * numFalse <= 1.0;
}



/* @brief Return a bounding rectangle of the point-set. */
cv::Rect pointsBoundingRect(
	const cv::Size&               size,
	const std::vector<cv::Point>& points
)
{
	if (points.size() < 2)
		return cv::Rect();

	cv::Rect rect = cv::boundingRect(points);
	
	rect.x = MAX(0, rect.x - 1);
	rect.y = MAX(0, rect.y - 1);
	
	rect.width = rect.x + rect.width < size.width ?
		rect.width + 1 : rect.width;
	rect.height = rect.x + rect.width < size.width ?
		rect.height + 1 : rect.height;

	return rect;
}


/* @brief Return a bounding rectangle of the line segment. */
cv::Rect segmentBoundingRect(
	const cv::Size&    size,
	const LineSegment& segment
)
{
	PixelList pixels;
	bresenham(segment.begPx, segment.endPx, pixels);

	std::vector<cv::Point> points;
	for (const auto& px : pixels)
		points.emplace_back(px.point());

	return pointsBoundingRect(size, points);
}





HistInfoPtr createHistInfo(int bins, double lowerB, double upperB)
{
	HistInfoPtr ptr = std::make_shared<HistInfo>();

	ptr->bins = bins;
	ptr->hist.resize(bins, 0);
	ptr->lowerB = lowerB;
	ptr->upperB = upperB;

	return ptr;
};


/* @brief Draw histogram. */
void drawHistogram(
	cv::Mat&           histImg,
	const HistInfoPtr& histPtr,
	int                binWidth,
	double             maxVal,
	double             interval,
	const cv::Scalar&  color
)
{
	if (maxVal < interval)
		return;

	auto& hist = histPtr->hist;
	int height = int(maxVal / interval);
	histImg = cv::Mat::zeros(height, histPtr->bins * binWidth, CV_8UC3);

	for (int i = 0; i != hist.size(); ++i)
	{
		int binHeight = hist[i] / interval;
		cv::rectangle(histImg, cv::Rect(i * binWidth, 0, binWidth, binHeight), color, -1);
	}

	cv::flip(histImg, histImg, 0);

	return;
}