#ifndef __LSD_HPP__
#define __LSD_HPP__


#include <algorithm>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <unordered_set>

#include "utilities.hpp"
#include "segments.hpp"


/* @brief Pseudo-sort the input image's each pixel. */
template <typename T = float>
void pseudoSort(
	const cv::Mat&              src,
	std::vector<PixelLinkList>& pxLists,
	int                         bins = 512
)
{
	if (src.empty() || src.channels() > 1)
		return;

	pxLists.clear();
	pxLists.resize(bins);

	// max gradient magnitude is set to 255.
	double binStep = 255.0 / bins;

	const T* ptr = (T*)src.ptr<T>();
	size_t sz = src.rows * src.cols;

	for (size_t ind = 0; ind != sz; ++ind)
	{
		const auto& val = ptr[ind];

		int binInd = val / binStep;
		binInd = binInd >= bins ? bins - 1 : binInd;	// avoid out of range

		pxLists[binInd].emplace_back(
			/*x*/ind % src.cols, /*y*/ind / src.cols, val);
	}

	// descending sort
	for (size_t ind = 0; ind != bins; ++ind)
	{
		pxLists[ind].sort([](const Pixel& lhs, 
			const Pixel& rhs)->bool {return lhs.val > rhs.val; });
	}

	return;
}


/* @brief Pixel test for Edge Drawing. */
extern bool isAnchorED(
	const GradientInfo* pGradInfo,
	const Pixel&        px,
	float               anchorThresh
);


/* @brief ED's method for anchors extraction. */
extern void extractAnchorED(
	const GradientInfo*               pGradInfo,
	const std::vector<PixelLinkList>& pxLinkLists,
	PixelList&                        anchorPixels,
	float                             anchorThresh = 8.0f
);





/* @brief Extract aligned anchors. */
extern void extractAlignedAnchors(
	const GradientInfo*               pGradInfo,
	const std::vector<PixelLinkList>& pxLinkLists,
	PixelList&                        alignedAnchors,
	float                             anchorThresh = 8.0f,
	float                             angleTolerance = 11.25f
);


/* @brief Walk to next pixel. */
extern Pixel walkToNextPixel(
	const GradientInfo* pGradInfo, 
	const cv::Vec4f&    line,
	const Pixel&        currPx, 
	float               prevAng,
	bool                posDir
);


extern Pixel jump(
	const cv::Mat&          ori,
	const cv::Mat&          labels,
	const PixelList&        alignedAnchors,
	const Pixel&            begPx,
	std::vector<cv::Point>& points,
	cv::Vec4f&              line,
	std::vector<bool>&      used,
	int                     jumpStep = 3,
	bool                    posDir = true
);


/* @brief Extend line from aligned anchors. */
extern LineSegment extend(
	const GradientInfo* pGradInfo,
	const PixelList&    alignedAnchors,
	cv::Mat&            labels,
	std::vector<bool>&  used,
	int                 groupInd
);


/* @brief My routing method. */
extern void routing(
	const GradientInfo* pGradInfo,
	const PixelList&    alignedAnchors,
	LineSegList&        lineSegments
);


/* @brief Validate a line by its aligned-point density. */
extern bool densityValidate(
	const cv::Mat& ori,
	const LineSegment& seg,
	float              densityThresh = 0.7F
);


/* @brief Extract line segment from strong anchors. p.s. Metohd1 */
extern void extractLineSegment(
	const GradientInfo* pGradInfo,
	const PixelList& strongPixels,
	LineSegList&     lineSegments,
	float            distTolerance = 1.42f
);


#endif // !__LSD_HPP__

