#ifndef __ALIGN_EDGE_DRAWING__
#define __ALIGN_EDGE_DRAWING__


#include <opencv2/opencv.hpp>
#include "utilities.hpp"
#include "segments.hpp"


namespace AED
{
	struct SegmentInfo
	{
		LineSegment seg;

	};


	/* @brief Pixel test for Edge Drawing. */
	bool isAnchorED(
		const GradientInfo* pGradInfo,
		const Pixel&        px,
		float               anchorThresh
	);


	/* @brief ED's method for anchors extraction. */
	void extractAnchorED(
		const GradientInfo*               pGradInfo,
		const std::vector<PixelLinkList>& pxLinkLists,
		PixelList&                        anchorPixels,
		float                             anchorThresh = 1.f
	);


	/* @brief Extract aligned anchors. */
	extern 
	void extractAlignedAnchors(
		const GradientInfo*               pGradInfo,
		const std::vector<PixelLinkList>& pxLinkLists,
		PixelList&                        alignedAnchors,
		float                             anchorThresh = 3.0f,
		float                             angleTolerance = 22.5f
	);


	/* @brief Validate a line by its aligned-point density. */
	extern
	bool alignedDensityValidate(
		const cv::Mat&     ori,
		const LineSegment& seg,
		float              densityThresh = 0.7f
	);

	bool alignedDensityValidate(
		const cv::Mat&                ori,
		const LineSegment&            seg,
		const std::vector<cv::Point>& pts,
		float                         densityThresh
	);


	/* @brief Validate a line by the density of anchor-point in point-set. */
	extern
	bool anchorDensityValidate(
		const cv::Mat&     labels,
		const LineSegment& seg,
		float              densityThresh = 0.55f
	);

	
	/* @brief Walk to next pixel according to line orientation. */
	extern
	Pixel walkToNextPixel(
		const GradientInfo* pGradInfo,
		const cv::Vec4f&    prevLine,
		const cv::Vec4f&    line,
		const Pixel&        currPx,
		bool                posDir,
		bool&               reverseFlag
	);


	/* @brief Meet aligned-group which direction changed. */
	extern
	Pixel extendAlongLineDirection(
		const GradientInfo*     pGradInfo,
		const cv::Mat&          labels,
		cv::Mat&                visited,
		const PixelList&        alignedAnchors,
		std::vector<cv::Vec4f>& alignedLines,
		const cv::Vec4f&        prevLine,
		cv::Vec4f&              currLine,
		std::vector<cv::Point>& points,
		std::vector<bool>&      isLink,
		const Pixel&            begPx,
		int                     remainStep,
		bool                    posDir,
		bool&                   reverseFlag
	);


	/* @brief Link aligned anchors to other aligned anchors. */
	extern
	LineSegment linkAlignedAnchorGroup(
		const GradientInfo*     pGradInfo,
		const PixelList&        alignedAnchors,
		cv::Mat&                visited,
		const cv::Mat&          labels,
		LineSegList&            candidateSegments,
		std::vector<cv::Vec4f>& alignedLines,
		std::vector<bool>&      isLink,
		int                     groupInd
	);


	/* @brief Link anchors from un-linked-anchor-group by Edge-Drawing. */
	extern
	LineSegment linkPixels(
		const GradientInfo*     pGradInfo,
		const PixelList&        alignedAnchors,
		const cv::Mat&          labels,
		std::vector<cv::Vec4f>& alignedLines,
		std::vector<bool>&      isLink,
		int                     groupInd
	);


	/* @brief Merge near line segment */



	/* @brief My routing method. */
	extern
	void detect(
		const GradientInfo* pGradInfo,
		const PixelList&    alignedAnchors,
		const PixelList&    normalAnchors,
		LineSegList&        lineSegments,
		LineSegList&        candidateSegments
	);


	/* @brief Validate candidate line segments. */
	extern
	void validateCandidateSegments(
		const GradientInfo* pGradInfo,
		LineSegList&        candidateSegments
	);


	/* @brief Pseudo-sort the input image's each pixel. */
	template <typename T = float>
	void pseudoSort(
		const cv::Mat& src,
		std::vector<PixelLinkList>& pxLists,
		int                         bins = 1024
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
		//for (size_t ind = 0; ind != bins; ++ind)
		//{
		//	pxLists[ind].sort([](const Pixel& lhs,
		//		const Pixel& rhs)->bool {return lhs.val > rhs.val; });
		//}

		return;
	}
}

#endif // !__ALIGN_EDGE_DRAWING__

