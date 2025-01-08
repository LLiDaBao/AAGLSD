#ifndef __ALIGN_EDGE_DRAWING__
#define __ALIGN_EDGE_DRAWING__


#include <opencv2/opencv.hpp>
#include "utilities.hpp"
#include "segments.hpp"


namespace AED
{
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


	/* @brief Merge near line segment */
	extern
	LineSegment mergeLineSegments(
		const LineSegment& lhs,
		const LineSegment& rhs
	);


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
}

#endif // !__ALIGN_EDGE_DRAWING__

