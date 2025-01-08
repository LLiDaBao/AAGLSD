#include "alignED.hpp"
#include "utilities.hpp"
#include "segments.hpp"

namespace AED
{

	/* @brief Extract aligned anchors.
	For pixel(x, y), if Gx(x, y) > Gy(x, y), it's Vertical Pixel, else Horizontal Pixel.
	If pixel is aligned, e.g. Horizontal Pixel,
		mag(x + i, y - 1) < mag(x, y) < mag(x + i, y + 1),
		ori(x - 1, y + i) < ori(x,y) < ori(x + 1, y + i)
	and aligned with its neighbor which is perpendicular to gradient orientation. */
	void extractAlignedAnchors(
		const GradientInfo*               pGradInfo,
		const std::vector<PixelLinkList>& pxLinkLists,
		PixelList&                        alignedAnchors,
		float                             anchorThresh,
		float                             angleTolerance
	)
	{
		alignedAnchors.clear();

		auto& gradx = pGradInfo->gradx;
		auto& grady = pGradInfo->grady;
		auto& mag = pGradInfo->mag;
		auto& ori = pGradInfo->ori;

		PixelList localMaxPixels;

		// to avoid multiple anchors share the same pixel.
		std::vector<bool> used(gradx.rows * gradx.cols, false);	

		for (int ind = pxLinkLists.size() - 1; ind >= 0; --ind)
		{
			auto it = pxLinkLists[ind].begin();
			if (it == pxLinkLists[ind].end() || it->val < MIN_GRAD_THRESH)
				continue;

			for (; it != pxLinkLists[ind].end(); ++it)
			{
				const Pixel& px = *it;

				if (used[int(px.x) + int(px.y) * gradx.cols])
					continue;

				if (px.val < MIN_GRAD_THRESH)
					break;

				// Split to 0, 45, 90, 135, 180 deg
				const auto& gradAng = atPixel<float>(ori, px);

				// split pixel to horizontal, vertical, 45-diagonal and 135-diagonal types.
				std::vector<int> dx(6);
				std::vector<int> dy(6);

				Pixel px1, px2;	// temp variable, for local maximal
				Pixel px3, px4;	// temp variable, for aligned pixel 

				// Inspect neighbor pixels, check if it's local maximum.
				if (gradAng >= 22.5f && gradAng < 67.5f) // 45-diagonal gradient orientation
				{
					dx = { 0, -1, -1, 0, 1, 1 };
					dy = { -1, -1, 0, 1, 1, 0 };
				}
				else if (gradAng >= 67.5f && gradAng < 112.5f) // vertical gradient orientation
				{
					dx = { 1, 0, -1, -1, 0, 1 };
					dy = { -1, -1, -1, 1, 1, 1 };
				}
				else if (gradAng >= 112.5f && gradAng < 157.5f) // 135-diagonal gradient orientation
				{
					dx = { -1, -1, 0, 1, 1, 0 };
					dy = { 0,  1, 1, 0, -1, -1 };
				}
				else	// horizontal gradient orientation
				{
					dx = { -1, -1, -1, 1, 1, 1 };
					dy = { -1, 0, 1, 1, 0, -1 };
				}

				bool isLocalMax = true;

				// Travel the top-down or left-right neighbors
				for (int i = 0; i != 6 && isLocalMax; ++i)
				{
					// Check index if is valid
					Pixel currPx(px.x + dx[i], px.y + dy[i]);

					if (!currPx.isInMatrix(gradx))
						break;

					currPx.val = atPixel<float>(mag, currPx);

					// Find local maximum of neighbors
					if (i < 3 && px1.val < currPx.val)
					{
						px1 = currPx;
					}
					if (i >= 3 && px2.val < currPx.val)
					{
						px2 = currPx;
					}

					isLocalMax &= (px.val > currPx.val);
				}

				if (!isLocalMax || px1.val == FLT_MIN || px2.val == FLT_MIN)
					continue;	// no valid pixels were found
				
				if (px.val - px1.val < anchorThresh &&
					px.val - px2.val < anchorThresh)
					continue;	// not local maximal

				// Then, inspect neighbor pixels, check if it's aligned with its neighbors.
				if (gradAng >= 22.5f && gradAng < 67.5f) // -45-diagonal level-line orientation
				{
					dx = { -1, -1, 0, 1, 1, 0 };
					dy = { 0, 1, 1, 0, -1, -1 };
				}
				else if (gradAng > 67.5f && gradAng < 112.5f) // horizontal level-line orientation
				{
					dx = { -1, -1, -1, 1, 1, 1 };
					dy = { -1, 0, 1, 1, 0, -1 };
				}
				else if (gradAng >= 112.5f && gradAng <= 157.5f) // 45-diagonal level-line orientation
				{
					dx = { 0, -1, -1, 0, 1, 1 };
					dy = { -1, -1, 0, 1, 1, 0 };
				}
				else	// vertical level-line orientation
				{
					dx = { 1, 0, -1, -1, 0, 1 };
					dy = { -1, -1, -1, 1, 1, 1 };
				}

				for (int i = 0; i != 6; ++i)
				{
					// Check index if is valid
					Pixel currPx(px.x + dx[i], px.y + dy[i]);

					if (!currPx.isInMatrix(gradx))
						break;

					currPx.val = atPixel<float>(mag, currPx);

					// Find local maximum in left-right or top-down 3 connected components.
					if (i < 3 && px3.val < currPx.val)	// top or left
					{
						px3 = currPx;
					}
					if (i >= 3 && px4.val < currPx.val)	// down or right
					{
						px4 = currPx;
					}
				}

				// Only if valid pixel was found or not used, then
				if (px3.val != FLT_MIN && px4.val != FLT_MIN &&
					used[int(px3.x) + int(px3.y) * gradx.cols] == false &&
					used[int(px4.x) + int(px4.y) * gradx.cols] == false)
				{
					// Test if is aligned
					const auto& ang = atPixel<float>(ori, px);
					const auto& ang1 = atPixel<float>(ori, px3);
					const auto& ang2 = atPixel<float>(ori, px4);

					// if aligned, push pixel and its neighbor to vector
					if (angleDiff(ang, ang1) <= ANG_TOLERANCE ||
						angleDiff(ang, ang2) <= ANG_TOLERANCE)
					{
						alignedAnchors.emplace_back(px3);
						alignedAnchors.emplace_back(px);
						alignedAnchors.emplace_back(px4);

						// set to used
						used[int(px3.x) + int(px3.y) * gradx.cols] = true;
						used[int(px.x) + int(px.y) * gradx.cols] = true;
						used[int(px4.x) + int(px4.y) * gradx.cols] = true;
					}
				}
			}
		}

		return;
	}


	/* @brief Validate a line by its aligned-point density. */
	bool alignedDensityValidate(
		const cv::Mat&     ori,
		const LineSegment& seg,
		float              densityThresh
	)
	{
		PixelList pixels;
		bresenham(seg.begPx, seg.endPx, pixels);

		int totalNum = 0;
		int alignedNum = 0;

		double gradAng = seg.angleDeg() + 90.0;

		for (const auto& px : pixels)
		{
			if (px.isInMatrix(ori))
			{
				++totalNum;
				if (angleDiff(atPixel<float>(ori, px), gradAng) <= ANG_TOLERANCE)
					++alignedNum;
			}
		}

		return totalNum > 0 && 1.0 * alignedNum / totalNum >= densityThresh;
	}


	/* @brief Validate a line by the density of anchor-point in point-set. */
	bool anchorDensityValidate(
		const cv::Mat&     labels,
		const LineSegment& seg,
		float              densityThresh
	)
	{
		PixelList pixels;
		bresenham(seg.begPx, seg.endPx, pixels);

		int totalNum = 0;
		int anchorNum = 0;

		for (const auto& px : pixels)
		{
			if (px.isInMatrix(labels))
			{
				++totalNum;
				if (atPixel<int>(labels, px) != -1)
					++anchorNum;
			}
		}

		return totalNum > 0 && 1.0 * anchorNum / totalNum >= densityThresh;
	}


	/* @brief Walk to next pixel according to line orientation. */
	Pixel walkToNextPixel(
		const GradientInfo* pGradInfo,
		const cv::Vec4f&    prevLine,
		const cv::Vec4f&    line,
		const Pixel&        currPx,
		bool                posDir,
		bool&               reverseFlag
	)
	{
		auto& gradx = pGradInfo->gradx;
		auto& grady = pGradInfo->grady;
		auto& mag = pGradInfo->mag;
		auto& ori = pGradInfo->ori;

		// line angle guided
		double prevAng = lineAngle(prevLine, true);
		double lineAng = lineAngle(line, true);	// [-90.0, 90.0]

		if (prevAng >= -90.0 && prevAng < -67.5 && lineAng >= -67.5 && lineAng < -22.5 ||
			prevAng >= -67.5 && prevAng < -22.5 && lineAng >= -90.0 && lineAng < -67.5)
			reverseFlag = true;

		std::vector<int> dx(3), dy(3);


		if (lineAng >= -67.5 && lineAng < -22.5)	// -45-diagonal
		{
			dx = { 1, 1, 0 };
			dy = { 0, -1, -1 };
			if (!posDir)
			{
				dx = { -1, -1, 0 };
				dy = { 0, 1, 1 };
			}

			if (reverseFlag)
			{
				dx = { -1, -1, 0 };
				dy = { 0, 1, 1 };
				if (!posDir)
				{
					dx = { 1, 1, 0 };
					dy = { 0, -1, -1 };
				}
			}
		}
		else if (lineAng >= -22.5 && lineAng < 22.5)	// horizontal
		{
			dx = { 1, 1, 1 };
			dy = { -1, 0, 1 };
			if (!posDir)
			{
				dx = { -1, -1, -1 };
				dy = { -1, 0, 1 };
			}
		}
		else if (lineAng >= 22.5 && lineAng < 67.5)	// 45-diagonal
		{
			dx = { 0, 1, 1 };
			dy = { 1, 1, 0 };
			if (!posDir)
			{
				dx = { 0, -1, -1 };
				dy = { -1, -1, 0 };
			}
		}
		else // vertical
		{
			dx = { -1, 0, 1 };
			dy = { 1, 1, 1 };
			if (!posDir)
			{
				dx = { -1, 0, 1 };
				dy = { -1, -1, -1 };
			}

			if (reverseFlag)
			{
				dx = { -1, 0, 1 };
				dy = { -1, -1, -1 };
				if (!posDir)
				{
					dx = { -1, 0, 1 };
					dy = { 1, 1, 1 };
				}
			}
		}

		Pixel nextPx;
		// Select the max one as next
		for (int i = 0; i != dx.size(); ++i)
		{
			Pixel temp(currPx.x + dx[i], currPx.y + dy[i]);
			if (!temp.isInMatrix(gradx)) 
				continue;

			temp.val = atPixel<float>(mag, temp);

			if (temp.val > nextPx.val) 
				nextPx = temp;
		}

		if (nextPx.val == FLT_MIN) 
			return Pixel();

		return nextPx;
	}


	/* @brief Meet aligned-group which direction changed. */
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
	)
	{
		bool isHorizontal = currLine[1] / currLine[0] <= 1.0;

		int dx = isHorizontal ? 1 : 0;
		int dy = isHorizontal ? 0 : 1;

		if (!posDir) 
			dx *= -1, dy *= -1;

		Pixel retPx;

		for (int step = remainStep; step > 0; --step)
		{
			Pixel nextPx(begPx + Pixel(dx, dy));
			
			if (isHorizontal)
				nextPx.y = retY(nextPx.x, currLine);
			else
				nextPx.x = retX(nextPx.y, currLine);

			nextPx = nextPx.round();

			if (!nextPx.isInMatrix(labels) || atPixel<bool>(visited, nextPx))
				break;	// out of range

			int nextGroupInd = atPixel<int>(labels, nextPx);
			if (nextGroupInd >= 0 && isLink[nextGroupInd] == false)
			{
				float currLineAng = lineAngle(currLine, true);
				float candidateAng = lineAngle(alignedLines[nextGroupInd], true);

				// only difference between two anchor-lines is less than angle tolerance, then
				if (angleDiff(currLineAng, candidateAng) <= ANG_TOLERANCE)
				{
					isLink[nextGroupInd] = true;

					// add current group of anchors to point-set and update
					for (int i = 0; i != 3; ++i)
					{
						points.emplace_back(alignedAnchors[3 * nextGroupInd + i].point());
						atPixel<bool>(visited, alignedAnchors[3 * nextGroupInd + i]) = true;
					}

					cv::fitLine(points, currLine, cv::DIST_L2, 0, 0.01, 0.01);
					retPx = walkToNextPixel(pGradInfo, prevLine, currLine, 
						alignedAnchors[3 * nextGroupInd + 1], posDir, reverseFlag);
					break;
				}
			}
		}

		return retPx;
	}


	/* @brief Merge near line segment. */
	LineSegment mergeLineSegments(
		const LineSegment& lhs,
		const LineSegment& rhs
	)
	{
		// lambda expression: intersection?
		auto isIntersect = [](const LineSegment& lhs, const LineSegment& rhs) -> bool
		{
			const Pixel& endPt0 = rhs.project2line(lhs.begPx);
			const Pixel& endPt1 = rhs.project2line(lhs.endPx);
			const Pixel& endPt2 = lhs.project2line(rhs.begPx);
			const Pixel& endPt3 = lhs.project2line(rhs.endPx);

			bool dot_pt0_r = (endPt2 - endPt0) * (endPt3 - endPt0) <= 0;
			bool dot_pt1_r = (endPt2 - endPt1) * (endPt3 - endPt1) <= 0;
			bool dot_pt2_l = (endPt0 - endPt2) * (endPt1 - endPt2) <= 0;
			bool dot_pt3_l = (endPt0 - endPt3) * (endPt1 - endPt3) <= 0;

			if (!dot_pt0_r && !dot_pt1_r && !dot_pt2_l && !dot_pt3_l)
				return false;
			else 
				return true;
		};

		// lambda expression: merge 2 segments.
		auto merge = [](const LineSegment& lhs, const LineSegment& rhs) -> LineSegment
		{
			PixelList pixelsL, pixelsR;
			bresenham(lhs.begPx, lhs.endPx, pixelsL);
			bresenham(rhs.begPx, rhs.endPx, pixelsL);

			std::vector<cv::Point> pts;
			for (const auto& px : pixelsL)
				pts.emplace_back(px.point());
			for (const auto& px : pixelsR)
				pts.emplace_back(px.point());

			cv::Vec4f lineRes;
			cv::fitLine(pts, lineRes, cv::DIST_L2, 0.0, 0.01, 0.01);

			// select end point
			Pixel endPt0, endPt1;
			float maxDist = 0.0;
			
			float dist0 = (lhs.begPx - lhs.endPx) * (lhs.begPx - lhs.endPx);
			if (dist0 > maxDist) 
			{ 
				maxDist = dist0; 
				endPt0 = lhs.begPx;
				endPt1 = lhs.endPx;
			}
			float dist1 = (lhs.begPx - rhs.begPx) * (lhs.begPx - rhs.begPx);
			if (dist1 > maxDist)
			{
				maxDist = dist1;
				endPt0 = lhs.begPx;
				endPt1 = rhs.begPx;
			}
			float dist2 = (lhs.begPx - rhs.endPx) * (lhs.begPx - rhs.endPx);
			if (dist2 > maxDist)
			{
				maxDist = dist2;
				endPt0 = lhs.begPx;
				endPt1 = rhs.endPx;
			}
			float dist3 = (rhs.begPx - rhs.endPx) * (rhs.begPx - rhs.endPx);
			if (dist3 > maxDist)
			{
				maxDist = dist3;
				endPt0 = rhs.begPx;
				endPt1 = rhs.endPx;
			}
			float dist4 = (lhs.endPx - rhs.begPx) * (lhs.endPx - rhs.begPx);
			if (dist4 > maxDist)
			{
				maxDist = dist4;
				endPt0 = lhs.endPx;
				endPt1 = rhs.begPx;
			}
			float dist5 = (lhs.endPx - rhs.endPx) * (lhs.endPx - rhs.endPx);
			if (dist5 > maxDist)
			{
				maxDist = dist5;
				endPt0 = lhs.endPx;
				endPt1 = rhs.endPx;
			}

			LineSegment segRes;
			if (std::abs(lineRes[1] / lineRes[0]) > 1.0)
			{
				segRes.begPx = Pixel(retX(endPt0.y, lineRes), endPt0.y);
				segRes.endPx = Pixel(retX(endPt1.y, lineRes), endPt1.y);
			}
			else
			{
				segRes.begPx = Pixel(endPt0.x, retY(endPt0.x, lineRes));
				segRes.endPx = Pixel(endPt1.x, retY(endPt1.x, lineRes));
			}

			Pixel temp0 = segRes.project2line(endPt0);
			Pixel temp1 = segRes.project2line(endPt1);

			segRes.begPx = temp0;
			segRes.endPx = temp1;

			return segRes;
		};

		// test perpendicular center distance
		float distl2r = perpendCenterDist(lhs, rhs);
		float distr2l = perpendCenterDist(rhs, lhs);

		// test angle difference
		float angDiff = angleDiff(lhs.angleDeg(), rhs.angleDeg());

		//default:5, 1.5, 9
		constexpr float ANG_TOLERANCE = 8.0;	
		constexpr float C_DIST_TOLERANCE = 1.5;
		constexpr float END_PT_DIST_TOLERANCE = 12;

		if (angDiff <= ANG_TOLERANCE &&
			distl2r <= C_DIST_TOLERANCE &&
			distr2l <= C_DIST_TOLERANCE)
		{
			// case1: they have intersection
			if (isIntersect(lhs, rhs))
			{
				return merge(lhs, rhs);
			}

			// case2: no intersection, but end points are in range
			else if (
				(lhs.begPx - rhs.begPx) * (lhs.begPx - rhs.begPx) < END_PT_DIST_TOLERANCE * END_PT_DIST_TOLERANCE ||
				(lhs.begPx - rhs.endPx) * (lhs.begPx - rhs.endPx) < END_PT_DIST_TOLERANCE * END_PT_DIST_TOLERANCE ||
				(lhs.endPx - rhs.begPx) * (lhs.endPx - rhs.begPx) < END_PT_DIST_TOLERANCE * END_PT_DIST_TOLERANCE ||
				(lhs.endPx - rhs.endPx) * (lhs.endPx - rhs.endPx) < END_PT_DIST_TOLERANCE * END_PT_DIST_TOLERANCE)
			{
				return merge(lhs, rhs);
			}
		}

		return LineSegment();
	}


	/* @brief Link aligned anchors to other aligned anchors. */
	LineSegment linkAlignedAnchorGroup(
		const GradientInfo*     pGradInfo,
		const PixelList&        alignedAnchors,
		const cv::Mat&          labels,
		LineSegList&            candidateSegments,
		std::vector<cv::Vec4f>& alignedLines,
		std::vector<bool>&      isLink,
		int                     groupInd
	)
	{
		if (isLink[groupInd]) 
			return LineSegment();

		bool reverseFlag = false;

		auto& gradx = pGradInfo->gradx;
		auto& grady = pGradInfo->grady;
		auto& mag = pGradInfo->mag;
		auto& ori = pGradInfo->ori;

		int currGroupInd = groupInd;

		// a pixel if is visted in this round
		cv::Mat_<bool> visited(gradx.size(), false);

		// current linked aligned-anchor group
		std::vector<int> linkIndices;

		// current line segment's number of aligned-group
		int alignedCnt = 0;

		// initialize points set, [cos_theta, sin_theta, x0, y0]
		cv::Vec4f lineRes(alignedLines[groupInd]);
		cv::Vec4f prevLine(lineRes);

		std::vector<cv::Point> pts;	// for cv::fitLine
		for (size_t i = 0; i != 3; ++i)
		{
			pts.emplace_back(alignedAnchors[3 * groupInd + i].point());
			atPixel<bool>(visited, alignedAnchors[3 * groupInd + i]) = true;
		}

		// 2 end-points of a line segment, initialize as the mid-point of aligned anchors.
		Pixel endPx1(walkToNextPixel(pGradInfo, prevLine, lineRes, Pixel(pts[1]), true, reverseFlag));
		Pixel endPx2(walkToNextPixel(pGradInfo, prevLine, lineRes, Pixel(pts[1]), false, reverseFlag));

		// Go toward positive direction, until arriving boundary, 
		// no remain steps, the distance of current pixel to line is greater than tolerance.
		constexpr int REMAIN_STEPS = 7;
		int remainSteps = REMAIN_STEPS; // steps remain
		
		Pixel currPx(endPx1);
		currPx.val = atPixel<float>(mag, endPx1);

		while (remainSteps > 0)
		{
			Pixel nextPx(walkToNextPixel(pGradInfo, prevLine, lineRes, currPx, true, reverseFlag));

			if (nextPx.val == FLT_MIN ||
				atPixel<bool>(visited, nextPx))	// out of matrix or visited
				break;

			// set be visted
			atPixel<bool>(visited, nextPx) = true;

			// Calculate distance point to line
			LineSegment tempSeg(lineRes);
			if (tempSeg.point2lineDist(nextPx) > DIST_TOLERANCE)
				break;

			int nextGroupInd = atPixel<int>(labels, nextPx);

			if (nextGroupInd == -2)	
			{
				// meet an ED anchor,  just put it into point-set and update line
				pts.emplace_back(nextPx.point());
				prevLine = lineRes;
				cv::fitLine(pts, lineRes, cv::DIST_L2, 0, 0.01, 0.01);

				// update status
				currPx = nextPx;
				endPx1 = nextPx;

				currGroupInd = nextGroupInd;

				remainSteps = REMAIN_STEPS + 2;
			}
			else if (nextGroupInd != currGroupInd && nextGroupInd >= 0 && isLink[nextGroupInd] == false)
			{
				// find other not-linked aligned anchors
				// then, check their direction if is aligned
				const cv::Vec4f& candidateLine = alignedLines[nextGroupInd];
				
				float currLineAng = lineAngle(lineRes, true);
				float candidateAng = lineAngle(candidateLine, true);

				// only difference between two anchor-lines is less than angle tolerance, then
				if (angleDiff(currLineAng, candidateAng) <= ANG_TOLERANCE)
				{
					// add current group of anchors to point-set and update
					for (int i = 0; i != 3; ++i)
					{
						pts.emplace_back(alignedAnchors[3 * nextGroupInd + i].point());
						atPixel<bool>(visited, alignedAnchors[3 * nextGroupInd + i]) = true;
					}
					prevLine = lineRes;
					cv::fitLine(pts, lineRes, cv::DIST_L2, 0, 0.01, 0.01);
					
					//cv::fitLine(pts, lineRes, cv::DIST_L2, 0, 0.01, 0.01);

					// update status
					isLink[groupInd] = true;	// only link to other aligned anchors
					isLink[nextGroupInd] = true;
					linkIndices.push_back(nextGroupInd);
					
					endPx1 = alignedAnchors[3 * nextGroupInd + 1];
					currPx = walkToNextPixel(pGradInfo, prevLine, lineRes, endPx1, true, reverseFlag);
					currGroupInd = nextGroupInd;

					remainSteps = REMAIN_STEPS + 4;
					++alignedCnt;
				}
				// we may meet an aligend-anchor group, but direction change
				else
				{
					Pixel tempPx = extendAlongLineDirection(pGradInfo, labels, visited, alignedAnchors, 
						alignedLines, prevLine, lineRes, pts, isLink, nextPx, REMAIN_STEPS, true, reverseFlag);

					if (!(tempPx == Pixel()))
					{
						currPx = tempPx;
						endPx1 = currPx;

						currGroupInd = atPixel<int>(labels, currPx);
						remainSteps = REMAIN_STEPS + 4;
						++alignedCnt;
					}
				}
			}
			else
			{
				//pts.emplace_back(nextPx.point());
				//prevLine = lineRes;
				//cv::fitLine(pts, lineRes, cv::DIST_L2, 0, 0.01, 0.01);
				
				// just update current pixel and steps
				--remainSteps;
				currPx = nextPx;
				//endPx1 = nextPx;

				currGroupInd = nextGroupInd;
			}

		}

		// Go toward negative direction, until arriving boundary, 
		// no remain steps, the distance of current pixel to line is greater than tolerance.
		remainSteps = REMAIN_STEPS; // steps remain
		currPx = endPx2;
		currPx.val = atPixel<float>(mag, endPx2);

		while (remainSteps > 0)
		{
			Pixel nextPx(walkToNextPixel(pGradInfo, prevLine, lineRes, currPx, false, reverseFlag));

			if (nextPx.val == FLT_MIN ||
				atPixel<bool>(visited, nextPx))	// out of matrix or visited
				break;

			// set be visted
			atPixel<bool>(visited, nextPx) = true;

			// Calculate distance point to line
			LineSegment tempSeg(lineRes);
			if (tempSeg.point2lineDist(nextPx) > DIST_TOLERANCE)
				break;

			int nextGroupInd = atPixel<int>(labels, nextPx);

			if (nextGroupInd == -2)
			{
				// meet an ED anchor,  just put it into point-set and update line
				pts.emplace_back(nextPx.point());
				prevLine = lineRes;
				cv::fitLine(pts, lineRes, cv::DIST_L2, 0, 0.01, 0.01);

				// update status
				currPx = nextPx;
				endPx2 = nextPx;

				currGroupInd = nextGroupInd;

				remainSteps = REMAIN_STEPS + 2;
			}
			else if (nextGroupInd != currGroupInd && 
				nextGroupInd >= 0 && isLink[nextGroupInd] == false)
			{
				// find other not-linked aligned anchors
				// then, check their direction if is aligned
				const cv::Vec4f& candidateLine = alignedLines[nextGroupInd];

				float currLineAng = lineAngle(lineRes, true);
				float candidateAng = lineAngle(candidateLine, true);

				// only difference between two anchor-lines is less than angle tolerance, then
				if (angleDiff(currLineAng, candidateAng) <= ANG_TOLERANCE)
				{
					// add current group of anchors to point-set and update
					for (int i = 0; i != 3; ++i)
					{
						pts.emplace_back(alignedAnchors[3 * nextGroupInd + i].point());
						atPixel<bool>(visited, alignedAnchors[3 * nextGroupInd + i]) = true;
					}
					prevLine = lineRes;
					cv::fitLine(pts, lineRes, cv::DIST_L2, 0, 0.01, 0.01);


					// update status
					isLink[nextGroupInd] = true;
					linkIndices.push_back(nextGroupInd);

					endPx2 = alignedAnchors[3 * nextGroupInd + 1];
					currPx = walkToNextPixel(pGradInfo, prevLine, lineRes, endPx2, false, reverseFlag);

					currGroupInd = nextGroupInd;

					remainSteps = REMAIN_STEPS + 4;
					++alignedCnt;
				}
				// we may meet an aligend-anchor group, but direction change
				else
				{
					Pixel tempPx = extendAlongLineDirection(pGradInfo, labels, visited, alignedAnchors,
						alignedLines, prevLine, lineRes, pts, isLink, nextPx, REMAIN_STEPS, false, reverseFlag);

					if (!(tempPx == Pixel()))
					{
						currPx = tempPx;
						endPx2 = currPx;

						currGroupInd = atPixel<int>(labels, currPx);
						remainSteps = REMAIN_STEPS + 4;
						++alignedCnt;
					}
				}
			}
			else
			{
				//pts.emplace_back(nextPx.point());
				//prevLine = lineRes;
				//cv::fitLine(pts, lineRes, cv::DIST_L2, 0, 0.01, 0.01);
				

				// just update current pixel and steps
				--remainSteps;
				currPx = nextPx;
				//endPx2 = nextPx;

				currGroupInd = nextGroupInd;
			}
		}

		LineSegment segRes;

		if (std::abs(lineRes[1] / lineRes[0]) > 1.0)
		{
			segRes.begPx = Pixel(retX(endPx1.y, lineRes), endPx1.y);
			segRes.endPx = Pixel(retX(endPx2.y, lineRes), endPx2.y);
		}
		else
		{
			segRes.begPx = Pixel(endPx1.x, retY(endPx1.x, lineRes));
			segRes.endPx = Pixel(endPx2.x, retY(endPx2.x, lineRes));
		}

		Pixel temp1 = segRes.project2line(endPx1);
		Pixel temp2 = segRes.project2line(endPx2);

		segRes.begPx = temp1;
		segRes.endPx = temp2;

		// filter non-aligned segment
		if (alignedCnt < 1 || 
			segRes.length() < 5 || 
			!anchorDensityValidate(labels, segRes, 0.5))
			return LineSegment();

		// for short or weak segment
		if (alignedCnt < 3 || 
			!alignedDensityValidate(ori, segRes, 0.5))
		{
			for (auto& linkInd : linkIndices)
				isLink[linkInd] = false;

			candidateSegments.emplace_back(segRes);

			return LineSegment();
		}

		return segRes;
	}


	/* @brief My routing method. */
	void detect(
		const GradientInfo* pGradInfo,
		const PixelList&    alignedAnchors,
		const PixelList&    edAnchors,
		LineSegList&        lineSegments,
		LineSegList&        candidateSegments
	)
	{
		auto& gradx = pGradInfo->gradx;
		auto& grady = pGradInfo->grady;
		auto& mag = pGradInfo->mag;
		auto& ori = pGradInfo->ori;

		/* label map, value means:
		-1: background
		-2: ED anchor point
		>=0: aligned anchor point. */
		cv::Mat_<int> labels(gradx.rows, gradx.cols, -1);

		cv::Mat segmentMap = cv::Mat::zeros(gradx.size(), CV_8UC1);

		for (size_t ind = 0; ind != edAnchors.size(); ++ind)
		{
			const auto& px = edAnchors[ind];
			labels.ptr<int>(px.y)[int(px.x)] = -2;
		}

		for (size_t ind = 0; ind != alignedAnchors.size(); ++ind)
		{
			const auto& px = alignedAnchors[ind];
			labels.ptr<int>(px.y)[int(px.x)] = ind / 3;
		}

		// link status
		std::vector<bool> isLink(alignedAnchors.size() / 3, false);

		// aligned-anchor-line
		std::vector<cv::Vec4f> alignedLines(isLink.size(), cv::Vec4f(0, 0, 0, 0));
		for (size_t ind = 0; ind != alignedLines.size(); ++ind)
		{
			for (int i = 0; i != 3; ++i)
			{
				const auto& ang = atPixel<float>(ori, alignedAnchors[3 * ind + i]) - 90.0;

				alignedLines[ind][0] += std::cos(ang * CV_PI / 180.0);
				alignedLines[ind][1] += std::sin(ang * CV_PI / 180.0);
			}
			// middle pixel as init point
			alignedLines[ind][2] = alignedAnchors[3 * ind + 1].x;
			alignedLines[ind][3] = alignedAnchors[3 * ind + 1].y;	
		}

		for (int groupInd = 0; groupInd != isLink.size(); ++groupInd)
		{
			LineSegment seg = linkAlignedAnchorGroup(
				pGradInfo, alignedAnchors, labels, 
				candidateSegments, alignedLines, isLink, groupInd);

			if (seg != LineSegment())
			{
				LineSegment mergedSeg;
				bool isMerged = false;

				for (auto& _seg : lineSegments)
				{
					mergedSeg = mergeLineSegments(_seg, seg);
					if (mergedSeg != LineSegment())
					{
						isMerged = true;
						_seg = mergedSeg;
						//break;
					}
				}

				if (!isMerged)
					lineSegments.emplace_back(seg);
			}
				
		}

		//for (int groupInd = 0; groupInd != isLink.size(); ++groupInd)
		//{
		//	LineSegment seg = linkPixels(
		//		pGradInfo, alignedAnchors, labels, alignedLines, isLink, groupInd);

		//	if (seg != LineSegment())
		//		lineSegments.emplace_back(seg);
		//}

		return;
	}

	/* @brief Validate candidate line segments. */
	void validateCandidateSegments(
		const GradientInfo* pGradInfo,
		LineSegList&        candidateSegments
	)
	{
		auto& mag = pGradInfo->mag;

		LineSegList remains;

		for (const auto& segment : candidateSegments)
		{
			std::vector<double> rectData;
			std::vector<double> segData;

			cv::Rect rect = segmentBoundingRect(mag.size(), segment);

			double rectMean = 0.0;
			double rectStd = 0.0;
			if (!getMeanStd<float>(mag, rect, rectMean, rectStd))
				continue;

			size_t sz = rect.width * rect.height;
			float* ptr = (float*)mag.ptr<float>();


			for (int row = rect.y; row < rect.y + rect.height; ++row)
			{
				for (int col = rect.x; col < rect.x + rect.width; ++col)
				{
					rectData.push_back(mag.ptr<float>(row)[col]);
				}
			}

			double segMean = 0.0;
			double segStd = 0.0;
			PixelList pixels;
			bresenham(segment.begPx, segment.endPx, pixels);
			for (const auto& px : pixels)
			{
				if (px.isInMatrix(mag))
				{
					segData.push_back(atPixel<float>(mag, px));
					segMean += atPixel<float>(mag, px);
				}

			}
			segMean /= pixels.size();

			for (const auto& px : pixels)
			{
				if (px.isInMatrix(mag))
					segStd += (atPixel<float>(mag, px) - segMean) * 
						(atPixel<float>(mag, px) - segMean);
			}
			segStd = std::sqrt(segStd / pixels.size());

			if (rectStd <= 0 || segStd <= 0)
				continue;

			double segKurt = kurtosis<double>(segData, segMean, segStd);
			double rectKurt = kurtosis<double>(rectData, rectMean, rectStd);

			double segSkew = skewness<double>(segData, segMean, segStd);
			double rectSkew = skewness<double>(rectData, rectMean, rectStd);

			if (segKurt - rectKurt > 0 && 
				segSkew < 0 && rectSkew > 0)
			{
				remains.emplace_back(segment);

				// draw histogram
				//{
				//	cv::Mat _roi = mag(rect);

				//	cv::Mat histImg0, histImg1;

				//	auto pHist0 = segmentHist<float>(pGradInfo->mag, segment, 32, 0, 512);
				//	auto pHist1 = imgHist<float>(pGradInfo->mag, 32, 0, 512);

				//	pHist0->normalize(), pHist1->normalize();

				//	drawHistogram(histImg0, pHist0, 2, 1.0, 0.005, cv::Scalar(0, 0, 255));
				//	drawHistogram(histImg1, pHist1, 2, 1.0, 0.005, cv::Scalar(0, 255, 0));
				//}
			}

		}

		candidateSegments = remains;

		return;
	}
}
