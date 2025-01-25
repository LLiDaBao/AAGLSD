#include "lsd.hpp"
#include "drawutils.hpp"
#include "segments.hpp"

/* @brief Pixel test for Edge Drawing. */
bool isAnchorED(
	const GradientInfo* pGradInfo,
	const Pixel&        px,
	float               anchorThresh
)
{
	auto& gradx = pGradInfo->gradx;
	auto& grady = pGradInfo->grady;
	auto& mag = pGradInfo->mag;

	if (px.val < MIN_GRAD_THRESH)
		return false;

	// Vertical pixel if ang < 45 degree, else horizontal
	bool isVerticalPx =
		std::abs(gradx.ptr<float>(px.y)[int(px.x)]) >
		std::abs(grady.ptr<float>(px.y)[int(px.x)]);

	// initialize as current point.
	Pixel px1(px), px2(px);
	if (isVerticalPx)
	{
		px1.x -= 1;
		px2.x += 1;
	}
	else
	{
		px1.y -= 1;
		px2.y += 1;
	}

	// if is greater than anchor threshold
	if (px1.isInMatrix(mag) && px2.isInMatrix(mag) &&
		px.val - anchorThresh > mag.ptr<float>(px1.y)[int(px1.x)] &&
		px.val - anchorThresh > mag.ptr<float>(px2.y)[int(px2.x)])

		return true;
	
	return false;
}


/* @brief ED's method for anchors extraction. */
void extractAnchorED(
	const GradientInfo*               pGradInfo,
	const std::vector<PixelLinkList>& pxLinkLists,
	PixelList&                        anchorPixels,
	float                             anchorThresh
)
{
	anchorPixels.clear();

	auto& gradx = pGradInfo->gradx;
	auto& grady = pGradInfo->grady;
	auto& mag = pGradInfo->mag;

	for (int ind = pxLinkLists.size() - 1; ind >= 0; --ind)
	{
		for (auto it = pxLinkLists[ind].begin(); it != pxLinkLists[ind].end(); ++it)
		{
			const Pixel& px = *it;
			
			if(isAnchorED(pGradInfo, px, anchorThresh))
				anchorPixels.emplace_back(px);
		}
	}

	return;
}


/* @brief Extract aligned anchors. */
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

	/*
	* Extract aligned anchors
	* def: 
	* For pixel(x, y), if Gx(x, y) > Gy(x, y), it's Vertical Pixel, else Horizontal Pixel.
	* If pixel is aligned, e.g. Horizontal Pixel, 
	*           mag(x + i, y - 1) < mag(x, y) < mag(x + i, y + 1),
	*           ori(x - 1, y + i) < ori(x,y) < ori(x + 1, y + i)
	* and aligned with its neighbor which is perpendicular to gradient orientation.
	*/

	PixelList localMaxPixels;

	std::vector<bool> used(gradx.rows * gradx.cols, false);	// to avoid multiple anchors share the same pixel.

	for (int ind = pxLinkLists.size() - 1; ind >= 0; --ind)
	{
		for (auto it = pxLinkLists[ind].begin(); it != pxLinkLists[ind].end(); ++it)
		{
			const Pixel& px = *it;

			if (used[int(px.x) + int(px.y) * gradx.cols])
				continue;

			if (px.val < MIN_GRAD_THRESH) 
				break;

			// Split to 0, 45, 90, 135, 180
			const auto& gradAng = ori.ptr<float>(px.y)[int(px.x)];


			// split pixel to horizontal, vertical, 45-diagonal and 135-diagonal types.
			std::vector<int> dx(6);
			std::vector<int> dy(6);

			if (gradAng >= 22.5f && gradAng <= 67.5f) // 135-diagonal level-line orientation
			{
				dx = { -1, -1, 0, 1, 1, 0 };
				dy = { 0, 1, 1, 0, -1, -1 };
			}
			else if (gradAng > 67.5f && gradAng < 115.5f) // horizontal level-line orientation
			{
				dx = { -1, -1, -1, 1, 1, 1 };
				dy = { -1, 0, 1, 1, 0, -1 };
			}
			else if (gradAng >= 115.5f && gradAng <= 157.5f) // 45-diagonal level-line orientation
			{
				dx = { 0, -1, -1, 0, 1, 1 };
				dy = { -1, -1, 0, 1, 1, 0 };
			}
			else	// vertical level-line orientation
			{
				dx = { 1, 0, -1, -1, 0, 1 };
				dy = { -1, -1, -1, 1, 1, 1 };
			}


			Pixel px1, px2;	// temp variable, for aligned pixel 
			Pixel px3, px4;	// temp variable, for local maximal

			// Inspect neighbor pixels, check if it's aligned with its neighbors.
			for (int i = 0; i != 6; ++i)
			{
				// Check index if is valid
				Pixel currPx(px.x + dx[i], px.y + dy[i]);

				if (!currPx.isInMatrix(gradx))
					break;

				currPx.val = atPixel<float>(mag, currPx);

				if (currPx.val < MIN_GRAD_THRESH)
					break;

				// Find local maximum in left-right or top-down 3 connected components.
				if (i < 3 && px1.val < currPx.val)	// top or left
				{
					px1 = currPx;
				}
				if (i >= 3 && px2.val < currPx.val)	// down or right
				{
					px2 = currPx;
				}
			}

			if (px1.val == FLT_MIN || px2.val == FLT_MIN)
				continue;
			
			if (used[int(px1.x) + int(px1.y) * gradx.cols] == true ||
				used[int(px2.x) + int(px2.y) * gradx.cols] == true)
				continue;

			// Test if is aligned
			const auto& ang = atPixel<float>(ori, px);
			const auto& ang1 = atPixel<float>(ori, px1);
			const auto& ang2 = atPixel<float>(ori, px2);

			if (angleDiff(ang, ang1) > angleTolerance ||
				angleDiff(ang, ang2) > angleTolerance)
				continue;// not aligned, continue to next pixel
			

			// Inspect neighbor pixels, check if it's local maximum.
			if (gradAng >= 22.5f && gradAng <= 67.5f) // 45-diagonal gradient orientation
			{
				dx = { 0, -1, -1, 0, 1, 1 };
				dy = { -1, -1, 0, 1, 1, 0 };
			}
			else if (gradAng > 67.5f && gradAng < 115.5f) // vertical gradient orientation
			{
				dx = { 1, 0, -1, -1, 0, 1 };
				dy = { -1, -1, -1, 1, 1, 1 };
			}
			else if (gradAng >= 115.5f && gradAng <= 157.5f) // 135-diagonal gradient orientation
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

				if (currPx.val < MIN_GRAD_THRESH)
					break;

				// Find local maximum of neighbor
				if (i < 3 && px3.val < currPx.val)
				{
					px3 = currPx;
				}
				if (i >= 3 && px4.val < currPx.val)
				{
					px4 = currPx;
				}

				isLocalMax &= (px.val >= currPx.val);
			}

			if (!isLocalMax || px3.val == FLT_MIN || px4.val == FLT_MIN)
				continue;
			
			px3.val /= 3, px4.val /= 3;	// mean

			// Test if is local maximum
			if (isLocalMax && px.val - px3.val >= anchorThresh && px.val - px4.val >= anchorThresh)
			{
				alignedAnchors.emplace_back(px1);
				alignedAnchors.emplace_back(px);
				alignedAnchors.emplace_back(px2);

				// set to used
				used[int(px1.x) + int(px1.y) * gradx.cols] = true;
				used[int(px.x) + int(px.y) * gradx.cols] = true;
				used[int(px2.x) + int(px2.y) * gradx.cols] = true;
			}
		}
	}

	return;
}



Pixel walkToNextPixel(
	const GradientInfo* pGradInfo,
	const cv::Vec4f&    line,
	const Pixel&        currPx,
	float               prevAng,
	bool                posDir
)
{
	auto& gradx = pGradInfo->gradx;
	auto& grady = pGradInfo->grady;
	auto& mag = pGradInfo->mag;
	auto& ori = pGradInfo->ori;

	// line angle guided
	double lineAng = std::atan(line[1] / line[0]) * 180.0 / CV_PI;	// [-90.0, 90.0]

	std::vector<int> dx(3), dy(3);
	if (prevAng >= -90.0 && prevAng < -67.5 && lineAng >= -67.5 && lineAng < -22.5)
	{
		dx = { -1, -1, 0 };
		dy = { 0, 1, 1 };

		if (!posDir)
		{
			dx = { 1, 1, 0 };
			dy = { 0, -1, -1 };
		}
	}
	else if (lineAng >= -67.5 && lineAng < -22.5)	// -45-diagonal
	{
		// Test line is horizontal or vertical in previous step
		dx = { 1, 1, 0 };
		dy = { 0, -1, -1 };

		if (!posDir)
		{
			dx = { -1, -1, 0 };
			dy = { 0, 1, 1 };
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



Pixel jump(
	const cv::Mat&          ori,
	const cv::Mat&          labels,
	const PixelList&        alignedAnchors,
	const Pixel&            begPx,
	std::vector<cv::Point>& points,
	cv::Vec4f&              line,
	std::vector<bool>&      used,
	int                     jumpStep,
	bool                    posDir
)
{
	Pixel retPx;
	if (!begPx.isInMatrix(labels))
		return retPx;

	// should use Bresenham, update later
	bool isHorizontal = std::abs(line[1] / line[0]) <= 1.0;
	bool findAligned = false;
	int dx = 0, dy = 0;

	if (isHorizontal) 
		dx = 1;
	else 
		dy = 1;

	if (!posDir) dx *= -1, dy *= -1;

	for (int i = 1; i <= jumpStep && findAligned == false; ++i)
	{
		Pixel temp;

		if (isHorizontal)
		{
			temp.x = begPx.x + dx * i;
			temp.y = retY(temp.x, line);
		}
		else
		{
			temp.y = begPx.y + dy * i;
			temp.x = retX(temp.y, line);
		}

		temp = temp.round();
		if (!temp.isInMatrix(labels))
			continue;

		int label = atPixel<int>(labels, temp); 
		
		float cosTheta = 0.0f, sinTheta = 0.0f;
		if (label != -1 && !used[label])	// detect aligned anchor
		{
			// initialize points set
			for (int i = 0; i != 3; ++i)
			{
				const auto& ang = atPixel<float>(ori, alignedAnchors[3 * label + i]) - 90.0;
				cosTheta += std::cos(ang * CV_PI / 180.0);
				sinTheta += std::sin(ang * CV_PI / 180.0);
			}

			float anchorAng = std::atan(sinTheta / cosTheta) * 180.0 / CV_PI;
			float lineAng = std::atan(line[1] / line[0]) * 180.0 / CV_PI;

			if (angleDiff(anchorAng, lineAng) <= /*aligned-diff*/ANG_TOLERANCE)
			{
				used[label] = true;

				// select farest pixel as next one
				float dist2begPx = FLT_MIN;
				for (int i = 0; i != 3; ++i)
				{
					const auto& tempPx = alignedAnchors[label * 3 + i];

					points.emplace_back(tempPx.point());

					if ((tempPx - begPx) * (tempPx - begPx) > dist2begPx)
					{
						dist2begPx = (tempPx - begPx) * (tempPx - begPx);
						retPx = tempPx;
					}
				}

				cv::fitLine(points, line, cv::DIST_L2, 0.0, 0.01, 0.01);

				findAligned = true;
			}
		}
	}
	
	return retPx;
}

/* @brief Extend line. */
LineSegment extend(
	const GradientInfo* pGradInfo,
	const PixelList&    alignedAnchors,
	cv::Mat&            labels,
	std::vector<bool>&  used,
	int                 groupInd
)
{
	if (used[groupInd])
		return LineSegment();

	auto& gradx = pGradInfo->gradx;
	auto& grady = pGradInfo->grady;
	auto& mag = pGradInfo->mag;
	auto& ori = pGradInfo->ori;

	std::vector<bool> visited(gradx.rows * gradx.cols, false);

	// initialize points set
	cv::Vec4f lineRes;	// [cos_theta, sin_theta, x0, y0]
	std::vector<cv::Point> pts;
	for (int i = 0; i != 3; ++i)
	{
		const auto& ang = atPixel<float>(ori, alignedAnchors[3 * groupInd + i]) - 90.0;

		lineRes[0] += std::cos(ang * CV_PI / 180.0);
		lineRes[1] += std::sin(ang * CV_PI / 180.0);

		pts.emplace_back(alignedAnchors[3 * groupInd + i].point());
	}
	lineRes[2] = pts[1].x, lineRes[3] = pts[1].y;	// middle one as init point
	visited[pts[1].x + pts[1].y * gradx.cols] = true;

	used[groupInd] = true;

	// Previous line angle, for walk to next pixel
	float prevAng = FLT_MIN;

	// 2 end-points of a line segment
	Pixel endPx1(walkToNextPixel(pGradInfo, lineRes, alignedAnchors[3 * groupInd + 1], prevAng, true));
	Pixel endPx2(walkToNextPixel(pGradInfo, lineRes, alignedAnchors[3 * groupInd + 1], prevAng, false));
	
	int numAddPx = 0;
	LineSegment segRes;	// output

	/* Start to extend, go positive direction firstly. */
	Pixel currPx(endPx1);
	currPx.val = atPixel<float>(mag, endPx1);

	while (currPx.val != FLT_MIN && currPx.isInMatrix(gradx))
	{
		visited[int(currPx.y * gradx.cols + currPx.x)] = true;
		Pixel nextPx(walkToNextPixel(pGradInfo, lineRes, currPx, prevAng, true));

		if (!nextPx.isInMatrix(gradx) || 
			visited[int(nextPx.y * gradx.cols + nextPx.x)] ||
			nextPx.val == FLT_MIN ||
			nextPx.val < MIN_GRAD_THRESH)
			break; // out of matrix's range, break
		
		// angle range is [-90.0, 90.0]
		float nextAng = atPixel<float>(ori, nextPx) - 90.0;
		float lineAng = std::atan(lineRes[1] / lineRes[0]) * 180.0 / CV_PI;

		if (angleDiff(nextAng, lineAng) > ANG_TOLERANCE )	// Detect direction change, or weak pixel
		{
			currPx = jump(ori, labels, alignedAnchors, 
				nextPx, pts, lineRes,  used, 9, true);
			if (currPx == Pixel()) break;

			numAddPx += 3;
			continue;
		}

		// Calculate distance point to line
		LineSegment tempSeg;
		if (std::abs(lineRes[1] / lineRes[0]) > 1.0) // vertical line
		{
			tempSeg.begPx = Pixel(retX(endPx2.y, lineRes), endPx2.y);
			tempSeg.endPx = Pixel(retX(currPx.y, lineRes), currPx.y);
		}
		else // horizontal line
		{
			tempSeg.begPx = Pixel(endPx2.x, retY(endPx2.x, lineRes));
			tempSeg.endPx = Pixel(currPx.x, retY(currPx.x, lineRes));
		}

		if (tempSeg.point2lineDist(nextPx) > DIST_TOLERANCE)
			break;
		
		int label = atPixel<int>(labels, nextPx);
		// add aligned anchors to this group
		if (label != -1)	
		{
			if (used[label] == false)// first detect aligned-anchors group
			{
				used[label] = true;

				// select farest pixel as next one
				float dist2endPx = FLT_MIN;
				for (int i = 0; i != 3; ++i)
				{
					const auto& tempPx = alignedAnchors[label * 3 + i];
					pts.emplace_back(tempPx.point());

					if ((tempPx - endPx2) * (tempPx - endPx2) > dist2endPx)
					{
						dist2endPx = (tempPx - endPx2) * (tempPx - endPx2);
						nextPx = tempPx;
					}
				}

				numAddPx += 3;
			}
		}
		else
		{
			pts.emplace_back(nextPx.x, nextPx.y);
			numAddPx += 1;
		}

		// update current pixel
		currPx = nextPx;	
		endPx1 = currPx;


		prevAng = lineRes[1] / lineRes[0] * 180.0 / CV_PI;
		cv::fitLine(pts, lineRes, cv::DIST_L2, 0, 0.01, 0.01);
	}

	/* Then, extend towards negative direction. */
	currPx = endPx2;
	currPx.val = atPixel<float>(mag, currPx);

	while (currPx.val != FLT_MIN && currPx.isInMatrix(gradx))
	{
		visited[int(currPx.y * gradx.cols + currPx.x)] = true;
		Pixel nextPx(walkToNextPixel(pGradInfo, lineRes, currPx, prevAng, false));

		if (!nextPx.isInMatrix(gradx) || 
			visited[int(nextPx.y * gradx.cols + nextPx.x)] ||
			nextPx.val == FLT_MIN ||
			nextPx.val < MIN_GRAD_THRESH)
			break; // out of matrix's range, break

		// angle range is [-90.0, 90.0]
		float nextAng = atPixel<float>(ori, nextPx) - 90.0;
		float lineAng = std::atan(lineRes[1] / lineRes[0]) * 180.0 / CV_PI;

		if (angleDiff(nextAng, lineAng) > ANG_TOLERANCE)	// Detect direction change
		{
			currPx = jump(ori, labels, alignedAnchors,
				nextPx, pts, lineRes, used, 9, false);
			if (currPx == Pixel()) break;

			numAddPx += 3;
			continue;
		}

		// Calculate distance point to line
		LineSegment tempSeg;
		if (std::abs(lineRes[1] / lineRes[0]) > 1.0) // vertical line
		{
			tempSeg.begPx = Pixel(retX(endPx1.y, lineRes), endPx1.y);
			tempSeg.endPx = Pixel(retX(currPx.y, lineRes), currPx.y);
		}
		else // horizontal line
		{
			tempSeg.begPx = Pixel(endPx1.x, retY(endPx1.x, lineRes));
			tempSeg.endPx = Pixel(currPx.x, retY(currPx.x, lineRes));
		}

		if (tempSeg.point2lineDist(nextPx) > DIST_TOLERANCE)
			break;

		int label = atPixel<int>(labels, nextPx);
		// add aligned anchors to this group
		if (label != -1)
		{
			if (used[label] == false)// first detect aligned-anchors group
			{
				used[label] = true;

				// select farest pixel as next one
				float dist2endPx = FLT_MIN;
				for (int i = 0; i != 3; ++i)
				{
					const auto& tempPx = alignedAnchors[label * 3 + i];
					pts.emplace_back(tempPx.point());

					if ((tempPx - endPx1) * (tempPx - endPx1) > dist2endPx)
					{
						dist2endPx = (tempPx - endPx1) * (tempPx - endPx1);
						nextPx = tempPx;
					}
				}
				numAddPx += 3;
			}
		}
		else
		{
			pts.emplace_back(nextPx.x, nextPx.y);
			numAddPx += 1;
		}

		// update current pixel
		currPx = nextPx;
		endPx2 = currPx;

		prevAng = lineRes[1] / lineRes[0] * 180.0 / CV_PI;
		cv::fitLine(pts, lineRes, cv::DIST_L2, 0, 0.01, 0.01);
	}

	if (numAddPx < 6) return LineSegment();

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

	return segRes;
}


/* @brief My routing method. */
void routing(
	const GradientInfo* pGradInfo,
	const PixelList&    alignedAnchors,
	LineSegList&        lineSegments
)
{
	auto& gradx = pGradInfo->gradx;
	auto& grady = pGradInfo->grady;
	auto& mag = pGradInfo->mag;
	auto& ori = pGradInfo->ori;

	// init label map
	cv::Mat_<int> labels(gradx.rows, gradx.cols, -1);
	for (size_t ind = 0; ind != alignedAnchors.size(); ++ind)
	{
		const auto& px = alignedAnchors[ind];
		labels.ptr<int>(px.y)[int(px.x)] = ind / 3;
	}

	// init 
	std::vector<bool> used(alignedAnchors.size() / 3, false);

	// 从中心幅值最大处的aligned anchor开始，根据其拟合出初始直线 line->(dir, center).
	for (int groupInd = 0; groupInd != used.size(); ++groupInd)
	{
		if (used[groupInd]) continue;
		LineSegment seg = extend(pGradInfo, alignedAnchors, labels, used, groupInd);

		if (seg == LineSegment()) continue;

		lineSegments.emplace_back(seg);
	}

	return;
}



/* @brief Validate a line by its aligned-point density. */
bool densityValidate(
	const cv::Mat& ori,
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


/* @brief Extract line segment from strong anchors. p.s. Metohd1 */
void extractLineSegment(
	const GradientInfo* pGradInfo,
	const PixelList&    strongPixels,
	LineSegList&        lineSegments,
	float               distTolerance
)
{
	// 最小二乘法版本搜索直线
	auto& gradx = pGradInfo->gradx;
	auto& grady = pGradInfo->grady;
	auto& mag = pGradInfo->mag;
	auto& ori = pGradInfo->ori;

	std::vector<int> dx, dy;

	for (size_t i = 0; i < strongPixels.size(); i += 3)
	{
		std::cout << i << "\r";
		const auto& px1 = strongPixels[i];			// top or left pixel
		const auto& px2 = strongPixels[i + 1];		// middle pixel
		const auto& px3 = strongPixels[i + 2];		// down or right pixel

		// to output line segment
		Pixel begPx(px1);
		Pixel endPx(px3);

		// fit initial line by OpenCV API
		std::vector<cv::Point> points;
		points.emplace_back(px1.point());
		points.emplace_back(px2.point());
		points.emplace_back(px3.point());

		cv::Vec4f line;	// output
		cv::fitLine(points, line, cv::DIST_L2, 0.0, 1e-2, 1e-2);

		Pixel currPx(px1);
		float currDist = 0.0f;
		bool isValid = true;

		// Toward Negative Direction

		// Vertical pixel if ang < 45 degree, else horizontal
		if (std::abs(line[1] / line[0]) > 1.0)
		{
			dx = { -1, 0, 1 };
			dy = { -1, -1, -1 };
		}
		else
		{
			dx = { -1, -1, -1 };
			dy = { -1, 0, 1 };
		}

		while (isValid && currDist <= distTolerance && currPx.isInMatrix(mag))
		{
			isValid = false;
			Pixel nextPx;
			float maxMag = FLT_MIN;

			// select the best next point
			for (int i = 0; i != 3; ++i)
			{
				Pixel temp(currPx.x + dx[i], currPx.y + dy[i]);

				isValid |= temp.isInMatrix(mag);
				if (!temp.isInMatrix(mag))
					continue;

				temp.val = mag.ptr<float>(temp.y)[int(temp.x)];
				if (temp.val > 0 && temp.val >= maxMag)
				{
					nextPx = temp;
					maxMag = temp.val;
				}
			}
			
			// Update current pixel
			currPx = nextPx;
			//visited.ptr<uchar>(nextPx.y)[int(nextPx.x)] = 255;

			if (!isValid) break;	// All 3 neighbor pixels are not in matrix.

			Pixel tempPx1, tempPx2;
			if (std::abs(line[1] / line[0]) > 1.0)
			{
				tempPx1.y = 0;
				tempPx2.y = mag.rows - 1;

				tempPx1.x = retX(tempPx1.y, line);
				tempPx2.x = retX(tempPx2.y, line);
			}
			else
			{
				tempPx1.x = 0;
				tempPx2.x = mag.cols - 1;

				tempPx1.y = retY(tempPx1.x, line);
				tempPx2.y = retY(tempPx2.x, line);
			}

			LineSegment tempSeg(tempPx1, tempPx2);

			currDist = tempSeg.point2lineDist(nextPx);
			if (currDist <= distTolerance)
			{
				points.emplace_back(nextPx.point());
				cv::fitLine(points, line, cv::DIST_L2, 0.0, 1e-2, 1e-2);
				begPx = nextPx;	// update begin pixel
			}
		}
		

		// Toward Positive Direction
		if (std::abs(line[1] / line[0]) > 1.0)
		{
			dx = { -1, 0, 1 };
			dy = { 1, 1, 1 };
		}
		else
		{
			dx = { 1, 1, 1 };
			dy = { -1, 0, 1 };
		}

		currPx = px2;
		currDist = 0.0f;
		isValid = true;

		while (isValid && currDist <= distTolerance && currPx.isInMatrix(mag))
		{
			isValid = false;
			Pixel nextPx;
			float maxMag = FLT_MIN;

			// select the best next point
			for (int i = 0; i != 3; ++i)
			{
				Pixel temp(currPx.x + dx[i], currPx.y + dy[i]);

				isValid |= temp.isInMatrix(mag);
				if (!temp.isInMatrix(mag))
					continue;

				temp.val = mag.ptr<float>(temp.y)[int(temp.x)];
				if (temp.val > 0 && temp.val >= maxMag)
				{
					nextPx = temp;
					maxMag = temp.val;
				}
			}

			// Update current pixel
			currPx = nextPx;
			//visited.ptr<uchar>(nextPx.y)[int(nextPx.x)] = 255;

			if (!isValid) break;	// All 3 neighbor pixels are not in matrix.

			Pixel tempPx1, tempPx2;
			if (std::abs(line[1] / line[0]) > 1.0)
			{
				tempPx1.y = 0;
				tempPx2.y = mag.rows - 1;

				tempPx1.x = retX(tempPx1.y, line);
				tempPx2.x = retX(tempPx2.y, line);
			}
			else
			{
				tempPx1.x = 0;
				tempPx2.x = mag.cols - 1;

				tempPx1.y = retY(tempPx1.x, line);
				tempPx2.y = retY(tempPx2.x, line);
			}

			LineSegment tempSeg(tempPx1, tempPx2);

			currDist = tempSeg.point2lineDist(nextPx);
			if (currDist <= distTolerance)
			{
				points.emplace_back(nextPx.point());
				cv::fitLine(points, line, cv::DIST_L2, 0.0, 1e-2, 1e-2);
				endPx = nextPx;	// update begin pixel
			}
		}
		

		// output
		float begx = 0.f, begy = 0.f, endx = 0.f, endy = 0.f;
		if (std::abs(line[1] / line[0]) > 1.0) // true is vertical line, else horizontal.
		{
			begy = begPx.y;
			begx = retX(begy, line);
			endy = endPx.y;
			endx = retX(endy, line);
		}
		else
		{
			begx = begPx.x;
			begy = retY(begx, line);
			endx = endPx.x;
			endy = retY(endPx.x, line);
		}

		lineSegments.emplace_back(Pixel(begx, begy), Pixel(endx, endy));
	}

	return;
}