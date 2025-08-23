#include "drawutils.hpp"


/* @brief Draw Rectangle. */
void drawRect(
	cv::Mat& canvas,
	const cv::Rect& rect,
	const cv::Scalar& color
)
{
	if (canvas.empty())
		return;

	if (canvas.type() == CV_8UC1)
		cv::cvtColor(canvas, canvas, cv::COLOR_GRAY2BGR);

	cv::Point pt0(rect.x, rect.y);
	cv::Point pt1(rect.x + rect.width, rect.y);
	cv::Point pt2(rect.x + rect.width, rect.y + rect.height);
	cv::Point pt3(rect.x, rect.y + rect.height);

	cv::Point pts[4]{ pt0, pt1, pt2, pt3 };

	for (int i = 0; i != 4; ++i)
	{
		cv::line(canvas, pts[i], pts[(i + 1) % 4], color, 1, cv::LINE_AA);
	}

	return;
}


void drawPixelList(
	cv::Mat&         canvas, 
	PixelList&       pxList, 
	int              groupNum,
	bool             useRand,
	const cv::Vec3b& color
)
{
	if (canvas.empty())
		return;

	if (canvas.type() == CV_8UC1)
		cv::cvtColor(canvas, canvas, cv::COLOR_GRAY2BGR);

	cv::Vec3b bgr(color);
	for (int i = 0; i < pxList.size(); i += groupNum)
	{
		if (useRand)
			bgr = randBGR();
		for (int j = 0; j < groupNum; ++j)
			canvas.ptr<cv::Vec3b>(pxList[i + j].y)[int(pxList[i + j].x)] = bgr;
	}

	return;
}


/* @brief Draw line segment. */
cv::Mat drawLineSegments(
	const LineSegList& lineSegments,
	const cv::Size& imgSize,
	const cv::Vec3b& color
)
{
	cv::Mat result(imgSize, CV_8UC3, cv::Scalar(255, 255, 255));

	cv::Vec3b bgr(0, 0, 0);
	if (color != cv::Vec3b())
		bgr = color;

	for (const auto& seg : lineSegments)
	{
		cv::line(result, (seg.begPx).point(), (seg.endPx).point(), bgr, 1, cv::LINE_AA);
	}

	return result;
}


/* @brief Draw line segment. */
void drawLineSegments(
	cv::Mat&           canvas,
	const LineSegList& lineSegments,
	bool               useRand,
	const cv::Vec3b&   color
)
{
	if (canvas.empty())
		return;

	if (canvas.type() == CV_8UC1)
		cv::cvtColor(canvas, canvas, cv::COLOR_GRAY2BGR);

	cv::Vec3b bgr(color);
	for (const auto& seg : lineSegments)
	{
		if (useRand)
			bgr = randBGR();
		
		cv::line(canvas, (seg.begPx).point(), (seg.endPx).point(), cv::Scalar(bgr[0], bgr[1], bgr[2]), 1, cv::LINE_AA);
	}

	return;
}


/* @brief Draw the YorkUrban dataset. */
cv::Mat drawYUK(
	const std::string& txtname,
	const char splitText,
	const cv::Size& imgSize
)
{
	LineSegList segments;
	extractYUK(txtname, segments, splitText);
	cv::Mat ret(imgSize, CV_8UC1, 255);
	for (const auto& seg : segments)
	{
		cv::line(ret, seg.begPx.point(), seg.endPx.point(), cv::Scalar(0), 1, cv::LINE_AA);
	}

	return ret;
}


/* @brief Draw the YorkUrban dataset. */
cv::Mat drawCompare(
	const std::string& txtname,
	const char splitText,
	const cv::Size& imgSize
)
{
	LineSegList segments;
	extractYUK(txtname, segments, '\t');
	cv::Mat ret(imgSize, CV_8UC1, 255);
	for (const auto& seg : segments)
	{
		cv::line(ret, seg.begPx.point(), seg.endPx.point(), cv::Scalar(0), 1, cv::LINE_AA);
	}

	return ret;
}


void extractYUK(
	const std::string& txtname,
	LineSegList&       segments,
	const char         splitText
)
{
	std::ifstream ifs(txtname);

	if (!ifs.is_open())
	{
		ifs.close();
		return;
	}

	std::string line;
	std::unordered_set<char> iset {'.', 'e', 'E', '+', '-'};

	while (std::getline(ifs, line))
	{
		if (line.empty() || std::isspace(line[0]))
			continue;

		int cnt = 0;
		float val[4] = { FLT_MIN, FLT_MIN, FLT_MIN, FLT_MIN, };

		std::string digit;
		for (size_t ind = 0; ind != line.size(); ++ind)
		{
			if (!std::isdigit(line[ind]) &&
				!std::isspace(line[ind]) &&
				iset.find(line[ind]) == iset.end() &&
				line[ind] != splitText)
				return;

			if (ind == line.size() - 1 && std::isdigit(line[ind]))
				digit.push_back(line[ind]);

			if (line[ind] == splitText || ind == line.size() - 1)
			{
				val[cnt++] = std::stof(digit);
				digit.clear();
				continue;
			}
			digit.push_back(line[ind]);
		}


		segments.emplace_back(val[0], val[1], val[2], val[3]);
	}

	ifs.close();
	return;
}