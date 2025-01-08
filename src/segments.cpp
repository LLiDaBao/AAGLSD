#include "segments.hpp"


bool Pixel::operator==(const Pixel& _px) const
{
	return this->val == _px.val &&
		this->x == _px.x &&
		this->y == _px.y;
}

bool Pixel::operator<(const Pixel& _px) const
{
	return this->val > _px.val;
}

Pixel& Pixel::operator=(const Pixel& _px)
{
	this->val = _px.val;
	this->x = _px.x;
	this->y = _px.y;

	return *this;
}

Pixel Pixel::operator+(const Pixel& _px) const
{
	return Pixel(this->x + _px.x, this->y + _px.y);
}

Pixel Pixel::operator-(const Pixel& _px) const
{
	return Pixel(this->x - _px.x, this->y - _px.y);
}

Pixel& Pixel::operator-=(const Pixel& _px)
{
	this->x -= _px.x;
	this->y -= _px.y;
	return *this;
}

Pixel& Pixel::operator+=(const Pixel& _px)
{
	this->x += _px.x;
	this->y += _px.y;
	return *this;
}

// dot
float Pixel::operator*(const Pixel& _px) const
{
	return this->x * _px.x + this->y * _px.y;
}

// scale vector
Pixel Pixel::operator*(float _v) const
{
	return Pixel(this->x * _v, this->y * _v);;
}

/* @brief Constructor. */
LineSegment::LineSegment(const cv::Vec4f& _line)
{
	float begX = 0.0f, begY = 0.0f, endX = 0.0f, endY = 0.0f;
	if (std::abs(_line[1] / _line[0]) > 1.0f)
	{
		begY = 0.0f;
		endY = _line[3];
		begX = retX(begY, _line);
		endX = retX(endY, _line);
	}
	else
	{
		begX = 0.0f;
		endX = _line[2];
		begY = retY(begX, _line);
		endY = retY(endX, _line);
	}

	this->begPx = Pixel(begX, begY);
	this->endPx = Pixel(endX, endY);
}

bool LineSegment::operator==(const LineSegment& _seg) const
{
	return begPx == _seg.begPx && endPx == _seg.endPx;
}

bool LineSegment::operator!=(const LineSegment& _seg) const
{
	return !(*this == _seg);
}

/* @brief Project given pixel to the line. */
Pixel LineSegment::project2line(const Pixel& _px) const
{
	Pixel lineVec(endPx - begPx);
	Pixel tempVec(_px - begPx);

	auto a = tempVec * lineVec;
	auto b = lineVec * lineVec;

	Pixel projPx = lineVec * (tempVec * lineVec / (lineVec * lineVec));

	return begPx + projPx;
}

/* @brief Distance from given pixel to the line. */
double LineSegment::point2lineDist(const Pixel& _px) const
{
	Pixel projPx(project2line(_px));

	return std::sqrt((_px - projPx) * (_px - projPx));
}


/* @brief Perpendicular distance of a segment's mid pixel to another line segment. */
double perpendDist(
	const LineSegment& seg,
	const LineSegment& targetSeg
)
{
	Pixel midPx((seg.begPx + seg.endPx) * 0.5);
	return targetSeg.point2lineDist(midPx);
}


/* @brief Test a prjected pixel if is in a segment. */
bool isInLineSegment(
	const Pixel&       px,
	const LineSegment& seg
)
{
	Pixel projPx = seg.project2line(px);

	Pixel vec0 = seg.endPx - seg.begPx;
	Pixel vec1 = seg.endPx - projPx;

	Pixel vec2 = seg.begPx - seg.endPx;
	Pixel vec3 = seg.begPx - projPx;

	return vec0 * vec1 > 0 && vec2 * vec3 > 0;
}