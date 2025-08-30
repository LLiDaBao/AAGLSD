#include <opencv2/opencv.hpp>
#include <iostream>
#include "utilities.hpp"
#include "lsd.hpp"
#include "iofile.hpp"
#include "drawutils.hpp"
#include "alignED.hpp"


// if define, we evaluate on the YorkUrban dataset
#define YORK_URBAN_EVALUATE


int main(void)
 {

#ifdef YORK_URBAN_EVALUATE
	std::vector<std::string> filenames;
	// your path that saved images,
	// e.g. listDirRecursively(filenames, "../datasets/YorkUrbanDB", ".jpg");

	listDirRecursively(filenames, "your image path", "suffix");

	int counter = 0;

	for (const auto& imgPath : filenames)
	{
		std::string labelname = imgPath.substr(imgPath.find_last_of('/') + 1);
		labelname = labelname.substr(0, labelname.find_last_of('.'));
		
		LineSegList gt;
		// your folder where saves the labels,
		// e.g. extractYUK("../data/yuk-linelet-labels/" + labelname + ".txt", gt);
		extractYUK("your label path" + labelname + ".txt", gt);

		std::cout << "\r" << imgPath;

		cv::Mat testImg = cv::imread(imgPath, 0);	// grayscale
		cv::Mat canvas = cv::imread(imgPath, cv::IMREAD_COLOR);	// color
		cv::Mat tempImg = testImg.clone();
		GradientInfoPtr pGradInfo = std::make_shared<GradientInfo>();

		cv::GaussianBlur(testImg, testImg, cv::Size(5, 5), 1.0);

		calcGradInfo(testImg, pGradInfo.get());

		std::vector<PixelLinkList> pxLists;
		AED::pseudoSort<float>(pGradInfo->mag, pxLists);
		PixelList anchors, anchorsED;

		/* TEST */
		AED::extractAlignedAnchors(pGradInfo.get(), pxLists, anchors);
		//AED::extractAnchorED(pGradInfo.get(), pxLists, anchorsED);
		NMS(pGradInfo.get(), anchorsED);
		drawPixelList(testImg, anchors, 3, true);
		drawPixelList(tempImg, anchorsED, 1, true);
		
		LineSegList lineSegments;
		LineSegList candidateSegments;
		AED::detect(pGradInfo.get(), anchors, anchorsED, lineSegments, candidateSegments);

		AED::validateCandidateSegments(pGradInfo.get(), candidateSegments);

		cv::Mat res = drawLineSegments(lineSegments, testImg.size());

		// visualization
		drawLineSegments(res, candidateSegments, false, cv::Vec3b(0, 0, 255));
		
		drawLineSegments(canvas, candidateSegments, false, cv::Vec3b(0, 0, 255));
		
		//drawLineSegments(res, lineSegments, false, cv::Vec3b(0, 0, 255));

		// save results
 		std::string imgname = imgPath.substr(imgPath.find_last_of('/') + 1);

		// e.g. cv::imwrite("F:/projects-learning/line-segment-detect/data/yuk-cmp/" + imgname, res);
		// e.g. write2txt(lineSegments, "F:/projects-learning/line-segment-detect/data/results", imgPath);
		cv::imwrite("your path" + imgname, res);
		write2txt(lineSegments, "your path", imgPath);
	}
#endif

#ifndef YORK_URBAN_EVALUATE

	std::vector<std::string> imgNames;

	std::string HPATCHES_DIR = "your hpatches path";
	std::string imgList = "your image list path saving all image names, e.g. 'data/hpatches/list.txt'";

	readFileNames(imgList, imgNames);


	for (int i = 0; i != imgNames.size(); ++i)
	{
		const std::string& imgPath = HPATCHES_DIR + imgNames[i];
		std::cout << "\rReading " << imgPath << "  ";

		cv::Mat testImg = cv::imread(imgPath, 0);	// grayscale
		cv::Mat canvas = cv::imread(imgPath, cv::IMREAD_COLOR);	// color
		cv::Mat tempImg = testImg.clone();

		GradientInfoPtr pGradInfo = std::make_shared<GradientInfo>();
		cv::GaussianBlur(testImg, testImg, cv::Size(5, 5), 1.0);
		calcGradInfo(testImg, pGradInfo.get());

		std::vector<PixelLinkList> pxLists;
		AED::pseudoSort<float>(pGradInfo->mag, pxLists);
		PixelList anchors, anchorsED;

		/* TEST */
		AED::extractAlignedAnchors(pGradInfo.get(), pxLists, anchors);
		//AED::extractAnchorED(pGradInfo.get(), pxLists, anchorsED);
		NMS(pGradInfo.get(), anchorsED);

		// draw anchors
		//drawPixelList(testImg, anchors, 3, true);
		//drawPixelList(tempImg, anchorsED, 1, true);

		LineSegList lineSegments;
		LineSegList candidateSegments;
		AED::detect(pGradInfo.get(), anchors, anchorsED, lineSegments, candidateSegments);

		//AED::validateCandidateSegments(pGradInfo.get(), candidateSegments);
		//for (const auto& seg : candidateSegments)
		//	lineSegments.emplace_back(seg);


		// draw line segments
		//cv::Mat res = drawLineSegments(lineSegments, testImg.size());
		//drawLineSegments(res, candidateSegments, false, cv::Vec3b(0, 0, 255));
		drawLineSegments(canvas, lineSegments, true, cv::Vec3b(0, 0, 255));

		std::string dstDir = "your output path";
		int ind = imgNames[i].find_last_of('/');
		dstDir = dstDir + "/" + imgNames[i].substr(0, ind);
		std::string imgname = imgNames[i].substr(ind + 1);

		write2txt(lineSegments, dstDir, imgname);
		cv::imwrite(dstDir + "/" + imgname.substr(0, imgname.find_last_of('.')) + ".jpg", canvas);

	}

#endif // !YORK_URBAN_EVALUATE


	return 0;
}
