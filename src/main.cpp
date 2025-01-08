#include <opencv2/opencv.hpp>
#include <iostream>
#include "utilities.hpp"
#include "iofile.hpp"
#include "drawutils.hpp"
#include "alignED.hpp"


#define YORK_URBAN_EVALUATE


int main(void)
 {

#ifdef YORK_URBAN_EVALUATE
	std::vector<std::string> filenames;
	//listDir(filenames, "F:/projects-learning/datasets/wireframe/v1.1/test", ".jpg");
	listDirRecursively(filenames, "F:/projects-learning/datasets/YorkUrbanDB", ".jpg");

	int counter = 0;

	for (const auto& imgPath : filenames)
	{
		std::string labelname = imgPath.substr(imgPath.find_last_of('/') + 1);
		labelname = labelname.substr(0, labelname.find_last_of('.'));
		
		LineSegList gt;
		extractYUK("F:/projects-learning/line-segment-detect/AG3line-master/Quantitative_Evaluation/YorkUrbanRes/york/" + labelname + "sold.txt", gt, ' ');
		//extractYUK("F:/projects-learning/datasets/wireframe/line_segments/" + labelname + ".txt", gt, ' ');
		std::cout << "\r" << imgPath;

		cv::Mat testImg = cv::imread(imgPath, 0);	// grayscale
		cv::Mat canvas = cv::imread(imgPath, cv::IMREAD_COLOR);	// color
		cv::Mat tempImg = testImg.clone();
		cv::Mat aagShow = canvas.clone();

		cv::Mat visual = drawLineSegments(gt, testImg.size());

		GradientInfoPtr pGradInfo = std::make_shared<GradientInfo>();

		cv::GaussianBlur(testImg, testImg, cv::Size(5, 5), 1.0);

		calcGradInfo(testImg, pGradInfo.get(), 0);

		std::vector<PixelLinkList> pxLists;
		pseudoSort<float>(pGradInfo->mag, pxLists, 256);
		PixelList alignedAnchors, anchors;

		/* TEST */
		AED::extractAlignedAnchors(pGradInfo.get(), pxLists, alignedAnchors);
		//AED::extractAnchorED(pGradInfo.get(), pxLists, anchorsED);
		NMS(pGradInfo.get(), anchors);
		drawPixelList(aagShow, alignedAnchors, 3, true);

		drawPixelList(tempImg, anchors, 1, true);
		
		LineSegList lineSegments;
		LineSegList candidateSegments;
		AED::detect(pGradInfo.get(), alignedAnchors, anchors, lineSegments, candidateSegments);

		AED::validateCandidateSegments(pGradInfo.get(), candidateSegments);
		//for (const auto& seg : candidateSegments)
		//	lineSegments.emplace_back(seg);

		cv::Mat res = drawLineSegments(lineSegments, testImg.size());
		//drawLineSegments(res, candidateSegments, false, cv::Vec3b(0, 0, 255));
		
		drawLineSegments(canvas, lineSegments, true, cv::Vec3b(0, 255, 0));
		
		//drawLineSegments(res, lineSegments, false, cv::Vec3b(0, 0, 255));

 		std::string imgname = imgPath.substr(imgPath.find_last_of('/') + 1);
		cv::imwrite("D:/Learning/paper/my/AAGLSD/lsd-results/" + imgname, res);

		write2txt(
			lineSegments, 
			"F:/projects-learning/line-segment-detect/AG3line-master/Quantitative_Evaluation/YorkUrbanRes/wireframe", 
			imgPath,
			".txt",
			" "
		);
	}
#endif

#ifndef YORK_URBAN_EVALUATE

	std::vector<std::string> imgNames;

	std::string HPATCHES_DIR = "F:/projects-learning/datasets/hpatches-sequences-release/";
	std::string imgList = "F:/projects-learning/line-segment-detect/Aligned-Group-Edge-Drawing/evaluation/repeatability-evalutaion/image-names.txt";

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
		pseudoSort<float>(pGradInfo->mag, pxLists, 512);
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
		drawLineSegments(canvas, lineSegments, false, cv::Vec3b(0, 255, 0));

		std::string dstDir = "F:/projects-learning/line-segment-detect/data/hpatches/my";
		int ind = imgNames[i].find_last_of('/');
		dstDir = dstDir + "/" + imgNames[i].substr(0, ind);
		std::string imgname = imgNames[i].substr(ind + 1);

		write2txt(lineSegments, dstDir, imgname);
		cv::imwrite(dstDir + "/" + imgname.substr(0, imgname.find_last_of('.')) + ".jpg", canvas);

	}

#endif // !YORK_URBAN_EVALUATE


	return 0;
}
