/*
* General.cpp
*
*  Created on: Nov 13, 2013
*      Author: coert
*/

#include <General.h>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

namespace nl_uu_science_gmt
{

	const string General::CBConfigFile = "checkerboard.xml";
	const string General::CalibrationVideo = "calibration.avi";
	const string General::CheckerboadVideo = "checkerboard.avi";
	const string General::BackgroundVideoFile = "background.avi";
	const string General::BackgroundImageFile = "background.png";
	const string General::VideoFile = "video.avi";
	const string General::IntrinsicsFile = "intrinsics.xml";
	const string General::CheckerboadCorners = "boardcorners.xml";
	const string General::ConfigFile = "config.xml";

	/**
	* Linux/Windows friendly way to check if a file exists
	*/
	bool General::fexists(const std::string &filename)
	{
		ifstream ifile(filename.c_str());
		return ifile.is_open();
	}

	float General::pointDistance(const Point& p1, const Point& p2) {
		return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
	}

	void General::popupCallback(int event, int x, int y, int, void* param) {
		int* key = (int*)param;
		if (event != EVENT_LBUTTONDOWN)
			return;

		if (x > 30 && x < 100 && y > 100 && y < 130)
			*key = 'y';
		if (x > 200 && x < 270 && y > 100 && y < 130)
			*key = 'n';
	}

	bool General::popup(const std::string &title, const std::string &message) {
		int key = NULL;
		namedWindow(title);
		setMouseCallback(title, General::popupCallback, &key);

		while (key == NULL) {
			Mat popup(150, 300, CV_8UC3, Scalar(255, 255, 255));
			putText(popup, message, Point(30, 30), 1, 1, Scalar(0, 0, 0));
			rectangle(popup, Point(30, 130), Point(100, 100), Scalar(150, 150, 150), CV_FILLED);
			putText(popup, "Yes (Y)", Point(40, 120), 1, 0.9, Scalar(0, 0, 0));
			rectangle(popup, Point(200, 130), Point(270, 100), Scalar(150, 150, 150), CV_FILLED);
			putText(popup, "No (N)", Point(212, 120), 1, 0.9, Scalar(0, 0, 0));
			imshow(title, popup);
			int k = waitKey(15);
			if (k == 'y' || k == 'n' || k == 'Y' || k == 'N')
				key = k;
		}

		destroyWindow(title);

		if (key == 'y' || key == 'Y')
			return true;
		if (key == 'n' || key == 'N')
			return false;
	}

} /* namespace nl_uu_science_gmt */
