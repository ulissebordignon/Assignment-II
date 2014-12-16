/*
* Tracker.cpp
*
*  Created on: Dec 13, 2014
*      Author: Ulisse Bordignon, Nicola Chinellato
*/

#include "General.h"
#include "Tracker.h"

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

namespace nl_uu_science_gmt
{

	/**
	* Voxel reconstruction class
	*/
	Tracker::Tracker(const vector<Camera*> &cs, const string& dp, Scene3DRenderer &s3d) :
		_cameras(cs), _data_path(dp), _scene3d(s3d), _active(false)
	{
		// if file exists load color model
	}

	void Tracker::update() {
		if (_color_model.empty())
			createColorModel();

		// update voxels' colors based on colr model

	}

	void Tracker::createColorModel() {
		string winName = "Frame selection";
		namedWindow(winName);

		int selectedFrame = 0;

		int k = 0;
		while (k != 'c'){
			Mat image = _cameras[0]->getVideoFrame(selectedFrame);
			hconcat(image, _cameras[1]->getVideoFrame(selectedFrame), image);
			Mat tmpImage = _cameras[2]->getVideoFrame(selectedFrame);
			hconcat(tmpImage, _cameras[3]->getVideoFrame(selectedFrame), tmpImage);

			vconcat(image, tmpImage, image);

			resize(image, image, Size(), 0.5, 0.5);

			imshow(winName, image);
			createTrackbar("Frame", winName, &selectedFrame, _cameras.front()->getFramesAmount());

			k = waitKey(15);
		}

		destroyWindow(winName);

		for (int i = 0; i < _cameras.size(); i++)
			_scene3d.processForeground(_cameras[i]);

		Reconstructor &rec = _scene3d.getReconstructor();

		rec.update();

		vector<Reconstructor::Voxel*> voxels = rec.getVisibleVoxels();

		Mat labels;

		Mat coordinates;

		for (int i = 0; i < voxels.size(); i++)
			coordinates.push_back(Point2f(voxels[i]->x, voxels[i]->y));

		TermCriteria criteria;
		criteria.maxCount = 10;

		kmeans(coordinates, 4, labels, criteria, 2, KMEANS_RANDOM_CENTERS);

		for (int i = 0; i < voxels.size(); i++){
			Reconstructor::Voxel* v = voxels[i];
			switch (labels.at<int>(i)) {
			case 0:
				v->color = Scalar(0.f,0.f,0.f,1);
				break;
			case 1:
				v->color = Scalar(1.f, 0.f, 0.f, 1);
				break;
			case 2:
				v->color = Scalar(0.f, 1.f, 0.f, 1);
				break;
			case 3:
				v->color = Scalar(0.f, 0.f, 1.f, 1);
				break;
			}
		}
		
		// create color model from selected frame

		/*		!!! SUPER PESANTE !!!	
		for (int i = 0; i < voxels.size(); i++)

			for (int j = 0; j < _cameras.size(); j++)

				_cameras[j]->projectOnView(Point3f(voxels[i]->x, voxels[i]->y, voxels[i]->z),
				_cameras[j]->getRotationValues(), _cameras[j]->getTranslationValues(),
				_cameras[j]->getCameraMatrix(), _cameras[j]->getDistortionCoefficients());*/

	}


} /* namespace nl_uu_science_gmt */
