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
		_cameras(cs), _data_path(dp), _scene3d(s3d)
	{
		// if file exists load color model
	}

	void Tracker::update(const vector<Reconstructor::Voxel*>& voxels) {
		if (_color_model.empty()) {

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
			// create color model from selected frame
		}

		// update voxels' colors based on colr model

	}

} /* namespace nl_uu_science_gmt */
