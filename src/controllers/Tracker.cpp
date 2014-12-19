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
	Tracker::Tracker(const vector<Camera*> &cs, const string& dp, Scene3DRenderer &s3d, int cn = 4) :
		_cameras(cs), _data_path(dp), _scene3d(s3d), _active(false), _clusters_number(cn)
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
		
		// create color model from selected frame

		vector<vector<VoxelAttributes>> visibleVoxelsMat;

		// look for non-occluded voxels for each view
		for (int i = 0; i < _cameras.size(); i++){

			vector<VoxelAttributes> visibleVoxels;

			Point3f camLocation = _cameras[i]->getCameraLocation();

			// for each voxel
			for (int j = 0; j < voxels.size(); j++){
				
				// determine the projection
				Point2i projection;

				projection = _cameras[i]->projectOnView(Point3f(voxels[j]->x, voxels[j]->y, voxels[j]->z));
				
				// determine if the projection has already been used
				for (int k = 0; k < visibleVoxels.size(); k++){
					if (projection == visibleVoxels[k].projection){

						float distOld, distNew;
						// distance from old voxel to camera
						distOld = sqrt(pow(visibleVoxels[k].voxel->x - camLocation.x, 2) +
							pow(visibleVoxels[k].voxel->y - camLocation.y, 2) +
							pow(visibleVoxels[k].voxel->z - camLocation.z, 2));
						// distance from new voxel to camera
						distNew =
							sqrt(pow(voxels[j]->x - camLocation.x, 2) +
							pow(voxels[j]->y - camLocation.y, 2) +
							pow(voxels[j]->z - camLocation.z, 2));
						// if it has, and the new voxel is closer to the camera than the old one, substitute
						if (distOld > distNew) {
							visibleVoxels[k].voxel = voxels[j];
							visibleVoxels[k].label = labels.at<int>(j);
						}

						break;
					}

					if (k == visibleVoxels.size()) {
						VoxelAttributes va;
						va.voxel = voxels[j];
						va.projection = projection;
						va.label = labels.at<int>(j);
						// if it hasn't, add projection and projected voxel to the respective vectors
						visibleVoxels.push_back(va);
						break;
					}

				} // end visible voxels loop

			} // end voxel loop

			visibleVoxelsMat.push_back(visibleVoxels);

		} // end camera loop

		// create color model for each label, using all views

		Mat colorModel;

		for (int i = 0; i < _cameras.size(); i++){

			vector<VoxelAttributes> currentVoxels = visibleVoxelsMat[i];

			// vector that stores the color bins 
			vector<int> colorBins;
			for (int j = 0; j < currentVoxels.size(); j++){
				
				Reconstructor::Voxel* v = currentVoxels[j].voxel;
				
			}
		}

	}


} /* namespace nl_uu_science_gmt */
