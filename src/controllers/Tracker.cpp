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

	Tracker::Tracker(const vector<Camera*> &cs, const string& dp, Scene3DRenderer &s3d, int cn) :
		_cameras(cs), _data_path(dp), _scene3d(s3d), _active(false), _clusters_number(cn)
	{
		if (General::fexists(_data_path + CM_FILENAME))
			loadColorModel();
	}

	void Tracker::update() {
		if (_scene3d.getReconstructor().getVisibleVoxels().size() > _scene3d.getReconstructor().getVoxels().size() / 4) {
			if (!General::popup("Warning", "HSV unbalanced, Proceed?")) {
				_active = false;
				return;
			}
		}

		if (_color_models.size() == 0)
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
			putText(image, "Select a frame where all people are visible, then press 'c'", Point(10, 20), 1, 1, Scalar(0, 0, 255));
			imshow(winName, image);
			createTrackbar("Frame", winName, &selectedFrame, _cameras.front()->getFramesAmount()-1);

			k = waitKey(15);
		}
		destroyWindow(winName);

		cout << "Creating color model...";

		for (int i = 0; i < _cameras.size(); i++)
			_scene3d.processForeground(_cameras[i]);

		Reconstructor &rec = _scene3d.getReconstructor();

		rec.update();

		vector<Reconstructor::Voxel*> voxels = rec.getVisibleVoxels();

		Mat labels, coordinates;

		for (int i = 0; i < voxels.size(); i++)
			coordinates.push_back(Point2f(voxels[i]->x, voxels[i]->y));

		TermCriteria criteria;
		criteria.maxCount = 10;

		kmeans(coordinates, 4, labels, criteria, 2, KMEANS_RANDOM_CENTERS);
		
		// create color model from selected frame

		vector<vector<VoxelAttributes*>> visibleVoxelsMat;

		projectVoxels(visibleVoxelsMat, labels);

		// create color model for each label, using all views

		for (int i = 0; i < _clusters_number; i++) {
			ColorModel* cm = new ColorModel();
			cm->bHistogram.resize(25);
			cm->gHistogram.resize(25);
			cm->rHistogram.resize(25);
			_color_models.push_back(cm);
		}


		for (int i = 0; i < _cameras.size(); i++){

			vector<VoxelAttributes*> currentVoxels = visibleVoxelsMat[i];
			Mat frame = _cameras[i]->getVideoFrame(selectedFrame);

			for (int j = 0; j < currentVoxels.size(); j++){
				
				VoxelAttributes* va = currentVoxels[j];
				ColorModel* cm = _color_models[va->label];

				Vec3b intensity = frame.at<Vec3b>(va->projection);
				int blue = intensity.val[0]/10;
				int green = intensity.val[1]/10;
				int red = intensity.val[2]/10;

				cm->bHistogram[blue]++;
				cm->gHistogram[green]++;
				cm->rHistogram[red]++;
			}
		}

		// Normalization
		for (int i = 0; i < _color_models.size(); i++) {
			ColorModel* cm = _color_models[i];

			int tot = 0;
			for (int j = 0; j < cm->bHistogram.size(); j++)
				tot += cm->bHistogram[j];

			for (int j = 0; j < cm->bHistogram.size(); j++)
				cm->bHistogram[j] = cm->bHistogram[j] * 100 / tot;
			for (int j = 0; j < cm->gHistogram.size(); j++)
				cm->gHistogram[j] = cm->gHistogram[j] * 100 / tot;
			for (int j = 0; j < cm->rHistogram.size(); j++)
				cm->rHistogram[j] = cm->rHistogram[j] * 100 / tot;
		}

		cout << " done!" << endl;

		saveColorModel();

	}

	/**
	* Save the color model to the file system as an xml file
	*/
	void Tracker::saveColorModel() {
		cout << "Saving color model to " << _data_path << CM_FILENAME << "...";
		FileStorage fs(_data_path + CM_FILENAME, FileStorage::WRITE);

		for (int i = 0; i < _color_models.size(); i++) {
			ColorModel* cm = _color_models[i];

			stringstream ss;
			ss << "Cluster" << i;
			
			fs << ss.str() << "{";
			fs << "bHistogram" << cm->bHistogram;
			fs << "gHistogram" << cm->gHistogram;
			fs << "rHistogram" << cm->rHistogram;
			fs << "}";
		}

		fs.release();
		cout << " done!" << endl;
	}

	/**
	* Load the color model from an xml file
	*/
	void Tracker::loadColorModel() {
		cout << "Loading color model...";
		FileStorage fs(_data_path + CM_FILENAME, FileStorage::READ);

		for (int i = 0; i < _clusters_number; i++) {
			ColorModel* cm = new ColorModel();

			stringstream ss;
			ss << "Cluster" << i;

			FileNode fn = fs[ss.str()];

			fn["bHistogram"] >> cm->bHistogram;
			fn["gHistogram"] >> cm->gHistogram;
			fn["rHistogram"] >> cm->rHistogram;
			
			_color_models.push_back(cm);
		}

		fs.release();
		cout << " done!" << endl;
	}

	void Tracker::projectVoxels(vector<vector<VoxelAttributes*>>& outputVector, const Mat labels) {
		
		Reconstructor &rec = _scene3d.getReconstructor();
		vector<Reconstructor::Voxel*> voxels = rec.getVisibleVoxels();

		// look for non-occluded voxels for each view
		for (int i = 0; i < _cameras.size(); i++){

			vector<VoxelAttributes*> visibleVoxels;

			Point3f camLocation = _cameras[i]->getCameraLocation();

			// for each voxel
			for (int j = 0; j < voxels.size(); j++){

				// determine the projection
				Point2i projection;

				projection = _cameras[i]->projectOnView(Point3f(voxels[j]->x, voxels[j]->y, voxels[j]->z));

				// determine if the projection has already been used
				bool found = false;
				int k = 0;
				while (!found){
					if (k < visibleVoxels.size() && projection == visibleVoxels[k]->projection){

						float distOld, distNew;
						// distance from old voxel to camera
						distOld = sqrt(pow(visibleVoxels[k]->voxel->x - camLocation.x, 2) +
							pow(visibleVoxels[k]->voxel->y - camLocation.y, 2) +
							pow(visibleVoxels[k]->voxel->z - camLocation.z, 2));
						// distance from new voxel to camera
						distNew =
							sqrt(pow(voxels[j]->x - camLocation.x, 2) +
							pow(voxels[j]->y - camLocation.y, 2) +
							pow(voxels[j]->z - camLocation.z, 2));
						// if it has, and the new voxel is closer to the camera than the old one, substitute
						if (distOld > distNew) {
							visibleVoxels[k]->voxel = voxels[j];
							if (!labels.empty())
								visibleVoxels[k]->label = labels.at<int>(j);
						}

						found = true;
					}
					else if (k == visibleVoxels.size()) {
						VoxelAttributes* va = new VoxelAttributes();
						va->voxel = voxels[j];
						va->projection = projection;
						if (!labels.empty())
							va->label = labels.at<int>(j);
						// if it hasn't, add projection and projected voxel to the respective vectors
						visibleVoxels.push_back(va);
						found = true;
					}
					k++;
				} // end visible voxels loop

			} // end voxel loop

			outputVector.push_back(visibleVoxels);

		} // end camera loop

	}

	float Tracker::chiSquared(const vector<float>& colorModel, const vector<float>& colorHistogram) {

		assert(colorModel.size() == colorHistogram.size());

		float sum = 0;
		for (int i = 0; i < colorModel.size(); i++) {
			sum += pow(colorModel[i] - colorHistogram[i], 2) / (colorModel[i] + colorHistogram[i]);
		}
		return sum / 2.0f;
	}

} /* namespace nl_uu_science_gmt */
