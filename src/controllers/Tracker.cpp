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
		_unrefined_centers.resize(_clusters_number);
		_refined_centers.resize(_clusters_number);

		if (General::fexists(_data_path + CM_FILENAME))
			loadColorModel();
		_refined_centers.resize(cn);
	}

	void Tracker::update() {
		vector<Reconstructor::Voxel*> voxels = _scene3d.getReconstructor().getVisibleVoxels();
		if (voxels.size() > _scene3d.getReconstructor().getVoxels().size() / 4) {
			if (!General::popup("Warning", "HSV unbalanced, Proceed?")) {
				_active = false;
				return;
			}
		}

		if (_color_models.size() == 0) {
			createColorModel();
			return;
		}

		// update voxels' colors based on color model
		vector<map<float,VoxelAttributes*>> visibleVoxelsMat;
		
		projectVoxels(visibleVoxelsMat);
		vector < vector<Point2f> > points4Relabelling(_clusters_number);
		vector <Point2f> centers4Clustering;
		
		for (int i = 0; i < visibleVoxelsMat.size(); i++) {
			map<float,VoxelAttributes*> currentVoxels = visibleVoxelsMat[i];
			
			for (map<float, VoxelAttributes*>::iterator it = currentVoxels.begin(); it != currentVoxels.end(); it++) {

				VoxelAttributes* va = it->second;
				ColorModel* cm = new ColorModel();
				cm->bHistogram.resize(26);
				cm->gHistogram.resize(26);
				cm->rHistogram.resize(26);
				Mat frame = _cameras[i]->getFrame();
				
				Vec3b intensity = frame.at<Vec3b>(va->projection);

				int blue = floor(intensity.val[0] / 51);
				int green = floor(intensity.val[1] / 51);
				int red = floor(intensity.val[2] / 51);

				cm->bHistogram[blue] = 100;
				cm->gHistogram[green] = 100;
				cm->rHistogram[red] = 100;

				// current label
				int m = 0;

				float bestResult = FLT_MAX;
				
				for (int k = 0; k < _clusters_number; k++) {

					float currentResult = chiSquared(_color_models[k], cm);
					if (currentResult < bestResult) {
						m = k;
						bestResult = currentResult;
					}
				}

				va->label = m;
				points4Relabelling[m].push_back(Point2f(va->voxel->x, va->voxel->y));
				centers4Clustering.push_back(Point2f(va->voxel->x, va->voxel->y));
				va->voxel->color = _color_models[m]->color;
				
				/*if (bestResult < 1200) {
					voxels[i]->color = _color_models[m]->color;
					points4Relabelling[m].push_back(Point2f(voxels[i]->x, voxels[i]->y));
				}
				else {
					voxels[i]->color = _color_models[m]->color;
					voxels[i]->color[3] = 0.f;
				}*/
			}
		}

		for (int i = 0; i < _clusters_number; i++) {
			int sumx = 0, sumy = 0;
			for (int j = 0; j < points4Relabelling[i].size(); j++) {
				sumx += points4Relabelling[i][j].x;
				sumy += points4Relabelling[i][j].y;
			}
			_unrefined_centers[i].push_back(Point2f(sumx / points4Relabelling[i].size(), sumy / points4Relabelling[i].size()));
		}


		Mat discard, centers;
		TermCriteria criteria;
		criteria.maxCount = 10;

		kmeans(centers4Clustering, _clusters_number, discard, criteria, 2, KMEANS_RANDOM_CENTERS, centers);

		for (int i = 0; i < _clusters_number; i++) {
			int b; 
			for (int j = 0; j < _clusters_number; j++) {

			}
			// compare to find best match to substitute to
		}
		
		vector < vector<Point2f> > relabelledPoints(_clusters_number);
		for (int i = 0; i < voxels.size(); i++) {

			float closestCenterDst = FLT_MAX;
			int c;
			for (int j = 0; j < _clusters_number; j++) {
				float currentCenterDst = sqrt(pow(voxels[i]->x - _unrefined_centers[j].back().x, 2) + pow(voxels[i]->y - _unrefined_centers[j].back().y, 2));
				if (currentCenterDst < closestCenterDst) {
					c = j;
					closestCenterDst = currentCenterDst;
				}
			}
			cout << closestCenterDst << endl;
			voxels[i]->color = _color_models[c]->color;
			relabelledPoints[c].push_back(Point2f(voxels[i]->x, voxels[i]->y));

		}

		for (int i = 0; i < _clusters_number; i++) {
			int sumx = 0, sumy = 0;
			for (int j = 0; j < relabelledPoints[i].size(); j++) {
				sumx += relabelledPoints[i][j].x;
				sumy += relabelledPoints[i][j].y;
			}
			_refined_centers[i].push_back(Point2f(sumx / relabelledPoints[i].size(), sumy / relabelledPoints[i].size()));
		}

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

		kmeans(coordinates, _clusters_number, labels, criteria, 2, KMEANS_RANDOM_CENTERS);
		
		// create color model from selected frame

		vector<map<float,VoxelAttributes*>> visibleVoxelsMat;

		projectVoxels(visibleVoxelsMat, labels);

		// create color model for each label, using all views

		for (int i = 0; i < _clusters_number; i++) {
			ColorModel* cm = new ColorModel();
			cm->bHistogram.resize(26);
			cm->gHistogram.resize(26);
			cm->rHistogram.resize(26);

			switch (i) {
			case 0:
				cm->color = Scalar(0.f, 0.f, 1.f, 1.f);
				break;
			case 1:
				cm->color = Scalar(1.f, 0.f, 0.f, 1.f);
				break;
			case 2:
				cm->color = Scalar(0.f, 1.f, 0.f, 1.f);
				break;
			default:
				cm->color = Scalar(0.f, 0.f, 0.f, 1.f);
			}
			_color_models.push_back(cm);
		}


		for (int i = 0; i < _cameras.size(); i++){

			map<float,VoxelAttributes*> currentVoxels = visibleVoxelsMat[i];
			Mat frame = _cameras[i]->getVideoFrame(selectedFrame);

			for (map<float,VoxelAttributes*>::iterator it = currentVoxels.begin(); it != currentVoxels.end(); it++){
				
				VoxelAttributes* va = it->second;
				ColorModel* cm = _color_models[va->label];

				Vec3b intensity = frame.at<Vec3b>(va->projection);
				int blue = floor(intensity.val[0]/51);
				int green = floor(intensity.val[1]/51);
				int red = floor(intensity.val[2]/51);

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
		FileStorage fs(_data_path + CM_FILENAME, FileStorage::WRITE);

		for (int i = 0; i < _color_models.size(); i++) {
			ColorModel* cm = _color_models[i];

			stringstream ss;
			ss << "Cluster" << i;
			
			fs << ss.str() << "{";
			fs << "color" << cm->color;
			fs << "bHistogram" << cm->bHistogram;
			fs << "gHistogram" << cm->gHistogram;
			fs << "rHistogram" << cm->rHistogram;
			fs << "}";
		}

		fs.release();
		cout << "Color model saved to " << _data_path << CM_FILENAME << endl;
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

			fn["color"] >> cm->color;
			fn["bHistogram"] >> cm->bHistogram;
			fn["gHistogram"] >> cm->gHistogram;
			fn["rHistogram"] >> cm->rHistogram;
			
			_color_models.push_back(cm);
		}

		fs.release();
		cout << " done!" << endl;
	}

	void Tracker::projectVoxels(vector<map<float,VoxelAttributes*>>& outputVector, const Mat labels) {
		
		Reconstructor &rec = _scene3d.getReconstructor();
		vector<Reconstructor::Voxel*> voxels = rec.getVisibleVoxels();

		// look for non-occluded voxels for each view
		for (int i = 0; i < _cameras.size(); i++){

			map<float,VoxelAttributes*> visibleVoxels;

			Point3f camLocation = _cameras[i]->getCameraLocation();

			// for each voxel
			for (int j = 0; j < voxels.size(); j++){

				// determine the projection
				Point2i projection;

				projection = _cameras[i]->projectOnView(Point3f(voxels[j]->x, voxels[j]->y, voxels[j]->z));
				int x = projection.x;
				int y = projection.y;
				float key = (x + y)*(x + y + 1) / 2 + y;

				// determine if the projection has already been used
				if (visibleVoxels.find(key) == visibleVoxels.end()) {
					VoxelAttributes* va = new VoxelAttributes();
					va->voxel = voxels[j];
					va->projection = projection;
					if (!labels.empty())
						va->label = labels.at<int>(j);
					// if it hasn't, add projection and projected voxel to the respective vectors
					visibleVoxels[key] = va;
					voxels.erase(voxels.begin() + j);
				}
				else {
					float distOld, distNew;
					VoxelAttributes* va = visibleVoxels[key];
					// distance from old voxel to camera
					distOld = sqrt(pow(va->voxel->x - camLocation.x, 2) +
						pow(va->voxel->y - camLocation.y, 2) +
						pow(va->voxel->z - camLocation.z, 2));
					// distance from new voxel to camera
					distNew =
						sqrt(pow(voxels[j]->x - camLocation.x, 2) +
						pow(voxels[j]->y - camLocation.y, 2) +
						pow(voxels[j]->z - camLocation.z, 2));
					// if it has, and the new voxel is closer to the camera than the old one, substitute
					if (distOld > distNew) {
						va->voxel = voxels[j];
						if (!labels.empty())
							va->label = labels.at<int>(j);
						voxels.erase(voxels.begin() + j);
					}
				}

			} // end voxel loop

			outputVector.push_back(visibleVoxels);

		} // end camera loop

	}

	float Tracker::chiSquared(const ColorModel* reference, const ColorModel* data) {

		assert(colorModel.size() == colorHistogram.size());

		float bDist = 0;
		for (int i = 0; i < reference->bHistogram.size(); i++) {
			if (reference->bHistogram[i] + data->bHistogram[i] != 0)
				bDist += pow(reference->bHistogram[i] - data->bHistogram[i], 2) / (reference->bHistogram[i] + data->bHistogram[i]);
		}
		bDist = bDist / 2.0f;

		float gDist = 0;
		for (int i = 0; i < reference->gHistogram.size(); i++) {
			if (reference->gHistogram[i] + data->gHistogram[i] != 0)
				bDist += pow(reference->gHistogram[i] - data->gHistogram[i], 2) / (reference->gHistogram[i] + data->gHistogram[i]);
		}
		gDist = gDist / 2.0f;

		float rDist = 0;
		for (int i = 0; i < reference->rHistogram.size(); i++) {
			if (reference->rHistogram[i] + data->rHistogram[i] != 0)
				bDist += pow(reference->rHistogram[i] - data->rHistogram[i], 2) / (reference->rHistogram[i] + data->rHistogram[i]);
		}
		rDist = rDist / 2.0f;

		return (bDist + gDist + rDist) / 3;
	}

} /* namespace nl_uu_science_gmt */
