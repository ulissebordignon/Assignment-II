/*
* Tracker.h
*
*  Created on: Dec 13, 2014
*      Author: Ulisse Bordignon, Nicola Chinellato
*/

#ifndef TRACKER_H_
#define TRACKER_H_

#include <opencv2/opencv.hpp>

#ifdef _WIN32
#include <Windows.h>
#endif
#include <vector>

#include "General.h"
#include "Reconstructor.h"
#include "Scene3DRenderer.h"
#include "Camera.h"

namespace nl_uu_science_gmt
{

#define CM_FILENAME "color_model.xml"

	class Tracker
	{
	public:
		struct VoxelAttributes
		{
			Reconstructor::Voxel* voxel;
			cv::Point2i projection;
			int label;
		};

		struct ColorModel 
		{
			cv::Scalar color;
			std::vector<float> bHistogram;
			std::vector<float> gHistogram;
			std::vector<float> rHistogram;
		};

	private:

		const std::vector<Camera*> &_cameras;
		const std::string _data_path;
		Scene3DRenderer &_scene3d;
		std::vector<ColorModel*> _color_models;
		bool _active;
		int _clusters_number;
		std::vector<std::vector<cv::Point2f>> _unrefined_centers;
		std::vector<std::vector<cv::Point2f>> _refined_centers;

		void createColorModel();
		void saveColorModel();
		void loadColorModel();

	public:
		Tracker(const std::vector<Camera*> &, const std::string&, Scene3DRenderer&, int = 4);

		void update();

		const std::vector<Camera*>& getCameras() const
		{
			return _cameras;
		}

		bool isActive() {
			return _active;
		}

		bool setActive(bool active) {
			_active = active;
		}

		void toggleActive() {
			_active = !_active;
		}

		std::vector<std::vector<cv::Point2f>> getRefinedCenters() {
			return _refined_centers;
		}

		std::vector<ColorModel*> getColorModels() {
			return _color_models;
		}

		float chiSquared(const ColorModel&, const ColorModel&);

		void projectVoxels(std::vector<std::vector<VoxelAttributes*>>&, const cv::Mat = cv::Mat());
	};

} /* namespace nl_uu_science_gmt */

#endif /* SCENE3DRENDERER_H_ */
