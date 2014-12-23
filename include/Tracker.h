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

		float chiSquared(const std::vector<float>&, const std::vector<float>&);

		void projectVoxels(std::vector<std::vector<VoxelAttributes*>>&, const cv::Mat = cv::Mat());
	};

} /* namespace nl_uu_science_gmt */

#endif /* SCENE3DRENDERER_H_ */
