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

	class Tracker
	{
	public:
		struct VoxelAttributes
		{
			Reconstructor::Voxel* voxel;
			Point2i projection;
			int label;
		};

		const std::vector<Camera*> &_cameras;
		const std::string _data_path;
		Scene3DRenderer &_scene3d;
		cv::Mat _color_model;
		bool _active;
		int _clusters_number;

		void createColorModel();

	public:
		Tracker(const std::vector<Camera*> &, const std::string&, Scene3DRenderer&, int);

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
	};

} /* namespace nl_uu_science_gmt */

#endif /* SCENE3DRENDERER_H_ */
