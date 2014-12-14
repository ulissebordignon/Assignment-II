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
		const std::vector<Camera*> &_cameras;
		const std::string _data_path;
		Scene3DRenderer &_scene3d;
		cv::Mat _color_model;

#ifdef _WIN32
		HDC _hDC;
#endif

	public:
		Tracker(const std::vector<Camera*> &, const std::string&, Scene3DRenderer&);

		void update(const std::vector<Reconstructor::Voxel*>&);

		const std::vector<Camera*>& getCameras() const
		{
			return _cameras;
		}
	};

} /* namespace nl_uu_science_gmt */

#endif /* SCENE3DRENDERER_H_ */
