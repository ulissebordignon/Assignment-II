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
#include <string>

#include "General.h"
#include "Camera.h"

namespace nl_uu_science_gmt
{

	class Tracker
	{
		const std::vector<Camera*> &_cameras;
		const std::string _data_path;
		cv::Mat* _color_model;

#ifdef _WIN32
		HDC _hDC;
#endif

	public:
		Tracker(const std::vector<Camera*> &, const std::string&);

		void update(std::vector<Reconstructor::Voxel*>);

		const std::vector<Camera*>& getCameras() const
		{
			return _cameras;
		}
	};

} /* namespace nl_uu_science_gmt */

#endif /* SCENE3DRENDERER_H_ */
