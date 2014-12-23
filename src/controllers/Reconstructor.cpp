/*
* Reconstructor.cpp
*
*  Created on: Nov 15, 2013
*      Author: coert
*/

#include "General.h"
#include "Reconstructor.h"

#include <opencv2/opencv.hpp>
#include <cassert>
#include <iostream>

using namespace std;
using namespace cv;

namespace nl_uu_science_gmt
{

	/**
	* Voxel reconstruction class
	*/
	Reconstructor::Reconstructor(const vector<Camera*> &cs, const string& dp) :
		_cameras(cs), _data_path(dp)
	{
		for (size_t c = 0; c < _cameras.size(); ++c)
		{
			if (_plane_size.area() > 0)
				assert(_plane_size.width == _cameras[c]->getSize().width && _plane_size.height == _cameras[c]->getSize().height);
			else
				_plane_size = _cameras[c]->getSize();
		}

		_step = 32;
		_size = 512;
		const size_t h_edge = _size * 4;
		const size_t edge = 2 * h_edge;
		_voxels_amount = (edge / _step) * (edge / _step) * (h_edge / _step);

		initialize();
	}

	/**
	* Free the memory of the pointer vectors
	*/
	Reconstructor::~Reconstructor()
	{
		for (size_t c = 0; c < _corners.size(); ++c)
			delete _corners.at(c);
		for (size_t v = 0; v < _voxels.size(); ++v)
			delete _voxels.at(v);
	}

	/**
	* Create some Look Up Tables
	* 	- LUT for the scene's box corners
	* 	- LUT with a map of the entire voxelspace: point-on-cam to voxels
	* 	- LUT with a map of the entire voxelspace: voxel to cam points-on-cam
	*/
	void Reconstructor::initialize()
	{
		const int h_edge = _size * 4;
		const int xL = -h_edge;
		const int xR = h_edge;
		const int yL = -h_edge;
		const int yR = h_edge;
		const int zL = 0;
		const int zR = h_edge;

		// Save the volume corners
		// bottom
		_corners.push_back(new Point3f((float)xL, (float)yL, (float)zL));
		_corners.push_back(new Point3f((float)xL, (float)yR, (float)zL));
		_corners.push_back(new Point3f((float)xR, (float)yR, (float)zL));
		_corners.push_back(new Point3f((float)xR, (float)yL, (float)zL));

		// top
		_corners.push_back(new Point3f((float)xL, (float)yL, (float)zR));
		_corners.push_back(new Point3f((float)xL, (float)yR, (float)zR));
		_corners.push_back(new Point3f((float)xR, (float)yR, (float)zR));
		_corners.push_back(new Point3f((float)xR, (float)yL, (float)zR));

		cout << "Initializing voxels... ";

		// Acquire some memory for efficiency
		_voxels.resize(_voxels_amount);

		ifstream inputFile(_data_path + "voxels.csv");


		// Load the voxels from a CSV file (if it exists)
		if (inputFile.good()) {
			string line;

			int p = 0;

			while (getline(inputFile, line))
			{
				stringstream lineStream(line);
				string cell;
				vector<string> result;

				while (getline(lineStream, cell, ','))
					result.push_back(cell);

				Voxel* voxel = new Voxel();

				// The CSV format is x,y,z,x1,y1,v1,x2,y2,v2,...,xn,yn,vn,p
				// Where:
				// x,y,z are the 3D coordinates of the voxel
				// xk,yk are the 2D coordinates of the voxel's projection on camera k
				// vk indicates if the voxels projection on camera k is valid
				// n is the number of cameras
				// p is the index of the voxel

				voxel->x = stoi(result[0]);
				voxel->y = stoi(result[1]);
				voxel->z = stoi(result[2]);

				voxel->camera_projection = vector<Point>(_cameras.size());
				voxel->valid_camera_projection = vector<int>(_cameras.size());

				// i starts from 1 because the first three values have already been used above
				for (int i = 1; i < _cameras.size() + 1; i++) {

					// The ith triplet on the file corresponds to the i-1th camera
					voxel->camera_projection[i - 1] = Point(stoi(result[i * 3]), stoi(result[i * 3 + 1]));
					voxel->valid_camera_projection[i - 1] = stoi(result[i * 3 + 2]);
				}

				_voxels[p] = voxel;
				p++;
			}

		}

		// Initialize voxels internally and save them to a CSV file
		else {
			ofstream outputFile;
			outputFile.open(_data_path + "voxels.csv");

			for (int z = zL; z < zR; z += _step)
			{
				cout << "." << flush;

				for (int y = yL; y < yR; y += _step)
				{
					for (int x = xL; x < xR; x += _step)
					{
						Voxel* voxel = new Voxel;
						voxel->x = x;
						voxel->y = y;
						voxel->z = z;

						outputFile << x << "," << y << "," << z << ",";

						voxel->camera_projection = vector<Point>(_cameras.size());
						voxel->valid_camera_projection = vector<int>(_cameras.size(), 0);

						const int zp = ((z - zL) / _step);
						const int yp = ((y - yL) / _step);
						const int xp = ((x - xL) / _step);
						const int plane_y = (yR - yL) / _step;
						const int plane_x = (xR - xL) / _step;
						const int plane = plane_y * plane_x;
						const int p = zp * plane + yp * plane_x + xp;  // The voxel's index

						for (size_t c = 0; c < _cameras.size(); ++c)
						{
							Point point = _cameras[c]->projectOnView(Point3f((float)x, (float)y, (float)z));

							// Save the pixel coordinates 'point' of the voxel projections on camera 'c'
							voxel->camera_projection[(int)c] = point;

							if (point.x >= 0 && point.x < _plane_size.width && point.y >= 0 && point.y < _plane_size.height)
								voxel->valid_camera_projection[(int)c] = 1;

							outputFile << point.x << "," << point.y << "," << voxel->valid_camera_projection[c] << ",";
						}

						//'p' is not critical as it's unique
						_voxels[p] = voxel;

						outputFile << p << endl;
					}
				}
			}

			outputFile.close();
		}
		cout << "done!" << endl;

		inputFile.close();
	}

	/**
	* Count the amount of camera's each voxel in the space appears on,
	* if that amount equals the amount of cameras, add that voxel to the
	* visible_voxels vector
	*
	* Optimized by inverting the process (iterate over voxels instead of camera pixels for each camera)
	*/
	void Reconstructor::update()
	{
		_visible_voxels.clear();
		std::vector<Voxel*> visible_voxels;

#ifdef _OPENMP
		omp_set_num_threads(NUM_THREADS);
#pragma omp parallel for shared(visible_voxels)
#endif
		for (int v = 0; v < (int)_voxels_amount; ++v)
		{
			int camera_counter = 0;
			Voxel* voxel = _voxels[v];

			for (size_t c = 0; c < _cameras.size(); ++c)
			{
				if (voxel->valid_camera_projection[c])
				{
					const Point point = voxel->camera_projection[c];

					//If there's a white pixel on the foreground image at the projection point, add the camera
					if (_cameras[c]->getForegroundImage().at<uchar>(point) == 255) ++camera_counter;
				}
			}

			// If the voxel is present on all cameras
			if (camera_counter == _cameras.size())
			{
#ifdef _OPENMP
#pragma omp critical //push_back is critical
#endif
				visible_voxels.push_back(voxel);
			}
		}

		_visible_voxels.insert(_visible_voxels.end(), visible_voxels.begin(), visible_voxels.end());
	}

} /* namespace nl_uu_science_gmt */
