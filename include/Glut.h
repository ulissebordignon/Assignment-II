/*
* Glut.h
*
*  Created on: Nov 15, 2013
*      Author: coert
*/

#ifndef GLUT_H_
#define GLUT_H_

#ifdef _WIN32
#include <Windows.h>
#endif
#include <GL/gl.h>
#include <GL/glu.h>

#include <opencv2/opencv.hpp>

#include "arcball.h"

#include "General.h"
#include "Scene3DRenderer.h"
#include "Reconstructor.h"
#include "Tracker.h"

// i am not sure about the compatibility with this...
#define MOUSE_WHEEL_UP   3
#define MOUSE_WHEEL_DOWN 4

namespace nl_uu_science_gmt
{

	class Scene3DRenderer;

	class Glut
	{
		Scene3DRenderer &_scene3d;
		Tracker &_tracker;

		static Glut* _glut;

		static void drawGrdGrid();
		static void drawCamCoord();
		static void drawVolume();
		static void drawArcball();
		static void drawVoxels();
		static void drawWCoord();
		static void drawInfo();

		static void optimizeHSV(bool);
		static bool drawOptimization(cv::Mat, const cv::Scalar&, const cv::Scalar&);

		static inline void perspectiveGL(GLdouble, GLdouble, GLdouble, GLdouble);

#ifdef _WIN32
		static void SetupPixelFormat(HDC hDC);
		static LRESULT CALLBACK WndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam);
#endif

	public:
		Glut(Scene3DRenderer &, Tracker &);
		virtual ~Glut();

#ifdef __linux__
		void initializeLinux(const char*, int, char**);
		static void mouse(int, int, int, int);
#endif
#ifdef _WIN32
		int initializeWindows(const char*);
		void mainLoopWindows();
#endif

		static void keyboard(unsigned char, int, int);
		static void motion(int, int);
		static void reshape(int, int);
		static void reset();
		static void idle();
		static void display();
		static void update(int);
		static void quit();

		Scene3DRenderer& getScene3d() const
		{
			return _scene3d;
		}

		Tracker& getTracker() const
		{
			return _tracker;
		}
	};

} /* namespace nl_uu_science_gmt */

#endif /* GLUT_H_ */
