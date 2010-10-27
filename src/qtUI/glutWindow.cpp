/*
 *  glutWindow.cpp
 *  OOMove3D
 *
 *  Created by Jim Mainprice on 25/10/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "glutWindow.hpp"

#include "Graphic-pkg.h"
#include "Move3D-pkg.h"

#include "qtUI/qtOpenGL/glwidget.hpp"

#include "API/Graphic/drawModule.hpp"

using namespace std;

extern void* GroundCostObj;
extern int mainMhp(int argc, char** argv);

//! reshape the OPenGL display
//! changes the matrix when the window changes sizes
void reshapeGL(int w, int h)
{
	glViewport(0, 0, (GLint) w, (GLint) h);
	//	qglClearColor(QColor::fromCmykF(0.0, 0.0, 0.0, 0.0));
	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	
	double size = G3D_WIN->vs.size;
	
	gluPerspective(40.0, (GLdouble) w / (GLdouble) h, size / 10000.0, size * 1000.0);
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

//! Initilizes the OpenGL display
//! Sets the matrices at start
void initializeGL(int w,int h)
{
	glViewport(0,0,(GLint) w,(GLint) h);
	//glClearColor(G3D_WIN->vs.bg[0],G3D_WIN->vs.bg[1],G3D_WIN->vs.bg[2],.0);
	
	//   glMatrixMode(GL_PROJECTION);
	//   glLoadIdentity();
	g3d_set_projection_matrix(G3D_WIN->vs.projection_mode);
	
	glMatrixMode(GL_MODELVIEW);
	
	glLoadIdentity();
	
	
	/** on desactive tout mode OpenGL inutile ***/
	glDisable(GL_STENCIL_TEST);
	glDisable(GL_SCISSOR_TEST);
	glDisable(GL_ALPHA_TEST);
	
	glEnable(GL_DEPTH_TEST);
	
	if(G3D_WIN->vs.GOURAUD) 
	{
		glShadeModel(GL_SMOOTH);
	} else 
	{
		glShadeModel(GL_FLAT);
	}
	
	
	if(GroundCostObj)
	{
		cout << "GroundCostObj, vs.displayFloor = false" << endl;
		G3D_WIN->vs.displayFloor = false;
	}
}

bool init = true;

void paintGL()
{
	if(init)
	{
		 initializeGL(800,600);
			init = false;
	}
	
	glPushMatrix();
	
	//	computeNewVectors(Xc,Xw,up);
	g3d_cam_param p;
	qt_ui_calc_param(p);
	
	gluLookAt(p.Xc[0], p.Xc[1], p.Xc[2], 
						p.Xw[0], p.Xw[1], p.Xw[2], 
						p.up[0], p.up[1], p.up[2]);
	
	G3D_WIN->vs.cameraPosition[0]= p.Xc[0];
	G3D_WIN->vs.cameraPosition[1]= p.Xc[1];
	G3D_WIN->vs.cameraPosition[2]= p.Xc[2];  
	
	g3d_draw();
	
	glPopMatrix();
}

//! Creates a Glut window
GlutWindowDisplay::GlutWindowDisplay(int argc, char *argv[])
{
	int argc2 = 0;
	char* argv2 = "";
	
	/* initialisation de GLUT */
	glutInit (&argc2, &argv2);
	
	/* création d'une fenêtre OpenGL RVBA avec en simple mémoire tampon
	 avec un tampon de profondeur */
	glutInitDisplayMode (GLUT_RGB | GLUT_SINGLE | GLUT_ALPHA | GLUT_DEPTH | GLUT_STENCIL | GLUT_STEREO);
	glutInitWindowSize (800, 600);
	glutCreateWindow ("Move3D Glut");
	
	// Initializes Move3D
	mainMhp(argc, argv);
	
	// Initializes Draw Functions
	ext_get_win_mouse = (void (*) (int*,int*))(qt_get_win_mouse);
	ext_g3d_draw_allwin_active = (void (*)())(qt_draw_allwin_active);
	ext_calc_cam_param = (void (*) (g3d_cam_param&) )(qt_ui_calc_param);
	
	Graphic::initDrawFunctions();
	
	// Initializes the G3D Window
	qtG3DWindow* win = new qtG3DWindow;
	
	G3D_WIN->vs.displayFloor = true;
	G3D_WIN->vs.displayTiles = true;
	G3D_WIN->vs.GOURAUD = true;
	
	/* initialisation des fonctions callback appelées par glut 
	 pour respectivement le redimensionnement de la fenêtre
	 et le rendu }de la scène */
	glutReshapeFunc (reshapeGL);
	glutDisplayFunc (paintGL);
}