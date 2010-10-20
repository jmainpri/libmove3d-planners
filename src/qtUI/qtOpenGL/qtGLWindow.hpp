/**
 * QT Open GL Window
 */

#ifndef QT_GL_WINDOW_H
#define QT_GL_WINDOW_H

#include "Graphic-pkg.h"
#include "qtUI/qtOpenGL/glwidget.hpp"

#ifdef CXX_PLANNER
#include "qtLibrary.hpp"
#endif

#ifdef OOMOVE3D_CORE
#include "qtUI/qtLibrary.hpp"
#endif

/**
  * @ingroup qtOldWidget
  * @brief Open GL widget containing the qtOpenGL viewer implemetation in Qt
  */
class qtGLWindow: public QWidget
{
Q_OBJECT

public:
	qtGLWindow();

	GLWidget* getOpenGLWidget() { return glWidget; }

private:
	void createCheckBoxes();
	QSlider *createSlider();

	GLWidget *glWidget;

	QSlider *xSlider;
	QSlider *ySlider;
	QSlider *zSlider;
	QSlider *zoom;

	QCheckBox* wcop; // "Copy"
	QCheckBox* vsav; // "Save\nView"
	QCheckBox* vres; // "Restore\n View"
	QCheckBox* vfil; // "Poly/\nLine"
	QCheckBox* vcont; // "Contours"
	QCheckBox* vGhost; //"Ghost"
	QCheckBox* vBb; // "BB"
	QCheckBox* vgour; // "Smooth"
	QCheckBox* wfree; // "Freeze"
	QCheckBox* mcamera; // "Mobile\n Camera"
	QCheckBox* done; // "Done"
	QCheckBox* unselect; // "Unselect\n Joint"

	QCheckBox* opfloor; //"Floor"
	QCheckBox* optiles; // "Tiles"
	QCheckBox* walls; // "Walls"
	QCheckBox* shadows; // "Shadows"

	G3D_Window* win;

private slots:
	void setBoolGhost(bool value);
	void setBoolBb(bool value);
	void setBoolFloor(bool value);
	void setBoolTiles(bool value);
	void setBoolWalls(bool value);
	void setBoolShadows(bool value);

};

#endif
