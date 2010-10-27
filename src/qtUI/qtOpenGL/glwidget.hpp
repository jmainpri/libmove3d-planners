/*
 * Header File for the qtOpenGL Widget
 */

#ifndef GLWIDGET_H
#define GLWIDGET_H

#include "p3d_matrix.h"
#include "qtUI/qtLibrary.hpp"

#ifndef MAINWINDOW_HPP
class MainWindow;
#endif

class Move3D2OpenGl;

#ifndef WITH_XFORMS
#include "Graphic-pkg.h"
#endif

void qt_get_win_mouse(int* i, int* j);
void qt_draw_allwin_active();
void qt_ui_calc_param(g3d_cam_param& p);

/**
  * @ingroup qtWindow
  * @brief Open GL viewer implemetation in Qt
  */
class GLWidget: public QGLWidget
{
Q_OBJECT

public:
	GLWidget(QWidget *parent = 0);
	~GLWidget();

	void setMainWindow(MainWindow* w) { m_mainWindow = w; }
	void setWinSize(double size);
	void resetImageVector();
	void setThreadWorking(bool isWorking);
	void newG3dWindow();
	void initG3DFunctions();

public slots:
	void saveView();
	void reinitGraphics();

	void addCurrentImage();
	void saveImagesToDisk();
	
signals:
	void xRotationChanged(int angle);
	void yRotationChanged(int angle);
	void zRotationChanged(int angle);
	void zoomChanged(int value);

protected:
	// OpenGL functions
	void initializeGL();
	void paintGL(); // This is called when changing environments.
	void resizeGL(int width, int height);
	void computeNewVectors(p3d_vector4& Xc,p3d_vector4& Xw,p3d_vector4& up);
	
	// Mouse events
	void mousePressEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void keyPressEvent(QKeyEvent *e);
	void keyReleaseEvent(QKeyEvent *e);
	void mouseDoubleClickEvent(QMouseEvent *event);

private:
	// Pointer that allows resizing
	MainWindow*	m_mainWindow;
	
	// OpenGl variables
	GLdouble   x,y,z,el,az,zo;

	// size of the OpenGl scene
	double size;

	p3d_vector4  up;

	QPoint lastPos;

	// Colors for background
	QColor trolltechGreen;
	QColor trolltechPurple;
	QColor trolltechGrey;
	QColor trolltechBlack;
	QColor trolltechWhite;

	bool _light;
	bool _watingMouseRelease;

	// Do not draw when this 
	// variable is true
	bool _isThreadWorking;
	
	// Vector of recorded images
	QVector<QImage*> _pictures;
	
	// Counts the number of draw
	int paintNum;

#ifndef WITH_XFORMS
	qtG3DWindow* mG3DOld;
#endif
};

extern int mouse_mode;

#endif
