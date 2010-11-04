/*
 * Source File for the Open GL Window
 */

#include "glwidget.hpp"

#include "qtUI/qtMainInterface/mainwindow.hpp"
#include "qtUI/qtMainInterface/mainwindowGenerated.hpp"

#include "API/Graphic/drawModule.hpp"

#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Graphic-pkg.h"
#include "Collision-pkg.h" // Necessary to get access to the robotboxlist variable.

#include <iostream>
#include <iomanip>
#include <iosfwd>
#include <sstream>
#include <fstream>

using namespace std;

extern void draw_opengl();

extern void* GroundCostObj;

GLWidget::GLWidget(QWidget *parent) :
QGLWidget(parent)
{
	x = 0;
	y = 0;
	z = 0;
	
	up[0] = 0;
	up[1] = 0;
	up[2] = 0;
	up[3] = 0;
	
	az = INIT_AZ;
	el = INIT_EL;
	double x1, x2, y1, y2, z1, z2;
	p3d_get_env_box(&x1, &x2, &y1, &y2, &z1, &z2);
	zo = 0.2 * MAX(MAX(x2-x1,y2-y1),z2-z1);
	
	trolltechGreen = QColor::fromCmykF(0.40, 0.0, 1.0, 0.0);
	trolltechPurple = QColor::fromCmykF(0.39, 0.39, 0.0, 0.0);
	trolltechBlack = QColor::fromCmykF(1.0, 1.0, 1.0, 0.0);
	trolltechWhite =  QColor::fromCmykF(0.0, 0.0, 0.0, 0.0);
	trolltechGrey = trolltechWhite;
	
	_isThreadWorking = false;
	_light = false;
	
	initG3DFunctions();
	
	mG3DOld = new qtG3DWindow;
	
	g3d_set_win_floor_color(g3d_get_cur_states(), 1, 0.87, 0.49);
	//g3d_set_win_bgcolor(g3d_get_cur_win(), 0.5, 0.6, 1.0);
	g3d_set_win_wall_color(g3d_get_cur_states(), 0.4, 0.45, 0.5);
	
	g3d_set_win_bgcolor(g3d_get_cur_states(), 
											XYZ_ENV->background_color[0], 
											XYZ_ENV->background_color[1], 
											XYZ_ENV->background_color[2]);
	//	setFocusPolicy(Qt::StrongFocus);
}


GLWidget::~GLWidget()
{
	makeCurrent();
}

void GLWidget::setWinSize(double size)
{
	this->size = size;
}

// -------------------------------------------------------
// Save images to disk functions
// -------------------------------------------------------
void GLWidget::addCurrentImage()
{
	//	QPixmap* image = new QPixmap(renderPixmap());
	QImage* image = new QImage(grabFrameBuffer());
	_pictures.push_back(image);
}

void GLWidget::saveImagesToDisk()
{
	ostringstream oss(ostringstream::out);
	oss << "cd "<< getenv("HOME_MOVE3D") <<"/video;rm *.jpg";
	system(oss.str().c_str());
	
	for (int i = 0; i < _pictures.size(); i++)
	{
		oss.str("");
		oss << "video/Image_" << setfill('0') << setw(4) << i << ".jpg";
		cout << "Saving : " << oss.str() << endl;
		_pictures.at(i)->save(oss.str().c_str(), "JPG", 100);
	}
	resetImageVector();
	cout << "Images saved to video/" << endl;
	
	oss.str("");
	//change to video directory then compress jpg files to AVI video for more parameters and video format see man pages of mencoder
	oss << "cd "<<getenv("HOME_MOVE3D")<<"/video;mencoder mf://*.jpg -mf w=800:h=600:fps=25:type=jpeg -ovc lavc -lavcopts vcodec=mpeg4 -oac copy -o output.avi";
	system(oss.str().c_str());
}

void GLWidget::resetImageVector()
{
	for (int i = 0; i < _pictures.size(); i++)
	{
		delete _pictures.at(i);
	}
	_pictures.clear();
}

void GLWidget::saveView()
{
	QImage image = grabFrameBuffer();
	//	QPixmap image = renderPixmap(0,0,false);
	image.save("OpenGLView.jpg", "JPG", 100);
}

// -------------------------------------------------------
// Initialize G3D functions
// -------------------------------------------------------

// Mode for rotation translation 
// control of the camera frame
int mouse_mode = 0;

// place on the screen
int _i; int _j;

void qt_get_win_mouse(int* i, int* j)
{
	//cout << "qt_get_win_mouse" <<  endl;
	*i = _i;
	*j = _j;
}

void qt_ui_calc_param(g3d_cam_param& p)
{
	//cout << "qt_ui_calc_param" <<  endl;
	p3d_vector4 up;
	
	calc_cam_param(G3D_WIN, p.Xc, p.Xw);
	
	if (G3D_WIN)
	{
		p3d_matvec4Mult(*G3D_WIN->cam_frame, G3D_WIN->vs.up, up);
	}
	else
	{
		up[0] = 0;
		up[1] = 0;
		up[2] = 1;
	}
	
	p.up[0] = up[0];
	p.up[1] = up[1];
	p.up[2] = up[2];
}

void GLWidget::initG3DFunctions()
{
	ext_get_win_mouse = (void (*) (int*,int*))(qt_get_win_mouse);
	ext_g3d_draw_allwin_active = draw_opengl;
	ext_calc_cam_param = (void (*) (g3d_cam_param&) )(qt_ui_calc_param);
	
	Graphic::initDrawFunctions();
}

// -------------------------------------------------------
// Paint functions
// -------------------------------------------------------
void GLWidget::initializeGL()
{
	glViewport(0,0,(GLint) 800,(GLint) 600);
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
	
	//	glViewport(0, 0, (GLint) 800, (GLint) 600);
	//
	//        if(!GroundCostObj)
	//        {
	//            qglClearColor(trolltechGrey);
	//        }
	//        else
	//        {
	//            qglClearColor(trolltechWhite);
	//            G3D_WIN->vs.displayFloor = false;
	//        }
	//
	//	glMatrixMode(GL_PROJECTION);
	//	glLoadIdentity();
	//
	//	gluPerspective(40.0, (GLdouble) 800 / (GLdouble) 600, size / 1000., size
	//			* 1000.0);
	//
	//	glMatrixMode(GL_MODELVIEW);
	//	glLoadIdentity();
	
	//	cout << "initGL" << endl; 
}

void GLWidget::setThreadWorking(bool isWorking)
{
	_isThreadWorking = isWorking;
}

// Camera vectors
//p3d_vector4 JimXc;
//p3d_vector4 JimXw;
//p3d_vector4 Jimup;

void GLWidget::paintGL()
{
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPushMatrix();
	
	p3d_vector4 Xc, Xw;
	p3d_vector4 up;
	
	//	computeNewVectors(Xc,Xw,up);
	g3d_cam_param p;
	qt_ui_calc_param(p);
	
	Xc[0] = p.Xc[0];
	Xc[1] = p.Xc[1];
	Xc[2] = p.Xc[2];
	
	Xw[0] = p.Xw[0];
	Xw[1] = p.Xw[1];
	Xw[2] = p.Xw[2];
	
	up[0] = p.up[0];
	up[1] = p.up[1];
	up[2] = p.up[2];
	
	gluLookAt(Xc[0], Xc[1], Xc[2], Xw[0], Xw[1], Xw[2], up[0], up[1], up[2]);
	
	G3D_WIN->vs.cameraPosition[0]= Xc[0];
	G3D_WIN->vs.cameraPosition[1]= Xc[1];
	G3D_WIN->vs.cameraPosition[2]= Xc[2];

	//g3d_draw();

	glPopMatrix();
	
	//cout << "paintGL() : " << paintNum++ << endl;
	//cout << "------------------------------------------------" << endl;
	//cout << Xc[0] << " " << Xc[1] << " " << Xc[2] << endl;
	//cout << Xw[0] << " " << Xw[1] << " " << Xw[2] << endl;
	//cout << up[0] << " " << up[1] << " " << up[2] << endl;
}

void GLWidget::reinitGraphics()
{
  pp3d_env env;
  
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  g3d_delete_all_poly(-1);
  
  if (boxlist != -1)
  {
    glDeleteLists(boxlist, 1);
    boxlist = -1;
  }
  
#ifdef P3D_COLLISION_CHECKING
  if (p3d_get_robotboxlist() != -1) {
    glDeleteLists(p3d_get_robotboxlist(), 1);
    p3d_reset_robotboxlist();
  }
#endif
  
  env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  if (env)
  {
    env->INIT = 1;
  }
}

void GLWidget::resizeGL(int width, int height)
{
	glViewport(0, 0, (GLint) width, (GLint) height);
	//	qglClearColor(QColor::fromCmykF(0.0, 0.0, 0.0, 0.0));
	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	
	gluPerspective(40.0, (GLdouble) width / (GLdouble) height, size / 10000.0, size * 1000.0);
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void GLWidget::computeNewVectors(p3d_vector4& Xc, p3d_vector4& Xw,
																 p3d_vector4& up)
{
	static p3d_matrix4 Txc =
	{
		{ 1, 0, 0, 0 },
		{ 0, 1, 0, 0 },
		{ 0, 0, 1, 0 },
		{ 0, 0, 0, 1 } };
	p3d_matrix4 Id =
	{
		{ 1, 0, 0, 0 },
		{ 0, 1, 0, 0 },
		{ 0, 0, 1, 0 },
		{ 0, 0, 0, 1 } };
	p3d_matrix4 m_aux;
	p3d_vector4 Xx;
	
	Xx[0] = x;
	Xx[1] = y;
	Xx[2] = z;
	Xx[3] = 1.0;
	p3d_matvec4Mult(Id, Xx, Xw);
	
	Txc[0][3] = this->zo * (cos(this->az) * cos(this->el));
	Txc[1][3] = this->zo * (sin(this->az) * cos(this->el));
	Txc[2][3] = this->zo * sin(this->el);
	
	p3d_mat4Mult(Id, Txc, m_aux);
	p3d_matvec4Mult(m_aux, Xx, Xc);
	
	p3d_matvec4Mult(Id, this->up, up);
}

// -------------------------------------------------------
// Mouse board events
// -------------------------------------------------------

void GLWidget::mousePressEvent(QMouseEvent *event)
{
	if (!_light)
	{
		_i = event->x();// - lastPos.x());
		_j = event->y();// - lastPos.y());
		qt_canvas_viewing(1, 0);
		//		_watingMouseRelease = true;
	}
	
	lastPos = event->pos();
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
	if (!_light)
	{
		// Viewing mode
		_i = event->x();
		_j = event->y();
		
		//cout << "Mouse Button : " << event->buttons() << endl;
		
		if ((event->buttons() ==			( Qt::LeftButton | Qt::MidButton )) &&(mouse_mode==0) )
		{
			//cout << "Qt::LeftButton	& Qt::MidButton" << endl;
			qt_canvas_viewing(0, 3);
		}
		else if ((event->buttons() == ( Qt::MidButton | Qt::RightButton )) &&(mouse_mode==0) )
		{
			//cout << "Qt::MidButton		& Qt::RightButton" << endl;
			qt_canvas_viewing(0, 4);
		}
		else if ((event->buttons() == ( Qt::RightButton | Qt::LeftButton)) &&(mouse_mode==0) )
		{
			//cout << "Qt::RightButton & Qt::LeftButton" << endl;
			qt_canvas_viewing(0, 5);
		}
		else if ((event->buttons() == Qt::LeftButton)&&(mouse_mode==0))
		{
			qt_canvas_viewing(0, 0);
		}
		else if ((event->buttons() == Qt::MidButton) || ((event->buttons() & Qt::LeftButton)&&(mouse_mode==1)) )
		{
			qt_canvas_viewing(0, 1);
		}
		else if ((event->buttons() == Qt::RightButton) || ((event->buttons() & Qt::LeftButton)&&(mouse_mode==2)) )
		{
			qt_canvas_viewing(0, 2);
		}
		_watingMouseRelease = true;
	}
	else
	{
		// Light managment
		int dx = event->x() - lastPos.x();
		int dy = event->y() - lastPos.y();
		
		double xmin, xmax, ymin, ymax, zmin, zmax;
		p3d_get_env_box(&xmin, &xmax, &ymin, &ymax, &zmin, &zmax);
		
		if (event->buttons() & Qt::LeftButton)
		{
			if( (dx<0 && qt_get_cur_g3d_win()->vs.lightPosition[0]>xmin )
				 || (dx>0 && qt_get_cur_g3d_win()->vs.lightPosition[0]>xmax ) )
			{
				qt_get_cur_g3d_win()->vs.lightPosition[0] += (dx*(xmax-xmin) / 100);
			}
		}
		else if (event->buttons() & Qt::MidButton)
		{
			if( (dy<0 && qt_get_cur_g3d_win()->vs.lightPosition[1]>ymin )
				 || (dy>0 && qt_get_cur_g3d_win()->vs.lightPosition[1]>ymax ) )
			{
				qt_get_cur_g3d_win()->vs.lightPosition[1] += (dy * (ymax-ymin ) / 100);
			}
		}
		else if (event->buttons() & Qt::RightButton)
		{
			if( (dy<0 && qt_get_cur_g3d_win()->vs.lightPosition[2]>zmin )
				 || (dy>0 && qt_get_cur_g3d_win()->vs.lightPosition[2]>zmax ) )
			{
				qt_get_cur_g3d_win()->vs.lightPosition[2] += (dy*(zmax - zmin) / 100);
			}
		}
	}
	//	updateGL();
	if(!ENV.getBool(Env::isRunning))
	{
		updateGL();
	}
}

void GLWidget::mouseDoubleClickEvent (QMouseEvent *event)
{
	if(   ( event->buttons()			&& ( Qt::LeftButton )) &&
				( event->modifiers()	&& ( Qt::ControlModifier )) )
	{
		QRect geom;
		
		geom = m_mainWindow->Ui()->centralWidget->geometry();
		
		QList<int> list;
		
		list.clear();
		list.push_back( 0 );
		list.push_back( geom.width() );
	
		m_mainWindow->Ui()->vSplitter->setSizes ( list );
		
		list.clear();
		list.push_back( geom.height() );
		list.push_back( 0 );
		
		m_mainWindow->Ui()->hSplitter->setSizes ( list );
		
		m_mainWindow->showNormal();
		m_mainWindow->showMaximized();
	}
}


// -------------------------------------------------------
// Key board events
// -------------------------------------------------------
void GLWidget::keyPressEvent(QKeyEvent *e)
{
	cout << "A key is pressed" << endl;
	
	switch (e->key())
	{
		case Qt::Key_Shift:
			_light = !_light;
			cout << "Shift pressed" << endl;
			break;
			
		case Qt::Key_Plus:
			// 		if (qt_get_cur_g3d_win()->shadowContrast < 9.94)
			// 		{
			// 			qt_get_cur_g3d_win()->shadowContrast += 0.05;
			// 		}
			cout << "+ pressed" << endl;
			break;
			
		case Qt::Key_Minus:
			// 		if (qt_get_cur_g3d_win()->shadowContrast > 0.06)
			// 		{
			// 			qt_get_cur_g3d_win()->shadowContrast -= 0.05;
			// 		}
			cout << "- pressed" << endl;
			break;
			
		case Qt::Key_A:
			cout << "A pressed" << endl;
			break;
			
		case Qt::Key_B:
			cout << "B pressed" << endl;
			break;
	}
}

void GLWidget::keyReleaseEvent(QKeyEvent *e)
{
	cout << "A key is release" << endl;
	
	switch (e->key())
	{
			
			break;
	}
}
