#ifdef QT_UI_XML_FILES
#include "qtMainInterface/mainwindow.hpp"
#endif

#include "main.hpp"
#include "cppToQt.hpp"

#if defined( OOMOVE3D_CORE ) || defined( CXX_PLANNER )
#include "API/scene.hpp"
#include "API/project.hpp"
#endif

#include <iostream>
#include <fcntl.h>
#include <QDesktopWidget>

#ifdef QT_GL
#include "qtUI/qtOpenGL/g3dQtConnection.hpp"
QSemaphore* sem;
GLWidget* openGlWidget;
#endif

#include "API/Graphic/drawModule.hpp"
#include "glutWindow.hpp"
#include "glut.h"

extern int mainMhp(int argc, char** argv);

using namespace std;


/**
 * @ingroup qtWindow
 * @brief Main double thread class (X-Forms Thread)
 */
Fl_thread::Fl_thread(QObject* parent) :
QThread(parent)
{
}

Fl_thread::Fl_thread(int argc, char** argv, QObject* parent) :
QThread(parent)
{
	_argc = argc;
	_argv = argv;
}

void Fl_thread::run()
{
	mainMhp(_argc, _argv);
	cout << "Ends main_old" << endl;
	//    terminate();
	//    wait();
}


/**
 * @ingroup qtWindow
 * @brief Main application with the QT_WidgetMain double thread class (X-Forms Thread)
 */
Main_threads::Main_threads()
{
#ifdef QT_GL
	sem = new QSemaphore(0);
#endif
}

Main_threads::~Main_threads()
{
	
}


int Main_threads::run(int argc, char** argv)
{
	app = new QApplication(argc, argv);
	app->setWindowIcon(QIcon::QIcon(QPixmap::QPixmap(molecule_xpm)));
	//    app->setStyle(new QCleanlooksStyle());
	//    app->setStyle(new QWindowsStyle());
	//    app->setStyle(new QMacStyle());
	
	Fl_thread move3dthread(argc, argv);
	connect(&move3dthread, SIGNAL(terminated()), this, SLOT(exit()));
	move3dthread.start();
	
#ifdef QT_GL
	cout << "Waiting end of parser to draw OpenGL and create Qt Forms ..."<< endl;
	sem->acquire();
	waitDrawAllWin = new QWaitCondition();
	lockDrawAllWin = new QMutex();
#endif
	
	// Creates the wrapper to the project 
	// Be carefull to initilize in the right thread
	global_Project = new Project(new Scene(XYZ_ENV));
	
#ifdef QT_UI_XML_FILES
	MainWindow w;
	//  w.showMaximized();
	
 	QRect g = QApplication::desktop()->screenGeometry();
 	cout << " x = " << g.x() << " y = " << g.y() << endl;
 	cout << " width = " << g.width() << " height = " << g.height() << endl;
 	
 	QRect g_window = w.geometry();
 	g_window.setWidth( g.width() );
 	g_window.setHeight( 0.707*g.height() ); // sqrt(2) / 2
 	g_window.moveTo( 0, 0 );
	
 	w.setGeometry( g_window );
	
	w.show();
	w.raise();
#endif
	
	return app->exec();
}


void Main_threads::exit()
{
	cout << "Ends all threads" << endl;
	app->quit();
}


/**
 * @defgroup qtWindow The Qt Window
 * The qt Module implements the interface in a separate thread so that it is necessary to use
 * such things as semaphore, locks and pipes to have everything behaving nicely. The maine function is as follows
 * \code
 app = new QApplication(argc, argv);
 
 Fl_thread move3dthread(argc, argv);
 connect(&move3dthread, SIGNAL(terminated()), this, SLOT(exit()));
 move3dthread.start();
 
 sem->acquire();
 cout << "Waiting"<< endl;
 waitDrawAllWin = new QWaitCondition();
 lockDrawAllWin = new QMutex();
 
 MainWindow w;
 w.show();
 
 return app->exec();
 \endcode
 */

int qt_fl_pipe[2];

/**
 * @ingroup qtWindow
 * @brief Main function of Move3D
 */
int main(int argc, char *argv[])
{
	
	enum DisplayMode 
	{
		MainMHP,
		qtWindow,
		Glut,
	} mode;
	
	mode = qtWindow;
	
	switch (mode) 
	{
		case MainMHP:
		{
			return mainMhp(argc, argv);
		}
			
		case qtWindow:
		{
#ifdef WITH_XFORMS
			pipe(qt_fl_pipe);
			fcntl(qt_fl_pipe[0], F_SETFL, O_NONBLOCK);
#endif
			
			Main_threads main;
			cout << "main.run(argc, argv)"  << endl;
			return main.run(argc, argv);
		}
		case Glut:
		{
			GlutWindowDisplay win(argc,argv);
			glutMainLoop ();
		}
			break;
			
		default:
			break;
	}
}
