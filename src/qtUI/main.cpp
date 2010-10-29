#ifdef QT_UI_XML_FILES
#include "qtMainInterface/mainwindow.hpp"
#endif

#include "main.hpp"
#include "planner_handler.hpp"
#include "cppToQt.hpp"

#include "API/scene.hpp"
#include "API/project.hpp"

#include <iostream>
#include <QDesktopWidget>

#ifdef QT_GL
QSemaphore* sem;
GLWidget* openGlWidget;
#endif

#include "API/Graphic/drawModule.hpp"

#ifdef USE_GLUT
#include "glutWindow.hpp"
#include "glut.h"
#endif

extern int mainMhp(int argc, char** argv);

using namespace std;


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

// Temporary mechanism to redraw the opengl scene.
// Not elegant, but it works.
MainWindow* global_w(NULL);
void draw_opengl()
{
  if(global_w != NULL)
  {
    QMetaObject::invokeMethod(global_w->getOpenGL(),
			      "updateGL",
			      Qt::BlockingQueuedConnection);
  }
}

int Main_threads::run(int argc, char** argv)
{
	app = new QApplication(argc, argv);
	app->setWindowIcon(QIcon::QIcon(QPixmap::QPixmap(molecule_xpm)));
	//    app->setStyle(new QCleanlooksStyle());
	//    app->setStyle(new QWindowsStyle());
	//    app->setStyle(new QMacStyle());

	QThread plannerThread;
	PlannerHandler plannerHandler(argc, argv);
	plannerHandler.moveToThread(&plannerThread);
	plannerThread.start();
	QMetaObject::invokeMethod(&plannerHandler,
				  "init",
				  Qt::BlockingQueuedConnection);
	
	// Creates the wrapper to the project 
	// Be carefull to initialize in the right thread
	global_Project = new Project(new Scene(XYZ_ENV));
	
#ifdef QT_UI_XML_FILES
	MainWindow w;
	global_w = &w;
	// Start
	connect(&w, SIGNAL(runClicked()), this, SLOT(selectPlanner()));
	connect(&w, SIGNAL(runClicked()), &w, SLOT(enableStopButton()));
	connect(this, SIGNAL(selectedPlanner(QString)),
		&plannerHandler, SLOT(startPlanner(QString)));
	// Stop
	connect(&w, SIGNAL(stopClicked()), &plannerHandler, SLOT(stopPlanner()), Qt::DirectConnection);
	connect(&plannerHandler, SIGNAL(plannerIsStopped()),
		&w, SLOT(enableRunAndResetButtons()));
	connect(&plannerHandler, SIGNAL(plannerIsStopped()),
		&w, SLOT(drawAllWinActive()));
	// Reset
	connect(&w, SIGNAL(resetClicked()), &plannerHandler, SLOT(resetPlanner()));
	connect(&plannerHandler, SIGNAL(plannerIsReset()),
		&w, SLOT(enableRunButton()));
	connect(&plannerHandler, SIGNAL(plannerIsReset()),
		&w, SLOT(drawAllWinActive()));
	//  w.showMaximized();

#ifdef QT_GL
	connect(&plannerHandler, SIGNAL(redraw()),
		&w, SLOT(drawAllWinActiveHack()), Qt::BlockingQueuedConnection);
#endif
	
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

void Main_threads::selectPlanner()
{
  if(ENV.getBool(Env::isPRMvsDiffusion))
  {
    emit(selectedPlanner(QString("PRM")));
  }
  else
  {
    emit(selectedPlanner(QString("Diffusion")));
  }
}

void Main_threads::exit()
{
	cout << "Ends all threads" << endl;
	app->quit();
}

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
			Main_threads main;
			//cout << "main.run(argc, argv)"  << endl;
			return main.run(argc, argv);
		}
		case Glut:
		{
#ifdef USE_GLUT
			GlutWindowDisplay win(argc,argv);
			glutMainLoop ();
#else
			cout << "Error : Glut is not linked" << endl;
#endif
		}
			break;
			
		default:
			break;
	}
}
