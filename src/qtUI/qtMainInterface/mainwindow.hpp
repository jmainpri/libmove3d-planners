#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include "p3d_sys.h"
#include "env.hpp"

#if defined( CXX_PLANNER )

#include "kcdpropertieswindow.hpp"
#include "qtBase/qt_widgets.hpp"
#include "qtFormRobot/moverobot.hpp"
#include "ParametersEnv.hpp"

#ifdef QWT
#include "qtPlot/histoWin.hpp"
#endif

#endif

#if defined( MOVE3D_CORE )

#include "qtUI/qtMainInterface/kcdpropertieswindow.hpp"
#include "qtUI/qtBase/qt_widgets.hpp"
#include "qtUI/qtFormRobot/moverobot.hpp"
#include "ParametersEnv.hpp"

#ifdef QWT
#include "qtUI/qtPlot/histoWin.hpp"
#endif

#endif

#ifndef GLWIDGET_H
class GLWidget;
#endif
#ifndef MAIN_WINDOW_TEST_FUNCTIONS
class MainWindowTestFunctions;
#endif


#include <vector>

namespace Ui
{
	class MainWindow;
}

/**
 * @ingroup qtMainWindow
 * @brief Qt Main Window container
 * Tow Widget are derived from other classes The GLWidget widget and the MoveRobot Widget
 \image html Designer.png
 */
class MainWindow : public QMainWindow
{
	Q_OBJECT
	
public:
	MainWindow(QWidget *parent = 0);
	~MainWindow();
	
	/**
	 * Function tro create sliders and checkboxes TODO move somwhere else
	 */
	void connectCheckBoxToEnv(QCheckBox* box, Env::boolParameter p);
	void connectCheckBoxToEnv(QCheckBox* box, PlanParam::boolParameter p);
	
	LabeledSlider* createSlider(QString s, Env::intParameter p,int lower, int upper);
	LabeledDoubleSlider* createDoubleSlider(QString s,Env::doubleParameter p, double lower, double upper);

	Ui::MainWindow* Ui() { return m_ui; }
	
	GLWidget*		getOpenGL();
	MoveRobot*	getMoveRobot();
	
	public slots:
	void drawAllWinActive();
	void isPlanning();
	void planningFinished();
	
	void setBoolGhost(bool value);
	void setBoolBb(bool value);
	void setBoolFloor(bool value);
	void setBoolTiles(bool value);
	void setBoolWalls(bool value);
	void setBoolSmooth(bool value);
	void setBoolShadows(bool value);
	void setBoolFilaire(bool value);
	void setBoolJoints(bool value);
	void setBoolContour(bool value);
	void setBoolEnableLight(bool value);
	
	void setJointToDraw(int joint);
	
	void setCurrentTraj(p3d_traj* traj);

signals:
  void runClicked();
  void stopClicked();
  void resetClicked();
	
protected:
	void keyPressEvent(QKeyEvent *e);
	void keyReleaseEvent(QKeyEvent *e);
	
	
private slots:
	
	void initRobotsMenu();
	void setRobotAsCurrent();
	
	void openScenario();
	void saveScenario();
	
	void loadGraph();
	void saveGraph();
	void saveXYZGraphToDot();
	
	void loadTraj();
	void saveTraj();
	
	void changeLightPosX();
	void changeLightPosY();
	void changeLightPosZ();
	void addTrajToDraw();
	void clearTrajToDraw();
	void colorTrajChange(int color);
  void enableRunAndResetButtons();
  void enableStopButton();
  void enableRunButton();
	void showTraj();
	void restoreView();
	void mobileCamera();
	
	void test();
	
	// Global
	//    void setLineEditWithNumber(Env::intParameter p , int val );
	void changeEvent(QEvent *e);
	
private:
	
	Ui::MainWindow*							m_ui;
	
	KCDpropertiesWindow*				mKCDpropertiesWindow;
	
	MainWindowTestFunctions*		m_testFunctions;
	
	std::vector<QAction*>				m_RobotsInMenu;
	
	void connectCheckBoxes();

	void initRunButtons();
	void initViewerButtons();
	void initLightSource();
	
	p3d_traj* m_currentTraj;
};

// Global MainWindow Pointer 
extern MainWindow* global_w;

/**
 * @ingroup qtWindow
 * @brief Showtraj thread class 
 */
class Showtrajthread: public QThread
{
	Q_OBJECT
	
public:
	Showtrajthread(QObject* parent = 0);
	
protected:
	void run();
	
};


#endif // MAINWINDOW_H
