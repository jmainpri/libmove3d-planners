#ifndef MOVEROBOT_HPP
#define MOVEROBOT_HPP

#include "qtLibrary.hpp"
#include <QtGui/QStackedLayout>

#if defined( CXX_PLANNER )
#include "qtBase/SpinBoxSliderConnector_p.hpp"
#endif

#if defined( MOVE3D_CORE )
#include "qtUI/qtBase/SpinBoxSliderConnector_p.hpp"
#endif

#if defined( CXX_PLANNER ) || defined( MOVE3D_CORE ) 
#include "API/Device/joint.hpp"
#include "API/ConfigSpace/configuration.hpp"
#ifndef ROBOT_HPP
class Robot;
#endif
#endif

#ifndef GLWIDGET_H
class GLWidget;
#endif

class FormRobot;

namespace Ui {
	class MoveRobot;
}

/**--------------------------------------------------------------
 * @ingroup qtMainWindow
 * @brief Creates one DoF slider structure
 */
class DofSlider : public QObject
{
	Q_OBJECT
	
public:
	DofSlider() {}
	
	DofSlider(Robot* R, GLWidget* Gl, FormRobot* FR) : 
	mRobot(R),
	mOpenGl(Gl),
	mFormRobot(FR)
	{}
	
	~DofSlider() {}
	
	/**
	 * Creates a slider with a spinbox and a label
	 */
	void makeSlider(QGridLayout* gridLayout, Joint *jntPt, int DofNumOnJnt);
	
	void setValue(double value) { mConnector->setValue(value); }
	
	QDoubleSpinBox* getDoubleSpinBox() { return mDoubleSpinBox;}
	QSlider* getHorizontalSlider() { return mHorizontalSlider; }
	
	QtShiva::SpinBoxSliderConnector* getConnector() {return mConnector;}
	
	
#if defined( CXX_PLANNER ) || defined( MOVE3D_CORE ) 
	Robot* getRobot() { return mRobot; }
#endif
	
	public slots:
	void dofValueChanged(double value);
	
private:
#if defined( CXX_PLANNER ) || defined( MOVE3D_CORE ) 
	Robot*  mRobot;
#endif
	
	int								mDofNum;
	GLWidget*					mOpenGl;
	
	FormRobot*				mFormRobot;
	
	QLabel*						mLabel;
	QDoubleSpinBox*		mDoubleSpinBox;
	QSlider*					mHorizontalSlider;
	
	QtShiva::SpinBoxSliderConnector* mConnector;
};

/**--------------------------------------------------------------
 * @ingroup qtMainWindow
 * @brief Creates the sliders structure, Contains a vector of DofSlider
 */
class FormRobot : public QObject {
	
	Q_OBJECT
	
public:
	FormRobot() {}
	
	FormRobot(Robot* R,QGridLayout* GL,QComboBox* pos,QComboBox* traj,GLWidget* openGl) :
	mRobot(R),
	mGridLayout(GL),
	mPositions(pos),
	mTrajectoriesNames(traj),
	mOpenGl(openGl)
	{}
	
	~FormRobot() {};
	
	/**
	 * Initializes the sliders associated to the Dofs of ptrRob
	 */
	void initSliders();
	
	/**
	 * Sets the associated sliders to the values int ptrConf
	 */
	void setSliders(Configuration& ptrConf);
	
	/**
	 * Resets the constrained DoFs
	 */
	void resetConstraintedDoFs();
	
	/**
	 * Returns the robot structure
	 */
	Robot* getRobot() { return mRobot; }
	
	/**
	 *
	 */
	QComboBox* getComboBox() { return mPositions; }
	
	public slots:
	/**
	 * Sets the current Configuration to Init or Goto
	 */
	void setCurrentPosition(int position);
	
	/**
	 * Save current configuration to position (Init or GoTo)
	 */
	void saveCurrentConfigToPosition();
	
	/**
	 * Add Traj
	 */
	void addTraj( std::string& name, p3d_traj* trajPt );
	
	/**
	 * Function called to set the current trajectory
	 */
	void setCurrentTraj(int id);
	
private:
	
	int calc_real_dof(void);
	
	Robot*										mRobot;
	
	std::vector<DofSlider*>		mSliders;
	QGridLayout*							mGridLayout;
	
	QComboBox*								mPositions;
	
	QComboBox*								mTrajectoriesNames;
	std::vector<p3d_traj*>		mTrajectories;
	
	GLWidget*									mOpenGl;
};


/**--------------------------------------------------------------
 * @ingroup qtMainWindow
 * @brief Creates the FormRobots Stucture, Contains a vector of FormRobot
 */
class MoveRobot : public QWidget 
{
	Q_OBJECT
	
public:
	MoveRobot(QWidget *parent = 0);
	~MoveRobot();
	
	/**
	 * Initilizes all forms
	 */
	void initAllForms(GLWidget* ptrOpenGl);
	
	/**
	 * Updates all robot pos
	 */
	void updateAllRobotInitPos();
	
	/**
	 * Get the robot form by names
	 */
	FormRobot* getRobotFormByName(std::string name);
	
	/**
	 * Sets the constraints
	 */
	void setRobotConstraintedDof(Robot* ptrRob);
	
protected:
	void changeEvent(QEvent *e);
	
private:
	Ui::MoveRobot *m_ui;
	
#if defined( CXX_PLANNER ) || defined( MOVE3D_CORE ) 
	/**
	 * Creates a new gridLayout inside a tabWidget
	 */
	FormRobot* newGridLayoutForRobot(Robot* ptrRob);
	FormRobot* newGridLayoutForRobotStacked(Robot* ptrRob);
	
#endif
	
	/**
	 * Members
	 */
	std::vector<FormRobot*>			mRobots;
	
	// Tab Widget
	QTabWidget*                 mTabWidget;
	
	// Stacked Widgets
	QStackedLayout*							m_StackedLayout;
	QComboBox*									m_pageComboBox;
	
	GLWidget*										mOpenGl;
};

#endif // MOVEROBOT_HPP
