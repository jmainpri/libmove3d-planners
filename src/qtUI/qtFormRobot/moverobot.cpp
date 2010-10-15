#include "moverobot.hpp"
#include "ui_moverobot.h"

#include "qtOpenGL/glwidget.hpp"

#include <iostream>
#include <tr1/memory>

#include "P3d-pkg.h"
#include "Collision-pkg.h"

#include "cost_space.hpp"
#include "API/project.hpp"

#ifdef HRI_COSTSPACE
#include "HRICS_costspace.hpp"
#endif

#ifdef WITH_XFORMS
#include "cppToQt.hpp"
#endif

using namespace std;
using namespace tr1;

MoveRobot::MoveRobot(QWidget *parent) :
QWidget(parent),
m_ui(new Ui::MoveRobot)
{
	m_ui->setupUi(this);
	
}

MoveRobot::~MoveRobot()
{
	delete m_ui;
}

/*!
 * This function intialize all Robot Forms
 * and sets them in a Tab widget
 */
void MoveRobot::initAllForms(GLWidget* ptrOpenGl)
{
	mOpenGl= ptrOpenGl;
	
	Scene* envPt = global_Project->getActiveScene();
	
	for(unsigned int i=0;i<envPt->getNumberOfRobots();i++)
	{
//		if(i==0)
//		{
//			mTabWidget = new QTabWidget(this);
//			mTabWidget->setUsesScrollButtons(true);
//		}

		if(i==0)
		{
			m_pageComboBox = new QComboBox;
			m_StackedLayout = new QStackedLayout;
			
			m_ui->MainLayout->addWidget( m_pageComboBox );
			m_ui->MainLayout->addLayout( m_StackedLayout );
			
			connect(m_pageComboBox, SIGNAL(activated(int)), m_StackedLayout, SLOT(setCurrentIndex(int)));
		}
		
#if defined( CXX_PLANNER ) || defined( WITH_OOMOVE3D ) 
		Robot* ptrRob = envPt->getRobot(i);
		
		FormRobot* form = newGridLayoutForRobotStacked(ptrRob);
		
		form->initSliders();
		form->resetConstraintedDoFs();
		
		shared_ptr<Configuration> ptrConf = ptrRob->getInitialPosition();
		
		ptrRob->setAndUpdate( *ptrConf );
		form->setSliders( *ptrConf );
		mRobots.push_back( form );
		//cout << "MoveRobot::ptrRob->getRobotStruct()->getNumberOfJoints() = "  << ptrRob->getNumberOfJoints() << endl;
#endif
#ifdef WITH_XFORMS
		std::string str = "g3d_draw_allwin_active";
		write(qt_fl_pipe[1],str.c_str(),str.length()+1);
#else
		if(!ENV.getBool(Env::isRunning))
		{
			mOpenGl->updateGL();
		}
#endif
	}
		//m_ui->MainLayout->addLayout(m_StackedLayout);
}

FormRobot* MoveRobot::getRobotFormByName(string name)
{
	for(unsigned int i=0;i<mRobots.size();i++)
	{
		if (mRobots[i]->getRobot()->getName().compare(name) == 0) 
		{
			return mRobots[i];
		}
	}
	cout << "Robot " << name << "doesn't exist" << endl;
	return 0x00;
}

void MoveRobot::setRobotConstraintedDof(Robot* ptrRob)
{
	FormRobot* form = getRobotFormByName(ptrRob->getName());
	form->resetConstraintedDoFs();
	form->setSliders(*ptrRob->getCurrentPos());
}

/*!
 * This function intialize a Robot Form from a 
 * Robot structure
 */
FormRobot* MoveRobot::newGridLayoutForRobot(Robot* ptrRob)
{
	QString robotName(ptrRob->getName().c_str());
	QWidget *tab;
	QVBoxLayout *robotLayoutTab;
	QScrollArea *scrollArea;
	QWidget *scrollAreaWidgetContents;
	QHBoxLayout *horizontalLayoutScrollArea;
	QGridLayout *gridLayout;
	
	tab = new QWidget();
	
	robotLayoutTab = new QVBoxLayout(tab);
	scrollArea = new QScrollArea(tab);
	scrollArea->setWidgetResizable(true);
	scrollAreaWidgetContents = new QWidget();
	//scrollAreaWidgetContents->setGeometry(QRect(0, 0, 700, 250));
	horizontalLayoutScrollArea = new QHBoxLayout(scrollAreaWidgetContents);
	
	gridLayout = new QGridLayout();
	horizontalLayoutScrollArea->addLayout(gridLayout);
	scrollArea->setWidget(scrollAreaWidgetContents);
	robotLayoutTab->addWidget(scrollArea);
	
	QComboBox* positions = new QComboBox();
	positions->addItem(QString::QString("Start"));
	positions->addItem(QString::QString("Goal"));
	robotLayoutTab->addWidget(positions);
	
	QPushButton* saveButton = new QPushButton("Save Current");
	robotLayoutTab->addWidget(saveButton);
	
	mTabWidget->addTab(tab, QString());
	mTabWidget->setTabText(mTabWidget->indexOf(tab), robotName );
	
	m_ui->MainLayout->addWidget(mTabWidget);
	
	FormRobot* formRobot = new FormRobot(ptrRob,gridLayout,positions,mOpenGl);
	
	connect(positions, SIGNAL(currentIndexChanged(int)),formRobot, SLOT(setCurrentPosition(int)));   
	positions->setCurrentIndex( 0 );
	
	connect(saveButton,SIGNAL(clicked()),formRobot,SLOT(saveCurrentConfigToPosition()));
	
	return formRobot;
}

/*!
 * This function intialize a Robot Form from a 
 * Robot structure
 */
FormRobot* MoveRobot::newGridLayoutForRobotStacked(Robot* ptrRob)
{
	QString						robotName(ptrRob->getName().c_str());
	QWidget*					tab;
	QVBoxLayout*			robotLayoutTab;
	QScrollArea*			scrollArea;
	QWidget*					scrollAreaWidgetContents;
	QHBoxLayout*			horizontalLayoutScrollArea;
	QGridLayout*			gridLayout;
	
	tab = new QWidget();
	
	robotLayoutTab = new QVBoxLayout(tab);
	scrollArea = new QScrollArea(tab);
	scrollArea->setWidgetResizable(true);
	scrollAreaWidgetContents = new QWidget();
	//scrollAreaWidgetContents->setGeometry(QRect(0, 0, 700, 250));
	horizontalLayoutScrollArea = new QHBoxLayout(scrollAreaWidgetContents);
	
	gridLayout = new QGridLayout();
	horizontalLayoutScrollArea->addLayout(gridLayout);
	scrollArea->setWidget(scrollAreaWidgetContents);
	robotLayoutTab->addWidget(scrollArea);
	
	QComboBox* positions = new QComboBox();
	positions->addItem(QString::QString("Start"));
	positions->addItem(QString::QString("Goal"));
	robotLayoutTab->addWidget(positions);
	
	QPushButton* saveButton = new QPushButton("Save Current");
	robotLayoutTab->addWidget(saveButton);
	
	m_StackedLayout->addWidget( tab );
	m_pageComboBox->addItem(robotName);
	
	FormRobot* formRobot = new FormRobot( ptrRob , gridLayout , positions , mOpenGl );
	
	connect(positions, SIGNAL(currentIndexChanged(int)),formRobot, SLOT(setCurrentPosition(int)));   
	positions->setCurrentIndex( 0 );
	
	connect(saveButton,SIGNAL(clicked()),formRobot,SLOT(saveCurrentConfigToPosition()));
	
	return formRobot;
}

/*!
 * Sets the Robot Forms to the initial
 * configuration of their associated Robots
 */
void MoveRobot::updateAllRobotInitPos()
{
	for(unsigned int i=0;i<mRobots.size();i++)
	{
		Robot* robot = mRobots[i]->getRobot();
		robot->setAndUpdate( *robot->getInitialPosition() );
		mRobots[i]->setSliders( *robot->getCurrentPos() );
		mRobots[i]->getComboBox()->setCurrentIndex(0);
		cout << "Set Robot " << robot->getName() << " to its initial position" << endl;
	}
}

void MoveRobot::changeEvent(QEvent *e)
{
	QWidget::changeEvent(e);
	switch (e->type()) {
    case QEvent::LanguageChange:
			m_ui->retranslateUi(this);
			break;
    default:
			break;
	}
}

//---------------------------------------------------------------------
// FormRobot
//---------------------------------------------------------------------

/*!
 * Computes the number of dof of the Robot
 */
int FormRobot::calc_real_dof(void)
{
	int nrd;
	int njnt,i,j,k;
	p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
	
	nrd = robotPt->nb_user_dof;
	njnt = p3d_get_robot_njnt();
	
	//#ifdef P3D_COLLISION_CHECKING
	//    if(njnt > MAX_NJNTS_IN_ROBOTFORM) {
	//        return 0;
	//    }
	//#endif
	
	for(i=0; i<=njnt; i++) {
		for(j=0; j<robotPt->joints[i]->dof_equiv_nbr; j++) {
			k = robotPt->joints[i]->index_dof + j;
			if((! p3d_jnt_get_dof_is_user(robotPt->joints[i], j)) &&
				 (robotPt->cntrt_manager->in_cntrt[k] == 1)) {
				nrd++;
			}
		}
	}
	
	return nrd;
}

/*!
 * Initializes the 
 * DofSlider Object (one per dof)
 */
void FormRobot::initSliders()
{
	//    int       i, j, k, ir, ord;
	int k;
	int /*njnt,*/ nb_dof;
	//configPt robot_pos_deg;
	p3d_rob *robotPt;
	Joint * jntPt;
	
	nb_dof =    mRobot->getRobotStruct()->nb_dof; //p3d_get_robot_ndof();
	//    ir =        ptrRob->getRobotStruct()->num; //p3d_get_desc_curnum(P3D_ROBOT);
	robotPt =   mRobot->getRobotStruct(); //(p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
	
	if(calc_real_dof() > 0)
	{
		for(unsigned int i=0; i<mRobot->getNumberOfJoints(); i++)
		{
			jntPt = mRobot->getJoint(i);
			
			for(unsigned int j=0; j<jntPt->getNumberOfDof(); j++)
			{
				k = jntPt->getIndexOfFirstDof() + j;
				
				if((p3d_jnt_get_dof_is_user(jntPt->getJointStruct(),j)) || 
					 (robotPt->cntrt_manager->in_cntrt[k] == 1))
				{
					DofSlider* oneSlider = new DofSlider(mRobot,mOpenGl,this);
					oneSlider->makeSlider( mGridLayout, jntPt, j );
					mSliders.push_back( oneSlider );
				}
			}
		}
	}
	
	QSpacerItem *verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);
	mGridLayout->addItem(verticalSpacer, k+1, 0, 1, 1);
}

void FormRobot::resetConstraintedDoFs()
{
	int k;
	int njnt, nb_dof;
	//configPt robot_pos_deg;
	p3d_rob *robotPt;
	p3d_jnt * jntPt;
	
	int numDof = 0;
	
	nb_dof =    mRobot->getRobotStruct()->nb_dof; //p3d_get_robot_ndof();
	njnt =      mRobot->getRobotStruct()->njoints; //p3d_get_robot_njnt();
	//    ir =        ptrRob->getRobotStruct()->num; //p3d_get_desc_curnum(P3D_ROBOT);
	robotPt =   mRobot->getRobotStruct(); //(p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
	
	if(calc_real_dof() > 0)
	{
		for(int i=0; i<=njnt; i++)
		{
			jntPt = robotPt->joints[i];
			
			for(int j=0; j<jntPt->dof_equiv_nbr; j++)
			{
				k = jntPt->index_dof + j;
				
				if((p3d_jnt_get_dof_is_user(jntPt,j)) || (robotPt->cntrt_manager->in_cntrt[k] == 1))
				{
					if (	robotPt->cntrt_manager->in_cntrt[k] == 2 )
					{
						mSliders[numDof]->getDoubleSpinBox()->setDisabled(true);
						mSliders[numDof]->getHorizontalSlider()->setDisabled(true);
					}
					else
					{
						mSliders[numDof]->getDoubleSpinBox()->setDisabled(false);
						mSliders[numDof]->getHorizontalSlider()->setDisabled(false);
					}
					
					numDof++;
					//cout << "in_cntrt[k] = " << robotPt->cntrt_manager->in_cntrt[k] << endl;
				}
			}
		}
	}	
}

/*!
 * Sets the slider value 
 * with an input configuration
 */
void FormRobot::setSliders(Configuration& ptrConfRad)
{
	string RobotName = ptrConfRad.getRobot()->getName();
	
	shared_ptr<Configuration> ptrConfDeg = ptrConfRad.getConfigInDegree();
	
	//    cout << "--------" << endl;
	//    cout << RobotName  << endl;
	//    cout << "--------" << endl;
	
	for(unsigned int numRob=0;numRob<mSliders.size();numRob++)
	{
		//        cout << mSliders[numRob][0]->getRobot()->getName() << mSliders.size() <<endl;
		if( mRobot->getName().compare( RobotName ) == 0 )
		{
			p3d_rob* robotPt = ptrConfDeg->getRobot()->getRobotStruct();
			int numDof = 0;
			
			if(calc_real_dof() > 0)
			{
				for(int i=0; i<=robotPt->njoints; i++)
				{
					p3d_jnt* jntPt = robotPt->joints[i];
					
					for(int j=0; j<jntPt->dof_equiv_nbr; j++)
					{
						int k = jntPt->index_dof + j;
						
						if((p3d_jnt_get_dof_is_user(jntPt,j)) || (robotPt->cntrt_manager->in_cntrt[k] == 1))
						{
							//                            cout << ptrConfDeg->getConfigStruct() << endl;
							disconnect(mSliders[numDof]->getConnector(),SIGNAL(valueChanged(double)),
												 mSliders[numDof],SLOT(dofValueChanged(double)));
							
							mSliders[numDof]->getConnector()->setValue( ptrConfDeg->at(k) );
							
							connect(mSliders[numDof]->getConnector(),SIGNAL(valueChanged(double)),
											mSliders[numDof],SLOT(dofValueChanged(double)));
							
							numDof++;
							
							//                            if(RobotName.compare("ROBOT") == 0)
							//                            {
							//                                cout << " Jidof dof[" << k <<"] = "<< ptrConfDeg->at(k) << endl;
							//                            }
							
							if (robotPt->cntrt_manager->in_cntrt[k] == 2)
							{
								//                                mSliders.back().back()->doubleSpinBox->setDisabled(true);
								//                                mSliders.back().back()->horizontalSlider->setDisabled(true);
							}
						}
					}
				}
			}
		}
	}
}

/*!
 * Sets the robot in
 *  0 - Init
 *  1 - Goto
 */
void FormRobot::setCurrentPosition(int position)
{
	shared_ptr<Configuration> ptrConf;
	
	if( position == 0 )
	{
		cout << "Robot " << mRobot->getName() << " to Initial Pos" << endl;
		
		ptrConf = mRobot->getInitialPosition();
		setSliders(*ptrConf);
		mRobot->setAndUpdate(*ptrConf);
	}
	
	if (position == 1) 
	{
		cout << "Robot " << mRobot->getName() << " to Goto Pos" << endl;
		ptrConf = mRobot->getGoTo();
		setSliders(*ptrConf);
		mRobot->setAndUpdate(*ptrConf);
	}
	
#ifdef WITH_XFORMS
	std::string str = "g3d_draw_allwin_active";
	write(qt_fl_pipe[1],str.c_str(),str.length()+1);
#else
	if(!ENV.getBool(Env::isRunning))
	{
		mOpenGl->updateGL();
	}
#endif
}

/*!
 * Saves the current Config into the robot configuration
 */
void FormRobot::saveCurrentConfigToPosition()
{	
	int index = mPositions->currentIndex();
	
	if( index == 0 )
	{
		mRobot->setInitialPosition(*mRobot->getCurrentPos());
		cout << "Save Config in Pos: " << index << endl;
	}
	
	if( index == 1 )
	{
		mRobot->setGoTo(*mRobot->getCurrentPos());
		cout << "Save Config in Pos: " << index << endl;
	}
}

//---------------------------------------------------------------------
// DofSlider
//---------------------------------------------------------------------
/*!
 * Makes a slider
 *
 * LABEL + SPINBOX + SLIDER
 */
void DofSlider::makeSlider(QGridLayout* gridLayout, Joint *jnt, int DofNumOnJnt)
{
	p3d_jnt* jntPt = jnt->getJointStruct();
	int DofNum = jntPt->index_dof + DofNumOnJnt;
	const char* name = p3d_jnt_get_dof_name(jntPt, DofNumOnJnt);
	QString dofName(name);
	
	double max;
	double min;
	
	p3d_jnt_get_dof_bounds_deg(jntPt, DofNumOnJnt, &min, &max);
	
	mDofNum = DofNum;
	
	// Label
	mLabel = new QLabel();
	mLabel->setObjectName("LabelDof");
	mLabel->setText(dofName);
	
	QFont testFont( "Times", 10, QFont::Bold );
	mLabel->setFont ( testFont );
	
	gridLayout->addWidget(mLabel, mDofNum, 0, 1, 1);
	
	// SpinBox
	mDoubleSpinBox = new QDoubleSpinBox();
	mDoubleSpinBox->setObjectName(QString::fromUtf8("doubleSpinBox"));
	mDoubleSpinBox->setDecimals(2);
	mDoubleSpinBox->setSingleStep(1e-02);
	mDoubleSpinBox->setMaximum(max);
	mDoubleSpinBox->setMinimum(min);
	
	gridLayout->addWidget(mDoubleSpinBox, mDofNum, 1, 1, 1);
	
	// Slider
	mHorizontalSlider = new QSlider();
	mHorizontalSlider->setObjectName(QString::fromUtf8("horizontalSlider"));
	mHorizontalSlider->setOrientation(Qt::Horizontal);
	mHorizontalSlider->setMaximum(10000);
	
	gridLayout->addWidget(mHorizontalSlider, mDofNum, 2, 1, 1);
	
	// Connector
	mConnector = new QtShiva::SpinBoxSliderConnector(
																									 this, mDoubleSpinBox, mHorizontalSlider );
	
	connect(mConnector,SIGNAL(valueChanged(double)),this,SLOT(dofValueChanged(double)),Qt::DirectConnection);
}

/*!
 * Call Back called when a slider is moved
 */
void DofSlider::dofValueChanged(double value)
{
	//    std::string str = "ChangeDof";
	//    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
	//    CB_position_obj(NULL,value);
	
	configPt p=NULL, p_deg=NULL;
	int i,nb_dof,ir,i_dof;
	p3d_rob *robotPt;
	p3d_jnt *jntPt;
	int ncol=0;
	int I_can;
	int nreshoot;   // <-modif Juan
	int *ikSol = NULL;
	
	double val = value; //fl_get_slider_value(ob);
	int arg = mDofNum;
	
	//    ir = p3d_get_desc_curnum(P3D_ROBOT);
	//    robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
	//    nb_dof = p3d_get_robot_ndof();
	
#if defined( CXX_PLANNER ) || defined( WITH_OOMOVE3D ) 
	nb_dof =    mRobot->getRobotStruct()->nb_dof; //p3d_get_robot_ndof();
	ir =        mRobot->getRobotStruct()->num; //p3d_get_desc_curnum(P3D_ROBOT);
	robotPt =   mRobot->getRobotStruct(); //(p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
#endif
	
	
#ifdef HRI_GENERALIZED_IK
	/*
	if ( mRobot->getName().find("HUMAN") != string::npos ) 
	{
		shared_ptr<Configuration> qSeated = mRobot->getCurrentPos();
		HRI_AGENTS* agents = hri_create_agents();
		Eigen::Vector3d head = mRobot->getJoint(5)->getVectorPos();

		hri_agent_compute_posture(agents->humans[0],head[2],1.3,qSeated->getConfigStruct());
		mRobot->setAndUpdate(*qSeated);

		hri_destroy_agents(agents);
	}
	 */
#endif
	
	p = p3d_alloc_config(robotPt);
	//    p = mRobot->getNewConfig()->getConfigStruct();
	//     p_deg = p3d_alloc_config(robotPt);
	
	//        if(fl_get_choice(ROBOTS_FORM[ir].GOTO_OBJ) == 1){
	//          p3d_convert_config_rad_to_deg(robotPt, robotPt->ROBOT_POS, &p_deg);
	//          ikSol = robotPt->ikSolPos;
	//        }
	//        else{
	//          p3d_convert_config_rad_to_deg(robotPt, robotPt->ROBOT_GOTO, &p_deg);
	//          ikSol = robotPt->ikSolGoto;
	//        }
	
	p_deg = p3d_get_robot_config_deg(robotPt);
	//    p3d_convert_config_rad_to_deg(robotPt, robotPt->ROBOT_POS, &p_deg)
	//    p_deg = mRobot->getCurrentPos()->getConfigInDegree()->getConfigStruct();
	
	p_deg[arg] = val;
	p3d_convert_config_deg_to_rad(robotPt, p_deg, &p);
	
	/*update the configuration of the current robot */
	/* sustitution pour la partie de la fonction dans l'anciene version de Move3D */
	/*   p3d_set_and_update_robot_conf(p); */ /* <- dans la fonction  remplacee */
	
	p3d_set_robot_config(robotPt, p);
	
	//   I_can = p3d_update_robot_pos();
	I_can = p3d_update_this_robot_pos_multisol(robotPt, NULL, 0, ikSol);
	
	//cout << "I_can = "  << I_can << endl;
	
	if (robotPt->cntrt_manager->cntrts != NULL)
	{
		// modif Juan
		nreshoot = 0;
		if(!I_can && p3d_get_RLG())
		{
			if(robotPt->cntrt_manager->in_cntrt[arg] == 1)
			{
				while(!I_can && (nreshoot < 100))
				{
					if(p3d_random_loop_generator_without_parallel(robotPt, p))
					{
						I_can = p3d_update_robot_pos();
					}
					nreshoot++;
				}
			}
		}
		//         fmodif Juan
		if(I_can)
		{
			p3d_get_robot_config_into(robotPt, &p);
			p3d_get_robot_config_deg_into(robotPt, &p_deg);
			for(i=0; i<nb_dof; i++)
			{
				//                if (ROBOTS_FORM[ir].POSITION_OBJ[i] != NULL)
				//                {
				//                    fl_set_slider_value(ROBOTS_FORM[ir].POSITION_OBJ[i], p_deg[i]);
				//                }
			}
			
			//print_config(robotPt,p);
			//            p3d_copy_config_into(robotPt, p_deg, &last_p_deg[ir]);
		}
		else
		{
			//            p3d_copy_config_into(robotPt, last_p_deg[ir], &p_deg);
			p3d_convert_config_deg_to_rad(robotPt, p_deg, &p);
			for(i=0; i<nb_dof; i++)
			{
				//                if (ROBOTS_FORM[ir].POSITION_OBJ[i] != NULL)
				//                {
				//                    fl_set_slider_value(ROBOTS_FORM[ir].POSITION_OBJ[i], p_deg[i]);
				//                }
			}
			jntPt = p3d_robot_dof_to_jnt(robotPt, arg, &i_dof);
			p3d_jnt_set_dof_deg(jntPt, i_dof, p_deg[arg]);  /* ceneccasy lines for some cases of cntrts !!! */
			p3d_update_this_robot_pos_without_cntrt(robotPt);
		}
	}
	
	if (ENV.getBool(Env::isCostSpace))
	{
#ifdef P3D_PLANNER
		p3d_rob* costRobot = robotPt;
		configPt cost_q = p;
#ifdef HRI_COSTSPACE
		// Compute kinematic the object transfer point
		if ( ENV.getBool(Env::HRIComputeOTP) )
		{
			Eigen::Vector3d WSPoint;
			
			if( dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->computeBestFeasableTransferPoint(WSPoint) )
			{
				Robot* Object = global_Project->getActiveScene()->getRobotByNameContaining("OBJECT");
				
				shared_ptr<Configuration> q_curr = Object->getCurrentPos();
				
				(*q_curr)[6] = WSPoint[0];
				(*q_curr)[7] = WSPoint[1];
				(*q_curr)[8] = WSPoint[2];
				
				Object->setAndUpdate(*q_curr);
				
				cout << "Set and update : " << Object->getName() << endl << WSPoint << endl;
				
				p3d_col_deactivate_rob_rob(Object->getRobotStruct(), 
																	 dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getHuman()->getRobotStruct());
			}
		}
		
		// Change the cost robot
		if ( ENV.getBool(Env::enableHri) && (!ENV.getBool(Env::HRINoRobot))) 
		{
			std::string robotName(costRobot->name);
			
			// If the Robot moved is not ROBOT
			if( robotName.find("ROBOT") == string::npos && (global_Project->getActiveScene()->getNumberOfRobots() > 1)  ) // Does not contain Robot
			{
				costRobot = p3d_get_robot_by_name_containing("ROBOT");
				cost_q = p3d_get_robot_config(costRobot);
				//cout << "Change the robot position = " << robotPt->name << endl;
			}
			
			// Compute kinematic transfer point
			if ( ENV.getBool(Env::HRIcameraBehindHuman) )
			{
				//cout << "choseBestTransferPoint" << endl;
				if( HRICS_MotionPL != NULL )
				{
					Eigen::Vector3d WSPoint;
					
					if( dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->chooseBestTransferPoint(WSPoint) )
					{
						HRICS::Natural* reachSpace = HRICS_MotionPL->getReachability();
						reachSpace->computeIsReachable(WSPoint,reachSpace->getGrid()->isReachableWithLA(WSPoint)); 
					}
				}
			}
		}
#endif
		if (!ENV.getBool(Env::HRINoRobot)) 
		{
			Robot* costR(global_Project->getActiveScene()->getRobotByName(costRobot->name));
			Configuration costConfig(costR,cost_q);
			//std::cout << "Cost = " << p3d_GetConfigCost(costRobot,cost_q) << std::endl;
			std::cout << "Cost = " << global_costSpace->cost(costConfig) << std::endl;
		}
#endif
	}
	//	cout << "robotPt->name = " << robotPt->name << endl;
	//	cout << "XYZ_ROBOT = " << XYZ_ROBOT->name << endl;
	
#ifdef P3D_COLLISION_CHECKING
	ncol = mRobot->isInCollision();
	//cout << "Collision = " << ncol << endl;
#endif
	
	g3d_set_draw_coll(ncol);
	
	/* update the field current position or goal position of the
	 current robot depending on field GOTO_OBJ */
	//    if(fl_get_choice(ROBOTS_FORM[ir].GOTO_OBJ) == 1){
	//        p3d_copy_config_into(robotPt, p, &(robotPt->ROBOT_POS));
	//    }
	//    else{
	//        p3d_copy_config_into(robotPt, p, &(robotPt->ROBOT_GOTO));
	//    }
	
	/* update trajectory if track_traj mode is activated */
	//    if (ROBOTS_FORM[ir].TRACK_TRAJ)
	//    {
	//        CB_stop_obj(ob,0);
	//        CB_stop_optim_obj(ob,0);
	//        if (!p3d_trackCurrentTraj(robotPt,3, 0.001, p3d_get_d0(),p3d_get_QUICK_DESCENT()))
	//        {
	//            CB_specific_search_obj(ob,0);
	//        }
	//        p3d_set_and_update_robot_conf(p);
	//    }
	
	p3d_destroy_config(robotPt, p);
	p3d_destroy_config(robotPt, p_deg);
	
	
#ifdef WITH_XFORMS
	std::string str = "g3d_draw_allwin_active";
	write(qt_fl_pipe[1],str.c_str(),str.length()+1);
#else
	if(!ENV.getBool(Env::isRunning))
	{
		mOpenGl->updateGL();
	}
#endif
	
	// Sets the actual configuration to all sliders
	mFormRobot->setSliders(*mRobot->getCurrentPos());
}
