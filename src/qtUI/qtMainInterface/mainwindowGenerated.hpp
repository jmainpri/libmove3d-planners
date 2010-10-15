/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Wed Sep 22 16:13:30 2010
**      by: Qt User Interface Compiler version 4.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QScrollArea>
#include <QtGui/QSlider>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QSplitter>
#include <QtGui/QStatusBar>
#include <QtGui/QTabWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

#if defined( CXX_PLANNER )

#include "glwidget.hpp"

#include "qtCost.hpp"
#include "qtMotionPlanner.hpp"
#include "qtRobot.hpp"
#include "qtUtil.hpp"

#endif

#if defined( MULTILOCALPATH )
#include "qtMultiLocalPath.hpp"
#endif

#if defined( WITH_OOMOVE3D )

#include "qtUI/qtOpenGL/glwidget.hpp"

#include "qtUI/qtMainInterface/sideWidgets/qtCost.hpp"
#include "qtUI/qtMainInterface/sideWidgets/qtMotionPlanner.hpp"
#include "qtUI/qtMainInterface/sideWidgets/qtRobot.hpp"
#include "qtUI/qtMainInterface/sideWidgets/qtUtil.hpp"

#endif

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
		// Actions
    QAction *actionQuit;
    QAction *action3DViewer;
    QAction *actionKCDPropietes;
    QAction *actionAbout;
    QAction *actionCloseEnvironement;
    QAction *actionOpenScenario;
		QAction	*actionRobotForm;
		QAction	*actionLocalPathGroups;
    QAction *actionOpen;
    QAction *actionSaveScenario;
    QAction *actionNameOfEnv;
    QAction *actionSaveGraph;
    QAction *actionLoadGraph;
    QAction *actionSaveGraph_2;
    QAction *actionLoadTrajectory;
    QAction *actionSaveTrajectory;
    QAction *actionAllDevices;
    QAction *actionSave_Graph_To_Dot;
    QAction *actionSaveToDot;
	
		// Widgets
    QWidget *centralWidget;
		MoveRobot *formRobot;
#if defined( MULTILOCALPATH )
		MultiLocalPathWidget* localPathGroups;
#endif
    QVBoxLayout *verticalLayout_21;
    QSplitter *vSplitter;
    QWidget *sidePanel;
    QVBoxLayout *verticalLayout_6;
    QTabWidget *mainTabWidget;
    MotionPlanner *tabMotionPlanner;
    QVBoxLayout *verticalLayout_19;
    CostWidget *tabCost;
    QVBoxLayout *verticalLayout_11;
    RobotWidget *tabRobot;
    QVBoxLayout *verticalLayout_12;
    UtilWidget *tabUtil;
    QVBoxLayout *verticalLayout_17;
    QWidget *tabViewerSettings;
    QHBoxLayout *horizontalLayout;
    QScrollArea *scrollAreaViewer;
    QWidget *scrollAreaWidgetContents;
    QHBoxLayout *horizontalLayout_2;
    QWidget *widgetViewer;
    QVBoxLayout *verticalLayout_3;
    QGroupBox *groupBoxDrawEnv;
    QGridLayout *gridLayout_12;
    QCheckBox *checkBoxDrawGraph;
    QCheckBox *checkBoxDrawTraj;
    QCheckBox *checkBoxDisableDraw;
    QCheckBox *checkBoxDrawDebug;
    QCheckBox *checkBoxDrawTrajVector;
    QSpinBox *spinBoxJointToDraw;
    QWidget *widgetMobCamButton;
    QHBoxLayout *horizontalLayout_5;
    QPushButton *pushButtonMobileCamera;
    QSpacerItem *horizontalSpacer;
    QGroupBox *threeDGroupBox;
    QGridLayout *gridLayout_13;
    QCheckBox *checkBoxBB;
    QCheckBox *checkBoxGhosts;
    QCheckBox *checkBoxJoints;
    QCheckBox *checkBoxEnableLight;
    QGroupBox *sceneGroupBox;
    QGridLayout *gridLayout_11;
    QCheckBox *checkBoxFloor;
    QCheckBox *checkBoxTiles;
    QCheckBox *checkBoxShadows;
    QCheckBox *checkBoxAxis;
    QCheckBox *checkBoxSmooth;
    QCheckBox *checkBoxFilaire;
    QCheckBox *checkBoxWalls;
    QCheckBox *checkBoxContour;
    QGroupBox *lightGroupBox;
    QGridLayout *gridLayout_21;
    QWidget *widgetLightSliders;
    QGridLayout *gridLayout_43;
    QDoubleSpinBox *doubleSpinBoxLightX;
    QSlider *horizontalSliderLightX;
    QDoubleSpinBox *doubleSpinBoxLightY;
    QLabel *label_X;
    QLabel *label_Y;
    QSlider *horizontalSliderLightY;
    QDoubleSpinBox *doubleSpinBoxLightZ;
    QLabel *label_Z;
    QSlider *horizontalSliderLightZ;
    QWidget *widgetLightButtons;
    QVBoxLayout *verticalLayout_22;
    QPushButton *pushButtonRestoreLight;
    QCheckBox *checkBoxDrawLightSource;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_41;
    QPushButton *pushButtonAddTraj;
    QPushButton *pushButtonClearTraj;
    QSpacerItem *horizontalSpacerViewer;
    QComboBox *comboBoxColorTraj;
    QPushButton *pushButtonRestoreView;
    QSpacerItem *verticalSpacer_4;
    QWidget *verticalLayout;
    QVBoxLayout *verticalLayout_2;
    QSplitter *hSplitter;
    GLWidget *OpenGL;
    QWidget *controlWidget;
    QHBoxLayout *horizontalLayout_3;
    QGroupBox *groupBoxRunButtons;
    QGridLayout *gridLayout_30;
    QWidget *widget_4;
    QPushButton *pushButtonReset;
    QPushButton *pushButtonResetGraph;
    QPushButton *pushButtonRun;
    QPushButton *pushButtonStop;
    QLabel *labelRunning;
    QHBoxLayout *horizontalLayoutRunStrategy;
    QRadioButton *radioButtonDiff;
    QRadioButton *radioButtonPRM;
    QCheckBox *checkBoxWithSmoothing;
    QCheckBox *checkBoxUseP3DStructures;
    QGroupBox *groupBoxTrajectory;
    QVBoxLayout *verticalLayout_34;
    QHBoxLayout *horizontalLayoutShowTraj;
    QSpacerItem *horizontalSpacerShowTraj1;
    QPushButton *pushButtonShowTraj;
    QSpacerItem *horizontalSpacerShowTraj2;
    QHBoxLayout *horizontalLayoutTrajSpeed;
    QLabel *label;
    QDoubleSpinBox *doubleSpinBoxTrajSpeed;
    QSlider *horizontalSliderTrajSpeed;
    QGroupBox *groupBoxTests;
    QVBoxLayout *verticalLayout_4;
    QPushButton *pushButtonTest1;
    QPushButton *pushButtonTest2;
    QPushButton *pushButtonTest3;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuHelp;
    QMenu *menuCollisionCheker;
    QMenu *menuGraph;
    QMenu *menuEnvironement;
    QMenu *menuRobot;
    QStatusBar *statusBar;
	

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
			
        MainWindow->setWindowModality(Qt::NonModal);
        MainWindow->setEnabled(true);
        MainWindow->resize(1440, 700);
			
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
			
        MainWindow->setSizePolicy(sizePolicy);
        MainWindow->setSizeIncrement(QSize(0, 0));
        MainWindow->setTabShape(QTabWidget::Rounded);
			
			
				//----------------------------------------------------------------------
				// Upper window
				//----------------------------------------------------------------------
				actionQuit = new QAction(MainWindow);
        actionQuit->setObjectName(QString::fromUtf8("actionQuit"));
			
        action3DViewer = new QAction(MainWindow);
        action3DViewer->setObjectName(QString::fromUtf8("action3DViewer"));
			
        actionKCDPropietes = new QAction(MainWindow);
        actionKCDPropietes->setObjectName(QString::fromUtf8("actionKCDPropietes"));
			
        actionAbout = new QAction(MainWindow);
        actionAbout->setObjectName(QString::fromUtf8("actionAbout"));
			
        actionCloseEnvironement = new QAction(MainWindow);
        actionCloseEnvironement->setObjectName(QString::fromUtf8("actionCloseEnvironement"));
			
        actionOpenScenario = new QAction(MainWindow);
        actionOpenScenario->setObjectName(QString::fromUtf8("actionOpenScenario"));
			
				actionRobotForm = new QAction(MainWindow);
				actionRobotForm->setObjectName(QString::fromUtf8("actionRobotForm"));
			
				actionLocalPathGroups = new QAction(MainWindow);
				actionLocalPathGroups->setObjectName(QString::fromUtf8("actionLocalPathGroups"));
			
        actionOpen = new QAction(MainWindow);
        actionOpen->setObjectName(QString::fromUtf8("actionOpen"));
			
        actionSaveScenario = new QAction(MainWindow);
        actionSaveScenario->setObjectName(QString::fromUtf8("actionSaveScenario"));
			
        actionNameOfEnv = new QAction(MainWindow);
        actionNameOfEnv->setObjectName(QString::fromUtf8("actionNameOfEnv"));
			
        actionSaveGraph = new QAction(MainWindow);
        actionSaveGraph->setObjectName(QString::fromUtf8("actionSaveGraph"));
			
        actionLoadGraph = new QAction(MainWindow);
        actionLoadGraph->setObjectName(QString::fromUtf8("actionLoadGraph"));
			
        actionSaveGraph_2 = new QAction(MainWindow);
        actionSaveGraph_2->setObjectName(QString::fromUtf8("actionSaveGraph_2"));
			
        actionLoadTrajectory = new QAction(MainWindow);
        actionLoadTrajectory->setObjectName(QString::fromUtf8("actionLoadTrajectory"));
			
        actionSaveTrajectory = new QAction(MainWindow);
        actionSaveTrajectory->setObjectName(QString::fromUtf8("actionSaveTrajectory"));
			
        actionAllDevices = new QAction(MainWindow);
        actionAllDevices->setObjectName(QString::fromUtf8("actionAllDevices"));
			
        actionSave_Graph_To_Dot = new QAction(MainWindow);
        actionSave_Graph_To_Dot->setObjectName(QString::fromUtf8("actionSave_Graph_To_Dot"));
			
        actionSaveToDot = new QAction(MainWindow);
        actionSaveToDot->setObjectName(QString::fromUtf8("actionSaveToDot"));
			
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
			
        sizePolicy.setHeightForWidth(centralWidget->sizePolicy().hasHeightForWidth());
			
        centralWidget->setSizePolicy(sizePolicy);
			
        verticalLayout_21 = new QVBoxLayout(centralWidget);
        verticalLayout_21->setSpacing(6);
        verticalLayout_21->setContentsMargins(11, 11, 11, 11);
        verticalLayout_21->setObjectName(QString::fromUtf8("verticalLayout_21"));
        verticalLayout_21->setContentsMargins(-1, 0, -1, 2);
			
				//----------------------------------------------------------------------
				// Main splitter
			 //----------------------------------------------------------------------
        vSplitter = new QSplitter(centralWidget);
        vSplitter->setObjectName(QString::fromUtf8("vSplitter"));
        vSplitter->setOrientation(Qt::Horizontal);
			
				//----------------------------------------------------------------------
				// Side Window
				//----------------------------------------------------------------------
        sidePanel = new QWidget(vSplitter);
        sidePanel->setObjectName(QString::fromUtf8("sidePanel"));
			
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(2);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(sidePanel->sizePolicy().hasHeightForWidth());
        sidePanel->setSizePolicy(sizePolicy1);
			
        verticalLayout_6 = new QVBoxLayout(sidePanel);
        verticalLayout_6->setSpacing(6);
        verticalLayout_6->setContentsMargins(11, 11, 11, 11);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        verticalLayout_6->setContentsMargins(-1, 0, -1, 2);
			
        mainTabWidget = new QTabWidget(sidePanel);
        mainTabWidget->setObjectName(QString::fromUtf8("mainTabWidget"));
        mainTabWidget->setDocumentMode(true);
			
				// Main widget
        tabMotionPlanner = new MotionPlanner();
        tabMotionPlanner->setObjectName(QString::fromUtf8("tabMotionPlanner"));
				
        verticalLayout_19 = new QVBoxLayout(tabMotionPlanner);
        verticalLayout_19->setSpacing(6);
        verticalLayout_19->setContentsMargins(11, 11, 11, 11);
        verticalLayout_19->setObjectName(QString::fromUtf8("verticalLayout_19"));
        verticalLayout_19->setContentsMargins(0, 5, 0, 5);
			
        mainTabWidget->addTab(tabMotionPlanner, QString());
			
        tabCost = new CostWidget();
        tabCost->setObjectName(QString::fromUtf8("tabCost"));
			
        verticalLayout_11 = new QVBoxLayout(tabCost);
        verticalLayout_11->setSpacing(6);
        verticalLayout_11->setContentsMargins(11, 11, 11, 11);
        verticalLayout_11->setObjectName(QString::fromUtf8("verticalLayout_11"));
        verticalLayout_11->setContentsMargins(0, 5, 0, 5);
			
        mainTabWidget->addTab(tabCost, QString());
			
				// Tab robot
        tabRobot = new RobotWidget();
        tabRobot->setObjectName(QString::fromUtf8("tabRobot"));
			
				// Form robot
				formRobot = new MoveRobot();
				formRobot->setObjectName(QString::fromUtf8("formRobot"));
				formRobot->setMaximumSize(QSize(16777215, 16777215));
			
#if defined( MULTILOCALPATH )
				// Form robot
				localPathGroups = new MultiLocalPathWidget();
				localPathGroups->setObjectName(QString::fromUtf8("MultiLocalPathWidget"));	
#endif
			
        verticalLayout_12 = new QVBoxLayout(tabRobot);
        verticalLayout_12->setSpacing(6);
        verticalLayout_12->setContentsMargins(11, 11, 11, 11);
        verticalLayout_12->setObjectName(QString::fromUtf8("verticalLayout_12"));
        verticalLayout_12->setContentsMargins(0, 5, 0, 5);
			
        mainTabWidget->addTab(tabRobot, QString());
			
        tabUtil = new UtilWidget();
        tabUtil->setObjectName(QString::fromUtf8("tabUtil"));
        tabUtil->setFocusPolicy(Qt::NoFocus);
			
        verticalLayout_17 = new QVBoxLayout(tabUtil);
        verticalLayout_17->setSpacing(6);
        verticalLayout_17->setContentsMargins(11, 11, 11, 11);
        verticalLayout_17->setObjectName(QString::fromUtf8("verticalLayout_17"));
        verticalLayout_17->setContentsMargins(0, 5, 0, 5);
			
        mainTabWidget->addTab(tabUtil, QString());
			
        tabViewerSettings = new QWidget();
        tabViewerSettings->setObjectName(QString::fromUtf8("tabViewerSettings"));
			
			
				//----------------------------------------------------------------------
				// Horizontal and buttons
				//----------------------------------------------------------------------
        horizontalLayout = new QHBoxLayout(tabViewerSettings);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 5, 0, 5);
			
        scrollAreaViewer = new QScrollArea(tabViewerSettings);
        scrollAreaViewer->setObjectName(QString::fromUtf8("scrollAreaViewer"));
        scrollAreaViewer->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        scrollAreaViewer->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QString::fromUtf8("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, -169, 357, 785));
			
        horizontalLayout_2 = new QHBoxLayout(scrollAreaWidgetContents);
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
			
			
				//----------------------------------------------------------------------
				// Viewer Widget
				//----------------------------------------------------------------------
        widgetViewer = new QWidget(scrollAreaWidgetContents);
        widgetViewer->setObjectName(QString::fromUtf8("widgetViewer"));
			
        verticalLayout_3 = new QVBoxLayout(widgetViewer);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
			
        groupBoxDrawEnv = new QGroupBox(widgetViewer);
        groupBoxDrawEnv->setObjectName(QString::fromUtf8("groupBoxDrawEnv"));
			
        gridLayout_12 = new QGridLayout(groupBoxDrawEnv);
        gridLayout_12->setSpacing(6);
        gridLayout_12->setContentsMargins(11, 11, 11, 11);
        gridLayout_12->setObjectName(QString::fromUtf8("gridLayout_12"));
			
        checkBoxDrawGraph = new QCheckBox(groupBoxDrawEnv);
        checkBoxDrawGraph->setObjectName(QString::fromUtf8("checkBoxDrawGraph"));
        checkBoxDrawGraph->setChecked(false);

        gridLayout_12->addWidget(checkBoxDrawGraph, 1, 0, 1, 1);

        checkBoxDrawTraj = new QCheckBox(groupBoxDrawEnv);
        checkBoxDrawTraj->setObjectName(QString::fromUtf8("checkBoxDrawTraj"));

        gridLayout_12->addWidget(checkBoxDrawTraj, 2, 0, 1, 1);

        checkBoxDisableDraw = new QCheckBox(groupBoxDrawEnv);
        checkBoxDisableDraw->setObjectName(QString::fromUtf8("checkBoxDisableDraw"));

        gridLayout_12->addWidget(checkBoxDisableDraw, 0, 0, 1, 1);

        checkBoxDrawDebug = new QCheckBox(groupBoxDrawEnv);
        checkBoxDrawDebug->setObjectName(QString::fromUtf8("checkBoxDrawDebug"));

        gridLayout_12->addWidget(checkBoxDrawDebug, 4, 0, 1, 1);

        checkBoxDrawTrajVector = new QCheckBox(groupBoxDrawEnv);
        checkBoxDrawTrajVector->setObjectName(QString::fromUtf8("checkBoxDrawTrajVector"));

        gridLayout_12->addWidget(checkBoxDrawTrajVector, 3, 0, 1, 1);

        spinBoxJointToDraw = new QSpinBox(groupBoxDrawEnv);
        spinBoxJointToDraw->setObjectName(QString::fromUtf8("spinBoxJointToDraw"));
        spinBoxJointToDraw->setMinimum(-1);
        spinBoxJointToDraw->setMaximum(1000);

        gridLayout_12->addWidget(spinBoxJointToDraw, 1, 1, 1, 1);


        verticalLayout_3->addWidget(groupBoxDrawEnv);

        widgetMobCamButton = new QWidget(widgetViewer);
        widgetMobCamButton->setObjectName(QString::fromUtf8("widgetMobCamButton"));
			
        horizontalLayout_5 = new QHBoxLayout(widgetMobCamButton);
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
			
        pushButtonMobileCamera = new QPushButton(widgetMobCamButton);
        pushButtonMobileCamera->setObjectName(QString::fromUtf8("pushButtonMobileCamera"));

        horizontalLayout_5->addWidget(pushButtonMobileCamera);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer);


        verticalLayout_3->addWidget(widgetMobCamButton);

        threeDGroupBox = new QGroupBox(widgetViewer);
        threeDGroupBox->setObjectName(QString::fromUtf8("threeDGroupBox"));
			
        gridLayout_13 = new QGridLayout(threeDGroupBox);
        gridLayout_13->setSpacing(6);
        gridLayout_13->setContentsMargins(11, 11, 11, 11);
        gridLayout_13->setObjectName(QString::fromUtf8("gridLayout_13"));
			
        checkBoxBB = new QCheckBox(threeDGroupBox);
        checkBoxBB->setObjectName(QString::fromUtf8("checkBoxBB"));

        gridLayout_13->addWidget(checkBoxBB, 0, 0, 1, 1);

        checkBoxGhosts = new QCheckBox(threeDGroupBox);
        checkBoxGhosts->setObjectName(QString::fromUtf8("checkBoxGhosts"));

        gridLayout_13->addWidget(checkBoxGhosts, 1, 0, 1, 1);

        checkBoxJoints = new QCheckBox(threeDGroupBox);
        checkBoxJoints->setObjectName(QString::fromUtf8("checkBoxJoints"));

        gridLayout_13->addWidget(checkBoxJoints, 0, 1, 1, 1);

        checkBoxEnableLight = new QCheckBox(threeDGroupBox);
        checkBoxEnableLight->setObjectName(QString::fromUtf8("checkBoxEnableLight"));

        gridLayout_13->addWidget(checkBoxEnableLight, 1, 1, 1, 1);


        verticalLayout_3->addWidget(threeDGroupBox);

        sceneGroupBox = new QGroupBox(widgetViewer);
        sceneGroupBox->setObjectName(QString::fromUtf8("sceneGroupBox"));
			
        gridLayout_11 = new QGridLayout(sceneGroupBox);
        gridLayout_11->setSpacing(6);
        gridLayout_11->setContentsMargins(11, 11, 11, 11);
        gridLayout_11->setObjectName(QString::fromUtf8("gridLayout_11"));
			
        checkBoxFloor = new QCheckBox(sceneGroupBox);
        checkBoxFloor->setObjectName(QString::fromUtf8("checkBoxFloor"));

        gridLayout_11->addWidget(checkBoxFloor, 1, 0, 1, 1);

        checkBoxTiles = new QCheckBox(sceneGroupBox);
        checkBoxTiles->setObjectName(QString::fromUtf8("checkBoxTiles"));

        gridLayout_11->addWidget(checkBoxTiles, 4, 0, 1, 1);

        checkBoxShadows = new QCheckBox(sceneGroupBox);
        checkBoxShadows->setObjectName(QString::fromUtf8("checkBoxShadows"));

        gridLayout_11->addWidget(checkBoxShadows, 4, 1, 1, 1);

        checkBoxAxis = new QCheckBox(sceneGroupBox);
        checkBoxAxis->setObjectName(QString::fromUtf8("checkBoxAxis"));

        gridLayout_11->addWidget(checkBoxAxis, 6, 1, 1, 1);

        checkBoxSmooth = new QCheckBox(sceneGroupBox);
        checkBoxSmooth->setObjectName(QString::fromUtf8("checkBoxSmooth"));

        gridLayout_11->addWidget(checkBoxSmooth, 1, 1, 1, 1);

        checkBoxFilaire = new QCheckBox(sceneGroupBox);
        checkBoxFilaire->setObjectName(QString::fromUtf8("checkBoxFilaire"));

        gridLayout_11->addWidget(checkBoxFilaire, 5, 1, 1, 1);

        checkBoxWalls = new QCheckBox(sceneGroupBox);
        checkBoxWalls->setObjectName(QString::fromUtf8("checkBoxWalls"));

        gridLayout_11->addWidget(checkBoxWalls, 5, 0, 1, 1);

        checkBoxContour = new QCheckBox(sceneGroupBox);
        checkBoxContour->setObjectName(QString::fromUtf8("checkBoxContour"));

        gridLayout_11->addWidget(checkBoxContour, 6, 0, 1, 1);


        verticalLayout_3->addWidget(sceneGroupBox);

        lightGroupBox = new QGroupBox(widgetViewer);
        lightGroupBox->setObjectName(QString::fromUtf8("lightGroupBox"));
        gridLayout_21 = new QGridLayout(lightGroupBox);
        gridLayout_21->setSpacing(6);
        gridLayout_21->setContentsMargins(11, 11, 11, 11);
        gridLayout_21->setObjectName(QString::fromUtf8("gridLayout_21"));
        widgetLightSliders = new QWidget(lightGroupBox);
        widgetLightSliders->setObjectName(QString::fromUtf8("widgetLightSliders"));
        gridLayout_43 = new QGridLayout(widgetLightSliders);
        gridLayout_43->setSpacing(6);
        gridLayout_43->setContentsMargins(11, 11, 11, 11);
        gridLayout_43->setObjectName(QString::fromUtf8("gridLayout_43"));
        doubleSpinBoxLightX = new QDoubleSpinBox(widgetLightSliders);
        doubleSpinBoxLightX->setObjectName(QString::fromUtf8("doubleSpinBoxLightX"));

        gridLayout_43->addWidget(doubleSpinBoxLightX, 0, 0, 1, 1);

        horizontalSliderLightX = new QSlider(widgetLightSliders);
        horizontalSliderLightX->setObjectName(QString::fromUtf8("horizontalSliderLightX"));
        horizontalSliderLightX->setMaximum(1000);
        horizontalSliderLightX->setOrientation(Qt::Horizontal);

        gridLayout_43->addWidget(horizontalSliderLightX, 0, 2, 1, 1);

        doubleSpinBoxLightY = new QDoubleSpinBox(widgetLightSliders);
        doubleSpinBoxLightY->setObjectName(QString::fromUtf8("doubleSpinBoxLightY"));

        gridLayout_43->addWidget(doubleSpinBoxLightY, 1, 0, 1, 1);

				//----------------------------------------------------------------------
				// Light Sliders
				//----------------------------------------------------------------------
        label_X = new QLabel(widgetLightSliders);
        label_X->setObjectName(QString::fromUtf8("label_X"));

        gridLayout_43->addWidget(label_X, 0, 1, 1, 1);

        label_Y = new QLabel(widgetLightSliders);
        label_Y->setObjectName(QString::fromUtf8("label_Y"));

        gridLayout_43->addWidget(label_Y, 1, 1, 1, 1);
			
				label_Z = new QLabel(widgetLightSliders);
				label_Z->setObjectName(QString::fromUtf8("label_Z"));
			
				gridLayout_43->addWidget(label_Z, 2, 1, 1, 1);

        horizontalSliderLightY = new QSlider(widgetLightSliders);
        horizontalSliderLightY->setObjectName(QString::fromUtf8("horizontalSliderLightY"));
        horizontalSliderLightY->setMaximum(1000);
        horizontalSliderLightY->setOrientation(Qt::Horizontal);

        gridLayout_43->addWidget(horizontalSliderLightY, 1, 2, 1, 1);

        doubleSpinBoxLightZ = new QDoubleSpinBox(widgetLightSliders);
        doubleSpinBoxLightZ->setObjectName(QString::fromUtf8("doubleSpinBoxLightZ"));

        gridLayout_43->addWidget(doubleSpinBoxLightZ, 2, 0, 1, 1);

        horizontalSliderLightZ = new QSlider(widgetLightSliders);
        horizontalSliderLightZ->setObjectName(QString::fromUtf8("horizontalSliderLightZ"));
        horizontalSliderLightZ->setMaximum(1000);
        horizontalSliderLightZ->setOrientation(Qt::Horizontal);

        gridLayout_43->addWidget(horizontalSliderLightZ, 2, 2, 1, 1);


        gridLayout_21->addWidget(widgetLightSliders, 0, 0, 1, 1);

        widgetLightButtons = new QWidget(lightGroupBox);
        widgetLightButtons->setObjectName(QString::fromUtf8("widgetLightButtons"));
        verticalLayout_22 = new QVBoxLayout(widgetLightButtons);
        verticalLayout_22->setSpacing(6);
        verticalLayout_22->setContentsMargins(11, 11, 11, 11);
        verticalLayout_22->setObjectName(QString::fromUtf8("verticalLayout_22"));
        pushButtonRestoreLight = new QPushButton(widgetLightButtons);
        pushButtonRestoreLight->setObjectName(QString::fromUtf8("pushButtonRestoreLight"));

        verticalLayout_22->addWidget(pushButtonRestoreLight);

        checkBoxDrawLightSource = new QCheckBox(widgetLightButtons);
        checkBoxDrawLightSource->setObjectName(QString::fromUtf8("checkBoxDrawLightSource"));

        verticalLayout_22->addWidget(checkBoxDrawLightSource);


        gridLayout_21->addWidget(widgetLightButtons, 0, 1, 1, 1);


        verticalLayout_3->addWidget(lightGroupBox);

        groupBox = new QGroupBox(widgetViewer);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        gridLayout_41 = new QGridLayout(groupBox);
        gridLayout_41->setSpacing(6);
        gridLayout_41->setContentsMargins(11, 11, 11, 11);
        gridLayout_41->setObjectName(QString::fromUtf8("gridLayout_41"));
        pushButtonAddTraj = new QPushButton(groupBox);
        pushButtonAddTraj->setObjectName(QString::fromUtf8("pushButtonAddTraj"));

        gridLayout_41->addWidget(pushButtonAddTraj, 0, 0, 1, 1);

        pushButtonClearTraj = new QPushButton(groupBox);
        pushButtonClearTraj->setObjectName(QString::fromUtf8("pushButtonClearTraj"));

        gridLayout_41->addWidget(pushButtonClearTraj, 1, 0, 1, 1);

        horizontalSpacerViewer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_41->addItem(horizontalSpacerViewer, 1, 1, 1, 1);

        comboBoxColorTraj = new QComboBox(groupBox);
        comboBoxColorTraj->setObjectName(QString::fromUtf8("comboBoxColorTraj"));

        gridLayout_41->addWidget(comboBoxColorTraj, 0, 1, 1, 1);


        verticalLayout_3->addWidget(groupBox);

        pushButtonRestoreView = new QPushButton(widgetViewer);
        pushButtonRestoreView->setObjectName(QString::fromUtf8("pushButtonRestoreView"));

        verticalLayout_3->addWidget(pushButtonRestoreView);

        verticalSpacer_4 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer_4);


        horizontalLayout_2->addWidget(widgetViewer);

        scrollAreaViewer->setWidget(scrollAreaWidgetContents);

        horizontalLayout->addWidget(scrollAreaViewer);

        mainTabWidget->addTab(tabViewerSettings, QString());

        verticalLayout_6->addWidget(mainTabWidget);

			
				//----------------------------------------------------------------------
				// Layouts
				//----------------------------------------------------------------------
        vSplitter->addWidget(sidePanel);
        verticalLayout = new QWidget(vSplitter);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        QSizePolicy sizePolicy2(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(3);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(verticalLayout->sizePolicy().hasHeightForWidth());
        verticalLayout->setSizePolicy(sizePolicy2);
        verticalLayout_2 = new QVBoxLayout(verticalLayout);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(-1, -1, -1, 2);
        hSplitter = new QSplitter(verticalLayout);
        hSplitter->setObjectName(QString::fromUtf8("hSplitter"));
        hSplitter->setOrientation(Qt::Vertical);
			
				//----------------------------------------------------------------------
				// OpenGl Widget
				//----------------------------------------------------------------------
        OpenGL = new GLWidget(hSplitter);
        OpenGL->setObjectName(QString::fromUtf8("OpenGL"));
			
        QSizePolicy sizePolicy3(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy3.setHorizontalStretch(2);
        sizePolicy3.setVerticalStretch(2);
        sizePolicy3.setHeightForWidth(OpenGL->sizePolicy().hasHeightForWidth());
			
        OpenGL->setSizePolicy(sizePolicy3);
        OpenGL->setMinimumSize(QSize(400, 300));
			
        hSplitter->addWidget(OpenGL);
			
        controlWidget = new QWidget(hSplitter);
        controlWidget->setObjectName(QString::fromUtf8("controlWidget"));
        sizePolicy.setHeightForWidth(controlWidget->sizePolicy().hasHeightForWidth());
        controlWidget->setSizePolicy(sizePolicy);
			
        horizontalLayout_3 = new QHBoxLayout(controlWidget);
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(-1, -1, -1, 2);
			
        groupBoxRunButtons = new QGroupBox(controlWidget);
        groupBoxRunButtons->setObjectName(QString::fromUtf8("groupBoxRunButtons"));
			
        gridLayout_30 = new QGridLayout(groupBoxRunButtons);
        gridLayout_30->setSpacing(6);
        gridLayout_30->setContentsMargins(11, 11, 11, 11);
        gridLayout_30->setObjectName(QString::fromUtf8("gridLayout_30"));
			
        widget_4 = new QWidget(groupBoxRunButtons);
        widget_4->setObjectName(QString::fromUtf8("widget_4"));
        QSizePolicy sizePolicy4(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy4.setHorizontalStretch(0);
        sizePolicy4.setVerticalStretch(0);
        sizePolicy4.setHeightForWidth(widget_4->sizePolicy().hasHeightForWidth());
        widget_4->setSizePolicy(sizePolicy4);
        widget_4->setMinimumSize(QSize(400, 45));
			
				//----------------------------------------------------------------------
				// Button Reset
				//----------------------------------------------------------------------
        pushButtonReset = new QPushButton(widget_4);
        pushButtonReset->setObjectName(QString::fromUtf8("pushButtonReset"));
        pushButtonReset->setGeometry(QRect(100, 0, 40, 40));
        sizePolicy4.setHeightForWidth(pushButtonReset->sizePolicy().hasHeightForWidth());
        pushButtonReset->setSizePolicy(sizePolicy4);
        pushButtonReset->setMinimumSize(QSize(0, 0));
        pushButtonReset->setMaximumSize(QSize(16777215, 16777215));
        pushButtonReset->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"border-image:url(images/media-playback-stop.svg) 1 1 1 1 ;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"border-image:url(images/media-playback-stop.svg) -2 -2 -2 -2 ;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"border-image:url(images/media-playback-stop.svg) -5 -5 -5 -5 ;\n"
"}\n"
"\n"
"QPushButton:disabled {\n"
"border-image:url(images/media-playback-stop-disabled.svg) 1 1 1 1 ;\n"
"}"));
        pushButtonResetGraph = new QPushButton(widget_4);
        pushButtonResetGraph->setObjectName(QString::fromUtf8("pushButtonResetGraph"));
        pushButtonResetGraph->setGeometry(QRect(150, 10, 117, 32));
      
				QSizePolicy sizePolicy5(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy5.setHorizontalStretch(0);
        sizePolicy5.setVerticalStretch(0);
        sizePolicy5.setHeightForWidth(pushButtonResetGraph->sizePolicy().hasHeightForWidth());
        pushButtonResetGraph->setSizePolicy(sizePolicy5);
			
				//----------------------------------------------------------------------
				// Button Run
				//----------------------------------------------------------------------
        pushButtonRun = new QPushButton(widget_4);
        pushButtonRun->setObjectName(QString::fromUtf8("pushButtonRun"));
        pushButtonRun->setGeometry(QRect(0, 0, 40, 40));
        sizePolicy4.setHeightForWidth(pushButtonRun->sizePolicy().hasHeightForWidth());
        pushButtonRun->setSizePolicy(sizePolicy4);
        pushButtonRun->setMinimumSize(QSize(0, 0));
        pushButtonRun->setMaximumSize(QSize(16777215, 16777215));
        pushButtonRun->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"border-image:url(images/media-playback-start.svg) 1 1 1 1 ;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"border-image:url(images/media-playback-start.svg) -2 -2 -2 -2 ;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"border-image:url(images/media-playback-start.svg) -8 -8 -8 -8 ;\n"
"}\n"
"\n"
"QPushButton:disabled {\n"
"border-image:url(images/media-playback-start-disabled.svg) 1 1 1 1 ;\n"
"}"));
        pushButtonRun->setIconSize(QSize(50, 50));
			
				//----------------------------------------------------------------------
				// Button Stop
				//----------------------------------------------------------------------
        pushButtonStop = new QPushButton(widget_4);
        pushButtonStop->setObjectName(QString::fromUtf8("pushButtonStop"));
        pushButtonStop->setGeometry(QRect(50, 0, 40, 40));
        sizePolicy4.setHeightForWidth(pushButtonStop->sizePolicy().hasHeightForWidth());
        pushButtonStop->setSizePolicy(sizePolicy4);
        pushButtonStop->setMinimumSize(QSize(0, 0));
        pushButtonStop->setMaximumSize(QSize(16777215, 16777215));
        pushButtonStop->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"border-image:url(images/media-playback-pause.svg) 1 1 1 1 ;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"border-image:url(images/media-playback-pause.svg) -2 -2 -2 -2 ;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"border-image:url(images/media-playback-pause.svg) -5 -5 -5 -5 ;\n"
"}\n"
"\n"
"QPushButton:disabled {\n"
"border-image:url(images/media-playback-pause-disabled.svg) 1 1 1 1 ;\n"
"}"));
        pushButtonStop->setIconSize(QSize(50, 50));
        labelRunning = new QLabel(widget_4);
        labelRunning->setObjectName(QString::fromUtf8("labelRunning"));
        labelRunning->setGeometry(QRect(290, 10, 101, 31));

        gridLayout_30->addWidget(widget_4, 0, 1, 1, 1);

        horizontalLayoutRunStrategy = new QHBoxLayout();
        horizontalLayoutRunStrategy->setSpacing(6);
        horizontalLayoutRunStrategy->setObjectName(QString::fromUtf8("horizontalLayoutRunStrategy"));
        radioButtonDiff = new QRadioButton(groupBoxRunButtons);
        radioButtonDiff->setObjectName(QString::fromUtf8("radioButtonDiff"));

        horizontalLayoutRunStrategy->addWidget(radioButtonDiff);

        radioButtonPRM = new QRadioButton(groupBoxRunButtons);
        radioButtonPRM->setObjectName(QString::fromUtf8("radioButtonPRM"));

        horizontalLayoutRunStrategy->addWidget(radioButtonPRM);

        checkBoxWithSmoothing = new QCheckBox(groupBoxRunButtons);
        checkBoxWithSmoothing->setObjectName(QString::fromUtf8("checkBoxWithSmoothing"));

        horizontalLayoutRunStrategy->addWidget(checkBoxWithSmoothing);

        checkBoxUseP3DStructures = new QCheckBox(groupBoxRunButtons);
        checkBoxUseP3DStructures->setObjectName(QString::fromUtf8("checkBoxUseP3DStructures"));

        horizontalLayoutRunStrategy->addWidget(checkBoxUseP3DStructures);


        gridLayout_30->addLayout(horizontalLayoutRunStrategy, 1, 1, 1, 1);


        horizontalLayout_3->addWidget(groupBoxRunButtons);

        groupBoxTrajectory = new QGroupBox(controlWidget);
        groupBoxTrajectory->setObjectName(QString::fromUtf8("groupBoxTrajectory"));
        verticalLayout_34 = new QVBoxLayout(groupBoxTrajectory);
        verticalLayout_34->setSpacing(6);
        verticalLayout_34->setContentsMargins(11, 11, 11, 11);
        verticalLayout_34->setObjectName(QString::fromUtf8("verticalLayout_34"));
        horizontalLayoutShowTraj = new QHBoxLayout();
        horizontalLayoutShowTraj->setSpacing(6);
        horizontalLayoutShowTraj->setObjectName(QString::fromUtf8("horizontalLayoutShowTraj"));
        horizontalSpacerShowTraj1 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayoutShowTraj->addItem(horizontalSpacerShowTraj1);

        pushButtonShowTraj = new QPushButton(groupBoxTrajectory);
        pushButtonShowTraj->setObjectName(QString::fromUtf8("pushButtonShowTraj"));

        horizontalLayoutShowTraj->addWidget(pushButtonShowTraj);

        horizontalSpacerShowTraj2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayoutShowTraj->addItem(horizontalSpacerShowTraj2);


        verticalLayout_34->addLayout(horizontalLayoutShowTraj);

        horizontalLayoutTrajSpeed = new QHBoxLayout();
        horizontalLayoutTrajSpeed->setSpacing(6);
        horizontalLayoutTrajSpeed->setObjectName(QString::fromUtf8("horizontalLayoutTrajSpeed"));
        label = new QLabel(groupBoxTrajectory);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayoutTrajSpeed->addWidget(label);

        doubleSpinBoxTrajSpeed = new QDoubleSpinBox(groupBoxTrajectory);
        doubleSpinBoxTrajSpeed->setObjectName(QString::fromUtf8("doubleSpinBoxTrajSpeed"));
        doubleSpinBoxTrajSpeed->setMinimum(0);
        doubleSpinBoxTrajSpeed->setMaximum(30);

        horizontalLayoutTrajSpeed->addWidget(doubleSpinBoxTrajSpeed);

        horizontalSliderTrajSpeed = new QSlider(groupBoxTrajectory);
        horizontalSliderTrajSpeed->setObjectName(QString::fromUtf8("horizontalSliderTrajSpeed"));
        horizontalSliderTrajSpeed->setMinimum(1);
        horizontalSliderTrajSpeed->setMaximum(1000);
        horizontalSliderTrajSpeed->setOrientation(Qt::Horizontal);

        horizontalLayoutTrajSpeed->addWidget(horizontalSliderTrajSpeed);


        verticalLayout_34->addLayout(horizontalLayoutTrajSpeed);


        horizontalLayout_3->addWidget(groupBoxTrajectory);

        groupBoxTests = new QGroupBox(controlWidget);
        groupBoxTests->setObjectName(QString::fromUtf8("groupBoxTests"));
        verticalLayout_4 = new QVBoxLayout(groupBoxTests);
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        pushButtonTest1 = new QPushButton(groupBoxTests);
        pushButtonTest1->setObjectName(QString::fromUtf8("pushButtonTest1"));

        verticalLayout_4->addWidget(pushButtonTest1);

        pushButtonTest2 = new QPushButton(groupBoxTests);
        pushButtonTest2->setObjectName(QString::fromUtf8("pushButtonTest2"));

        verticalLayout_4->addWidget(pushButtonTest2);

        pushButtonTest3 = new QPushButton(groupBoxTests);
        pushButtonTest3->setObjectName(QString::fromUtf8("pushButtonTest3"));

        verticalLayout_4->addWidget(pushButtonTest3);


        horizontalLayout_3->addWidget(groupBoxTests);

        hSplitter->addWidget(controlWidget);

        verticalLayout_2->addWidget(hSplitter);

        vSplitter->addWidget(verticalLayout);

        verticalLayout_21->addWidget(vSplitter);

        MainWindow->setCentralWidget(centralWidget);
        
				// --------------------------------------------------------
				// Menu
				// --------------------------------------------------------
				menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1440, 22));
        
				menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        
				menuHelp = new QMenu(menuBar);
        menuHelp->setObjectName(QString::fromUtf8("menuHelp"));
      
				menuCollisionCheker = new QMenu(menuBar);
        menuCollisionCheker->setObjectName(QString::fromUtf8("menuCollisionCheker"));
			
        menuGraph = new QMenu(menuBar);
        menuGraph->setObjectName(QString::fromUtf8("menuGraph"));
			
        menuEnvironement = new QMenu(menuBar);
        menuEnvironement->setObjectName(QString::fromUtf8("menuEnvironement"));
      
				menuRobot = new QMenu(menuEnvironement);
        menuRobot->setObjectName(QString::fromUtf8("menuRobot"));
        
				MainWindow->setMenuBar(menuBar);
        
				statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

				//----------------------------------------------------------------------
				// Menu Bar
				//----------------------------------------------------------------------
        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuEnvironement->menuAction());
        menuBar->addAction(menuGraph->menuAction());
        menuBar->addAction(menuCollisionCheker->menuAction());
        menuBar->addAction(menuHelp->menuAction());
			
        menuFile->addAction(actionOpen);
        menuFile->addAction(actionCloseEnvironement);
        menuFile->addSeparator();
        menuFile->addAction(actionQuit);
			
        menuHelp->addAction(actionAbout);
			
        menuCollisionCheker->addAction(actionKCDPropietes);
			
        menuGraph->addAction(actionLoadGraph);
        menuGraph->addAction(actionSaveGraph_2);
        menuGraph->addAction(actionSaveToDot);
        menuGraph->addSeparator();
        menuGraph->addAction(actionLoadTrajectory);
        menuGraph->addAction(actionSaveTrajectory);
			
        menuEnvironement->addAction(actionOpenScenario);
        menuEnvironement->addAction(actionSaveScenario);
        menuEnvironement->addSeparator();
        menuEnvironement->addAction(menuRobot->menuAction());
				menuEnvironement->addAction(actionRobotForm);
				menuEnvironement->addAction(actionLocalPathGroups);

        retranslateUi(MainWindow);
			
        QObject::connect(actionQuit, SIGNAL(triggered()), MainWindow, SLOT(close()));
				QObject::connect(actionQuit, SIGNAL(triggered()), formRobot, SLOT(close()));

        mainTabWidget->setCurrentIndex(4);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        actionQuit->setText(QApplication::translate("MainWindow", "Quit", 0, QApplication::UnicodeUTF8));
        action3DViewer->setText(QApplication::translate("MainWindow", "3D-Viewer", 0, QApplication::UnicodeUTF8));
        actionKCDPropietes->setText(QApplication::translate("MainWindow", "KCD Propietes", 0, QApplication::UnicodeUTF8));
        actionAbout->setText(QApplication::translate("MainWindow", "About", 0, QApplication::UnicodeUTF8));
        actionCloseEnvironement->setText(QApplication::translate("MainWindow", "Close", 0, QApplication::UnicodeUTF8));
        actionRobotForm->setText(QApplication::translate("MainWindow", "Robot Forms", 0, QApplication::UnicodeUTF8));
				actionLocalPathGroups->setText(QApplication::translate("MainWindow", "Local Path Groups", 0, QApplication::UnicodeUTF8));
				actionOpenScenario->setText(QApplication::translate("MainWindow", "Load Scenario", 0, QApplication::UnicodeUTF8));
				actionOpen->setText(QApplication::translate("MainWindow", "Open", 0, QApplication::UnicodeUTF8));
        actionSaveScenario->setText(QApplication::translate("MainWindow", "Save Scenario", 0, QApplication::UnicodeUTF8));
        actionNameOfEnv->setText(QApplication::translate("MainWindow", "NameOfEnv", 0, QApplication::UnicodeUTF8));
        actionSaveGraph->setText(QApplication::translate("MainWindow", "Save", 0, QApplication::UnicodeUTF8));
        actionLoadGraph->setText(QApplication::translate("MainWindow", "Load Graph", 0, QApplication::UnicodeUTF8));
        actionSaveGraph_2->setText(QApplication::translate("MainWindow", "Save Graph", 0, QApplication::UnicodeUTF8));
        actionLoadTrajectory->setText(QApplication::translate("MainWindow", "Load Trajectory", 0, QApplication::UnicodeUTF8));
        actionSaveTrajectory->setText(QApplication::translate("MainWindow", "Save Trajectory", 0, QApplication::UnicodeUTF8));
        actionAllDevices->setText(QApplication::translate("MainWindow", "All Devices", 0, QApplication::UnicodeUTF8));
        actionSave_Graph_To_Dot->setText(QApplication::translate("MainWindow", "Save Graph To Dot", 0, QApplication::UnicodeUTF8));
        actionSaveToDot->setText(QApplication::translate("MainWindow", "Save To Dot", 0, QApplication::UnicodeUTF8));
        
				mainTabWidget->setTabText(mainTabWidget->indexOf(tabMotionPlanner), QApplication::translate("MainWindow", "Motion Planner", 0, QApplication::UnicodeUTF8));
        mainTabWidget->setTabText(mainTabWidget->indexOf(tabCost), QApplication::translate("MainWindow", "Cost", 0, QApplication::UnicodeUTF8));
        mainTabWidget->setTabText(mainTabWidget->indexOf(tabRobot), QApplication::translate("MainWindow", "Robot", 0, QApplication::UnicodeUTF8));
        mainTabWidget->setTabText(mainTabWidget->indexOf(tabUtil), QApplication::translate("MainWindow", "Utils", 0, QApplication::UnicodeUTF8));
        
				groupBoxDrawEnv->setTitle(QApplication::translate("MainWindow", "Draw Env", 0, QApplication::UnicodeUTF8));
        checkBoxDrawGraph->setText(QApplication::translate("MainWindow", "Draw Graph", 0, QApplication::UnicodeUTF8));
        checkBoxDrawTraj->setText(QApplication::translate("MainWindow", "Draw Traj", 0, QApplication::UnicodeUTF8));
        checkBoxDisableDraw->setText(QApplication::translate("MainWindow", "No Draw", 0, QApplication::UnicodeUTF8));
        checkBoxDrawDebug->setText(QApplication::translate("MainWindow", "Draw Debug", 0, QApplication::UnicodeUTF8));
        checkBoxDrawTrajVector->setText(QApplication::translate("MainWindow", "Draw Traj Vector", 0, QApplication::UnicodeUTF8));
        pushButtonMobileCamera->setText(QApplication::translate("MainWindow", "Mob. Camera", 0, QApplication::UnicodeUTF8));
        threeDGroupBox->setTitle(QApplication::translate("MainWindow", "3D Model", 0, QApplication::UnicodeUTF8));
        checkBoxBB->setText(QApplication::translate("MainWindow", "Bounding Boxes", 0, QApplication::UnicodeUTF8));
        checkBoxGhosts->setText(QApplication::translate("MainWindow", "Ghosts", 0, QApplication::UnicodeUTF8));
        checkBoxJoints->setText(QApplication::translate("MainWindow", "Joints", 0, QApplication::UnicodeUTF8));
        checkBoxEnableLight->setText(QApplication::translate("MainWindow", "Light Model", 0, QApplication::UnicodeUTF8));
        sceneGroupBox->setTitle(QApplication::translate("MainWindow", "Scene", 0, QApplication::UnicodeUTF8));
        checkBoxFloor->setText(QApplication::translate("MainWindow", "Floor", 0, QApplication::UnicodeUTF8));
        checkBoxTiles->setText(QApplication::translate("MainWindow", "Tiles", 0, QApplication::UnicodeUTF8));
        checkBoxShadows->setText(QApplication::translate("MainWindow", "Shadow", 0, QApplication::UnicodeUTF8));
        checkBoxAxis->setText(QApplication::translate("MainWindow", "Axis", 0, QApplication::UnicodeUTF8));
        checkBoxSmooth->setText(QApplication::translate("MainWindow", "Smooth", 0, QApplication::UnicodeUTF8));
        checkBoxFilaire->setText(QApplication::translate("MainWindow", "Filaire", 0, QApplication::UnicodeUTF8));
        checkBoxWalls->setText(QApplication::translate("MainWindow", "Walls", 0, QApplication::UnicodeUTF8));
        checkBoxContour->setText(QApplication::translate("MainWindow", "Contours", 0, QApplication::UnicodeUTF8));
        lightGroupBox->setTitle(QApplication::translate("MainWindow", "Light", 0, QApplication::UnicodeUTF8));
        label_X->setText(QApplication::translate("MainWindow", "X", 0, QApplication::UnicodeUTF8));
        label_Y->setText(QApplication::translate("MainWindow", "Y", 0, QApplication::UnicodeUTF8));
        label_Z->setText(QApplication::translate("MainWindow", "Z", 0, QApplication::UnicodeUTF8));
        pushButtonRestoreLight->setText(QApplication::translate("MainWindow", "Restore Light", 0, QApplication::UnicodeUTF8));
        checkBoxDrawLightSource->setText(QApplication::translate("MainWindow", "Draw Light Source", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("MainWindow", "Traj to Draw", 0, QApplication::UnicodeUTF8));
        pushButtonAddTraj->setText(QApplication::translate("MainWindow", "Add Current Traj", 0, QApplication::UnicodeUTF8));
        pushButtonClearTraj->setText(QApplication::translate("MainWindow", "Clear All Traj", 0, QApplication::UnicodeUTF8));
        comboBoxColorTraj->clear();
        comboBoxColorTraj->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "Black", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "Blue", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "Yellow", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "Red", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "Green", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "White", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "Grey", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "Brown", 0, QApplication::UnicodeUTF8)
        );
        pushButtonRestoreView->setText(QApplication::translate("MainWindow", "Restore View", 0, QApplication::UnicodeUTF8));
        mainTabWidget->setTabText(mainTabWidget->indexOf(tabViewerSettings), QApplication::translate("MainWindow", "Viewer Settings", 0, QApplication::UnicodeUTF8));
        groupBoxRunButtons->setTitle(QApplication::translate("MainWindow", "Run Motion Planning", 0, QApplication::UnicodeUTF8));
        pushButtonReset->setText(QString());
        pushButtonResetGraph->setText(QApplication::translate("MainWindow", "Reset Graph", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        pushButtonRun->setToolTip(QApplication::translate("MainWindow", "Run Motion Planner", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_WHATSTHIS
        pushButtonRun->setWhatsThis(QApplication::translate("MainWindow", "Run planning", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_WHATSTHIS
        pushButtonRun->setText(QString());
#ifndef QT_NO_TOOLTIP
        pushButtonStop->setToolTip(QApplication::translate("MainWindow", "Run Motion Planner", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_WHATSTHIS
        pushButtonStop->setWhatsThis(QApplication::translate("MainWindow", "Run planning", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_WHATSTHIS
        pushButtonStop->setText(QString());
        labelRunning->setText(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Lucida Grande'; font-size:13pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:16pt; color:#008d00;\">Not Running</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        radioButtonDiff->setText(QApplication::translate("MainWindow", "Diffusion", 0, QApplication::UnicodeUTF8));
        radioButtonPRM->setText(QApplication::translate("MainWindow", "PRM", 0, QApplication::UnicodeUTF8));
        checkBoxWithSmoothing->setText(QApplication::translate("MainWindow", "With Smoothing", 0, QApplication::UnicodeUTF8));
        checkBoxUseP3DStructures->setText(QApplication::translate("MainWindow", "p3d Structs", 0, QApplication::UnicodeUTF8));
        groupBoxTrajectory->setTitle(QApplication::translate("MainWindow", "Trajectory", 0, QApplication::UnicodeUTF8));
        pushButtonShowTraj->setText(QApplication::translate("MainWindow", "Show Trajectory", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "Speed", 0, QApplication::UnicodeUTF8));
        groupBoxTests->setTitle(QApplication::translate("MainWindow", "Tests", 0, QApplication::UnicodeUTF8));
        pushButtonTest1->setText(QApplication::translate("MainWindow", "Test 1", 0, QApplication::UnicodeUTF8));
        pushButtonTest2->setText(QApplication::translate("MainWindow", "Test 2", 0, QApplication::UnicodeUTF8));
        pushButtonTest3->setText(QApplication::translate("MainWindow", "Test 3", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", 0, QApplication::UnicodeUTF8));
        menuHelp->setTitle(QApplication::translate("MainWindow", "Help", 0, QApplication::UnicodeUTF8));
        menuCollisionCheker->setTitle(QApplication::translate("MainWindow", "Collision Cheker", 0, QApplication::UnicodeUTF8));
        menuGraph->setTitle(QApplication::translate("MainWindow", "Roadmap", 0, QApplication::UnicodeUTF8));
        menuEnvironement->setTitle(QApplication::translate("MainWindow", "Environement", 0, QApplication::UnicodeUTF8));
        menuRobot->setTitle(QApplication::translate("MainWindow", "Robot", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
