#ifndef QT_BASE_WIN_HH
#define QT_BASE_WIN_HH

#include "env.hpp"

#if defined( CXX_PLANNER )
#include "qtLibrary.hpp"
#include "qt_widgets.hpp"
#endif

#if defined( WITH_OOMOVE3D )
#include "qtUI/qtLibrary.hpp"
#include "qtUI/qtBase/qt_widgets.hpp"
#endif

/**
 * @ingroup qtOldWidget
 * @brief Qt Window base container
 */
class qtBaseWindow : public QObject
{
	Q_OBJECT;

protected:
	QString string;
	QGroupBox* box;
	QGridLayout* Layout;

public:
	// Constructor
	qtBaseWindow();

	// Getters
	QString 		getString();
	QGroupBox* 		getBox();
	QGridLayout* 	getLayout();

	// Initialization function
	//virtual void  init() = 0;
	//void init();

	// Create sliders and check boxes
	LabeledSlider* createSlider(QString s, Env::intParameter p, int lower, int upper);
	LabeledDoubleSlider* createDoubleSlider(QString s, Env::doubleParameter p, double lower, double upper);
	QCheckBox* createCheckBox(QString s, Env::boolParameter p);

public:
	// Destructor
	~qtBaseWindow();

};
#endif
