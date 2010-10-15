#include <QtGui/qapplication.h>
#include <QtGui/qmainwindow.h>
#include <qwt_counter.h>
#include <QtGui/qtoolbar.h>
#include <QtGui/qlabel.h>
#include <QtGui/qlayout.h>

#include "dataPlot.hpp"
#include "tempWin.hpp"

TempWin::TempWin()
{
	QToolBar *toolBar = new QToolBar(this);
        toolBar->setFixedHeight(50);

        this->resize(500,300);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        this->setSizePolicy(sizePolicy);

#if QT_VERSION < 0x040000
	setDockEnabled(TornOff, true);
	setRightJustification(true);
#else
	toolBar->setAllowedAreas(Qt::TopToolBarArea | Qt::BottomToolBarArea);
#endif
	QWidget *hBox = new QWidget(toolBar);
	QLabel *label = new QLabel("Timer Interval", hBox);
	QwtCounter *counter = new QwtCounter(hBox);
        counter->setRange(-1.0, 10.0, 1.0);

	QLabel *label2 = new QLabel("Maximum T", hBox);
	QwtCounter *counter2 = new QwtCounter(hBox);
        counter2->setRange(0.0000000001, 100.0, 0.1);

	QHBoxLayout *layout = new QHBoxLayout(hBox);
	layout->addWidget(label);
	layout->addWidget(counter);
	layout->addWidget(label2);
	layout->addWidget(counter2);
	layout->addWidget(new QWidget(hBox), 10); // spacer);

#if QT_VERSION >= 0x040000
	toolBar->addWidget(hBox);
#endif
	addToolBar(toolBar);

	DataPlot *plot = new DataPlot(this);

	setCentralWidget(plot);

	connect(counter, SIGNAL(valueChanged(double)), plot,
			SLOT(setTimerInterval(double)));

	connect(counter2, SIGNAL(valueChanged(double)), plot,
				SLOT(setMax(double)));

	counter->setValue(20.0);
	counter2->setValue(1.0);
}
