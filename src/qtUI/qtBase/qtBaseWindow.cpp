#include "qtBaseWindow.hpp"
#include <iostream>

// Constructor
qtBaseWindow::qtBaseWindow()
{
	string = tr("Name of Window");

	Layout = new QGridLayout();
	Layout->setSpacing(3);
	Layout->setContentsMargins(3, 3, 3, 3);

	box = new QGroupBox(string);
	box->setLayout(Layout);
}

// Getters
QString qtBaseWindow::getString()
{
	return string;
}

QGroupBox* qtBaseWindow::getBox()
{
	return box;
}

QGridLayout* qtBaseWindow::getLayout()
{
	return Layout;
}

LabeledSlider* qtBaseWindow::createSlider(QString s, Env::intParameter p,
		int lower, int upper)
{
	LabeledSlider* slider = new LabeledSlider(lower, upper, lower, s);
	connect(ENV.getObject(p), SIGNAL(valueChanged(int)), slider,
			SLOT(setValue(int)), Qt::DirectConnection);
	connect(slider, SIGNAL(valueChanged(int)), ENV.getObject(p),
			SLOT(set(int)), Qt::DirectConnection);
	slider->setValue(ENV.getInt(p));
	return (slider);
}

LabeledDoubleSlider* qtBaseWindow::createDoubleSlider(QString s,
		Env::doubleParameter p, double lower, double upper)
{
	LabeledDoubleSlider* slider = new LabeledDoubleSlider(lower, upper, lower,
			s);
	connect(ENV.getObject(p), SIGNAL(valueChanged(double)), slider,
			SLOT(setValue(double)), Qt::DirectConnection);
	connect(slider, SIGNAL(valueChanged(double)), ENV.getObject(p),
			SLOT(set(double)), Qt::DirectConnection);
	slider->setValue(ENV.getDouble(p));
	return (slider);
}

QCheckBox* qtBaseWindow::createCheckBox(QString s, Env::boolParameter p)
{
	QCheckBox* box = new QCheckBox(s);
	connect(ENV.getObject(p), SIGNAL(valueChanged(bool)), box,
			SLOT(setChecked(bool)), Qt::DirectConnection);
	connect(box, SIGNAL(toggled(bool)), ENV.getObject(p), SLOT(set(bool)),
			Qt::DirectConnection);
	box->setChecked(ENV.getBool(p));
	return (box);
}

qtBaseWindow::~qtBaseWindow()
{
	//delete box;
	//delete Layout;
}

