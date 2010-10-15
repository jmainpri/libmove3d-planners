#include "qt_widgets.hpp"
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <vector>
#include <limits>

//------------------------------------------------------------------------------

QDoubleSlider::QDoubleSlider(
        double min,
        double max,
        double init_value,
        QWidget *parent
        )
        : QSlider(Qt::Horizontal, parent)
{
    QSlider::setRange(/*std::numeric_limits<int>::min()*/-1000, /*std::numeric_limits<int>::max()*/1000);
    connect(this, SIGNAL(valueChanged(int)), this, SLOT(synchronizeDouble(int)));
    setRange(min, max);
    setValue(init_value, true, true);
}

//------------------------------------------------------------------------------

void QDoubleSlider::synchronizeDouble(int new_value)
{
    double double_value = min() +
                          ((double(new_value) - double(QSlider::minimum())) * (max() - min())) /
                          (double(QSlider::maximum()) - double(QSlider::minimum()));
    setValue(double_value, false);
}

void QDoubleSlider::setValue(double new_value, bool sync, bool init)
{
    if(new_value != value() || init) {
        doubleValue = new_value;
        if(sync) {
            int int_value = int(0.5 + ((double) QSlider::minimum()) +
                                ((new_value - min()) / (max() - min())) *
                                (((double) QSlider::maximum()) -
                                 ((double) QSlider::minimum())));
            disconnect(this, SIGNAL(valueChanged(int)), this, SLOT(synchronizeDouble(int)));
            QSlider::setValue(int_value);
            connect(this, SIGNAL(valueChanged(int)), this, SLOT(synchronizeDouble(int)));
        }
        emit valueChanged(new_value);
    }
}

//------------------------------------------------------------------------------

void QDoubleSlider::setValue(double new_value)
{
    setValue(new_value, true);
}

//------------------------------------------------------------------------------

void QDoubleSlider::setRange(double min, double max)
{
    max = min > max ? min : max;
    doubleMin = min;
    doubleMax = max;
    if(value() < min)
    { setValue(min); }
    else if(value() > max)
    { setValue(max); }
}

//------------------------------------------------------------------------------

double QDoubleSlider::value() const
{
    return(doubleValue);
}

//------------------------------------------------------------------------------

double QDoubleSlider::min() const
{
    return(doubleMin);
}

//------------------------------------------------------------------------------

double QDoubleSlider::max() const
{
    return(doubleMax);
}

//------------------------------------------------------------------------------

LabeledDoubleSlider::LabeledDoubleSlider(
        double min,
        double max,
        double init_value,
        QString text,
        int row,
        QGridLayout* layout,
        QWidget *parent
        )
        : QWidget(parent)
{
    if(!layout) {
        layout = new QGridLayout;
        layout->setHorizontalSpacing(3);
        layout->setVerticalSpacing(3);
        setLayout(layout);
    }
    QLabel* label = new QLabel(text);
    QLineEdit* edit = new QLineEdit;
    edit->setMaximumWidth(edit->fontMetrics().width('0') * 10);
    slider = new QDoubleSlider(min, max, init_value);
    slider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    connect(slider, SIGNAL(valueChanged(double)), this, SLOT(doubleToQString(double)));
    connect(this, SIGNAL(sliderValue(QString)), edit, SLOT(setText(QString)));
    connect(edit, SIGNAL(textEdited(QString)), this, SLOT(qStringToDouble(QString)));
    connect(this, SIGNAL(sliderValue(double)), slider, SLOT(setValue(double)));
    connect(slider, SIGNAL(valueChanged(double)), this, SIGNAL(valueChanged(double)));

    doubleToQString(slider->value());

    layout->addWidget(label, row, 0);
    layout->addWidget(edit, row, 1);
    layout->addWidget(slider, row, 2);
}

//------------------------------------------------------------------------------

void LabeledDoubleSlider::doubleToQString(double value)
{
    emit sliderValue(QString::number(value));
}

//------------------------------------------------------------------------------

void LabeledDoubleSlider::qStringToDouble(QString value)
{
    emit sliderValue(value.toDouble());
}

//------------------------------------------------------------------------------

void LabeledDoubleSlider::setValue(double new_value)
{
    slider->setValue(new_value);
}

//------------------------------------------------------------------------------

double LabeledDoubleSlider::value() const
{
    return(slider->value());
}

//------------------------------------------------------------------------------

LabeledSlider::LabeledSlider(
        int min,
        int max,
        int init_value,
        QString text,
        int row,
        QGridLayout* layout,
        QWidget *parent
        )
        : QWidget(parent)
{
    if(!layout) {
        layout = new QGridLayout;
        layout->setHorizontalSpacing(3);
        layout->setVerticalSpacing(3);
        setLayout(layout);
    }
    QLabel* label = new QLabel(text);
    QLineEdit* edit = new QLineEdit;
    edit->setMaximumWidth(edit->fontMetrics().width('0') * 10);
    slider = new QSlider(Qt::Horizontal, parent);
    slider->setRange(min, max);
    slider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    connect(slider, SIGNAL(valueChanged(int)), this, SLOT(intToQString(int)));
    connect(this, SIGNAL(sliderValue(QString)), edit, SLOT(setText(QString)));
    connect(edit, SIGNAL(textEdited(QString)), this, SLOT(qStringToInt(QString)));
    connect(this, SIGNAL(sliderValue(int)), slider, SLOT(setValue(int)));
    connect(slider, SIGNAL(valueChanged(int)), this, SIGNAL(valueChanged(int)));

    slider->setValue(init_value);

    layout->addWidget(label, row, 0);
    layout->addWidget(edit, row, 1);
    layout->addWidget(slider, row, 2);
}

//------------------------------------------------------------------------------

void LabeledSlider::intToQString(int value)
{
    emit sliderValue(QString::number(value));
}

//------------------------------------------------------------------------------

void LabeledSlider::qStringToInt(QString value)
{
    emit sliderValue(value.toInt());
}

//------------------------------------------------------------------------------

void LabeledSlider::setValue(int new_value)
{
    slider->setValue(new_value);
}

//------------------------------------------------------------------------------

int LabeledSlider::value() const
{
    return(slider->value());
}

//------------------------------------------------------------------------------

QVGroupBox::QVGroupBox(const QString& title, QWidget* parent)
        : QGroupBox(title, parent)
{
    QVBoxLayout* layout = new QVBoxLayout;
    layout->setSpacing(3);
    layout->setContentsMargins(3,3,3,3);
    setLayout(layout);
}

//------------------------------------------------------------------------------

void QVGroupBox::addWidget(QWidget* w, Qt::Alignment alignment)
{
    layout()->addWidget(w);
    layout()->setAlignment(w, alignment);
}

//------------------------------------------------------------------------------

OpenFile::OpenFile(
        QString button_label,
        QStringList filters,
        QBoxLayout::Direction dir
        )
        : _label(),
        dialog()
{
    QBoxLayout* layout = new QBoxLayout(dir);
    QPushButton* open_file_button = new QPushButton(button_label);
    dialog.setFileMode(QFileDialog::ExistingFile);
    dialog.setNameFilters(filters);
    _label.setFrameStyle(QFrame::Panel | QFrame::Sunken);

    connect(open_file_button, SIGNAL(clicked()), &dialog, SLOT(exec()));
    connect(&dialog, SIGNAL(filesSelected(QStringList)), this, SLOT(setTextFromList(QStringList)));

    layout->addWidget(open_file_button);
    layout->addWidget(&_label);
    layout->addStretch();
    setLayout(layout);
}

//------------------------------------------------------------------------------

void OpenFile::setTextFromList(QStringList list)
{
    if(list.size() > 0) {
        p_filename = list.first();
        _label.setText(QFileInfo(p_filename).fileName());
        emit fileChanged(filename());
    }
}

//------------------------------------------------------------------------------

QString OpenFile::filename() const
{
    return(p_filename);
}

//------------------------------------------------------------------------------
