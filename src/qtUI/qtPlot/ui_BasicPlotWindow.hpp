/********************************************************************************
** Form generated from reading UI file 'BasicPlotWindow.ui'
**
** Created: Thu Jan 28 00:08:44 2010
**      by: Qt User Interface Compiler version 4.6.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef BASICPLOTWINDOW_H
#define BASICPLOTWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QWidget>
#include <qwt_plot.h>

QT_BEGIN_NAMESPACE

class Ui_BasicPlotWindow
{
public:
    QwtPlot *plot;

    void setupUi(QWidget *BasicPlotWindow)
    {
        if (BasicPlotWindow->objectName().isEmpty())
            BasicPlotWindow->setObjectName(QString::fromUtf8("BasicPlotWindow"));
        BasicPlotWindow->resize(658, 381);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(BasicPlotWindow->sizePolicy().hasHeightForWidth());
        BasicPlotWindow->setSizePolicy(sizePolicy);
        plot = new QwtPlot(BasicPlotWindow);
        plot->setObjectName(QString::fromUtf8("plot"));
        plot->setGeometry(QRect(10, 10, 641, 361));
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(plot->sizePolicy().hasHeightForWidth());
        plot->setSizePolicy(sizePolicy1);

        retranslateUi(BasicPlotWindow);

        QMetaObject::connectSlotsByName(BasicPlotWindow);
    } // setupUi

    void retranslateUi(QWidget *BasicPlotWindow)
    {
        BasicPlotWindow->setWindowTitle(QApplication::translate("BasicPlotWindow", "Form", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class BasicPlotWindow: public Ui_BasicPlotWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // BASICPLOTWINDOW_H
