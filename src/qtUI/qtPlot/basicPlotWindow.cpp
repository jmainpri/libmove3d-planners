#include "basicPlotWindow.hpp"
#include "ui_BasicPlotWindow.hpp"

BasicPlotWindow::BasicPlotWindow(QWidget *parent) :
        QWidget(parent),
        m_ui(new Ui::BasicPlotWindow)
{
    m_ui->setupUi(this);
}

BasicPlotWindow::~BasicPlotWindow()
{
    delete m_ui;
}

void BasicPlotWindow::changeEvent(QEvent *e)
{
    QWidget::changeEvent(e);
    switch (e->type())
    {
    case QEvent::LanguageChange:
        m_ui->retranslateUi(this);
        break;
    default:
        break;
    }
}


void BasicPlotWindow::setPlot(QwtPlot* plot)
{
    m_ui->plot = plot;
}

QwtPlot* BasicPlotWindow::getPlot()
{
    return m_ui->plot;
}
