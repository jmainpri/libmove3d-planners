#ifndef PLOTWINDOW_HPP
#define PLOTWINDOW_HPP

#include <qwt/qwt_plot.h>

namespace Ui
{
    class BasicPlotWindow;
}

/**
  * @ingroup qtPlot
  * @brief Qt simple plot Window relies on qwt
  */
class BasicPlotWindow : public QWidget
{
    Q_OBJECT
public:
    BasicPlotWindow(QWidget *parent = 0);
    ~BasicPlotWindow();

    void setPlot(QwtPlot* plot);
    QwtPlot* getPlot();

protected:
    void changeEvent(QEvent *e);

private:
    Ui::BasicPlotWindow *m_ui;
};

#endif // PLOTWINDOW_HPP
