/*
 * histoWin.hpp
 *
 *  Created on: Sep 14, 2009
 *      Author: jmainpri
 */

#ifndef HISTOWIN_HPP_
#define HISTOWIN_HPP_

#include <qwt_plot.h>
#include <qmainwindow.h>

/**
  * @ingroup qtWindow
  * @defgroup qtPlot
  * This modules relies on qwt which is an open source library that relies on the C++ Qt library,
  * it implements such wigets as Plots, Histogram, logarithmic sliders and other scientific interfaces
  * The example code plots the trajectory cost
  \code
    BasicPlot* myPlot = new BasicPlot(this->plot);

    Trajectory* traj = currentTrajPt;

    int nbSample = myPlot->getPlotSize();
    double step = traj->getRangeMax() / (double) nbSample;

    vector<double> cost;

    for( double param=0; param<traj.getRangeMax(); param = param + step)
    {
        cost.push_back(traj->configAtParam(param)->cost());
    }

    myPlot->setData(cost);
    myPlot->show();
   \endcode
  */

/**
  * @ingroup qtPlot
  * @brief Qt Histogram widget relies on qwt
  */

class HistoWindow  {

public:
	HistoWindow();

	void startWindow();

private:
    QwtPlot* plot;
};

#endif /* HISTOWIN_HPP_ */
