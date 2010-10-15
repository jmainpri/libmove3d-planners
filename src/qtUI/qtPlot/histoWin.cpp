#include <stdlib.h>
#include <qapplication.h>
#include <qpen.h>
#include <qwt_plot.h>
#include <qwt_plot_grid.h>
#include <qwt_plot_marker.h>
#include <qwt_interval_data.h>
#include "histogramItem.hpp"

#include "histoWin.hpp"
#include <algorithm>
#include <iostream>

#ifdef CXX_PLANNER
#include "SaveContext.hpp"
#endif

using namespace std;

HistoWindow::HistoWindow()
{

	plot = new QwtPlot;
    plot->setCanvasBackground(QColor(Qt::white));
    plot->setTitle("Histogram");

    QwtPlotGrid *grid = new QwtPlotGrid;
    grid->enableXMin(true);
    grid->enableYMin(true);
    grid->setMajPen(QPen(Qt::black, 0, Qt::DotLine));
    grid->setMinPen(QPen(Qt::gray, 0 , Qt::DotLine));
    grid->attach(plot);

    HistogramItem *histogram = new HistogramItem();
    histogram->setColor(Qt::darkCyan);

    int numValues(0);

#ifdef CXX_PLANNER
    if( storedContext.getNumberStored() >0)
    {
    	numValues = storedContext.getTime(0).size();

    	for(int i=0;i<numValues;i++)
    	{
    		cout << "Time of run i= " << storedContext.getTime(0)[i] << endl;
    	}
    }

    QwtArray<QwtDoubleInterval> intervals(numValues);
    QwtArray<double> values(numValues);

    double pos = 0.0;
    for ( int i = 0; i < (int)intervals.size(); i++ )
    {
        const int width = 1;

        intervals[i] = QwtDoubleInterval(pos, pos + double(width));
        values[i] = storedContext.getTime(0)[i];

        pos += width;
    }

    histogram->setData(QwtIntervalData(intervals, values));
    histogram->attach(plot);

    double max =0;

    if( storedContext.getNumberStored() >0)
    {
		max = *std::max_element(
				storedContext.getTime(0).begin(),
				storedContext.getTime(0).end());
    }

    plot->setAxisScale(QwtPlot::yLeft, 0.0, max);
    plot->setAxisScale(QwtPlot::xBottom, 0.0, pos);
#endif
    plot->replot();

#if QT_VERSION < 0x040000
    a.setMainWidget(&plot);
#endif

}

void HistoWindow::startWindow()
{
    plot->resize(600,400);
    plot->show();
}
