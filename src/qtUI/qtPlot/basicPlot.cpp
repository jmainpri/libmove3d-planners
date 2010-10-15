#include <stdlib.h>
#include <qwt_painter.h>
#include <qwt_plot_canvas.h>
#include <qwt_plot_marker.h>
#include <qwt_plot_curve.h>
#include <qwt_scale_widget.h>
#include <qwt_legend.h>
#include <qwt_scale_draw.h>
#include <qwt_math.h>
#include "basicPlot.hpp"
#include "env.hpp"
#include <iostream>
#include <algorithm>
//
//  Initialize main window
//

using namespace std;

BasicPlot::BasicPlot( QWidget *parent):
    QwtPlot(parent)

{
    // Disable polygon clipping
    QwtPainter::setDeviceClipping(false);

    // We don't need the cache here
//    canvas()->setPaintAttribute(QwtPlotCanvas::PaintCached, false);
//    canvas()->setPaintAttribute(QwtPlotCanvas::PaintPacked, false);

#if QT_VERSION >= 0x040000
#ifdef Q_WS_X11
    /*
       Qt::WA_PaintOnScreen is only supported for X11, but leads
       to substantial bugs with Qt 4.2.x/Windows
     */
    canvas()->setAttribute(Qt::WA_PaintOnScreen, true);
#endif
#endif

//    alignScales();
    
    //  Initialize data
    for (int i = 0; i< PLOT_SIZE; i++)
    {
        d_x[i] = i;     // time axis
        d_y[i] = i;
    }

    // Assign a title
    setTitle("Cost along trajectory");
    insertLegend(new QwtLegend(), QwtPlot::BottomLegend);

    // Insert new curves
    cRight = new QwtPlotCurve("Cost");
    cRight->setPen(QPen(Qt::red));
    cRight->setRawData(d_x, d_y, PLOT_SIZE);
    cRight->attach(this);

    init = false;
    Max_y = 0.0;
    replot();
}

//
//  Set a plain canvas frame and align the scales to it
//
void BasicPlot::alignScales()
{
    // The code below shows how to align the scales to
    // the canvas frame, but is also a good example demonstrating
    // why the spreaded API needs polishing.

    canvas()->setFrameStyle(QFrame::Box | QFrame::Plain );
    canvas()->setLineWidth(1);

    for ( int i = 0; i < QwtPlot::axisCnt; i++ )
    {
        QwtScaleWidget *scaleWidget = (QwtScaleWidget *)axisWidget(i);

        if ( scaleWidget )
            scaleWidget->setMargin(0);

        QwtScaleDraw *scaleDraw = (QwtScaleDraw *)axisScaleDraw(i);

        if ( scaleDraw )
            scaleDraw->enableComponent(QwtAbstractScaleDraw::Backbone, false);
    }
}

//void BasicPlot::rescale()
//{
//    Max_y = *std::max_element(d_y,d_y+PLOT_SIZE);
//    setAxisScale(QwtPlot::yLeft, 0.0,Max_y*1.10);
//}

//  Generate new values 
void BasicPlot::setData(std::vector<double> y)
{
//    cout << "Adding data to plot" << endl;
    if(ENV.getBool(Env::initPlot) == false )
    {
//				for (unsigned int i=0; i<y.size(); i++) {
//					cout << "y [" << i << "] = " << y[i] << endl;
//				}
			
        Max_y = *std::max_element(y.begin(),y.end());
//				cout << "Max_y = " << Max_y << endl;
//        cout << "Setting fixed Axis"<< endl;
        setAxisScale(QwtPlot::yLeft, 0.0,Max_y*1.10);
        ENV.setBool(Env::initPlot,true);
    }

    for ( int i = 0; i<PLOT_SIZE ; i++)
    {
        d_y[i] = y[i];
//        cout << y[i] << endl;
    }

    replot();
//		rescale()
//    updateLayout();
 //   alignScales();

//    cout << "Replot" << endl;
}
