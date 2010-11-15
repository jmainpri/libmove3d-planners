#ifndef DOUBLEPLOT_HPP
#define DOUBLEPLOT_HPP

#if defined( CXX_PLANNER )
#include "basicPlot.hpp"
#endif

#if defined( MOVE3D_CORE )
#include "qtUI/qtPlot/basicPlot.hpp"
#endif

#include<vector>

//const int PLOT_SIZE = 100;      // 0 to 200

/**
 * @ingroup qtPlot
 * @brief Qt simple plot relies on qwt
 */
class DoublePlot : public QwtPlot
{
	Q_OBJECT
	
public:
	DoublePlot(QWidget* = NULL);
	
	int getPlotSize() { return PLOT_SIZE; }
	void setData(const std::vector< std::string >& names , 
							 const std::vector< std::vector <double> >& data );
	void rescale();
	
private:
	void alignScales();
	
	QwtArray< double >								d_x;
	std::vector< QwtArray< double > > d_y;
	
	bool init;
	
	std::vector<double> Max_y;
	std::vector< QwtPlotCurve* > cData;
};

#endif // DOUBLEPLOT_HPP
