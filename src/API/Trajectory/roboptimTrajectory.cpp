/*
 *  RoboptimTrajectory.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 22/06/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "RoboptimTrajectory.h"

#include <fstream>

#include <boost/assign/list_of.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <roboptim/core/io.hh>
#include <roboptim/core/finite-difference-gradient.hh>
#include <roboptim/core/solver-factory.hh>
#include <roboptim/core/plugin/cfsqp.hh>
#include <roboptim/core/indent.hh>
#include <roboptim/core/visualization/gnuplot.hh>
#include <roboptim/core/visualization/gnuplot-commands.hh>
#include <roboptim/core/visualization/gnuplot-function.hh>

#include <roboptim/trajectory/sys.hh>
#include <roboptim/trajectory/free-time-trajectory.hh>
#include <roboptim/trajectory/freeze.hh>
#include <roboptim/trajectory/fwd.hh>
#include <roboptim/trajectory/limit-speed.hh>
#include <roboptim/trajectory/spline-length.hh>
#include <roboptim/trajectory/cubic-b-spline.hh>
#include <roboptim/trajectory/trajectory-cost.hh>
#include <roboptim/trajectory/visualization/trajectory.hh>
#include <roboptim/trajectory/visualization/limit-speed.hh>

#include <vector>

#include "API/ConfigSpace/configuration.hpp"
#include "API/Trajectory/trajectory.hpp"

//#include "common.hh"

using namespace roboptim;
using namespace roboptim::visualization;
using namespace roboptim::visualization::gnuplot;

//-----------------------------------------------------------------
/*!
 * Change the arguments of the Linear Trajectory
 */
struct LinearDerivWrtParameters : public DerivableFunction
{
  LinearDerivWrtParameters (const LinearTrajectory& traj, value_type t)
	: DerivableFunction (traj.parameters ().size (), traj.outputSize (),
											 "spline derivable w.r.t parameters"),
	linTraj_ (traj),
	t_ (t)
  {}
	
  virtual void
  impl_compute (result_t& result, const argument_t& x)
	const throw ()
  {
    LinearTrajectory traj (linTraj_);
    traj.setParameters (x);
    result = traj (t_);
  }
	
  virtual void
  impl_gradient (gradient_t& gradient,
								 const argument_t& x,
								 size_type functionId = 0)
	const throw ()
  {
    LinearTrajectory traj (linTraj_);
    traj.setParameters (x);
    gradient = row (traj.variationConfigWrtParam (t_), functionId);
  }
	
private:
  const LinearTrajectory& linTraj_;
  value_type t_;
};

//-----------------------------------------------------------------
/*!
 * Change the arguments of the Linear Trajectory
 */

//FIXME: defined_lc_in has to be true (false untested).
LinearTrajectory::LinearTrajectory (interval_t tr, size_type outputSize,
																		const vector_t& p,
																		std::string name)
throw ()
: Trajectory<3> (tr, outputSize, p, name),
nbp_ (p.size () / outputSize)
{
	//Parameter size should be a multiple of spline dimension
	//assert (parameters_.size () % outputSize == 0);
	
	// number of control points should be at least 4.
	//assert (nbp_ >= 4);
	
	setParameters (p);
}

LinearTrajectory::LinearTrajectory (const LinearTrajectory& linearInterpol) throw ()
: Trajectory<3> (linearInterpol.timeRange (), linearInterpol.outputSize (),
								 linearInterpol.parameters ()),
nbp_ (linearInterpol.parameters ().size () / linearInterpol.outputSize ())
{
	//Parameter size should be a multiple of spline dimension
	assert (parameters_.size () % outputSize () == 0);
	// number of control points should be at least 4.
	assert (nbp_ >= 5);
	
	setParameters (linearInterpol.parameters ());
}


LinearTrajectory::~LinearTrajectory () throw ()
{
}

void
LinearTrajectory::setParameters (const vector_t& p) throw ()
{
	assert (p.size () == parameters_.size ());
	parameters_ = p;
}

void
LinearTrajectory::impl_compute (result_t& derivative, double t) const throw ()
{
	t = detail::fixTime (t, *this);
	assert (timeRange ().first <= t && t <= timeRange ().second);
	this->derivative (derivative, t, 0);
}

LinearTrajectory::value_type
LinearTrajectory::Dt () const
{
	return length () / nbp_;
}

LinearTrajectory::size_type
LinearTrajectory::interval (value_type t) const
{
	size_type i = std::floor (t - getLowerBound (timeRange ())) / Dt ();
	if (i == nbp_)
		i--; 
	return i;
}

void
LinearTrajectory::impl_derivative (gradient_t& derivative, double t,
																	 size_type order)
const throw ()
{	
	const size_type n = outputSize ();
	derivative = boost::numeric::ublas::zero_vector<double>(nbp_*n);
}

void
LinearTrajectory::impl_derivative (gradient_t& derivative,
																	 StableTimePoint stp,
																	 size_type order) const throw ()
{
	this->impl_derivative (derivative,
												 stp.getTime (this->timeRange ()),
												 order);
}

LinearTrajectory::jacobian_t
LinearTrajectory::variationConfigWrtParam (double t) const throw ()
{
	return variationDerivWrtParam (t, 0.);
}

LinearTrajectory::jacobian_t
LinearTrajectory::variationDerivWrtParam (double t, size_type order)
const throw ()
{
	const size_t n = outputSize ();
	
	if (order != 0.) {
		jacobian_t jac = ublas::zero_matrix<double> (n, nbp_ * n);
		return jac;
	}
	
	LinearDerivWrtParameters  linearDerivWrtParameters(*this,t);
	FiniteDifferenceGradient<> fdfunction(linearDerivWrtParameters);
	
	jacobian_t jac = fdfunction.jacobian( this->parameters ());
	return jac;
}

LinearTrajectory::jacobian_t
LinearTrajectory::variationConfigWrtParam (StableTimePoint stp)
const throw ()
{
	return this->variationConfigWrtParam (stp.getTime (this->timeRange ()));
}


LinearTrajectory::jacobian_t
LinearTrajectory::variationDerivWrtParam (StableTimePoint stp, size_type order)
const throw ()
{
	return this->variationDerivWrtParam(stp.getTime (this->timeRange ()), order);
}


LinearTrajectory::value_type
LinearTrajectory::singularPointAtRank (size_type rank) const
{
	return rank * length () / (nbp_- 3);
}

LinearTrajectory::vector_t
LinearTrajectory::derivBeforeSingularPoint (size_type rank, size_type order) const
{
	return derivative (singularPointAtRank (rank), order);
}

LinearTrajectory::vector_t
LinearTrajectory::derivAfterSingularPoint (size_type rank, size_type order) const
{
	return derivative (singularPointAtRank (rank), order);
}

std::ostream&
LinearTrajectory::print (std::ostream& o) const throw ()
{
	o << "LinearTrajectory" << incindent << std::endl
	<< "Number of parameters per spline function: " << nbp_ << std::endl
	<< "Length: " << length () << std::endl
	<< "Parameters: " << parameters ()
	<< decindent;
	return o;
}

//----------------------------------------------------------------
/*!
 * make a roboptim linear traj from move3d
 */
LinearTrajectory* RoboptimFactory::make_Roboptim(API::Trajectory& traj)
{
	using namespace boost::numeric;

	m_Robot = traj.getRobot();
	const unsigned int outputSize = m_Robot->getNumberOfActiveDoF();
	
	//std::cerr << "RoboptimFactory::make_Roboptim => New LinearTrajectory Intervall" << std::endl;
	LinearTrajectory::interval_t timeRange;
	
	try
	{
			timeRange = LinearTrajectory::makeInterval (0., traj.getRangeMax());
	}
	catch (...)
	{
			std::cerr << "RoboptimFactory::make_Roboptim => Problem in trajecory construcor, makeInterval" << std::endl;
			return NULL;
	}

	//std::cerr << "RoboptimFactory::make_Roboptim => New param vector" << std::endl;
	ublas::vector<double> params( (traj.getNbPaths() + 1)*outputSize );
	
	try
	{
		std::tr1::shared_ptr<Configuration> q;
		
		for ( int i=-1; i< traj.getNbPaths() ; i++) 
		{
			if ( i == -1 ) 
			{
				q = traj.getLocalPathPtrAt(i+1)->getBegin();
			}
			else 
			{
				q = traj.getLocalPathPtrAt(i)->getEnd();
			}
			
			for ( int j=0; j< (int)outputSize; j++) 
			{
				params((i+1)*outputSize+j) = q->getActiveDoF(j);
				//std::cout << "i*outputSize+j = " << (i+1)*outputSize+j << " , " <<  params((i+1)*outputSize+j) << std::endl;
			}
		}
	}
	catch (...) {
		std::cerr << "RoboptimFactory::make_Roboptim => Problem in assigning the param vector" <<  std::endl;
	}
	
//	std::cout << "Number of control points : " << traj.getNbPaths() + 1 << std::endl;
//	std::cout << "Output Size : " << outputSize << std::endl;
//	std::cout << "params = " << params << std::endl;
	
	//std::cerr << "RoboptimFactory::make_Roboptim => New roboptim traj" << std::endl;
	LinearTrajectory* linearTraj;
	
	try
	{
		 linearTraj =  new LinearTrajectory( timeRange, outputSize, params , "Move3D_Traj");
	}
	catch (...)
	{
		std::cerr << "RoboptimFactory::make_Roboptim => Problem in trajecory construcor" << std::endl;
		linearTraj = NULL;
	}
	
	return linearTraj;
}

/*!
 * make a move 3d linear traj from roboptim
 */
API::Trajectory*	RoboptimFactory::make_Move3D(LinearTrajectory& traj)
{
	
	using namespace boost::numeric;
	API::Trajectory* optimTrajPt;
	
	try 
	{
		const int outputSize = traj.outputSize ();
		ublas::vector<double> params = traj.parameters ();
		
		std::vector< std::tr1::shared_ptr<Configuration> > vect_conf;
		
		//std::cout << "Number of control points : " << traj.getNumberOfControlPoints() << std::endl;
		//std::cout << "Output Size : " << outputSize << std::endl;
		//std::cout << "params = " << params << std::endl;

		for ( int i=0; i< (int)(traj.getNumberOfControlPoints()); i++) 
		{
			std::tr1::shared_ptr<Configuration> q ( new Configuration(m_Robot) );
			
			for ( int j=0; j< (int)outputSize; j++) 
			{
				//std::cout << "i*outputSize+j = " << i*outputSize+j <<  " , "  << params(i*outputSize+j)<< std::endl;
				q->setActiveDoF( j, params(i*outputSize+j) );
			}
			vect_conf.push_back(q);
		}
		
		optimTrajPt = new API::Trajectory( vect_conf );
	}
	catch (...) 
	{
		std::cerr << "Error in make RoboptimFactory::make_Move3D" << std::endl;
		optimTrajPt = NULL;
	}	
	
	return optimTrajPt;
}

//-----------------------------------------------------------------
// Linear Problem
CostMapFunction::CostMapFunction (Robot* R, unsigned int nbControlPoints) throw() :  
Function( R->getNumberOfActiveDoF()*nbControlPoints,1,"CostMapFunction") 
{
	m_Robot = R;
	m_nbControlPoints = nbControlPoints;
}

void CostMapFunction::impl_compute (result_t& r , const argument_t& a) const throw ()
{
	std::vector< std::tr1::shared_ptr< Configuration > > vect_conf;
	size_type outputSize = m_Robot->getNumberOfActiveDoF();
	
	if ( m_nbControlPoints*outputSize != a.size() ) 
	{
		std::cerr << "CostMapFunction::impl_compute problem" << std::endl;
	}
	
	for ( int i=0; i< (int)(m_nbControlPoints); i++) 
	{
		std::tr1::shared_ptr<Configuration> q ( new Configuration(m_Robot) );
		
		for ( int j=0; j< (int)outputSize; j++) 
		{
			//std::cout << "i*outputSize+j = " << i*outputSize+j <<  " , "  << params(i*outputSize+j)<< std::endl;
			q->setActiveDoF( j, a(i*outputSize+j) );
		}
		vect_conf.push_back(q);
	}
	
	API::Trajectory traj( vect_conf );
	double Cost = traj.cost();
	std::cout << "Cost is : "<< Cost << std::endl;
	r(0) = Cost;
}

//-----------------------------------------------------------------

typedef CFSQPSolver::problem_t::constraints_t constraint_t;
typedef CFSQPSolver solver_t;

int RoboptimTrajectory::run_CostMap()
{
	try  
	{
		Robot* robotPt = global_Project->getActiveScene()->getActiveRobot();
		API::Trajectory* traj = new API::Trajectory( robotPt, robotPt->getTrajStruct() );
		
		roboptim::RoboptimFactory factory;
		LinearTrajectory optimTraj = *factory.make_Roboptim(*traj);
		delete traj;
		
		// Define cost.
		CostMapFunction cost (robotPt, optimTraj.getNumberOfControlPoints() );
		
		FiniteDifferenceGradient<> fdCost(cost);
		
		// Create problem.
		solver_t::problem_t problem (fdCost);
		
		// Add bounds
		Function::intervals_t bounds;
		for (unsigned int i=0; i<optimTraj.getNumberOfControlPoints(); i++)
			bounds.push_back( Function::makeInterval(-45,45) );
		
		problem.argumentBounds () =  bounds ;
		problem.startingPoint () = optimTraj.parameters ();
		
		std::cerr << "Cost function (before): " << fdCost (optimTraj.parameters ()) << std::endl;
		
		std::vector<Function::size_type> indices;
		
		indices.push_back (0);
		indices.push_back (1);
		indices.push_back (2);
		indices.push_back (3);
		
		indices.push_back (optimTraj.parameters ().size ()  - 4);
		indices.push_back (optimTraj.parameters ().size ()  - 3);
		indices.push_back (optimTraj.parameters ().size ()  - 2);
		indices.push_back (optimTraj.parameters ().size ()  - 1);
		
		makeFreeze (problem) (indices, optimTraj.parameters () );
		
		// Initialize solver
		CFSQPSolver solver (problem);
		std::cout << solver << std::endl;
		
		try  
		{
			solver_t::result_t res = solver.minimum ();
			std::cerr << res << std::endl;
		}
		catch (...) 
		{
			std::cerr << "Problem in trajectory optimization" << std::endl;
		}
		
		try 
		{
			// Retreive result
			const Result& result = solver.getMinimum<Result> ();
			optimTraj.setParameters (result.x);
		}
		catch (...) 
		{
			std::cerr << "Problem in geting the result" << std::endl;
		}
		
		traj = factory.make_Move3D(optimTraj);
		traj->replaceP3dTraj();
		delete traj;
		
	}
	catch (...) {
		std::cout << "Can not inizialize Problem" << std::endl;
	}
	
	
	std::cout << "End Of CFSQPSolver" << std::endl;
	return 0;
}

//-----------------------------------------------------------------
// Problem parameters.
typedef FreeTimeTrajectory<CubicBSpline> freeTime_t;

const unsigned nControlPoints = 15;
const unsigned nConstraintsPerCtrlPts = 10;
const double vMax = 85.;

int RoboptimTrajectory::run_testManipulator()
{
	return 0;
}

int RoboptimTrajectory::run_test1 ()
{
	using namespace boost;
	using namespace boost::assign;
	
	const double finalPos = 200.;
	CubicBSpline::vector_t params (nControlPoints);
	
	params[0] = 0;
	params[1] = 0;
	
	for (unsigned i = 0; i < nControlPoints-4; ++i)
		params[i+2] = finalPos / (nControlPoints - 5) * i;
	
	params[nControlPoints-2] = finalPos;
	params[nControlPoints-1] = finalPos;
	
	// Make trajectories.
	CubicBSpline::interval_t timeRange = CubicBSpline::makeInterval (0., 4.);
	CubicBSpline spline (timeRange, 1, params, "before");
	freeTime_t freeTimeTraj (spline, 1.);
	
	// Define cost.
	Function::matrix_t a (1, freeTimeTraj.parameters ().size ());
	a.clear ();
	a (0, 0) = -1.;
	Function::vector_t b (1);
	b.clear ();
	NumericLinearFunction cost (a, b);
	
	// Create problem.
	solver_t::problem_t problem (cost);
	problem.startingPoint () = freeTimeTraj.parameters ();
	
	// Scale has to remain positive.
	problem.argumentBounds ()[0] = Function::makeLowerInterval (0.);
	
	const freeTime_t::vector_t freeTimeParams = freeTimeTraj.parameters ();
	
	std::vector<Function::size_type> indices;
	indices.push_back (1);
	indices.push_back (2);
	indices.push_back (3);
	indices.push_back (freeTimeParams.size () - 3);
	indices.push_back (freeTimeParams.size () - 2);
	indices.push_back (freeTimeParams.size () - 1);
	makeFreeze (problem) (indices, freeTimeParams);
	
	Function::interval_t vRange = Function::makeUpperInterval (.5 * vMax * vMax);
	LimitSpeed<FreeTimeTrajectory<CubicBSpline> >::addToProblem
	(freeTimeTraj, problem, vRange, nControlPoints * nConstraintsPerCtrlPts);
	
	std::ofstream limitSpeedStream ("limit-speed.gp");
	Gnuplot gnuplot = Gnuplot::make_interactive_gnuplot ();
	
	gnuplot
	<< set ("multiplot layout 1,2 title "
					"'variation of speed before and after optimization'")
	<< set ("grid");
	gnuplot << plot_limitSpeed (freeTimeTraj, vMax);
	
	
	// SolverFactory<solver_t> factory ("cfsqp", problem);
	// solver_t& solver = factory ();
	
	// Initialize solver
	CFSQPSolver solver (problem);
	
	std::cout << solver << std::endl;
	
	solver_t::result_t res = solver.minimum ();
	std::cerr << res << std::endl;
	
	FreeTimeTrajectory<CubicBSpline> optimizedTrajectory =
	freeTimeTraj;
	
	switch (solver.minimumType ())
	{
		case GenericSolver::SOLVER_VALUE:
		{
			const Result& result = solver.getMinimum<Result> ();
			optimizedTrajectory.setParameters (result.x);
			break;
		}
			
		case GenericSolver::SOLVER_VALUE_WARNINGS:
		{
			const ResultWithWarnings& result =
			solver.getMinimum<ResultWithWarnings> ();
			optimizedTrajectory.setParameters (result.x);
			break;
		}
			
		case GenericSolver::SOLVER_NO_SOLUTION:
		case GenericSolver::SOLVER_ERROR:
			return 1;
	}
	
	gnuplot << plot_limitSpeed (optimizedTrajectory, vMax);
	limitSpeedStream << (gnuplot << unset ("multiplot"));
	return 0;
}

void RoboptimTrajectory::showSpline (const CubicBSpline& spline)
{
	std::cout
	<< "# Values:" << std::endl
	<< "# " << normalize (spline (0.)) << std::endl
	<< "# " << normalize (spline (2.5)) << std::endl
	<< "# " << normalize (spline (4.)) << std::endl
	
	<< "# 1st derivative:" << std::endl
	<< "# " << normalize (spline.derivative (0., 1)) << std::endl
	<< "# " << normalize (spline.derivative (2.5, 1)) << std::endl
	<< "# " << normalize (spline.derivative (4., 1)) << std::endl
	
	<< "# 2nd derivative:" << std::endl
	<< "# " << normalize (spline.derivative (0., 2)) << std::endl
	<< "# " << normalize (spline.derivative (2.5, 2)) << std::endl
	<< "# " << normalize (spline.derivative (4., 2)) << std::endl
	
	<< "# variationConfigWrtParam:" << std::endl
	<< "# " << normalize (spline.variationConfigWrtParam (0.)) << std::endl
	<< "# " << normalize (spline.variationConfigWrtParam (2.5)) << std::endl
	<< "# " << normalize (spline.variationConfigWrtParam (4.)) << std::endl;
}

int RoboptimTrajectory::run_test2 ()
{
	using namespace boost::assign;
	CubicBSpline::vector_t params (16);
	
	// Initial position.
	params[0] = 0.,  params[1] = 0.;
	params[2] = 0.,  params[3] = 0.;
	params[4] = 0.,  params[5] = 0.;
	// Control point 3.
	params[6] = 25.,  params[7] = 100.;
	// Control point 4.
	params[8] = 75.,  params[9] = 0.;
	// Final position.
	params[10] = 100., params[11] = 100.;
	params[12] = 100., params[13] = 100.;
	params[14] = 100., params[15] = 100.;
	
	CubicBSpline::interval_t timeRange = CubicBSpline::makeInterval (0., 4.);
	
	CubicBSpline spline (timeRange, 2, params, "before");
	discreteInterval_t interval (0., 4., 0.01);
	
	showSpline (spline);
	
	Gnuplot gnuplot = Gnuplot::make_interactive_gnuplot ();
	gnuplot
	<< set ("multiplot layout 1,2")
	<< set ("grid")
	<< plot_xy (spline);
	
	// Optimize.
	SplineLength cost (spline);
	
	// Check cost gradient.
	try
	{
		Function::vector_t x (params.size ());
		x.clear ();
		checkGradientAndThrow (cost, 0, x, 2e-3);
		
		x = params;
		checkGradientAndThrow (cost, 0, x, 2e-3);
	}
	catch (BadGradient& bg)
	{
		std::cerr << bg << std::endl;
		return 1;
	}
	
	solver_t::problem_t problem (cost);
	problem.startingPoint () = params;
	
	//	spline.freezeCurveStart (problem);
	//	spline.freezeCurveEnd (problem);
	
	//	SolverFactory<solver_t> factory ("cfsqp", problem);
	//	solver_t& solver = factory ();
	
	CFSQPSolver solver (problem);
	
	std::cerr << "Cost function (before): " << cost (params) << std::endl;
	std::cerr << "Parameters (before): " << params << std::endl;
	
	std::cerr << solver << std::endl;
	
	solver_t::result_t res = solver.minimum ();
	
	switch (res.which ())
	{
		case GenericSolver::SOLVER_VALUE:
		{
			Result& result = boost::get<Result> (res);
			CubicBSpline optimizedSpline (timeRange, 2, result.x, "after");
			showSpline (optimizedSpline);
			params = result.x;
			gnuplot << plot_xy (optimizedSpline);
			break;
		}
			
		case GenericSolver::SOLVER_NO_SOLUTION:
		{
			std::cerr << "No solution" << std::endl;
			return 1;
		}
		case GenericSolver::SOLVER_VALUE_WARNINGS:
		{
			ResultWithWarnings& result = boost::get<ResultWithWarnings> (res);
			CubicBSpline optimizedSpline (timeRange, 2, result.x, "after");
			showSpline (optimizedSpline);
			params = result.x;
			std::cerr << result << std::endl;
			//gnuplot << plot_xy (optimizedSpline);
			break;
		}
			
		case GenericSolver::SOLVER_ERROR:
		{
			SolverError& result = boost::get<SolverError> (res);
			std::cerr << result << std::endl;
			return 1;
		}
	}
	
	std::cerr << "Parameters (after): " << params << std::endl;
	
	// Check cost gradient (final).
	try
	{
		checkGradientAndThrow (cost, 0, params, 2e-3);
	}
	catch (BadGradient& bg)
	{
		std::cerr << bg << std::endl;
		return 1;
	}
	
	std::cout << (gnuplot << unset ("multiplot"));
	return 0;
}

/*
 const unsigned int m = 1;
 const unsigned int n = 4;
 
 struct F : public TwiceDerivableFunction
 {
 F () : TwiceDerivableFunction (4, 1)
 {
 }
 
 void
 impl_compute (result_t& result, const argument_t& x) const throw ()
 {
 vector_t res (m);
 res (0) = x[0] * x[3] * (x[0] + x[1] + x[2]) + x[3];
 return res;
 }
 
 void
 impl_gradient (gradient_t& grad, const argument_t& x, int) const throw ()
 {
 gradient_t grad (n);
 
 grad[0] = x[0] * x[3] + x[3] * (x[0] + x[1] + x[2]);
 grad[1] = x[0] * x[3];
 grad[2] = x[0] * x[3] + 1;
 grad[3] = x[0] * (x[0] + x[1] + x[2]);
 return grad;
 }
 
 void
 impl_hessian (hessian_t& h, const argument_t& x, int) const throw ()
 
 {
 matrix_t h (n, n);
 h (0, 0) = 2 * x[3];
 h (0, 1) = x[3];
 h (0, 2) = x[3];
 h (0, 3) = 2 * x[0] + x[1] + x[2];
 
 h (1, 0) = x[3];
 h (1, 1) = 0.;
 h (1, 2) = 0.;
 h (1, 3) = x[0];
 
 h (2, 0) = x[3];
 h (2, 1) = 0.;
 h (2, 2) = 0.;
 h (2, 3) = x[1];
 
 h (3, 0) = 2 * x[0] + x[1] + x[2];
 h (3, 1) = x[0];
 h (3, 2) = x[0];
 h (3, 3) = 0.;
 return h;
 }
 };
 
 struct G0 : public TwiceDerivableFunction
 {
 G0 ()
 : TwiceDerivableFunction (4, 1)
 {
 }
 
 void
 impl_compute (result_t& result, const argument_t& x) const throw ()
 {
 vector_t res (m);
 res (0) = x[0] * x[1] * x[2] * x[3];
 //return res;
 }
 
 void
 impl_gradient (gradient_t& grad, const argument_t& x, int) const throw ()
 {
 gradient_t grad (n);
 
 grad[0] = x[1] * x[2] * x[3];
 grad[1] = x[0] * x[2] * x[3];
 grad[2] = x[0] * x[1] * x[3];
 grad[3] = x[0] * x[1] * x[2];
 //return grad;
 }
 
 void
 impl_hessian (hessian_t& h, const argument_t& x, int) const throw ()
 {
 //matrix_t h (n, n);
 h (0, 0) = 0.;
 h (0, 1) = x[2] * x[3];
 h (0, 2) = x[1] * x[3];
 h (0, 3) = x[1] * x[2];
 
 h (1, 0) = x[2] * x[3];
 h (1, 1) = 0.;
 h (1, 2) = x[0] * x[3];
 h (1, 3) = x[0] * x[2];
 
 h (2, 0) = x[1] * x[3];
 h (2, 1) = x[0] * x[3];
 h (2, 2) = 0.;
 h (2, 3) = x[0] * x[1];
 
 h (3, 0) = x[1] * x[2];
 h (3, 1) = x[0] * x[2];
 h (3, 2) = x[0] * x[1];
 h (3, 3) = 0.;
 //return h;
 }
 };
 
 struct G1 : public TwiceDerivableFunction
 {
 G1 ()
 : TwiceDerivableFunction (4, 1)
 {
 }
 
 void
 impl_compute (result_t& result, const argument_t& x) const throw ()
 {
 vector_t res (m);
 res (0) = x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3];
 //return res;
 }
 
 void
 impl_gradient (gradient_t& grad, const argument_t& x, int) const throw ()
 {
 gradient_t grad (n);
 
 grad[0] = 2 * x[0];
 grad[1] = 2 * x[1];
 grad[2] = 2 * x[2];
 grad[3] = 2 * x[3];
 //return grad;
 }
 
 void
 impl_hessian (hessian_t& h, const argument_t& x, int) const throw ()
 {
 matrix_t h (n, n);
 h (0, 0) = 2.;
 h (0, 1) = 0.;
 h (0, 2) = 0.;
 h (0, 3) = 0.;
 
 h (1, 0) = 0.;
 h (1, 1) = 2.;
 h (1, 2) = 0.;
 h (1, 3) = 0.;
 
 h (2, 0) = 0.;
 h (2, 1) = 0.;
 h (2, 2) = 2.;
 h (2, 3) = 0.;
 
 h (3, 0) = 0.;
 h (3, 1) = 0.;
 h (3, 2) = 0.;
 h (3, 3) = 2.;
 // return h;
 }
 };
 
 void run_test() {
 F f;
 G0 g0;
 G1 g1;
 
 CFSQPSolver::problem_t pb (f);
 
 // Set bound for all variables.
 // 1. < x_i < 5. (x_i in [1.;5.])
 for (Function::size_type i = 0; i < pb.function ().n; ++i)
 pb.argBounds ()[i] = T::makeBound (1., 5.);
 
 // Add constraints.
 pb.addConstraint (&g0, T::makeUpperBound (25.));
 pb.addConstraint (&g1, T::makeBound (40., 40.));
 
 // Set the starting point.
 Function::vector_t start (pb.function ().n);
 start[0] = 1., start[1] = 5., start[2] = 5., start[3] = 1.;
 
 initialize_problem (pb, g0, g1);
 
 // Initialize solver
 CFSQPSolver solver (pb);
 
 // Compute the minimum and retrieve the result.
 CFSQPSolver::result_t res = solver.minimum ();
 
 // Display solver information.
 std::cout << solver << std::endl;
 
 // Check if the minimization has succeed.
 switch (solver.minimumType ())
 {
 case SOLVER_NO_SOLUTION:
 std::cerr << "No solution." << std::endl;
 return 1;
 case SOLVER_ERROR:
 std::cerr << "An error happened: "
 << solver.getMinimum<SolverError> ().what () << std::endl;
 return 2;
 
 case SOLVER_VALUE_WARNINGS:
 {
 // Get the ``real'' result.
 Result& result = solver.getMinimum<ResultWithWarnings> ();
 // Display the result.
 std::cout << "A solution has been found (minor problems occurred): "
 << std::endl
 << result << std::endl;
 return 0;
 }
 case SOLVER_VALUE:
 {
 // Get the ``real'' result.
 Result& result = solver.getMinimum<Result> ();
 // Display the result.
 std::cout << "A solution has been found: " << std::endl;
 std::cout << result << std::endl;
 return 0;
 }
 }
 
 // Should never happen.
 assert (0);
 return 42;
 }
 //GENERATE_TEST ()*/