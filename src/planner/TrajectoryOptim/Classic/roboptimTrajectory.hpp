/*
 *  RoboptimTrajectory.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 22/06/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef ROBOPTIMTRAJ_H
#define ROBOPTIMTRAJ_H

#ifdef Q_FOREACH
#undef Q_FOREACH
#endif

#include <roboptim/trajectory/trajectory.hh>
#include <roboptim/trajectory/cubic-b-spline.hh>

#include "API/Trajectory/trajectory.hpp"

/*!
 * Roboptim trajectory
 */
namespace roboptim
{
	class LinearTrajectory : public Trajectory<3> // Fix me should be one
	{
	public:
		// dimension N pour manip
		// parameters (dimension,NbPointControl)
		LinearTrajectory (interval_t timeRange, size_type dimension,
					  const vector_t& parameters,
					  const std::string name = "Linear Trajectory") throw ();
		
		/// \brief Copy constructor.
		/// \param spline spline that will be copied
		LinearTrajectory (const LinearTrajectory& linearInterpol) throw ();
		
		virtual ~LinearTrajectory () throw ();
		
		// returns the number of control points
		unsigned int getNumberOfControlPoints() { return nbp_; }
		
		/// \brief Modify spline parameters.
		virtual void setParameters (const vector_t&) throw ();
		
		// ds/dp
		virtual jacobian_t variationConfigWrtParam (double t) const throw ();
		virtual jacobian_t variationDerivWrtParam (double t, size_type order) const throw ();
		
		// Autant de points singuliers que de points de control
		virtual value_type singularPointAtRank (size_type rank) const;
		virtual vector_t derivBeforeSingularPoint (size_type rank, size_type order) const;
		virtual vector_t derivAfterSingularPoint (size_type rank, size_type order) const;
		
		ROBOPTIM_IMPLEMENT_CLONE(LinearTrajectory)
		
		virtual Trajectory<derivabilityOrder>* resize (interval_t timeRange)
		const throw ()
		{
			return new LinearTrajectory (timeRange, this->outputSize (), this->parameters ());
		}
		
		/// \brief Display the function on the specified output stream.
		///
		/// \param o output stream used for display
		/// \return output stream
		virtual std::ostream& print (std::ostream& o) const throw ();
		
		jacobian_t variationConfigWrtParam (StableTimePoint tp) const throw ();
		jacobian_t variationDerivWrtParam (StableTimePoint tp, size_type order)
		const throw ();
		
	protected:
		void impl_compute (result_t& r, double t) const throw ();
		void impl_derivative (gradient_t& g, double x, size_type order) const throw ();
		void impl_derivative (gradient_t& g, StableTimePoint, size_type order) const throw ();
		
		value_type Dt () const;
		size_type interval (value_type t) const;
		vector_t basisFunctions (value_type t, size_type order) const;
		
	private:
		/// \brief Number of control points.
		unsigned int nbp_;
	};
	
	/*!
	 * Cost Map Function
	 */
	class CostMapFunction : public Function
	{
	public:
    /// \brief Build a linear function from a matrix and a vector.                                                                                                                                                                                                                                                               
		CostMapFunction (Robot* R, unsigned int nbControlPoints) throw();
		
		// print the functions
    virtual std::ostream& print (std::ostream& s) const throw () { return s; }
		
  protected:
		// compute the cost of one solution
    void impl_compute (result_t& r , const argument_t& a) const throw ();
		
	private:
		// The Robot
		Robot* m_Robot;
		
		// Nombre de points de controls
		unsigned int m_nbControlPoints;
	};
	
	/*!
	 * Roboptim trajectory
	 */
	class RoboptimFactory
	{
	public:
		RoboptimFactory() {}
		
		LinearTrajectory* make_Roboptim(API::Trajectory& traj);
		API::Trajectory*	make_Move3D(LinearTrajectory& traj);
		
	private:
		Robot* m_Robot;
	};
	
	/*!
	 * Roboptim trajectory
	 */
	class RoboptimTrajectory
	{
	public:
		RoboptimTrajectory() {}
		
		void showSpline (const CubicBSpline& spline);
		
		int run_CostMap();
		int run_testManipulator();
		int run_test1();
		int run_test2();
	};
}

#endif