/*
 *  Distance.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 05/12/09.
 *  Copyright 2009 LAAS/CNRS. All rights reserved.
 *
 */
#include "HRICS_costspace.hpp"
#include "HRICS_Distance.hpp"

#include "P3d-pkg.h"
#include "Collision-pkg.h"

using namespace std;
using namespace tr1;
using namespace HRICS;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

/*!
 * Constructor that sets the Human and Robot structures
 */
Distance::Distance()
{
  Scene* sc =  global_Project->getActiveScene();
  
  m_Robot = sc->getRobotByNameContaining( global_ActiveRobotName );
  
  _Humans.push_back( sc->getRobotByNameContaining( "HUMAN" ) );
  
	cout  << "HRICS::Distance with " << " Robot " << m_Robot->getName() 
        << " and human " << _Humans[0]->getName() << endl;
  
	_SafeRadius = 0;
  m_InteractionRadius = 1.5;
}


/*!
 * Constructor that is given the
 * human and the robot
 */
Distance::Distance(Robot* rob, vector<Robot*> humans) :
m_Robot(rob),
_Humans(humans)
{
	string robName( m_Robot ? m_Robot->getName() : "No Robot" );
	
	cout << "HRICS::Distance with " << robName << " as robot and human " 
	<< _Humans[0]->getName() << endl;
	_SafeRadius = 0;
  m_InteractionRadius = 1.5;
}

/*!
 * Destructor sets back the safety
 * radius
 */
Distance::~Distance()
{
	cout << "Delete Distance" << endl;
	for(unsigned int j=0; j<_Humans.size(); j++)
	{
		for(int i =0; i<_Humans[j]->getRobotStruct()->no; i++)
		{
			string body = _Humans[j]->getRobotStruct()->o[i]->name;
			
			if( body.find("safety_zone_graphic") != string::npos )
			{
				//                cout << "i = " << i << endl;
				for(int k=0;k< _Humans[j]->getRobotStruct()->o[i]->np ; k++)
				{
					//                    cout << "_Humans[j]->getRobotStruct()->o[i]->np = " << _Humans[j]->getRobotStruct()->o[i]->np << endl;
					/* the shape : 0,1,2=polyh,
					 * 3=sphere, 4=cube,
					 * 5=box, 6=cylinder,
					 * 7=cone */
					
					int shape = _Humans[j]->getRobotStruct()->o[i]->pol[k]->entity_type;
					
					if( shape==0 || shape==1 || shape == 2)
					{
						//                        cout << "Shape is oval cylinder" << endl;
						break;
					}
					
					offSetPrim(_Humans[j]->getRobotStruct()->o[i]->pol[k],-_SafeRadius);
					//                    cout << "Set object " << i <<" with offset " << -_SafeRadius << endl;
				}
			}
		}
	}
}

/*!
 * Goes through the Human (p3d_rob) structure 
 * to find the safety zones
 */
void Distance::parseHumans()
{
	string body;
	string b_name;
	string buffer;
  
  vector<string> AchileActivePartsName;
  
  AchileActivePartsName.push_back( "headGhost" );
	AchileActivePartsName.push_back( "pelvisGhost" );
  AchileActivePartsName.push_back( "rFemurGhost" );
  AchileActivePartsName.push_back( "lFemurGhost" );
  
	//_SafeRadius = 0;
	ENV.setDouble(Env::zone_size,0.8);
  
	_SafetyZonesBodyId.clear();
	
	for(unsigned int i=0;i<_Humans.size();i++)
	{
		Robot* human = _Humans[i];
		vector<int> safetyZonesId;
		
		//        cout << "Looking for important zones on " << human->getName() << endl;
		for(int j=0;j<human->getRobotStruct()->no;j++)
		{
			body = human->getRobotStruct()->o[j]->name;
			//size_t found;
			b_name = body;
      
      bool isActiveAchilePartFound = false;
      
      for (unsigned int i=0; i<AchileActivePartsName.size(); i++) 
      {
        if( body.find( AchileActivePartsName[i] ) != string::npos)
        {
          safetyZonesId.push_back(j);
          isActiveAchilePartFound = true;
          break;
        }
      }
      
      if (isActiveAchilePartFound) 
      {
        continue;
      }
      
      if( body.find("safety_zone_ghost") != string::npos)
			{
				safetyZonesId.push_back(j);
				continue;
			}
			
			if( body.find("safety_zone_ghost") != string::npos)
			{
				safetyZonesId.push_back(j);
				continue;
			}
			
			if( body.find("safety_zone_graphic") != string::npos)
			{
				for(int k=0;k< human->getRobotStruct()->o[j]->np ; k++)
				{
					//                    cout << "j = " << j << endl;
					//                    cout << "human->getRobotStruct()->o[j]->np = " <<human->getRobotStruct()->o[j]->np << endl;
					/* the shape : 0,1,2=polyh,
					 * 3=sphere, 4=cube,
					 * 5=box, 6=cylinder,
					 * 7=cone */
					
					int shape = human->getRobotStruct()->o[j]->pol[k]->entity_type;
					
					if( shape==0 || shape==1 || shape == 2)
					{
						//                        cout << "Shape is oval cylinder" << endl;
						break;
					}
					
					if(shape==3){
						//                        cout << "Shape is sphere" << endl;
					}
					if(shape==4){
						//                        cout << "Shape is cube" << endl;
					}
					if(shape==5){
						//                        cout << "Shape is box" << endl;
					}
					if(shape==6){
						//                        cout << "Shape is cylinder" <<endl;
					}
					if(shape==7){
						//                        cout << "Shape is cone" << endl;
					}
					
					_SafeOffset = ENV.getDouble(Env::zone_size) - _SafeRadius;
					offSetPrim(human->getRobotStruct()->o[j]->pol[k],_SafeOffset);
					//                    cout << "offSetPrim on " << body << endl;
				}
			}
		}
		_SafetyZonesBodyId.push_back(safetyZonesId);
	}
	_SafeRadius = ENV.getDouble(Env::zone_size);
	activateNormalMode();
}

/*!
 * Changes dynamically the size of the zone shown
 * in the OpenGl display
 */
void Distance::offSetPrim(p3d_poly* poly,double offset)
{
	// p3d_scale_poly(poly->poly, scale, scale, scale);
	// polyhedre.h
	//	p3d_poly *poly;
	
	if (poly->primitive_data)
	{
		poly->primitive_data->radius += offset;
		poly->primitive_data->other_radius += offset;
		poly->primitive_data->height += (2*offset);
		poly->primitive_data->x_length += (2*offset);
		poly->primitive_data->y_length += (2*offset);
		poly->primitive_data->z_length += (2*offset);
	}
	
	if(poly->box.x1 > 0)
		poly->box.x1 += offset;
	else
		poly->box.x1 -= offset;
	
	if(poly->box.x2 > 0)
		poly->box.x2 += offset;
	else
		poly->box.x2 -= offset;
	
	if(poly->box.y1 > 0)
		poly->box.y1 += offset;
	else
		poly->box.y1 -= offset;
	
	if(poly->box.y2 > 0)
		poly->box.y2 += offset;
	else
		poly->box.y2 -= offset;
	
	if(poly->box.z1 > 0)
		poly->box.z1 += offset;
	else
		poly->box.z1 -= offset;
	
	if(poly->box.z2 > 0)
		poly->box.z2 += offset;
	else
		poly->box.z2 -= offset;
}

double Distance::getDistance()
{
	
	return 0;
}

/*!
 * Changes the collision context 
 * to compute distance only to the 
 * pritimives that represent the human
 */
void Distance::activateSafetyZonesMode()
{
	//sort(zone_id.begin(), zone_id.end());
	
	for(unsigned int j=0; j<_Humans.size(); j++)
	{
		//        cout << _Robot->getRobotStruct() << endl;
		
		p3d_col_activate_rob_rob(m_Robot->getRobotStruct(),_Humans[j]->getRobotStruct());
		
		for(int i =0; i<_Humans[j]->getRobotStruct()->no; i++)
		{
			if(binary_search(_SafetyZonesBodyId[j].begin(), _SafetyZonesBodyId[j].end(),i))
			{
				p3d_col_activate_rob_obj(m_Robot->getRobotStruct(),
																 _Humans[j]->getRobotStruct()->o[i]);
			}
			else
			{
				p3d_col_deactivate_rob_obj(m_Robot->getRobotStruct()
																	 ,_Humans[j]->getRobotStruct()->o[i]);
			}
		}
	}
}

/*!
 * Changes the collision context 
 * to compute the CD with the finer representation of the Human
 */
void Distance::activateNormalMode()
{
	for(unsigned int j=0; j<_Humans.size(); j++)
	{
		if (m_Robot)
		{
			// Activate robot to human collision checking
			p3d_col_activate_rob_rob(m_Robot->getRobotStruct(),
															 _Humans[j]->getRobotStruct());
		}
		
		// Deactivate all safety gosts self collision
		for ( vector<int>::iterator it  = _SafetyZonesBodyId[j].begin(); 
																it != _SafetyZonesBodyId[j].end(); ++it ) 
		{
			//  p3d_col_deactivate_rob_obj(_Robot->getRobotStruct(),
			//                             _Humans[j]->getRobotStruct()->o[i]);
			
			p3d_obj* obj =	_Humans[j]->getRobotStruct()->o[(*it)];
			
			for (int ib = 0; ib < _Humans[j]->getRobotStruct()->no; ib++) 
			{
				p3d_obj *o = _Humans[j]->getRobotStruct()->o[ib];
				if (o == obj) continue;
				p3d_col_deactivate_obj_obj(o, obj);
			}
		}
	}
}

double  Distance::computeCost(double distance)
{
	_PenetrationDist.resize(1);
	
	_PenetrationDist[0] = (_SafeRadius - distance)/_SafeRadius;
	
	double Cost = 0.00001;
	// Compute of the hri cost function
	if ( _PenetrationDist[0] > 0 )
	{
		Cost += (exp(_PenetrationDist[0]-1) - exp(-1) ) / ( 1 - exp(-1) );
		//            Cost += _PenetrationDist[k];
	}
	
	return Cost;
}

/*!
 * Get Workspace Cost 
 */
double Distance::getWorkspaceCost(const Eigen::Vector3d& WSPoint)
{
	p3d_vector3 body;
	p3d_vector3 other;
	
	double distance = computeBoundingBalls(WSPoint,body, other);
	
	return computeCost(distance);
}

/*!
 * Main function computing the distance
 * between the Human and the Robot
 */
vector<double> Distance::getDistToZones()
{
	//    activateSafetyZonesMode();
	
	int nof_bodies = m_Robot->getRobotStruct()->no;
	double* distances= MY_ALLOC(double,nof_bodies);
	
	p3d_vector3 *body= MY_ALLOC(p3d_vector3,nof_bodies);
	p3d_vector3 *other= MY_ALLOC(p3d_vector3,nof_bodies);
	
	int k=0;
	_PenetrationDist.resize(nof_bodies);
	
	/************************************************************/
	// Dist Type
	if (ENV.getBool(Env::useBallDist)) 
	{
		m_Method = Balls;
	} 
	else if (ENV.getBool(Env::useBoxDist)) 
	{
		m_Method = Boxes;
	} 
	else
	{
		m_Method = Full;
	}
	
	switch (m_Method) 
	{
		case Balls:
		{
			Vector3d WSPoint = m_Robot->getJointPos(ENV.getInt(Env::akinJntId));
			distances[k] = computeBoundingBalls(WSPoint,body[k], other[k]);
			_PenetrationDist[k] = (_SafeRadius - distances[k])/_SafeRadius;
			break;
		}
			
			
		case Boxes:
      //cout << "Boxes Distance" << endl;
			activateSafetyZonesMode();
			distances[k] = computeBBDist(body[k], other[k]);
			_PenetrationDist[k] = (_SafeRadius - distances[k])/_SafeRadius;
			activateNormalMode();
			break;
			
			
		case Full:
			switch (p3d_col_get_mode())
		{
			case p3d_col_mode_kcd:
			{
        activateSafetyZonesMode();
				int settings = get_kcd_which_test();
				set_kcd_which_test((p3d_type_col_choice)(20+3));
				// 40 = KCD_ROB_ENV
				// 3 = DISTANCE_EXACT
				
				p3d_col_test_choice();
				// Collision detection with other robots only
				
				p3d_kcd_closest_points_between_bodies(m_Robot->getRobotStruct(),body,other,distances);
				
				double buffer = -numeric_limits<double>::max();
				//double radius = ENV.getDouble(Env::zone_size);
				
				for(int i=0;i<nof_bodies;i++)
				{
					//        cout << "distance["<<i<<"] = "<< distances[i] << endl;
					_PenetrationDist[i] = (_SafeRadius - distances[i])/_SafeRadius;
					
					if(buffer<_PenetrationDist[i])
					{
						buffer = _PenetrationDist[i];
						k = i;
					}
				}
				
				set_kcd_which_test((p3d_type_col_choice)settings);// ROB_ALL + BOOL
        activateNormalMode();
			}
				break;
				
			case p3d_col_mode_pqp:
			{
        cout << "PQP distance = " << endl;
				distances[k] =  pqp_robot_robot_distance(
																								 m_Robot->getRobotStruct(),
																								 _Humans[k]->getRobotStruct(),
																								 body[k],
																								 other[k]);
				
				_PenetrationDist[k] = (_SafeRadius - distances[k])/_SafeRadius;
			}
				break;

			default:
				break;
		}
			break;
			
		default:
			break;
	}
	//    cout << " k = " << k << endl;
	
	/* ----------------------------------------------------
	 * Vecteur de distance aux zones HRI
	 **/
	mClosestPointToHuman[0] = body[k][0];
	mClosestPointToHuman[1] = body[k][1];
	mClosestPointToHuman[2] = body[k][2];
	
	if(ENV.getBool(Env::drawDistance))
	{
		vect_jim.clear();
		vect_jim.push_back(body[k][0]);
		vect_jim.push_back(body[k][1]);
		vect_jim.push_back(body[k][2]);
		vect_jim.push_back(other[k][0]);
		vect_jim.push_back(other[k][1]);
		vect_jim.push_back(other[k][2]);		
		cout << "vect_jim[0] = " << vect_jim[0] << " vect_jim[1] = " << vect_jim[1] << " vect_jim[2] = " << vect_jim[2] << endl;
		cout << "vect_jim[3] = " << vect_jim[3] << " vect_jim[4] = " << vect_jim[4] << " vect_jim[5] = " << vect_jim[5] << endl;
	}
	
	
	delete(distances);
	delete(body);
	delete(other);
	
	double Cost = 0.00001;
	// Compute of the hri cost function
	if ( _PenetrationDist[k] > 0 )
	{
		Cost += (exp(_PenetrationDist[k]-1) - exp(-1) ) / ( 1 - exp(-1) );
		//            Cost += _PenetrationDist[k];
	}
	
	vector<double> distCost;
	distCost.push_back( Cost );
	//    cout << "_PenetrationDist = " << _PenetrationDist[0] << endl;
	//    cout << "distCost = " << distCost[0] << endl;
	return distCost;
}

/*!
 * Bounding Box Distance
 */
double Distance::computeBBDist(p3d_vector3 robot, p3d_vector3 human)
{
	double minDistance1Prev = numeric_limits<double>::max();
	//    double minDistance2Prev = numeric_limits<double>::max();
	//    int nbBBTests =0;
	
	//    double tu,ts;
	//    ChronoOn();
	
  //cout << "Distance for Robot = " << m_Robot->getName() << endl;
  
	for(unsigned int i=0; i<_Humans.size(); i++)
	{
		//cout << "Distance for Human = " << _Humans[i]->getName() << endl;
    
		p3d_col_activate_rob_rob(m_Robot->getRobotStruct(),_Humans[i]->getRobotStruct());
		
		for(unsigned int j =0; j<_SafetyZonesBodyId[i].size();j++)
		{
			p3d_obj* Obj2= _Humans[i]->getRobotStruct()->o[_SafetyZonesBodyId[i][j]];
			
			bool pureGraphic = true;
			
      // Is object pure graphic
			for(int k =0; k<m_Robot->getRobotStruct()->no; k++)
			{
				for(int l=0; l<m_Robot->getRobotStruct()->o[k]->np;l++)
				{
					if(m_Robot->getRobotStruct()->o[k]->pol[l]->TYPE!=P3D_GRAPHIC)
					{
						pureGraphic = false;
					}
				}
				
				if(!pureGraphic)
				{
					p3d_obj* Obj1= m_Robot->getRobotStruct()->o[k];
					
					double minDistance2;
					double minDistance1 = p3d_BB_obj_obj_extern_dist ( Obj1 , Obj2, &minDistance2 );
          
          //cout << "minDistance1 = " << minDistance1 << endl;
					
					if(minDistance1 < minDistance1Prev)
					{
						minDistance1Prev = minDistance1;
						
						robot[0] = (Obj1->BB.xmax +  Obj1->BB.xmin)/2;
						robot[1] = (Obj1->BB.ymax +  Obj1->BB.ymin)/2;
						robot[2] = (Obj1->BB.zmax +  Obj1->BB.zmin)/2;
						
						human[0] = (Obj2->BB.xmax +  Obj2->BB.xmin)/2;
						human[1] = (Obj2->BB.ymax +  Obj2->BB.ymin)/2;
						human[2] = (Obj2->BB.zmax +  Obj2->BB.zmin)/2;
					}
				}
			}
		}
	}
	
//#if defined(LIGHT_PLANNER) && defined(PQP)
//	if(m_Robot->getRobotStruct()->isCarryingObject)
//	{
//		//        if(_Robot->getRobotStruct()->carriedObject->joints[1]->o > 1)
//		//        {
//		//            cout << "Warning Distance::computeBBDist => FF with more than 1 object"  << endl;
//		//        }
//		p3d_obj* Obj1 = m_Robot->getRobotStruct()->carriedObject->joints[1]->o;
//		
//		for( unsigned int i=0; i<_Humans.size(); i++ )
//		{
//			for( unsigned int j =0; j<_SafetyZonesBodyId[i].size();j++)
//			{
//				p3d_obj* Obj2= _Humans[i]->getRobotStruct()->o[_SafetyZonesBodyId[i][j]];
//				
//				double minDistance2;
//				double minDistance1 = p3d_BB_obj_obj_extern_dist ( Obj1 , Obj2, &minDistance2 );
//				
//				if(minDistance1 < minDistance1Prev)
//				{
//					minDistance1Prev = minDistance1;
//					
//					robot[0] = (Obj1->BB.xmax +  Obj1->BB.xmin)/2;
//					robot[1] = (Obj1->BB.ymax +  Obj1->BB.ymin)/2;
//					robot[2] = (Obj1->BB.zmax +  Obj1->BB.zmin)/2;
//					
//					human[0] = (Obj2->BB.xmax +  Obj2->BB.xmin)/2;
//					human[1] = (Obj2->BB.ymax +  Obj2->BB.ymin)/2;
//					human[2] = (Obj2->BB.zmax +  Obj2->BB.zmin)/2;
//				}
//			}
//		}
//	}
//#endif
	
	//    ChronoMicroTimes(&tu, &ts);
	//    ChronoOff();
	//    cout << "tu = " << tu << endl;
	
	return minDistance1Prev;
}

/*!
 * Computes the cylinder associated
 * with the human (body)
 */
bool Distance::computeCylinderZone(Vector3d &p1, Vector3d& p2)
{
	// First compute the human segment
	unsigned int ObjIndex;
	bool noZone = true;
	
/*	for(unsigned int i=0; i<_Humans.size(); i++)
	{
		// Checks that the safety zone exists
		if (_SafetyZonesBodyId[i].empty()) 
		{
			//cout << "Distance::computeCylinderZone::NoZones" << endl;
			noZone = true;
			break;
		}
		
		for(unsigned int j =0; j<_SafetyZonesBodyId[i].size();j++)
		{
			p3d_obj* Obj = _Humans[i]->getRobotStruct()->o[_SafetyZonesBodyId[i][j]];
			string ObjectName(Obj->name);
			//cout << ObjectName << endl;
			if ( ObjectName.find("safety_zone_ghost_0") != string::npos ) 
			{
				ObjIndex = _SafetyZonesBodyId[i][j];
				break;
			}
		}
	} */
	
	const double TopAndBottom = 1.0; // The top to bottom body is 1 meter
	
	if( noZone )
	{
		Vector3d Trans = _Humans[0]->getJoint(1)->getVectorPos();
		Vector3d Zaxis = _Humans[0]->getJoint(1)->getMatrixPos().linear().col(2);
		
		p1 =  TopAndBottom*Zaxis + Trans;
		p2 = -TopAndBottom*Zaxis + Trans;
	}
	else 
	{
		p3d_vector3 v; p3d_mat4ExtractTrans ( _Humans[0]->getRobotStruct()->o[ObjIndex]->jnt->abs_pos,v);
		p3d_matrix4* m = &(_Humans[0]->getRobotStruct()->o[ObjIndex]->jnt->abs_pos);
		
		p1(0) = (*m)[0][2]*(TopAndBottom)	+ v[0];
		p1(1) = (*m)[1][2]*(TopAndBottom)	+ v[1];
		p1(2) = (*m)[2][2]*(TopAndBottom)	+ v[2];
		
		p2(0) = (*m)[0][2]*(-TopAndBottom)	+ v[0];
		p2(1) = (*m)[1][2]*(-TopAndBottom)	+ v[1];
		p2(2) = (*m)[2][2]*(-TopAndBottom)	+ v[2];
		
	}
	
	return true;
}


//! Computes the closest point of a point to a segment line.
//! \param p the point
//! \param p1 the first point of the segment line
//! \param p2 the second point of the segment line
//! \param closestPoint the point on the segment that is the closest to p
//! \return the minimimal distance between the segment and the point
double Distance::pointToLineSegmentDistance(const Vector3d& p, const Vector3d& p1, const Vector3d& p2, Vector3d& closestPoint)
{
	double alpha, d1, d2, norm_p1p2;
	Vector3d p1p2, p1p2_n, p1p, proj;
	
	// Precompute the norm and the dot product
	p1p2			= p2 - p1;
	norm_p1p2	= p1p2.norm();
	p1p2_n		= p1p2.normalized();
	
	p1p			= p - p1;
	alpha		= p1p.dot(p1p2_n);
	
	// the projection is on the segment -> it is the closest point
	if( (alpha > 0) && (alpha < norm_p1p2) )
	{
		proj			= p1 + alpha*p1p2_n;
		closestPoint	= proj;
		return (proj-p).norm();
	}
	else
	{
		d1		= (p1-p).norm();
		d2		= (p2-p).norm();
		
		if( d1 < d2 )
		{
			closestPoint = p1;
			return d1;
		}
		else
		{
			closestPoint = p2;
			return d2;
		}
	}
}

//! Bounding Balls Distance
//! computes the distance to a line and a sphere
//! The Line is the body and the sphere is the head
double Distance::computeBoundingBalls(const Vector3d& WSPoint, p3d_vector3 robot, p3d_vector3 human)
{
	double pointneckdist;
	double pointbodydist = numeric_limits<double>::max();
	
	Vector3d p1,p2;
	Vector3d closestPoint;
	
	if (computeCylinderZone(p1,p2))
	{
		pointbodydist = pointToLineSegmentDistance(WSPoint,p1,p2,closestPoint)-0.20; // Body radius
	}
	
	Vector3d hneck;
	
	hneck = _Humans[0]->getJoint(HRICS_HUMANj_NECK_TILT)->getVectorPos();
	
	pointneckdist = ( WSPoint - hneck ).norm()-0.30; // Head radius
	
	if( pointneckdist < pointbodydist)
	{
    // Closest point is neck
		for(int i=0; i<3; i++)
		{
			robot[i] = WSPoint[i];
			human[i] = hneck[i];
		}
		
		return pointneckdist;
	}
	else
	{
    // Cosest point is Body
		for(int i=0; i<3; i++)
		{
			robot[i] = WSPoint[i];
			human[i] = closestPoint[i];
		}
		
		return pointbodydist;
	}
}

void Distance::drawInteractionZone()
{
  Vector3d WSPoint;
  Vector3d robotCenter = m_Robot->getJoint(1)->getVectorPos();
    
  Transform3d World_2_Robot( m_Robot->getJoint(1)->getMatrixPos() );
  
  GLint circle_points = 30;
  GLdouble color_vect[4];

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  
  g3d_set_color(Green,color_vect);
  
  glBegin(GL_POLYGON);
  for (int i = 0; i < circle_points; i++) 
  {
    double angle = M_PI*i/circle_points - (M_PI/2);
    
    Vector3d p;
    p[0] = m_InteractionRadius*cos(angle);
    p[1] = m_InteractionRadius*sin(angle);
    p[2] = 0.10;
    
    p = World_2_Robot*p;
    
    glVertex3d(p[0],p[1],p[2]);
  }
  glEnd();
  
//  int isHumanInInteractionZone = isPointInInteractionZone(_Humans[0]->getJoint(1)->getVectorPos());
//  cout << "isHumanInInteractionZone = " << isHumanInInteractionZone << endl;
}

int Distance::isPointInInteractionZone(const Vector3d& WSPoint)
{
  Vector3d robCenter = m_Robot->getJoint(1)->getVectorPos();
  
  Vector2d p1,p2;
  
  p1[0] = robCenter[0];
  p1[1] = robCenter[1];
  
  p2[0] = WSPoint[0];
  p2[1] = WSPoint[1];
  
  double dist = ( p1 - p2 ).norm();
  
  if ( dist > m_InteractionRadius ) {
    return 0;
  }
  
  Transform3d Robot_2_World( m_Robot->getJoint(1)->getMatrixPos().inverse() );
  
  Vector3d p = Robot_2_World*WSPoint;
  
  double alpha = atan2(p[1],p[0]);
  
  if ( alpha > 1.57 ) {
    return 0;
  }
  
  if ( alpha < -1.57 ) {
    return 0;
  }
  
  return 1;
}

