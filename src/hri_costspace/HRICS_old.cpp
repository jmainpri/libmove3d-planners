#include "HRICS_old.hpp"
#include <math.h>

#include "P3d-pkg.h"
#include "Collision-pkg.h"

#ifdef P3D_PLANNER
#include "Planner-pkg.h"
#endif

using namespace std;
using namespace HRICS;
using namespace Move3D;

//Hri hri_zones;

/**********************************************************************************
 * Code for Hri Transition-RRT
 **********************************************************************************/

Hri::Hri() :
	zones(0),
	vect_jim(6,0),
	dist_penetration(6,0),
	color(1)
	{
	//dist_penetration = null;
	//parseEnvForZone(·F);
	}

Hri::Hri(double zone_size) :
	zones(0),
	vect_jim(6,0),
	dist_penetration(6,0),
	color(1)
	{
	//parseEnvForZone();
	}

void Hri::setNodes(Node* fromComp , Node* toComp){
	this->fromComp = fromComp->getConfiguration()->getConfigStruct();
	this->toComp = toComp->getConfiguration()->getConfigStruct();
}

/**
 * Add zones to Hri module
 */
void Hri::parseEnvForZone(){

	int nb_robot = XYZ_ENV->nr;

	string name;
	string n_Human("Achile");
	string n_Robot("justin");

	for(int i=0;i<nb_robot;i++)
	{
		name = XYZ_ENV->robot[i]->name;

		if( name.compare(n_Human)==0 )
			Human = XYZ_ENV->robot[i];

		if( name.compare(n_Robot)==0 )
			Robot = XYZ_ENV->robot[i];
	}

	name = Human->name;

	if(name.compare("Achile")==0){
		cout << "Parsing " << Human->name << " for hri zones" << endl;
	}
	else{
		cout << Human->name << " is not the right robot" << endl;
		return;
	}

	string body;
	string b_name;
	string buffer;
	zoneHri new_zone;

	cout << new_zone.name << new_zone.id <<" "<< new_zone.radius << endl;

	for(int i=0;i<Human->no;i++){

		body = Human->o[i]->name;
		size_t found;
		b_name = body;

		for(int j=0;j<2;j++){
			found=b_name.find('.');
			buffer = b_name.substr(found+1);
			b_name = buffer;
		}

		buffer = b_name.substr(0,10);
		if(buffer.compare("zone_trans")==0){
			new_zone.radius = Human->o[i]->pol[0]->primitive_data->radius;
			//cout << "radius = " << new_zone.radius << endl;
		}

		buffer = b_name.substr(0,8);
		if(buffer.compare("zone_hri")==0){
			new_zone.name = body;
			new_zone.id = i;
			//cout << "name = " << new_zone.name << endl;
			//cout << "id = " << new_zone.id << endl;
		}
		//cout << new_zone.name << new_zone.id <<" "<< new_zone.radius << endl;
		if( new_zone.radius != 0 && new_zone.id != -1 ){
			cout << "radius = " << new_zone.radius << endl;
			cout << "name = " << new_zone.name << endl;
			cout << "push_back new_zone and reset at "<< new_zone.id << endl;
			cout << endl;
			zones.push_back(new_zone);
			new_zone.reset();
		}
	}

	vect_jim.resize(zones.size()*6);
	dist_penetration.resize(zones.size());

	cout << "zones.size() = " << zones.size() << endl;

	std::vector<double> v_radius;

        for(unsigned int i=0;i<zones.size();i++)
		v_radius.push_back(zones[i].radius);

	dist_min = *min_element(v_radius.begin(),v_radius.end());

	cout << "dist_min = " << dist_min << endl;
	cout << "dist_penetration.size() = " << dist_penetration.size() << endl;
	cout << "vect_jim.size() = " << vect_jim.size() << endl;
}


/**
 * Creates zone out of the Achile model
 */
void Hri::parseEnvHoleHuman(){

	int nb_robot = XYZ_ENV->nr;

	string name;
	string n_Human("Achile");
	string n_Robot("justin");

	for(int i=0;i<nb_robot;i++){

		name = XYZ_ENV->robot[i]->name;

		if( name.compare(n_Human)==0 )
			Human = XYZ_ENV->robot[i];

		if( name.compare(n_Robot)==0 )
			Robot = XYZ_ENV->robot[i];

	}

	toComp = Robot->ROBOT_GOTO;
	fromComp = Robot->ROBOT_POS;

	name = Human->name;

	if(name.compare("Achile")==0){
		cout << "Parsing " << Human->name << " for hri zones" << endl;
	}
	else{
		cout << Human->name << " is not the right robot" << endl;
		return;
	}

	string body;
	string b_name;
	string buffer;

	safe_offset = ENV.getDouble(Env::zone_size) - safe_radius;

	for(int i=0;i<Human->no;i++){

		body = Human->o[i]->name;
		size_t found;
		b_name = body;

		for(int j=0;j<2;j++){
			found=b_name.find('.');
			buffer = b_name.substr(found+1);
			b_name = buffer;
		}

		buffer = b_name.substr(0,10);
		if(buffer.compare("protection")==0){
			for(int j=0;j< Human->o[i]->np ; j++){

				/* the shape : 0,1,2=polyh,
				 * 3=sphere, 4=cube,
				 * 5=box, 6=cylinder,
				 * 7=cone */

				int shape = Human->o[i]->pol[j]->entity_type;

				if( shape==0 || shape==1 || shape == 2){
					cout << "Shape is oval cylinder" << endl;
					break;
				}
				//				if(shape == 3)
				//					cout << "Shape is sphere" << endl;
				//				if(shape==4)
				//					cout << "Shape is cube" << endl;
				//				if(shape==5)
				//					cout << "Shape is box" << endl;
				//				if(shape==6)
				//					cout << "Shape is cylinder" <<endl;
				//				if(shape==7)
				//					cout << "Shape is cone" << endl;

				//				cout << "radius = "<< Human->o[i]->pol[j]->primitive_data->radius << endl;
				//				cout << "other_radius = "<< Human->o[i]->pol[j]->primitive_data->other_radius << endl;
				//				cout << "height = "<< Human->o[i]->pol[j]->primitive_data->height << endl;
				//				cout << "sin_slope = "<< Human->o[i]->pol[j]->primitive_data->sin_slope << endl;
				//				cout << "x_length = "<< Human->o[i]->pol[j]->primitive_data->x_length << endl;
				//				cout << "y_length = "<< Human->o[i]->pol[j]->primitive_data->y_length << endl;
				//				cout << "z_length = "<< Human->o[i]->pol[j]->primitive_data->z_length << endl;

				offSetPrim(Human->o[i]->pol[j],safe_offset);
			}
		}
	}
	safe_radius = ENV.getDouble(Env::zone_size);
	safe_offset = 0;
}

/**
 * Creates zone out of the Bar model
 */
void Hri::parseEnvBarre(){
	int nb_robot = XYZ_ENV->nr;

	string name;
	string n_Barre("barre");
	string n_Robot("justin");
	string n_2Dof("2-3dof");

	for(int i=0;i<nb_robot;i++){

		name = XYZ_ENV->robot[i]->name;

		if( name.compare(n_Barre)==0 )
			Human = XYZ_ENV->robot[i];

		if( name.compare(n_Robot)==0 || name.compare(n_2Dof)==0 )
			Robot = XYZ_ENV->robot[i];
	}

	toComp = Robot->ROBOT_GOTO;
	fromComp = Robot->ROBOT_POS;

	safe_offset = ENV.getDouble(Env::zone_size) - safe_radius;
	offSetPrim(Human->o[1]->pol[0],safe_offset);
	safe_radius = ENV.getDouble(Env::zone_size);
	safe_offset = 0;

}

/**
 * Main function to create zones
 */
void Hri::parseEnv(){

	int nb_robot = XYZ_ENV->nr;

	string name;
	string n_Barre("barre");
	string n_Achile("Achile");
	string n_Robot("justin");

	for(int i=0;i<nb_robot;i++){

		name = XYZ_ENV->robot[i]->name;

		if( name.compare(n_Barre)==0 ){
			parseEnvBarre();
			break;
		}

		if( name.compare(n_Achile)==0 ){
			parseEnvHoleHuman();
			break;
		}

	}

	for(int i=0;i<nb_robot;i++){
		name = XYZ_ENV->robot[i]->name;

		if(name.compare(n_Robot)==0){
			computeMaxValues();
			break;
		}
	}

	//	configPt q;
	//	q = p3d_copy_config( Human, Human->ROBOT_POS );
	//	//print_config(Human,q);
	//	//Attention print necessite le calcul de cout
	//	// donc l'init complete
	//	p3d_set_robot_config(Human,q);
	//	p3d_update_this_robot_pos(Human);
	//	p3d_destroy_config(Human,q);

}

/**
 * Changes color of the zone
 */
void Hri::changeColor(/*double color*/)
{
	string body;
	string b_name;
	string buffer;

	double new_c[4];
	new_c[0]=1.0;
	new_c[1]=1.0;
	new_c[2]=0.0;

	if(this->color==1.0){
		this->color = 0.0;
		new_c[3]=0.0;
	}
	else{
		this->color = 1.0;
		new_c[3]=0.3;
	}

	cout << "change of color" << endl;

	for(int i=0;i<Human->no;i++){

		body = Human->o[i]->name;
		size_t found;
		b_name = body;

		for(int j=0;j<2;j++){
			found=b_name.find('.');
			buffer = b_name.substr(found+1);
			b_name = buffer;
		}

		buffer = b_name.substr(0,10);
		if(buffer.compare("protection")==0){
			for(int j=0;j< Human->o[i]->np ; j++){
				for(int k=0;k<4;k++){
					Human->o[i]->pol[j]->color_vect[k] = new_c[k];
				}
			}
		}
	}
}

/**
 * Changes dynamically the size of the zone shown
 * in the OpenGl display
 */
void Hri::offSetPrim(p3d_poly* poly,double offset)
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

/**
 * Intent to compute
 * the interval of value in the CSpace that the robot goes
 * through during specific planning
 */
void Hri::computeMaxValues(){
	// Distance in C-space to this config

	configPt q;
	q = p3d_alloc_config(Robot);

	q[0] = 0.000000;
	q[1] = 0.000000;
	q[2] = 0.000000;
	q[3] = 0.000000;
	q[4] = 0.000000;
	q[5] = 0.000000;
	q[6] = 1.573257;
	q[7] = -22.123896;
	q[8] = 31.659294;
	q[9] = -9.535398;
	q[10] = 0.000000;
	q[11] = 0.000000;
	q[12] = 5.014758;
	q[13] = -66.076698;
	q[14] = -15.044244;
	q[15] = 115.634224;
	q[16] = 93.608658;
	q[17] = -9.540314;
	q[18] = -3.672564;
	q[19] = -15.000000;
	q[20] = -46.000000;
	q[21] = -8.000000;
	q[22] = 119.000000;
	q[23] = 138.000000;
	q[24] = 62.000000;
	q[25] = 29.000000;

	qConfort = p3d_copy_config_deg_to_rad(Robot,q);

	int njnt = Robot->njoints, i, j, k;
	double vmin, vmax;
	p3d_jnt * jntPt;

	for(i=0; i<=njnt; i++) {
		jntPt = Robot->joints[i];
		for(j=0; j<jntPt->dof_equiv_nbr; j++) {
			k = jntPt->index_dof + j;
			if( (p3d_jnt_get_dof_is_user(jntPt,j)==1) &&
					(p3d_jnt_get_dof_is_active_for_planner(jntPt,j)==1) &&
					(Robot->cntrt_manager->in_cntrt[k]==0)) {
				p3d_jnt_get_dof_rand_bounds(jntPt, j, &vmin, &vmax);
				q[k] = vmax;
			}
			else
			{
				q[k] = qConfort[k];
			}
		}
	}


	double minDist = this->getCustomDistConfig(q,qConfort);
	q = p3d_copy_config_rad_to_deg(Robot,q);
	//print_config(Robot,q);

	for(i=0; i<=njnt; i++) {
		jntPt = Robot->joints[i];
		for(j=0; j<jntPt->dof_equiv_nbr; j++) {

			k = jntPt->index_dof + j;
			//		    	cout << "Robot->cntrt_manager->in_cntrt["<<k<<"] = "
			//						<< Robot->cntrt_manager->in_cntrt[k] << endl;

			if( (p3d_jnt_get_dof_is_user(jntPt,j)==1) &&
					(p3d_jnt_get_dof_is_active_for_planner(jntPt,j)==1) &&
					(Robot->cntrt_manager->in_cntrt[k]==0)) {
				p3d_jnt_get_dof_rand_bounds(jntPt, j, &vmin, &vmax);
				q[k] = vmin;
			}
			else
			{
				q[k] = qConfort[k];
			}
		}
	}

	double maxDist = this->getCustomDistConfig(q,qConfort);
	q = p3d_copy_config_rad_to_deg(Robot,q);
	//print_config(Robot,q);

	if(maxDist > minDist)
		max_jlimits = maxDist;
	else
		max_jlimits = minDist;

	// Right Hand
	int njoint = 15;

	configPt q_goal = toComp;
	configPt q_actual = fromComp;

	p3d_set_robot_config(Robot,q_goal);
	p3d_update_this_robot_pos(Robot);

	p3d_vector3 pos_goal;
	p3d_jnt_get_cur_vect_point(Robot->joints[njoint],pos_goal);

	p3d_set_robot_config(Robot,q_actual);
	p3d_update_this_robot_pos(Robot);

	p3d_vector3 pos_actual;
	p3d_jnt_get_cur_vect_point(Robot->joints[njoint],pos_actual);

	double dist;
	double a;
	max_taskdist = 0;

	for(int i=0;i<3;i++){
		dist = pos_goal[i] - pos_actual[i];

		if(i==2){
			// case of altitude
			//a= ENV.getDouble(Env::coeffHei);
			cout << "Warning : Coeff broken" << endl;
		}
		else{
			a=1;
		}

		max_taskdist += a*dist*dist;
	}

	max_taskdist = sqrt(max_taskdist);

	//cout << "q_actual = " << endl;
	//print_config(Robot,q_actual);
	//cout << "q_goal = " << endl;
	//print_config(Robot,q_goal);
	max_to_goal = this->getCustomDistConfig(q_goal,q_actual);

	//cout << "max_to_goal = " << max_to_goal << endl;
	//cout << "max_taskdist = " << max_taskdist << endl;
	//cout << "max_jlimits = " << max_jlimits << endl;

}

/**
 * p3d_GetMinDistCost
 * Get the cost of a current configuration based
 * on its minimal distance to the obstacles.
 * This function is used for algorithm based on configuration
 * spaces with cost functions
 */
double Hri::getHriDistCost(p3d_rob* robotPt, int disp) {

	/* ----------------------------------------------------
	 *
	 * Preparation du context KCD test de collision
	 *
	 * */

	//	for(int i=0;i<dist_penetration.size();i++)
	//		dist_penetration.at(i)=getDistBody(i,disp);
	//getDistBody(disp);

	/*****************************************************
	 * Natural looking cost
	 */
	if (ENV.getBool(Env::useHriNat)){
		getNaturalLookingCost(disp);
	}
	else{
		c_natural = 0;
	}

	/*****************************************************
	 * Penetration distance
	 */
	if (ENV.getBool(Env::useHriPen)){
		getSafeZoneCost(disp);
	}
	else{
		c_penetra = 0;
	}

	/*****************************************************
	 * To goal distance in the C-Space
	 */
	if (ENV.getBool(Env::useHriDis)){
		getDistToGoalCost(disp);
	}
	else{
		c_to_goal = 0;
	}

	/*****************************************************
	 *  Compute sum
	 **/

	cost=0;

	if (ENV.getBool(Env::useHriPen)){
		double coeff_penetra = ENV.getDouble(Env::coeffPen);
		cost += coeff_penetra*c_penetra;
	}

	if (ENV.getBool(Env::useHriDis)){
		double coeff_distanc = ENV.getDouble(Env::coeffDis);
		cost += coeff_distanc*c_to_goal;
	}

	if (ENV.getBool(Env::useHriNat)){
		double coeff_natural = ENV.getDouble(Env::coeffNat);
		cost += coeff_natural*c_natural;
	}


	if (disp) {
		//		PrintInfo(("dist_penetration = %3.2f \n", dist_penetration.at()));
		cout << "c_penetra = " << c_penetra << endl;
		cout << "c_to_goal = " << c_to_goal << endl;
		cout << "c_natural = " << c_natural << endl;
		PrintInfo(("Cost = \t%3.2f\n", cost));
	}

	if (disp)
		PrintInfo(("-----------------------------------------------------------\n"));

	return cost;
}

/**
 * Computes the penetration distance
 */
double Hri::getDistZone(int zone,int disp) {

	int settings = get_kcd_which_test();
	set_kcd_which_test((p3d_type_col_choice)(20+3));
	// 40 = KCD_ROB_ENV
	// 3 = DISTANCE_EXACT

	deactivateAllButHriAchile(zone,disp);
	// p3d_col_deactivate_rob_obj(r_Robot,r_Human->o[i]);

	p3d_rob* r_buffer = XYZ_ENV->robot[0];
	XYZ_ENV->robot[0] = XYZ_ENV->robot[1];
	XYZ_ENV->robot[1] = r_buffer;

	p3d_col_test_choice();

	int nof_bodies = Human->no;
	double* distances= MY_ALLOC(double,nof_bodies);

	p3d_vector3 *body= MY_ALLOC(p3d_vector3,nof_bodies);
	p3d_vector3 *other= MY_ALLOC(p3d_vector3,nof_bodies);

	p3d_kcd_closest_points_between_bodies(Human,body,other,distances);

	//	int k = 0;
	//	double buffer = numeric_limits<double>::max();
	////	get the mini value of the array distances
	//	for (int i=0; i<nof_bodies; i++){
	//		if(distances[i]<buffer){
	//			buffer = distances[i];
	//			k=i;
	//		}
	//	}
	//
	double dist_pene=*min_element(distances,distances+nof_bodies);;

	//	if(dist_pene>1.)
	//		dist_pene=1.;

	if (disp) {
		//		for(int i=0;i<zone_id.size();i++){
		//			p3d_matrix4 position;
		//			p3d_get_poly_pos(Human->o[zone_id[i]]->pol[0]->poly,position);
		//			printMatrix4(position);
		//			cout << endl;
		//		}
		cout << "dist_pene = " << dist_pene << endl;
	}

	/* ----------------------------------------------------
	 * Vecteur de distance aux zones HRI
	 **/
	for(int i=0;i<nof_bodies;i++){
		vect_jim[0+6*i] = body[i][0];
		vect_jim[1+6*i] = body[i][1];
		vect_jim[2+6*i] = body[i][2];
		vect_jim[3+6*i] = other[i][0];
		vect_jim[4+6*i] = other[i][1];
		vect_jim[5+6*i] = other[i][2];
	}

	activateAllAchile(disp);

	if(disp){
		cout << "All active now \n" << endl;
	}

	r_buffer = XYZ_ENV->robot[0];
	XYZ_ENV->robot[0] = XYZ_ENV->robot[1];
	XYZ_ENV->robot[1] = r_buffer;

	set_kcd_which_test((p3d_type_col_choice)settings);// ROB_ALL + BOOL

	return dist_pene;
}

/**
 * Computes the penetration distance
 */
void Hri::getDistBody(int disp) {

	int settings = get_kcd_which_test();
	set_kcd_which_test((p3d_type_col_choice)(20+3));
	// 40 = KCD_ROB_ENV
	// 3 = DISTANCE_EXACT

	deactivateAllButHriAchile(0,disp);
	// p3d_col_deactivate_rob_obj(r_Robot,r_Human->o[i]);


	p3d_col_test_choice();
	// Colision detection with other robots only

	int nof_bodies = Robot->no;
	double* distances= MY_ALLOC(double,nof_bodies);

	p3d_vector3 *body= MY_ALLOC(p3d_vector3,nof_bodies);
	p3d_vector3 *other= MY_ALLOC(p3d_vector3,nof_bodies);


	p3d_kcd_closest_points_between_bodies(Robot,body,other,distances);
	// Get robot closest points to human for each body

	vect_jim.resize(6*nof_bodies);
	dist_penetration.resize(nof_bodies);

	p3d_matrix4 position;

        for(unsigned int i=0;i<zones.size();i++)
		zones[0].dist_pene = numeric_limits<double>::max();

	double buffer = numeric_limits<double>::min();
	int k_a=0;
	int k_b=0;
	dist_penetration.resize(zones.size());

	for(int j=0;j<nof_bodies;j++){

		if(disp)
			cout << "Body num = " << j << endl;

		double radius(0);

                for(unsigned int i=0;i<zones.size();i++){

			p3d_get_poly_pos( Human->o[zones[i].id]->pol[0]->poly , position );

			if(( fabs((double)position[0][3] - (double)other[j][0]) <= 1.5) &&
					( fabs((double)position[1][3] - (double)other[j][1]) <= 1.5) &&
					( fabs((double)position[2][3] - (double)other[j][2]) <= 1.5)){



				radius = zones[i].radius;
				dist_penetration[i] =(radius - distances[j])/dist_min;

				if(dist_penetration[i]>1.)
					dist_penetration[i]=1.;

				if(disp){
					cout << "zones["<<i<<"] dist_penetration = "
					<< dist_penetration[i] << endl;
				}

				zones[i].dist_pene = dist_penetration[i];

				if(dist_penetration[i]>buffer){
					buffer = dist_penetration[i];
					k_a = i;
					k_b = j;
				}

				break;
			}
		}
	}

	if(disp){
		cout << "closer zone = "<< zones[k_a].name << endl;
		cout << "     id = "<< zones[k_a].id << endl;
		cout << "dist_pene = " << zones[k_a].dist_pene << endl;
	}

	/* ----------------------------------------------------
	 * Vecteur de distance aux zones HRI
	 **/
	vect_jim.resize(6);

	vect_jim[0] = body[k_b][0];
	vect_jim[1] = body[k_b][1];
	vect_jim[2] = body[k_b][2];
	vect_jim[3] = other[k_b][0];
	vect_jim[4] = other[k_b][1];
	vect_jim[5] = other[k_b][2];



	activateAllAchile(disp);

	if(disp){
		cout << "All active now \n" << endl;
	}

	set_kcd_which_test((p3d_type_col_choice)settings);// ROB_ALL + BOOL

}

/**
 * Computes the penetration distance
 */
void Hri::getDistHoleHuman(int disp) {

	int settings = get_kcd_which_test();

	set_kcd_which_test((p3d_type_col_choice)(20+3));
	// 40 = KCD_ROB_ENV
	// 3 = DISTANCE_EXACT

	p3d_col_test_choice();
	// Collision detection with other robots only

	int nof_bodies = Robot->no;
	double* distances= MY_ALLOC(double,nof_bodies);

	p3d_vector3 *body= MY_ALLOC(p3d_vector3,nof_bodies);
	p3d_vector3 *other= MY_ALLOC(p3d_vector3,nof_bodies);


	p3d_kcd_closest_points_between_bodies(Robot,body,other,distances);
	// Get robot closest points to human for each body

	vect_jim.resize(6);
	dist_penetration.resize(nof_bodies);

	int k=0;
	double buffer = -numeric_limits<double>::max();
	//double radius = ENV.getDouble(Env::zone_size);

	for(int i=0;i<nof_bodies;i++){


		dist_penetration[i] = (safe_radius - distances[i])/safe_radius;

		if(disp){
			//			cout << "id = " << i << endl;
			//			cout << dist_penetration[i] << endl;
			//cout << buffer << " < " << dist_penetration[i] << " : " << (buffer<dist_penetration[i]) << endl;
		}

		if(buffer<dist_penetration[i]){
			buffer = dist_penetration[i];
			k = i;
		}
	}

	if(disp){
		//		cout << "buffer = " << buffer << endl;
		//		cout << "k = " << k << endl;
	}

	/* ----------------------------------------------------
	 * Vecteur de distance aux zones HRI
	 **/
	vect_jim[0] = body[k][0];
	vect_jim[1] = body[k][1];
	vect_jim[2] = body[k][2];
	vect_jim[3] = other[k][0];
	vect_jim[4] = other[k][1];
	vect_jim[5] = other[k][2];

	set_kcd_which_test((p3d_type_col_choice)settings);// ROB_ALL + BOOL

	free(distances);
	free(body);
	free(other);

}
/**
 * p3d_p3d_GetGoalConfig
 **/
configPt Hri::getGoalConfig(p3d_rob* robotPt) {

	configPt q = p3d_copy_config_rad_to_deg(robotPt, robotPt->ROBOT_GOTO);

	//	p3d_jnt * jntPt;

	//	int index=0;

	//	printf("DOF PASSIFF\n");
	//
	//	for(int i=0; i<=robotPt->njoints; i++) {
	//
	//
	//		jntPt = robotPt->joints[i];
	//
	//		for(int j=0; j<jntPt->dof_equiv_nbr; j++) {
	//
	//		      if(robotPt->cntrt_manager->in_cntrt[jntPt->index_dof + j] == DOF_PASSIF){
	//		        printf("number %d is PASSIVE\n",jntPt->index_dof + j);
	//		      }
	//		      index++;
	//		    }
	//	}
	//
	//	printf("number %d of DOF\n",index);

	return q;
}

/**
 * Computes the distance to goal cost
 */
void Hri::getDistToGoalCost(int disp){

	configPt 	q_new = p3d_get_robot_config(Robot);
	q_new = p3d_copy_config_rad_to_deg(Robot,q_new);

	//		if(disp){
	//			printf("Q_NEW = \n");
	//			print_config(robotPt,q_new);
	//			printf("\n\n");
	//		}

	configPt q_goal = toComp;
	q_goal = p3d_copy_config_rad_to_deg(Robot,q_goal);

	//		if(disp){
	//			printf("Q_GOAL = \n");
	//			print_config(robotPt,q_goal);
	//			printf("\n\n");
	//		}


	double dist_to_goal = getCustomDistConfig( q_goal , q_new );

	p3d_destroy_config(Robot,q_new);
	p3d_destroy_config(Robot,q_goal);

	if ( dist_to_goal > max_to_goal ){
		dist_to_goal = max_to_goal;
	}

	c_to_goal = dist_to_goal / max_to_goal;
}

/*
 * Computes the safeZoneCost
 * using the HoleHumanDist
 */
void Hri::getSafeZoneCost(int disp){

	getDistHoleHuman(disp);

	// Maximal penetration distance
	// for each robot body
	c_penetra = *max_element(
			dist_penetration.begin(),
			dist_penetration.end());

	//		if(disp)
	//			cout << c_penetra << endl;


	// Compute of the hri cost function
	if (c_penetra < 0) {

		c_penetra = 0.000001;
	} else {
		c_penetra = (exp(c_penetra-1) - exp(-1) ) / ( 1 - exp(-1) );
		c_penetra += 0.000001;
	}
}

/**
 * Computes the natural looking cost
 * of a robot configuration
 */
void Hri::getNaturalLookingCost(int disp){

	configPt q_actual = p3d_get_robot_config(Robot);
	c_jlimits = getCustomDistConfig(qConfort,q_actual);

	//	if(c_jlimits > max_jlimits)
	//		c_jlimits = max_jlimits;
	c_jlimits /= max_jlimits;

	if(disp)
		cout << "c_jlimits =" << c_jlimits << endl;


	// Distance in task space

	// Right Hand
	int njoint = 15;

	configPt q_goal = toComp;

	p3d_set_robot_config(Robot,q_goal);
	p3d_update_this_robot_pos(Robot);

	p3d_vector3 pos_goal;
	p3d_jnt_get_cur_vect_point(Robot->joints[njoint],pos_goal);

	p3d_set_robot_config(Robot,q_actual);
	p3d_update_this_robot_pos(Robot);

	p3d_vector3 pos_actual;
	p3d_jnt_get_cur_vect_point(Robot->joints[njoint],pos_actual);

	p3d_destroy_config(Robot,q_actual);

	double dist;
	double a;

	if(disp){
		cout << "position actuelle : ";
		cout << pos_actual[0] << " " << pos_actual[1]<< " " << pos_actual[2] << endl;
		cout << "position du but : ";
		cout << pos_goal[0] << " " << pos_goal[1]<< " " << pos_goal[2] << endl;
	}

	for(int i=0;i<3;i++){
		dist = pos_goal[i] - pos_actual[i];

		if(disp)
			cout << dist << endl;

		if(i==2){
			// case of altitude
			//a= ENV.getDouble(Env::coeffHei);;
			cout << "Warning : Coeff broken" << endl;
		}
		else{
			a=1;
		}

		c_taskdist += a*dist*dist;
	}

	if(disp){
		cout << "c_taskdist =" << c_taskdist << endl;
		cout << endl;
	}

	c_taskdist = sqrt(c_taskdist);

	//	if(c_taskdist > max_taskdist)
	//		c_taskdist = max_taskdist;
	c_taskdist /= max_taskdist;


	if(disp){
		cout << "c_taskdist =" << c_taskdist << endl;
		cout << endl;
	}

//	double coeff_task = ENV.getDouble(Env::coeffTas)/100;
//	double coeff_limi = ENV.getDouble(Env::coeffLim)/100;
//
//	c_natural = coeff_task*c_taskdist + coeff_limi*c_jlimits;
	
	cout << "Warning : Coeff broken" << endl;
}

/*!
 * Compute the classic square distance between two configurations
 * with weight
 *
 * Input:  The robot,
 *         the two configurations
 *
 * See : p3d_dist_config
 */
double Hri::getCustomDistConfig(configPt q_i, configPt q_f) {
	double l = 0., ljnt = 0.;
	int i, j, njnt = Robot->njoints;
	p3d_jnt * jntPt;

	for (i=0; i<=njnt; i++) {

		jntPt = Robot->joints[i];

		for (j=0; j<jntPt->dof_equiv_nbr; j++) {

			if (Robot->cntrt_manager->in_cntrt[jntPt->index_dof + j]
			                                   != DOF_PASSIF) {
				//Jim Hri Modif

				double W;

				switch (jntPt->index_dof + j) {
				case 6:
					W = 2;
					break;
				case 7:
					W = 2;
					break;
				case 8:
					W = 2;
					break;

				case 12:
					W = 3;
					break;
				case 13:
					W = 3;
					break;
				case 14:
					W = 3;
					break;
				case 15:
					W = 3;
					break;

				case 16:
					W = 1;
					break;
				case 17:
					W = 1;
					break;

				default:
					W = 1;
				}
				double dof_dist = p3d_jnt_calc_dof_dist(jntPt, j, q_i, q_f);
				//printf("dof_dist[%d] = %f\n",jntPt->index_dof + j,dof_dist);
				ljnt += W*SQR(dof_dist);
			}
		}
	}
	l = sqrt(ljnt);

	return l;
}

/**
 * Gets the penetration cost
 * along a trajectory
 */
double Hri::getPeneTrajCost(p3d_graph* graphPt, p3d_traj* trajPt,
		double* minCostPt, double* maxCostPt,
		double* totalCostPt, int* nbConfigPt) {
	configPt config;
	double currentCost;
	double currentParam = 0.;
	double dMax = p3d_get_env_dmax();
	double prevCost;
	double Wsum = 0.;

	//double coeff_penetra = ENV.getDouble(Env::coeffPen);

	if (trajPt == NULL) {
		PrintInfo(("Warning: Failed to compute the cost of a solution path: \
		as no soltion solution path has been found\n"));
		return 0.;
	}

#if P3D_PLANNER
	// compute Cost
	config = p3d_config_at_param_along_traj(trajPt,currentParam);

	//----------------------------------------------
	p3d_set_and_update_robot_conf(config);

	//getSafeZoneCost(0);
	getDistHoleHuman(0);

	// Maximal penetration distance
	// for each robot body
	c_penetra = *max_element(
			dist_penetration.begin(),
			dist_penetration.end());

	currentCost= 100*c_penetra;
	//----------------------------------------------

	*totalCostPt = currentCost;
	*maxCostPt = currentCost;
	*minCostPt = currentCost;

	while(currentParam <= trajPt->range_param) {
		currentParam += dMax;
		prevCost = currentCost;
		p3d_destroy_config(graphPt->rob, config);

		// compute Cost
		config = p3d_config_at_param_along_traj(trajPt,currentParam);
		//----------------------------------------------
		p3d_set_and_update_robot_conf(config);
		//getSafeZoneCost(0);
		getDistHoleHuman(0);

		// Maximal penetration distance
		// for each robot body
		c_penetra = *max_element(
				dist_penetration.begin(),
				dist_penetration.end());

		currentCost= 100*c_penetra;
		//----------------------------------------------
		//computation of the cost associated to a mechanical work W
		Wsum += p3d_ComputeDeltaStepCost(prevCost, currentCost, dMax);
		*totalCostPt = (*totalCostPt) + currentCost;
		*minCostPt = MIN((*minCostPt),currentCost);
		*maxCostPt = MAX((*maxCostPt),currentCost);
		(*nbConfigPt)++;
	}
#endif
	return Wsum;
}

/**
 * Fonction commencant la description d'une zone hri (obstacle) p3d rattache
 * a l'environnement courant, qui devient l'objet courant
 * (au moins pour la description...)
 * In : son nom, son type (obstacle, body)
 * Out :
 **/

void * Hri::beg_zone_hri(char *name) {
	pp3d_obj o;

	o = MY_ALLOC(p3d_obj, 1);

	if (!o)
		return (NULL);

	o->name = strdup(name);
	o->env = XYZ_ENV;
	o->type = P3D_OBSTACLE;
	o->jnt = NULL;
	o->pol = NULL; // Modification Fabien
	o->np = 0;
	p3d_mat4Copy(p3d_mat4IDENTITY, o->opos);

	o->box.x1 = P3D_HUGE;
	o->box.x2 = -P3D_HUGE;
	o->box.y1 = P3D_HUGE;
	o->box.y2 = -P3D_HUGE;
	o->box.z1 = P3D_HUGE;
	o->box.z2 = -P3D_HUGE;

	o->BB.xmin = P3D_HUGE;
	o->BB.xmax = -P3D_HUGE;
	o->BB.ymin = P3D_HUGE;
	o->BB.ymax = -P3D_HUGE;
	o->BB.zmin = P3D_HUGE;
	o->BB.zmax = -P3D_HUGE;
	o->concat = 0;
	return ((void *) (XYZ_OBSTACLES = o));
}

/**
 * p3d_GetObjectByName
 */
p3d_obj* Hri::getObjectByName(char *name) {
	PrintInfo(("Nombre d'objets %d\n ", XYZ_ENV->nof_objs ));
	PrintInfo(("Nombre de robots %d\n ", XYZ_ENV->nr ));

	for (int i=0; i < XYZ_ENV->nr; i++) {
		PrintInfo(("\tfor robot \n"/*,XYZ_ENV->robot[i]->name*/));

		for (int j=0; j < XYZ_ENV->robot[i]->no; j++) {
			PrintInfo(("\t\t%s\n",XYZ_ENV->robot[i]->o[j]->name));

			//            return XYZ_ENV->o[i];
		}
	}
	PrintInfo(("/n"));
	//    PrintInfo(("Error Obj Null\n"));
	return NULL;
}

/**
 * p3d_SaveGPlotCostTraj
 *
 * Save the cost of configuration along the trajectory
 */
void Hri::saveGPlotCostTraj(int iteration) {
	char dir[128];
	char name[14];

	bool useHriPen;
	bool useHriDis;

	if (iteration>=1000)
		return;

	sprintf(name, "cost_traj_%03d", iteration);
	sprintf(dir, "%s/Matlab/cost_traj_dir/", getenv("HOME") );

	FILE* traj_file = fopen(strcat(dir, name) , "w");

	p3d_rob* robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
	configPt QSaved;

	useHriPen = ENV.getBool(Env::useHriPen);
	useHriDis = ENV.getBool(Env::useHriDis);

	ENV.setBool(Env::useHriPen, true);
	ENV.setBool(Env::useHriDis, true);
	
#if P3D_PLANNER

	for (double current_param = 0.; current_param < robotPt->tcur->range_param; current_param
	+= (p3d_get_env_dmax() * ENV.getDouble(Env::extensionStep)/2)) {
		configPt q = p3d_config_at_param_along_traj(robotPt->tcur,
				current_param);

		QSaved = p3d_get_robot_config(robotPt);
		p3d_set_and_update_robot_conf(q);

		cost = this->getHriDistCost(robotPt, FALSE);

		p3d_set_and_update_robot_conf(QSaved);
		p3d_destroy_config(robotPt, QSaved);

		// ATENTIOM AU COUT HRI + GOAL

		p3d_GetConfigCost(robotPt, q);
		fprintf(traj_file, "%f %f %f %f\n", current_param, c_to_goal,
				c_penetra, cost);
	}
	
#endif

	ENV.setBool(Env::useHriPen, useHriPen);
	ENV.setBool(Env::useHriDis, useHriDis);

	fclose(traj_file);
	printf("Sauvegarde => %s\n", dir);
}

/**
 * p3d_SaveGPlotCostNodeChro
 *
 * Save the costs of all nodes in the graph
 * with an chronological order of appearance
 */
void Hri::saveGPlotCostNodeChro(void) {
	// Save cost to file
	int i = 0;
	int j = 0;

	FILE* traj_file = fopen("cost_chro.txt", "w");

	for (p3d_compco *comp = XYZ_GRAPH->comp; comp != NULL && j < 10; comp
	= comp->suiv, j++) {

		for (p3d_list_node *n = comp->dist_nodes; n != NULL && i < 10000; n
		= n->next, i++) {
			fprintf(traj_file, "%d %f %f\n", n->N->num, n->N->cost, n->N->temp);
		}
	}

	fclose(traj_file);
	PrintInfo(("'cost_chro.txt' creation\n"));
}

/**
 * Sets the cost inside
 * a Table for 2D environments
 */
void Hri::setCostToTab(p3d_rob *robotPt, confCost * tab, int nbPart) {

	int i, j/*, k, count*/ = 0;

//	double ZminEnv= HUGE_VAL;
//	double ZmaxEnv = 0;
	double vMinDof1, vMaxDof1, vMinDof2, vMaxDof2/*, currentCost*/;

	configPt q, QSaved;
	p3d_jnt * jntPt;

	q = p3d_alloc_config(robotPt);

	jntPt = robotPt->joints[0];
	for (i = 0; i < 6; i++) {
		double vmin=0;
		double vmax=0;
		p3d_jnt_get_dof_bounds(jntPt, i, &vmin, &vmax);
		q[i] = vmax;
		PrintInfo(("Joint[0][%d] (vmax) = %f\n",i,vmax));
	}

	if (robotPt->joints[1]->type == P3D_PLAN) {

		jntPt = robotPt->joints[1];
		p3d_jnt_get_dof_rand_bounds(jntPt, 0, &vMinDof1, &vMaxDof1);
		p3d_jnt_get_dof_rand_bounds(jntPt, 1, &vMinDof2, &vMaxDof2);

	} else if (robotPt->joints[1]->type == P3D_ROTATE) {

		jntPt = robotPt->joints[1];
		p3d_jnt_get_dof_rand_bounds(jntPt, 0, &vMinDof1, &vMaxDof1);

		jntPt = robotPt->joints[2];
		p3d_jnt_get_dof_rand_bounds(jntPt, 0, &vMinDof2, &vMaxDof2);

	} else {

		PrintInfo(("Error in set_cost_to_tab : Joint type not taken into accout\n"));
		return;

	}

	PrintInfo(("vMinDof1 = %f\n",vMinDof1));
	PrintInfo(("vMaxDof1 = %f\n",vMaxDof1));

	PrintInfo(("vMinDof2 = %f\n",vMinDof2));
	PrintInfo(("vMaxDof2 = %f\n",vMaxDof2));

	PrintInfo(("nb_dof = %d\n",robotPt->nb_dof));
	PrintInfo(("nb_user_dof = %d\n",robotPt->nb_user_dof));

	for (i = 0; i < nbPart; i++) {
		for (j = 0; j < nbPart; j++) {
			q[6] = vMinDof1 + i*(vMaxDof1 - vMinDof1)/(nbPart-1);
			q[7] = vMinDof2 + j*(vMaxDof2 - vMinDof2)/(nbPart-1);

			QSaved = p3d_get_robot_config(robotPt);
			//			p3d_set_and_update_robot_conf(q);
			p3d_set_and_update_this_robot_conf(robotPt, q);

			/* Jim modif hri */
			tab[j*nbPart+i].cost = getHriDistCost(robotPt, FALSE);
			tab[j*nbPart+i].q = MY_ALLOC( double , robotPt->nb_dof );

			p3d_set_and_update_robot_conf(QSaved);
			p3d_destroy_config(robotPt, QSaved);

			/*for (int id = 0; id < robotPt->nb_dof; id++)*/
			tab[j*nbPart+i].q[6] = 90*q[6]/(vMaxDof1 - vMinDof1);
			tab[j*nbPart+i].q[7] = 90*q[7]/(vMaxDof2 - vMinDof2);
		}
	}
}

/**
 * Creates a Tab Cost out of a
 * function
 */
void Hri::costTabFormFunction(confCost* tab) {

	//double ZminEnv= HUGE_VAL;
//	double ZmaxEnv = 0;
//	double currentCost;

	int nbPart = 50;

	for (int i = 0; i < nbPart; i++) {
		for (int j = 0; j < nbPart; j++) {

			double x = (90.0 / (double)nbPart) * (double)i -45.0;
			double y = (90.0 / (double)nbPart) * (double)j -45.0;

			tab[j*nbPart+i].cost = costMapFunctions(x,y,1);
			tab[j*nbPart+i].q = new double [2];

			tab[j*nbPart+i].q[0] = x;
			tab[j*nbPart+i].q[1] = y;
			cout << "("<<i<<","<<j<<") = "<< x << " " << y << endl;
		}
	}
}

/**
 * Creates a simple
 * 2D Cost tab
 */
double Hri::costMapFunctions(double x, double y, int id_func){

	double x_max=30;
	double x_min=-30;
	double y_max=10;
	double y_min=-10;

	double cost_max = 30;

	double pente = 45;

	if( y>y_min && y <y_max ){
		if( x<x_min || x>x_max ){
			return cost_max;
		}
		else{
			if( x>x_min && x<=0){
				double op = x-x_min;
				double adj = op/tan(3.14*pente/180);
				if( cost_max-adj < 0)
					return 0;
				else
					return cost_max-adj;
			}
			if( x<x_max && x>=0){
				double op = x_max-x;
				double adj = op/tan(3.14*pente/180);
				if( cost_max-adj < 0)
					return 0;
				else
					return cost_max-adj;
			}
			else{
				cout << "ERROR computing the cost map" << endl;
			}
		}
	}
	else{
		return 0;
	}
 
	return 0;
}

/**
 * Writes Configuration Cost to
 * CSV file format
 */
void Hri::writeConfCostToCsv(confCost *tab, int size) {
	confCost cc;
	FILE* costFile;
	costFile = fopen("cost.csv", "w");

	for (int a=0; a<size; a++) {
		for (int b=0; b<size; b++) {

			cc = tab[b*size+a];

			fprintf(costFile, "%.10f,%.10f,%.10f\n", cc.q[6], cc.q[7], cc.cost);

		}
	}

	fclose(costFile);

	cout << "Save cost tab to CSV format"<<endl;
}

/**
 * Write a Cost Tab to
 * an ObPlane format, this output has
 * to go through a script to be used as
 * a Cost Map
 */
void Hri::writeConfCostToObPlane(confCost *tab, int size) {
	confCost cc;

	FILE* fd = fopen("OB_Plane.macro", "w");

	fprintf(fd, "p3d_beg_desc P3D_OBSTACLE\n\n\n");
	fprintf(fd, "\tp3d_add_desc_poly polyhedre1\n");

	for (int a=0; a<size; a++) {
		for (int b=0; b<size; b++) {
			cc = tab[b*size+a];
			fprintf(fd, "\t\tp3d_add_desc_vert %.9f %.9f %.9f\n",
					cc.q[6], /*Attention 6 et 7 pour le cas normal*/
					cc.q[7],
					cc.cost);
		}
	}

	fprintf(fd, "\n");

	for (int a=0; a<size-1; a++) {
		for (int b=0; b<size-1; b++) {
			fprintf(fd, "\t\tp3d_add_desc_face %d %d %d \n",
					b*size + a +1,
					(b+1)*size + a +1,
					b*size + (a+1)+1);

			fprintf(fd, "\t\tp3d_add_desc_face %d %d %d \n",
					(b+1)*size + a+1,
					(b+1)*size + (a+1)+1,
					b*size + (a+1)+1);
		}
	}

	fprintf(fd, "    p3d_end_desc_poly\n");
	fprintf(fd,
			"    p3d_set_prim_pos_by_mat polyhedre1 1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1\n");
	fprintf(fd, "p3d_end_desc\n\n");
	fprintf(fd, "p3d_set_obst_poly_color 1 Any 0.8 0.8 0.8\n");

	fclose(fd);

	cout << "Save cost tab to ObPlane format"<<endl;
}

/**
 * p3d_GetVectJim
 */
std::vector<double>& Hri::getVectJim() {
	return vect_jim;
}

/**
 * p3d_DeactivateAllButHri
 *
 * deactivate distance computation for KCD
 */
void Hri::deactivateAllButHri(int disp) {

	char str[] = "zone_hri_0";
	int deact=TRUE;

	//	PrintInfo(("DeactivateAllButHri jim\n"));

	for (int i = 0; i < XYZ_ENV->no; i++) {

		str[9]=48;

		for (int j=0; j<10 && deact; j++) {
			//    		PrintInfo(("%s\n",str));

			if (strcmp(XYZ_ENV->o[i]->name, str)==0)
				deact=FALSE;

			str[9]++;
		}

		if (deact) {
			p3d_kcd_deactivate_obstacle(XYZ_ENV->o[i]);

			if (disp)
				PrintInfo(( "Desactive l'objet %s\n", XYZ_ENV->o[i]->name ));
		}

		deact=TRUE;
	}

}

/**
 * p3d_DeactivateHris
 *
 * Deactevate distance computation for KCD
 */
void Hri::deactivateHri(int disp) {

	char str[] = "zone_hri_0";
	int deact=FALSE;

	//	PrintInfo(("DeactivateHri jim\n"));

	for (int i = 0; i < XYZ_ENV->no; i++) {

		str[9]=48;

		for (int j=0; j<10 && !deact; j++) {
			//    		PrintInfo(("%s\n",str));

			if (strcmp(XYZ_ENV->o[i]->name, str)==0)
				deact=TRUE;

			str[9]++;
		}

		if (deact) {
			p3d_kcd_deactivate_obstacle(XYZ_ENV->o[i]);

			if (disp)
				PrintInfo(( "Desactive l'objet %s\n", XYZ_ENV->o[i]->name ));
		}

		deact=FALSE;
	}

}

/**
 * p3d_ActivateAll
 */
void Hri::activateAll(int disp) {

	for (int i = 0; i < XYZ_ENV->no; i++) {
		p3d_kcd_activate_obstacle(XYZ_ENV->o[i]);

		if (disp)
			PrintInfo(( "Active l'objet %s\n", XYZ_ENV->o[i]->name ));
	}
}

/**
 * p3d_ActivateAll
 */
void Hri::activateAllAchile(int disp) {

	p3d_col_activate_rob_rob(Robot,Human);
	//sort(zone_id.begin(), zone_id.end());


	//	for(int i =0; i<Human->no; i++){
	//		//if(!binary_search(zone_id.begin(),zone_id.end(),i)){
	//			p3d_col_activate_rob_obj(Robot,Human->o[i]);
	////			if(disp)
	////				cout << "activates body" << Human->o[i]->name << endl;
	//		//}
	//	}
}



/**
 * p3d_DeactivateHris
 */
void Hri::deactivateAllButHriAchile(int zone,int disp) {

	//sort(zone_id.begin(), zone_id.end());

	std::vector<int> zone_id;

	for(unsigned int i=0; i<zones.size();i++)
		zone_id.push_back( zones[i].id );

	for(int i =0; i<Human->no; i++){
		if(!binary_search(zone_id.begin(), zone_id.end(),i)){
			//if(i!=zone_id[zone]){
			p3d_col_deactivate_rob_obj(Robot,Human->o[i]);
			//p3d_col_deactivate_rob_obj(Human,Robot->o[i]);
		}
		else{
			//p3d_col_activate_rob_obj(Robot,Human->o[i]);
			if(disp){
				//cout << "human name = " << Human->name << endl;
				//				cout << "zone_id[zone] = " << zone_id[zone] << endl;
				//
				//				cout << "deactivates body" << Human->o[i]->name << endl;
			}
		}
	}
}
