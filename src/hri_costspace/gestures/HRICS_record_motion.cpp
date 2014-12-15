/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001).
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014
 */
#include "HRICS_record_motion.hpp"

#include "HRICS_gest_parameters.hpp"
#include "HRICS_openrave_human_map.hpp"

#include "API/project.hpp"
#include "planner/planEnvironment.hpp"

#include "utils/NumsAndStrings.hpp"

#include <libmove3d/include/P3d-pkg.h>
#include <libmove3d/include/Planner-pkg.h>
#include <libmove3d/include/Graphic-pkg.h>

#include <sys/time.h>
#include <iomanip>
#include <sstream>
#include <fstream>

using namespace std;
using namespace HRICS;
using namespace Move3D;

std::vector<RecordMotion*> global_motionRecorders;

//--------------------------------------------------------
//--------------------------------------------------------
//--------------------------------------------------------

Move3D::Trajectory HRICS::motion_to_traj( const motion_t& traj, Move3D::Robot* robot, int max_index )
{
    if( traj.empty() )
        return Move3D::Trajectory();

    if( max_index < 0 )
        max_index = traj.size();

    Move3D::Trajectory traj1( robot );
    std::vector<double> dts;

    for( int i=0; i<int(traj.size()) && i<max_index; i++ ) {
        if( traj[i].second->getConfigStruct() == NULL ){
            std::cout << "NULL configuration in " << __PRETTY_FUNCTION__ << std::endl;
        }
        dts.push_back( traj[i].first );
        traj1.push_back( Move3D::confPtr_t( new Move3D::Configuration( robot, traj[i].second->getConfigStruct() )));
    }
    traj1.setUseTimeParameter( true );
    traj1.setUseConstantTime( false );
    traj1.setDeltaTimes( dts );

    // Resample
    Move3D::Trajectory traj2( robot );
    const double dt = 0.01; // 100Hz
    double t = 0.0;
    int ith=0;

    while(1)
    {
        if(!traj2.push_back( traj1.configAtTime(t) ) )
            cout << "no configuration added at : " << ith << endl;

        t += dt;

        ith++;

        if( ( t - traj1.getTimeLength() ) > 1e-3 ) {
            cout << "break in motion_to_traj,  t : " << t << ", traj1.getTimeLength() : " << traj1.getTimeLength() << endl;
            cout << " i : " << traj2.getNbOfViaPoints() << endl;
            cout << " ith : " << ith << endl;
            break;
        }
    }
    traj2.setUseTimeParameter( true );
    traj2.setUseConstantTime( true );
    traj2.setDeltaTime( dt ); // Set index 1 because 0 is often 0.0
    return traj2;
}

//--------------------------------------------------------
//--------------------------------------------------------
//--------------------------------------------------------

RecordMotion::RecordMotion() : m_robot(NULL)
{
    intialize();
}

RecordMotion::RecordMotion( Robot* robot ) : m_robot(robot)
{
    intialize();
}

RecordMotion::~RecordMotion()
{
    reset();
}

void RecordMotion::intialize()
{
    m_use_or_format = true;
    m_is_recording = false;
    m_id_motion = 0;

    reset();
    HRICS::set_human_maps();

    m_transX = 0.10; // IROS PAPER GMMs are using this frame
    m_transY = 0.50;
    m_transZ = 0.15;
    m_transR = 0.00;
}


void RecordMotion::setRobot(const std::string &robotname)
{
    cout << "set robot to : " << m_robot << endl;
    cout << "global_Project : " << global_Project << endl;
    m_robot = global_Project->getActiveScene()->getRobotByName( robotname );
    m_init_q = m_robot->getCurrentPos();
}

void RecordMotion::saveCurrentToFile()
{
    string home(getenv("HOME_MOVE3D"));
    ostringstream file_name;
    file_name << "/statFiles/recorded_motion/motion_saved_";
    file_name << std::setw( 5 ) << std::setfill( '0' ) << m_id_motion << "_";
    file_name << std::setw( 5 ) << std::setfill( '0' ) << m_id_file++ << ".xml";
    saveToXml( home+file_name.str() );
    m_motion.clear();
}

void RecordMotion::saveCurrentConfig()
{
    timeval tim;
    gettimeofday(&tim, NULL);

    double tu = tim.tv_sec+(tim.tv_usec/1000000.0);
    double dt = 0.0;
    if( m_time_last_saved != 0.0 )
        dt = tu - m_time_last_saved;
    m_time_last_saved = tu;

    confPtr_t q = m_robot->getCurrentPos();
    m_motion.push_back( std::make_pair(dt,q) );

    //cout << "Record config for " << m_robot->getName() << " , dt = " << dt << " sec" << endl;

    if( int(m_motion.size()) >= 100 )
    {
        saveCurrentToFile();
    }
}

void RecordMotion::reset()
{
    m_is_recording = false;
    m_id_file = 0;
    m_time_last_saved = 0.0;
    m_motion.clear();
    m_stored_motions.clear();
    m_ith_shown_motion = -1;
}

void RecordMotion::saveToXml(const std::string &filename)
{
    saveToXml( filename, m_motion );
}

void RecordMotion::saveToXml(const string &filename, const vector< pair<double,confPtr_t> >& motion )
{
    if( motion.empty() ) {
        cout << "No file to save empty vector in " << __PRETTY_FUNCTION__ << endl;
        return;
    }

    stringstream ss;
    string str;

    //Creating the file Variable version 1.0
    xmlDocPtr doc = xmlNewDoc( xmlCharStrdup("1.0") );

    //Writing the root node
    xmlNodePtr root = xmlNewNode( NULL, xmlCharStrdup("StoredConfiguration") );

    //nb node
    str.clear(); ss << motion.size(); ss >> str; ss.clear();
    xmlNewProp (root, xmlCharStrdup("nb_config"), xmlCharStrdup(str.c_str()));

    for (int i=0; i<int(motion.size()); i++)
    {
        ss << std::setw( ceil(log10(motion.size())) ) << std::setfill( '0' ) <<  i; ss >> str; ss.clear();
        string name = "config_" + str;
        xmlNodePtr cur            = xmlNewChild (root, NULL, xmlCharStrdup(name.c_str()), NULL);

        char str[80];
        sprintf( str, "%f", motion[i].first );
        xmlNodePtr curTime = xmlNewChild (cur, NULL, xmlCharStrdup("time"), NULL);
        xmlNewProp ( curTime, xmlCharStrdup("seconds"), xmlCharStrdup(str) );

        writeXmlRobotConfig( cur, static_cast<p3d_rob*>( m_robot->getP3dRobotStruct() ), motion[i].second->getConfigStruct() );
    }

    xmlDocSetRootElement(doc, root);

    //Writing the file on HD
    xmlSaveFormatFile (filename.c_str(), doc, 1);
    xmlFreeDoc(doc);

    cout << "Writing motion to : " << filename << endl;
}

motion_t RecordMotion::loadFromXml(const string& filename)
{
    motion_t vectConfs;
    vectConfs.clear();
    xmlDocPtr doc;
    xmlNodePtr cur;
    xmlNodePtr root;

    doc = xmlParseFile(filename.c_str());
    if(doc==NULL)
    {
        cout << "Document not parsed successfully (doc==NULL)" << endl;
        return vectConfs;
    }

    root = xmlDocGetRootElement(doc);
    if (root == NULL)
    {
        cout << "Document not parsed successfully" << endl;
        xmlFreeDoc(doc);
        return vectConfs;
    }

    if (xmlStrcmp(root->name, xmlCharStrdup("StoredConfiguration")))
    {
        cout << "Document of the wrong type root node not StoredConfiguration" << endl;
        xmlFreeDoc(doc);
        return vectConfs;
    }

    xmlChar* tmp = xmlGetProp(root, xmlCharStrdup("nb_config"));
    if (tmp == NULL)
    {
        cout << "No prop named nb_config" << endl;
        return vectConfs;
    }
    int nb_config = 0;
    sscanf((char *) tmp, "%d", &nb_config );
    xmlFree(tmp);

    //---------------------------------------------------------------
    //---------------------------------------------------------------

    cur = root->xmlChildrenNode->next;
    int i =0;

    vectConfs.clear();
    while ( i < nb_config )
    {
        if (cur == NULL)
        {
            cout << "Document error on the number of config" << endl;
            return vectConfs;
        }

        std::string name(reinterpret_cast<const char*>(cur->name));

        if (name.find("config") != string::npos )
        {
            i++;

            xmlNodePtr ptrTime = cur->xmlChildrenNode->next;
            if ( xmlStrcmp( ptrTime->name, xmlCharStrdup("time") ) )
            {
                cout << "Error: no time" << endl;
                return vectConfs;
            }
            xmlChar* tmp = xmlGetProp( ptrTime, xmlCharStrdup("seconds") );
            if (tmp == NULL)
            {
                cout << "Error: no prop named seconds" << endl;
                return vectConfs;
            }
            float time = 0;
            sscanf((char *) tmp, "%f", &time );
            xmlFree(tmp);

            //cout << "time : " << time << endl;

            configPt q = NULL;

            if( m_robot->getUseLibmove3dStruct() )
                q = readXmlConfig( static_cast<p3d_rob*>( m_robot->getP3dRobotStruct() ), cur->xmlChildrenNode->next->next->next );

            if( q == NULL ) {
                cout << "Error : in readXmlConfig" << endl;
                break;
            }
            confPtr_t q_tmp(new Configuration(m_robot,q,true));
            q_tmp->adaptCircularJointsLimits();
            vectConfs.push_back( make_pair(time,q_tmp) );
            //vectConfs.back().second->print();
        }

        cur = cur->next;
    }

    return vectConfs;
}

void RecordMotion::loadXMLFolder()
{
    std::string foldername = std::string( getenv("HOME_MOVE3D" ) ) + std::string( "/statFiles/recorded_motion/" );
    loadXMLFolder(foldername);
}

bool RecordMotion::loadXMLFolder(  const std::string& foldername  )
{
    //std::string foldername = "/home/jmainpri/workspace/move3d/libmove3d/statFiles/recorded_motion/";
    cout << "Load Folder : " << foldername << endl;

    std::string command = "ls " + foldername;
    FILE* fp = popen( command.c_str(), "r");
    if (fp == NULL) {
        cout << "ERROR in system call" << endl;
        return false;
    }
    char path[PATH_MAX]; int max_number_of_motions=0;
    while ( fgets( path, PATH_MAX, fp) != NULL ) max_number_of_motions++;
    pclose(fp);

    if( max_number_of_motions == 0) cout << "no file in folder" << endl;

    // Set the motion number you want to load
    max_number_of_motions = 200;
    int first_motion = 0;
    int number_of_motions_loaded = 0;
    const int max_number_of_files = 500;

    reset();

    for( int i=first_motion; i<(first_motion+max_number_of_motions); i++ )
    {
        for( int j=0; j<max_number_of_files; j++ )
        {
            std::ostringstream filename;
            filename << foldername << "motion_saved_";
            filename << std::setw( 5 ) << std::setfill( '0' ) << i << "_";
            filename << std::setw( 5 ) << std::setfill( '0' ) << j << ".xml";

            std::ifstream file_exists( filename.str().c_str() );

            if( file_exists )
            {
                //cout << "Load File : " << filename.str() << endl;
                motion_t partial_motion = loadFromXml( filename.str() );
                storeMotion( partial_motion, "", j == 0 );

                cout << "partial_motion.size() : " << partial_motion.size() << endl;

                if( j == 0 ) {
                    number_of_motions_loaded++;
                }
            }
            else {
                break;
            }
        }
    }
    cout << "Number of motion loaded : " << number_of_motions_loaded << endl;
    return true;
}

std::vector<std::string> RecordMotion::listFolder( const std::string& foldername, std::string ext, bool quiet ) const
{
    if( !quiet ) {
        cout << "Load Folder : " << foldername << endl;
    }

    std::vector<std::string> files;

    std::string command = "ls " + foldername;
    FILE* fp = popen( command.c_str(), "r");
    if (fp == NULL) {
        cout << "ERROR in system call" << endl;
        return files;
    }

    char str[PATH_MAX];
    while ( fgets( str, PATH_MAX, fp) != NULL )
    {
        std::string filename( str );
        filename = filename.substr(0, filename.size()-1);
        std::string extension( filename.substr( filename.find_last_of(".") + 1 ) );
        //cout << extension << endl;
        if( extension == ext )
        {
            char id = filename.at( filename.find_last_of(".")-1 );
            if( true /*id == '0'*/ ){
                if( !quiet ) {
                    cout << "add : " << filename << endl;
                }
                files.push_back( filename );
            }
        }
    }
    pclose(fp);
    return files;
}

void RecordMotion::loadCSVFolder( const std::string& foldername, bool quiet, std::string base_name )
{
    std::vector<std::string> files = listFolder( foldername, "csv", quiet );
    if( files.empty() )
    {
        cout << "Folder " << foldername << " is empty!!!" << endl;
        return;
    }

    for(size_t i=0; i<files.size(); i++)
    {
        if ( files[i].find( base_name ) != std::string::npos ) // true if file name contains base_name
        {
            motion_t motion = loadFromCSV( foldername + "/" + files[i], quiet );
            m_stored_motions.push_back( motion );
            m_stored_motions_names.push_back( files[i] );
        }
    }

    if( !quiet ) {
        cout << "m_stored_motions.size() : " << m_stored_motions.size() << endl;
    }
}

void RecordMotion::loadCSVFolder( const std::string& foldername, bool quiet, double threshold )
{
    std::vector<std::string> files = listFolder( foldername, "csv", quiet );
    if( files.empty() )
    {
        cout << "Folder " << foldername << " is empty!!!" << endl;
        return;
    }

    m_stored_motions.clear();
    m_stored_motions_names.clear();

    for(size_t i=0; i<files.size(); i++)
    {
        motion_t motion = loadFromCSV( foldername + "/" + files[i], quiet );

        if( threshold != 0.0 )
        {
            if( threshold < 0.0 )
            {
                if( (*motion[0].second)[6] < fabs(threshold) )
                {
                    m_stored_motions.push_back( motion );
                    m_stored_motions_names.push_back( files[i] );
                }
            }

            if( threshold > 0.0 )
            {
                if( (*motion[0].second)[6] > fabs(threshold) )
                {
                    m_stored_motions.push_back( motion );
                    m_stored_motions_names.push_back( files[i] );
                }
            }
        }
        else {
            m_stored_motions.push_back( motion );
            m_stored_motions_names.push_back( files[i] );
        }

        //        std::string filename( files[i].substr( 0, files[i].find_last_of(".") - 1 ) + "1.csv" );
        //        std::string path = foldername + "/" + filename;
        //        std::ifstream file_exists( path.c_str() );
        //        if( file_exists )
        //        {
        //            motion_t motion1 = loadFromCSV( path, quiet );
        //            m_stored_motions.back().insert( m_stored_motions.back().end(), motion1.begin(), motion1.end() );
        //        }
    }

    if( !quiet ) {
        cout << "m_stored_motions.size() : " << m_stored_motions.size() << endl;
    }
}

void RecordMotion::loadMotionFromMultipleFiles( const string& baseFilename, int number_of_files)
{
    m_motion.clear();

    stringstream ss;
    vector< pair<double,confPtr_t> > partial_motion;

    for( int i=0;i<number_of_files;i++)
    {
        string num;
        ss << std::setw( 5 ) << std::setfill( '0' ) << i; ss >> num; ss.clear();
        partial_motion = loadFromXml( baseFilename + num + ".xml");
        addToCurrentMotion( partial_motion );
    }
}

void RecordMotion::addToCurrentMotion( const motion_t& motion )
{
    m_motion.insert( m_motion.end(), motion.begin(), motion.end() );
}

void RecordMotion::storeMotion( const motion_t& motion, std::string name, bool new_motion )
{
    if( new_motion )
    {
        m_stored_motions_names.push_back( name );
        m_stored_motions.push_back( motion );
    }
    else
    {
        // Store motion at the end of the last motion
        if( !m_stored_motions.empty() )
        {
            m_stored_motions.back().insert( m_stored_motions.back().end(), motion.begin(), motion.end() );
        }
    }
}

bool RecordMotion::setRobotToConfiguration(int ith)
{
    if( ith < 0 || ith >= int(m_motion.size()) ) {
        cout << "index out of range in " << __PRETTY_FUNCTION__ << endl;
        return false;
    }

    cout << m_motion[ith].first << endl;
    m_robot->setAndUpdate( *m_motion[ith].second );
    return true;
}

bool RecordMotion::setRobotToStoredMotionConfig(int motion_id, int config_id)
{
    if( motion_id < 0 || ( motion_id > int(m_stored_motions.size())))
    {
        cout << "index out of stored motion range in " << __PRETTY_FUNCTION__ << endl;
        return false;
    }

    if( config_id < 0 || ( config_id >= int(m_stored_motions[motion_id].size()))) {
        cout << "index out of range in " << __PRETTY_FUNCTION__ << endl;
        return false;
    }

    m_robot->setAndUpdate( *m_stored_motions[motion_id][config_id].second );

    return true;
}

bool RecordMotion::setShowMotion(int ith)
{
    if( ith == -1) {
        cout << "disable ith show motion" << endl;
    }
    else if( ith < 0 || ith >= int(m_stored_motions.size()) ) {
        cout << "index out of range in " << __PRETTY_FUNCTION__ << endl;
        cout << "ith : " << ith << endl;
        cout << "m_stored_motions.size() : " << m_stored_motions.size() << endl;
        return false;
    }

    m_ith_shown_motion = ith;
    return true;
}

void RecordMotion::showStoredMotion()
{
    cout << "show motion (" << m_ith_shown_motion << ")" << endl;
    if( m_ith_shown_motion == -1 )
    {
        for (int i=0; i<int(m_stored_motions.size()); i++)
        {
            cout << "Show motion : " << i << " with " <<  m_stored_motions[i].size() << " frames" << endl;
            showMotion(  m_stored_motions[i] );
            //showMotion( getArmTorsoMotion( m_stored_motions[i], m_robot->getCurrentPos() ) );
        }
    }
    else
    {
        showMotion( getArmTorsoMotion( m_stored_motions[m_ith_shown_motion], m_robot->getCurrentPos() ) );
        //        showMotion( m_stored_motions[m_ith_shown_motion] );
    }
}

void RecordMotion::showCurrentMotion()
{
    showMotion( m_motion );
}

void RecordMotion::showMotion( const motion_t& motion )
{
    if( motion.empty() ) {
        cout << "warning : motion is empty" << endl;
        return;
    }

    int StopRun = false;
    int i=0;
    double tu_last = 0.0;
    double dt = 0.0;

    confPtr_t q_cur = motion[0].second;

    m_robot->setAndUpdate( *q_cur );
    g3d_draw_allwin_active();

    while ( !StopRun )
    {
        timeval tim;
        gettimeofday(&tim, NULL);
        double tu = tim.tv_sec+(tim.tv_usec/1000000.0);
        dt += ( tu - tu_last );
        tu_last = tu;

        if ( dt>=motion[i].first ) {

            // cout << "-------------------------------" << endl;
            // q_cur->equal( *motion[i].second, true );
            q_cur = motion[i].second;
            m_robot->setAndUpdate( *q_cur );
            // cout << "dt : " << dt << " , m_motion[i].first : " << motion[i].first << endl;
            // motion[i].second->print();
            // motion[i].second->adaptCircularJointsLimits();
            // cout << (*motion[i].second)[11] << endl;
            g3d_draw_allwin_active();
            dt = 0.0;
            i++;
        }

        if ( i >= int(motion.size())) {
            StopRun = true;
        }

        if ( PlanEnv->getBool(PlanParam::stopPlanner) ) {
            StopRun = true;
        }
    }
}

void RecordMotion::drawMotion( const motion_t& motion, int nb_frames=-1 )
{
    if( motion.empty() ) {
        return;
    }

    G3D_Window *win = g3d_get_cur_win();

    static_cast<p3d_rob*>( m_robot->getP3dRobotStruct() )->draw_transparent = false;
    p3d_rob* rob = (p3d_rob*)(p3d_get_desc_curid( P3D_ROBOT ));

    int delta = nb_frames > 0 ? int( double(motion.size()) / double(nb_frames)) : 1;

    for ( int i=0; i<int(motion.size()); i += delta )
    {
        win->vs.transparency_mode= G3D_TRANSPARENT_AND_OPAQUE;

        m_robot->setAndUpdate( *motion[i].second );

        p3d_sel_desc_num( P3D_ROBOT, static_cast<p3d_rob*>( m_robot->getP3dRobotStruct() )->num );
        g3d_draw_robot( static_cast<p3d_rob*>( m_robot->getP3dRobotStruct() )->num, win, 0 );
        drawHeraklesArms();
    }

    p3d_sel_desc_num( P3D_ROBOT, rob->num );
    static_cast<p3d_rob*>( m_robot->getP3dRobotStruct() )->draw_transparent = false;
}

void RecordMotion::dawColorSkinedCylinder(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2)
{
    // Skin color
    double color[4] = { 0.901961, 0.772549, 0.694118, 0.0 };

    p3d_vector3 pos1,pos2,pos3;
    pos1[0] = p1[0]; pos1[1] = p1[1]; pos1[2] = p1[2];
    pos2[0] = p2[0]; pos2[1] = p2[1]; pos2[2] = p2[2];

    // Find point that is still skin colored
    p3d_vectSub( pos2 , pos1 , pos3 );
    p3d_vectNormalize( pos3, pos3 );
    p3d_vectScale( pos3, pos3, 0.15 );
    p3d_vectAdd( pos1 , pos3, pos3 );
    g3d_set_color_vect( Any, color);
    g3d_draw_cylinder( pos3, pos2, 0.050, 100 );

}

void RecordMotion::drawHeraklesArms()
{
    bool error = false;

    if( m_robot->getName().find("HERAKLES_HUMAN") != string::npos )
    {
        Eigen::Vector3d p1, p2; Joint* jnt = NULL;

        // Bras Droit
        jnt = m_robot->getJoint( "rShoulderX" );
        if(!jnt)
            error = true;
        else
            p1 = jnt->getVectorPos();

        jnt = m_robot->getJoint( "rElbowZ" );
        if(!jnt)
            error = true;
        else
            p2 = jnt->getVectorPos();

        if ( ( p1 - p2 ).norm() > 0.10 )
        {
            dawColorSkinedCylinder( p1,  p2 );
        }

        // Bras Gauche
        jnt = m_robot->getJoint( "lShoulderX" );
        if(!jnt)
            error = true;
        else
            p1 = jnt->getVectorPos();

        jnt = m_robot->getJoint( "lElbowZ" );
        if(!jnt)
            error = true;
        else
            p2 = jnt->getVectorPos();

        if ( ( p1 - p2 ).norm() > 0.10 )
        {
            dawColorSkinedCylinder( p1,  p2 );
        }
    }
    else {
        error = true;
    }

    if (error) {
        cout << "Could not find one of the joints in hri_draw_kinect_human_arms" << endl;
    }
}

void RecordMotion::draw()
{
    drawHeraklesArms();

    if( GestEnv->getBool(GestParam::draw_recorded_motion) )
    {
        if( !m_stored_motions.empty()  && (m_ith_shown_motion > -1) )
        {
            confPtr_t q = m_robot->getCurrentPos();
            drawMotion( getArmTorsoMotion( resample( m_stored_motions[m_ith_shown_motion], 10 ), q ) );
        }
    }
}

motion_t RecordMotion::extractSubpart( int begin, int end )
{
    return extractSubpart( begin, end, m_motion );
}

motion_t RecordMotion::extractSubpart( int begin, int end, const motion_t& motion )
{
    motion_t vectConfs;

    if( begin < 0 || end >= int(motion.size()) )
    {
        cout << "indexes are not good in " << __PRETTY_FUNCTION__ << endl;
        return vectConfs;
    }

    motion_t::const_iterator it_1 = motion.begin() + begin;
    motion_t::const_iterator it_2 = motion.begin() + end;

    vectConfs = motion_t( it_1, it_2 );

    return vectConfs;
}

motion_t RecordMotion::resample(const motion_t& motion, int nb_sample )
{
    motion_t resampled_motion;

    if( int(motion.size()) == nb_sample )
    {
        resampled_motion = motion;
        for (int j=0; j<nb_sample; j++)
        {
            resampled_motion[j].second = motion[j].second->copy();
            resampled_motion[j].second->adaptCircularJointsLimits();
        }
        return resampled_motion;
    }

    cout << "RESAMPLE" << endl;

    double inc = double(motion.size())/double(nb_sample);
    double k =0;

    for (int j=0; j<nb_sample; j++)
    {
        confPtr_t q = motion[floor(k)].second->copy();
        q->adaptCircularJointsLimits();
        resampled_motion.push_back( make_pair(0.02,q) );
        k += inc;
    }

    return resampled_motion;
}

void RecordMotion::resampleAll( int nb_sample )
{
    for( size_t i=0;i<m_stored_motions.size();i++){
        m_stored_motions[i] = resample( m_stored_motions[i], nb_sample );
    }
}

motion_t RecordMotion::getArmTorsoMotion( const motion_t& motion, confPtr_t q )
{
    motion_t result( motion.size() );

    for( int i=0; i<int(motion.size()); i++ )
    {
        (*q)[6] = (*motion[i].second)[6];  // PelvisX
        (*q)[7] = (*motion[i].second)[7];  // PelvisY
        (*q)[8] = (*motion[i].second)[8];  // PelvisZ
        (*q)[11] =(*motion[i].second)[11]; // PelvisRX
        (*q)[12] =(*motion[i].second)[12]; // TorsoX
        (*q)[13] =(*motion[i].second)[13]; // TorsoY
        (*q)[14] =(*motion[i].second)[14]; // TorsoZ

        (*q)[18] =(*motion[i].second)[18]; // rShoulderX
        (*q)[19] =(*motion[i].second)[19]; // rShoulderZ
        (*q)[20] =(*motion[i].second)[20]; // rShoulderY
        (*q)[21] =(*motion[i].second)[21]; // rArmTrans
        (*q)[22] =(*motion[i].second)[22]; // rElbowZ

        result[i].first = motion[i].first;
        result[i].second = q->copy();
    }

    return result;
}

void RecordMotion::saveStoredToCSV( const std::string &filename , bool break_into_files )
{
    if( m_stored_motions.empty() )
    {
        cout << "No stored motion in " << __PRETTY_FUNCTION__ << endl;
        return;
    }

    //Breaks the prediction for IROS
    //    const int samples = 100;
    //    cout << "Down sampling to " << samples << endl;
    //    for (int i=0; i<int(m_stored_motions.size()); i++)
    //    {
    //        m_stored_motions[i] = resample( m_stored_motions[i], samples );
    //    }

    std::ofstream s;
    if( !break_into_files )
    {
        s.open( filename.c_str() );
        cout << "Opening save file : " << filename << endl;
    }

    Eigen::Vector3d pos;
    Eigen::Transform3d T;

    const bool save_cartesian=false;

    for (int i=0; i<int(m_stored_motions.size()); i++)
    {
        if( break_into_files )
        {
            s.close();

            std::stringstream converter;
            converter << std::setw( ceil(log10(m_stored_motions.size())) ) << std::setfill( '0' ) <<  i;
            std::string name =  filename + "_" + converter.str() + ".csv" ;

            cout << "Opening save file : " << name << endl;
            s.open( name.c_str() );
        }

        for (int j=0; j<int(m_stored_motions[i].size()); j++)
        {
            s << j << ",";

            if( save_cartesian )
            {
                m_robot->setAndUpdate( *m_stored_motions[i][j].second );

                if( j == 0)
                {
                    T = m_robot->getJoint("Pelvis")->getMatrixPos().inverse();
                }

                pos = T*m_robot->getJoint("rWristX")->getVectorPos();
                for (int k=0; k<int(pos.size()); k++)
                    s << pos[k] << ",";

                pos = T*m_robot->getJoint("rElbowZ")->getVectorPos();
                for (int k=0; k<int(pos.size()); k++)
                    s << pos[k] << ",";

                s << endl;
            }
            else
            {
                m_stored_motions[i][j].second->adaptCircularJointsLimits();

                // This is because the data was recorded near limits
                s << (*m_stored_motions[i][j].second)[6] << ","; // PelvisX
                s << (*m_stored_motions[i][j].second)[7] << ","; // PelvisY
                s << (*m_stored_motions[i][j].second)[8] << ","; // PelvisZ
                s << (*m_stored_motions[i][j].second)[11] << ","; // PelvisRX
                s << (*m_stored_motions[i][j].second)[12] << ","; // TorsoX
                s << (*m_stored_motions[i][j].second)[13] << ","; // TorsoY
                s << (*m_stored_motions[i][j].second)[14] << ","; // TorsoZ

                s << (*m_stored_motions[i][j].second)[18] << ","; // rShoulderX
                s << (*m_stored_motions[i][j].second)[19] << ","; // rShoulderZ
                s << (*m_stored_motions[i][j].second)[20] << ","; // rShoulderY
                s << (*m_stored_motions[i][j].second)[21] << ","; // rArmTrans
                s << (*m_stored_motions[i][j].second)[22] << endl; // rElbowZ

                // s << (*m_stored_motions[i][j].second)[27] << ","; // lShoulderX
                // s << (*m_stored_motions[i][j].second)[28] << ","; // lShoulderZ
                // s << (*m_stored_motions[i][j].second)[29] << ","; // lShoulderY
                // s << (*m_stored_motions[i][j].second)[30] << ","; // lArmTrans
                // s << (*m_stored_motions[i][j].second)[31] << endl; // lElbowZ
            }
        }
    }

    s << endl;
    cout << "Saved " << m_stored_motions.size() << " stored motions" << endl;
    s.close();
}

void RecordMotion::saveToCSV( const std::string &filename, const motion_t& motion )
{
    std::ofstream s;
    s.open( filename.c_str() );
    //cout << "Opening save file : " << oss.str() << endl;

    bool save_cartesian = false;

    for (int i=0; i<int(motion.size()); i++)
    {
        s << i << ",";

        if( save_cartesian )
        {
            Eigen::Vector3d pos;

            m_robot->setAndUpdate( *motion[i].second );

            Eigen::Transform3d T( m_robot->getJoint("Pelvis")->getMatrixPos().inverse() );

            pos = T*m_robot->getJoint("rWristX")->getVectorPos();
            for (int j=0; j<int(pos.size()); j++)
                s << pos[j] << ",";

            pos = T*m_robot->getJoint("rElbowZ")->getVectorPos();
            for (int j=0; j<int(pos.size()); j++)
                s << pos[j] << ",";
        }
        else
        {
            s << motion[i].second->at(6) << ","; // PelvisX
            s << motion[i].second->at(7) << ","; // PelvisY
            s << motion[i].second->at(8) << ","; // PelvisZ
            s << motion[i].second->at(11) << ","; // PelvisRX
            s << motion[i].second->at(12) << ","; // TorsoX
            s << motion[i].second->at(13) << ","; // TorsoY
            s << motion[i].second->at(14) << ","; // TorsoZ

            s << motion[i].second->at(18) << ","; // rShoulderX
            s << motion[i].second->at(19) << ","; // rShoulderZ
            s << motion[i].second->at(20) << ","; // rShoulderY
            s << motion[i].second->at(21) << ","; // rArmTrans
            s << motion[i].second->at(22) ; // rElbowZ
        }

        s << endl;
    }

    s << endl;
    //cout << "Closing save file" << endl;
    s.close();
}

void RecordMotion::translateStoredMotions()
{
    cout << "Translate all stored motions" << endl;

    for(int i=0;i<int(m_stored_motions.size());i++)
    {
        for(int j=0;j<int(m_stored_motions[i].size());j++)
        {
            confPtr_t q = m_stored_motions[i][j].second;
            (*q)[6]  += m_transX; // X
            (*q)[7]  += m_transY; // Y
            (*q)[8]  += m_transZ; // Z
            (*q)[11] += m_transR; // theta
        }
    }
}

void RecordMotion::invertTranslationStoredMotions()
{
    cout << "Invert translate all stored motions" << endl;

    for(int i=0;i<int(m_stored_motions.size());i++)
    {
        m_stored_motions[i] = invertTranslation( m_stored_motions[i] );
    }
}

motion_t RecordMotion::invertTranslation( const motion_t& motion )
{
    motion_t motion_trans = motion;

    for(int j=0;j<int(motion_trans.size());j++)
    {
        confPtr_t q = motion_trans[j].second->copy();
        (*q)[6]  -= m_transX; // X
        (*q)[7]  -= m_transY; // Y
        (*q)[8]  -= m_transZ; // Z
        (*q)[11] -= m_transR; // theta
        motion_trans[j].second = q;
    }
    return motion_trans;
}

Eigen::Transform3d RecordMotion::getOffsetTransform()
{
    Eigen::Transform3d T;

    T.translation()(0) = m_transX;
    T.translation()(1) = m_transY;
    T.translation()(2) = m_transZ;

    Eigen::Matrix3d rot = Eigen::Matrix3d( Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
                                           * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
                                           * Eigen::AngleAxisd(m_transR, Eigen::Vector3d::UnitZ()) );

    T.linear() = rot;

    return T;
}

bool  RecordMotion::loadRegressedFromCSV( const std::string& foldername )
{
    m_stored_motions.clear();

    m_use_or_format = false; // OpenRave format

    motion_t tmp;

    tmp = loadFromCSV( foldername + "traj_class_1.csv" );
    if( tmp.empty() ){
        return false;
    }
    m_stored_motions.push_back( tmp );

    tmp = loadFromCSV( foldername + "traj_class_2.csv" );
    if( tmp.empty() ){
        return false;
    }
    m_stored_motions.push_back( tmp );

    tmp = loadFromCSV( foldername + "traj_class_3.csv" );
    if( tmp.empty() ){
        return false;
    }
    m_stored_motions.push_back( tmp );

    tmp = loadFromCSV( foldername + "traj_class_4.csv" );
    if( tmp.empty() ){
        return false;
    }
    m_stored_motions.push_back( tmp );

    tmp = loadFromCSV( foldername + "traj_class_5.csv" );
    if( tmp.empty() ){
        return false;
    }
    m_stored_motions.push_back( tmp );

    tmp = loadFromCSV( foldername + "traj_class_6.csv" );
    if( tmp.empty() ){
        return false;
    }
    m_stored_motions.push_back( tmp );

    tmp = loadFromCSV( foldername + "traj_class_7.csv" );
    if( tmp.empty() ){
        return false;
    }
    m_stored_motions.push_back( tmp );

    tmp = loadFromCSV( foldername + "traj_class_8.csv" );
    if( tmp.empty() ){
        return false;
    }
    m_stored_motions.push_back( tmp );

    return true;
}

confPtr_t RecordMotion::getConfigOpenRave( const std::vector<std::string>& config ) const
{
    std::vector<double> tmp(config.size());

    for(int i=0;i<int(config.size());i++)
        convert_text_to_num<double>( tmp[i], config[i], std::dec );

    confPtr_t q = m_robot->getCurrentPos();

    for( std::map<std::string,int>::iterator it_map=herakles_openrave_map.begin();
         it_map!=herakles_openrave_map.end(); it_map++ )
    {
        (*q)[ herakles_move3d_map[it_map->first] ] = tmp[ it_map->second ];
    }

    return q;
}

std::pair<double,confPtr_t> RecordMotion::getConfigBio( const std::vector<std::string>& config ) const
{
    std::vector<double> tmp(config.size());

    for(int i=0;i<int(config.size());i++)
        convert_text_to_num<double>( tmp[i], config[i], std::dec );

    std::pair<double,confPtr_t> time_config;

    time_config.first = tmp[0];

    confPtr_t q = m_robot->getCurrentPos();

    for( std::map<std::string,int>::iterator it_map=herakles_bio_openrave_map.begin();
         it_map!=herakles_bio_openrave_map.end(); it_map++ )
    {
        (*q)[ herakles_bio_move3d_map[it_map->first] ] = tmp[ it_map->second+1 ]; // Add one for time

//        if ( it_map->first == "rShoulderTransY" ) {
//            cout << it_map->first << " : " << tmp[ it_map->second+1 ] << endl;
//        }
        // TODO REMOVE THAT SHOULDER HACK
        // Should work on left arm model to avoid seting the shoulder in the parser
        if ( it_map->first == "lShoulderX" ) {
            (*q)[ herakles_bio_move3d_map[it_map->first] ] = -M_PI/2; // Add one for time
        }
    }

    time_config.second = q;

    return time_config;
}

confPtr_t RecordMotion::getConfigTwelveDoF( const std::vector<std::string>& config ) const
{
    confPtr_t q = m_robot->getCurrentPos();

    convert_text_to_num<double>( (*q)[6], config[1], std::dec ); // Pelvis
    convert_text_to_num<double>( (*q)[7], config[2], std::dec ); // Pelvis
    convert_text_to_num<double>( (*q)[8], config[3], std::dec ); // Pelvis
    convert_text_to_num<double>( (*q)[11], config[4], std::dec ); // Pelvis
    convert_text_to_num<double>( (*q)[12], config[5], std::dec ); // TorsoX
    convert_text_to_num<double>( (*q)[13], config[6], std::dec ); // TorsoY
    convert_text_to_num<double>( (*q)[14], config[7], std::dec ); // TorsoZ

    convert_text_to_num<double>( (*q)[18], config[8], std::dec ); // rShoulderX
    convert_text_to_num<double>( (*q)[19], config[9], std::dec ); // rShoulderZ
    convert_text_to_num<double>( (*q)[20], config[10], std::dec ); // rShoulderY
    convert_text_to_num<double>( (*q)[21], config[11], std::dec ); // rArmTrans
    convert_text_to_num<double>( (*q)[22], config[12], std::dec ); // rElbowZ

    return q;
}

motion_t RecordMotion::loadFromCSV( const std::string& filename, bool quiet ) const
{
    if(!quiet) {
        cout << "Loading from CSV : " << filename << endl;
    }

    std::ifstream       file;
    std::vector< std::vector<std::string> >   matrix;
    std::vector< std::string >   row;
    std::string                line;
    std::string                cell;

    file.open( filename.c_str(), std::ifstream::in );
    if( !file.is_open()){
        cout << "Error opening file" << endl;
    }

    while( file.good() )
    {
        std::getline( file, line );
        std::stringstream lineStream( line );
        row.clear();

        while( std::getline( lineStream, cell, ',' ) )
        {
            row.push_back( cell );
        }

        if( !row.empty() )
            matrix.push_back( row );
    }

    motion_t motion;

    if( matrix.empty() ) {
        cout << "no data has been loaded" << endl;
        return motion;
    }
    //    cout << "matrix fully loaded" << endl;
    //    cout << "size : " << matrix.size() << " , " << matrix[0].size() << endl;
    //    cout << "m_robot->getNumberOfActiveDoF()" << m_robot->getNumberOfActiveDoF() << endl;
    for (int i=0; i<int(matrix.size()); i++)
    {
        std::pair<double,confPtr_t> config;

        if( m_use_bio_format )
        {
             config = getConfigBio( matrix[i] );
        }
        else if( m_use_or_format )
        {
            confPtr_t q = getConfigOpenRave( matrix[i] );
            config.first = 0.02;
            config.second = q;
            q->adaptCircularJointsLimits();
        }
        else
        {
            confPtr_t q = getConfigTwelveDoF( matrix[i] );
            config.first = 0.02;
            config.second = q;
            q->adaptCircularJointsLimits();
        }

//        cout << "dt : " << config.first << endl;
//        config.second->print();

        motion.push_back( config );
    }

//    exit(0);

    return motion;
}
