#include "HRICS_RecordMotion.hpp"

#include "HRICS_GestParameters.hpp"

#include "API/project.hpp"
#include "planner/planEnvironment.hpp"

#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Graphic-pkg.h"

#include <sys/time.h>
#include <iomanip>
#include <sstream>
#include <fstream>

using namespace std;
using namespace HRICS;

std::vector<RecordMotion*> global_motionRecorders;

template <class T>
bool convert_text_to_num(T& t,
                 const std::string& s,
                 std::ios_base& (*f)(std::ios_base&))
{
  std::istringstream iss(s);
  return !(iss >> f >> t).fail();
}


std::vector< std::vector<double> > convert_text_matrix_to_double(const std::vector< std::vector< std::string > >& matrix )
{
    std::vector< std::vector<double> > result(matrix.size());

    for( int i=0; i<int(matrix.size()); i++ )
    {
        result[i].resize( matrix[i].size() );

        for( int j=0; j<int(matrix[i].size()); j++ )
        {
            std::istringstream convert( matrix[i][j] );
            convert >> result[i][j];
        }
    }

    return result;
}

std::map<std::string,int> move3d_map;
std::map<std::string,int> or_map;

void set_maps()
{
    move3d_map["PelvisTransX"]  =6;
    move3d_map["PelvisTransY"]  =7;
    move3d_map["PelvisTransZ"]  =8;
    move3d_map["PelvisRotX"]    =9;
    move3d_map["PelvisRotY"]    =10;
    move3d_map["PelvisRotZ"]    =11;
    move3d_map["TorsoX"]        =12;
    move3d_map["TorsoY"]        =13;
    move3d_map["TorsoZ"]        =14;
    move3d_map["HeadZ"]         =15;
    move3d_map["HeadY"]         =16;
    move3d_map["HeadX"]         =17;
    move3d_map["rShoulderX"]    =18;
    move3d_map["rShoulderZ"]    =19;
    move3d_map["rShoulderY"]    =20;
    move3d_map["rArmTrans"]     =21;
    move3d_map["rElbowZ"]       =22;
    move3d_map["rWristX"]       =24;
    move3d_map["rWristY"]       =25;
    move3d_map["rWristZ"]       =26;
    move3d_map["lShoulderX"]    =27;
    move3d_map["lShoulderZ"]    =28;
    move3d_map["lShoulderY"]    =29;
    move3d_map["lArmTrans"]     =30;
    move3d_map["lElbowZ"]       =31;
    move3d_map["lWristX"]       =33;
    move3d_map["lWristY"]       =34;
    move3d_map["lWristZ"]       =35;
    move3d_map["rHipX"]         =36;
    move3d_map["rHipY"]         =37;
    move3d_map["rHipZ"]         =38;
    move3d_map["rKnee"]         =39;
    move3d_map["rAnkleX"]       =40;
    move3d_map["rAnkleY"]       =41;
    move3d_map["rAnkleZ"]       =42;
    move3d_map["lHipX"]         =43;
    move3d_map["lHipY"]         =44;
    move3d_map["lHipZ"]         =45;
    move3d_map["lKnee"]         =46;
    move3d_map["lAnkleX"]       =47;
    move3d_map["lAnkleY"]       =48;
    move3d_map["lAnkleZ"]       =49;

    or_map["PelvisTransX"] =  0;
    or_map["PelvisTransY"] =  1;
    or_map["PelvisTransZ"] =  2;
    or_map["PelvisRotX"] =    3;
    or_map["PelvisRotY"] =    4;
    or_map["PelvisRotZ"] =    5;
    or_map["TorsoX"] =        6;
    or_map["TorsoY"] =        7;
    or_map["TorsoZ"] =        8;
    or_map["HeadZ"] =         9;
    or_map["HeadY"] =         10;
    or_map["HeadX"] =         11;
    or_map["rShoulderX"] =    12;
    or_map["rShoulderZ"] =    13;
    or_map["rShoulderY"] =    14;
    or_map["rArmTrans"] =     15;
    or_map["rElbowZ"] =       16;
    or_map["rWristX"] =       17;
    or_map["rWristY"] =       18;
    or_map["rWristZ"] =       19;
    or_map["lShoulderX"] =    20;
    or_map["lShoulderZ"] =    21;
    or_map["lShoulderY"] =    22;
    or_map["lArmTrans"] =     23;
    or_map["lElbowZ"] =       24;
    or_map["lWristX"] =       25;
    or_map["lWristY"] =       26;
    or_map["lWristZ"] =       27;
    or_map["rHipX"] =         28;
    or_map["rHipY"] =         29;
    or_map["rHipZ"] =         30;
    or_map["rKnee"] =         31;
    or_map["rAnkleX"] =       32;
    or_map["rAnkleY"] =       33;
    or_map["rAnkleZ"] =       34;
    or_map["lHipX"] =         35;
    or_map["lHipY"] =         36;
    or_map["lHipZ"] =         37;
    or_map["lKnee"] =         38;
    or_map["lAnkleX"] =       39;
    or_map["lAnkleY"] =       40;
    or_map["lAnkleZ "] =      41;
}

//--------------------------------------------------------
//--------------------------------------------------------
//--------------------------------------------------------

RecordMotion::RecordMotion()
{
    m_use_or_format = true;
    m_is_recording = false;
    m_id_motion = 0;
    m_robot = NULL;
    reset();
    set_maps();
}

RecordMotion::RecordMotion( Robot* robot )
{
    m_use_or_format = true;
    m_is_recording = false;
    m_id_motion = 0;
    m_robot = robot;
    reset();
    set_maps();
}

RecordMotion::~RecordMotion()
{
    reset();
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
        cout << "No file to save empty vector in " << __func__ << endl;
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
        ss << std::setw( floor(log10(motion.size())) ) << std::setfill( '0' ) <<  i; ss >> str; ss.clear();
        string name = "config_" + str;
        xmlNodePtr cur            = xmlNewChild (root, NULL, xmlCharStrdup(name.c_str()), NULL);

        char str[80];
        sprintf( str, "%f", motion[i].first );
        xmlNodePtr curTime = xmlNewChild (cur, NULL, xmlCharStrdup("time"), NULL);
        xmlNewProp ( curTime, xmlCharStrdup("seconds"), xmlCharStrdup(str) );

        writeXmlRobotConfig( cur, m_robot->getRobotStruct(), motion[i].second->getConfigStruct() );
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

            configPt q = readXmlConfig( m_robot->getRobotStruct(), cur->xmlChildrenNode->next->next->next );
            if( q == NULL ) {
                cout << "Error : in readXmlConfig" << endl;
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
    std::string foldername = "/home/jmainpri/workspace/move3d/libmove3d/statFiles/recorded_motion/";
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
                storeMotion( partial_motion, j == 0 );

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

void RecordMotion::loadCSVFolder( const std::string& foldername, bool quiet )
{
    if( !quiet ) {
        cout << "Load Folder : " << foldername << endl;
    }

    std::string command = "ls " + foldername;
    FILE* fp = popen( command.c_str(), "r");
    if (fp == NULL) {
        cout << "ERROR in system call" << endl;
        return;
    }
    std::vector<std::string> files;

    char str[PATH_MAX];
    while ( fgets( str, PATH_MAX, fp) != NULL )
    {
        std::string filename( str );
        filename = filename.substr(0, filename.size()-1);
        std::string extension( filename.substr( filename.find_last_of(".") + 1 ) );
        //cout << extension << endl;
        if( extension == "csv" )
        {
            if( !quiet ) {
                cout << "add : " << filename << endl;
            }
            files.push_back( filename );
        }
    }
    pclose(fp);

    m_stored_motions.resize( files.size() );

    for(int i=0;i<int(files.size());i++)
    {
        m_stored_motions[i] = loadFromCSV( foldername + "/" + files[i], quiet );
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

void RecordMotion::storeMotion( const motion_t& motion, bool new_motion )
{
    if( new_motion )
    {
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
        cout << "index out of range in " << __func__ << endl;
        return false;
    }

    cout << m_motion[ith].first << endl;
    m_robot->setAndUpdate( *m_motion[ith].second );
    return true;
}

bool RecordMotion::setRobotToStoredMotionConfig(int motion_id, int config_id)
{
    if( motion_id < 0 || ( motion_id > int(m_stored_motions[motion_id].size())))
    {
        cout << "index out of stored motion range in " << __func__ << endl;
        return false;
    }

    if( config_id < 0 || config_id >= int(m_stored_motions[motion_id].size()) ) {
        cout << "index out of range in " << __func__ << endl;
        return false;
    }


    m_robot->setAndUpdate( *m_stored_motions[motion_id][config_id].second );

//    if(use_camera_)
//    {
//        _camera->pubImage(m_times[config_id]);
//    }

    return true;
}

bool RecordMotion::setShowMotion(int ith)
{
    if( ith == -1) {
        cout << "disable ith show motion" << endl;
    }
    else if( ith < 0 || ith >= int(m_stored_motions.size()) ) {
        cout << "index out of range in " << __func__ << endl;
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

//            cout << "-------------------------------" << endl;
//            q_cur->equal( *motion[i].second, true );
            q_cur = motion[i].second;
            m_robot->setAndUpdate( *q_cur );
//            cout << "dt : " << dt << " , m_motion[i].first : " << motion[i].first << endl;
//            motion[i].second->print();
//            motion[i].second->adaptCircularJointsLimits();
//            cout << (*motion[i].second)[11] << endl;
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

void RecordMotion::drawMotion( const motion_t& motion )
{
    if( motion.empty() ) {
        return;
    }

    G3D_Window *win = g3d_get_cur_win();

    m_robot->getRobotStruct()->draw_transparent = false;
    p3d_rob* rob = (p3d_rob*)(p3d_get_desc_curid( P3D_ROBOT ));

    for ( int i=0; i<int(motion.size()); i++ )
    {
        win->vs.transparency_mode= G3D_TRANSPARENT_AND_OPAQUE;

        m_robot->setAndUpdate( *motion[i].second );

        p3d_sel_desc_num( P3D_ROBOT, m_robot->getRobotStruct()->num );
        g3d_draw_robot( m_robot->getRobotStruct()->num, win, 0 );
        drawHeraklesArms();
    }

    p3d_sel_desc_num( P3D_ROBOT, rob->num );
    m_robot->getRobotStruct()->draw_transparent = false;
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
        cout << "indexes are not good in " << __func__ << endl;
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
        return motion;
    }

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

void RecordMotion::saveStoredToCSV( const std::string &filename )
{
    if( m_stored_motions.empty() )
    {
        cout << "No stored motion in " << __func__ << endl;
        return;
    }

    const int samples = 100;

    cout << "Down sampling to " << samples << endl;

    for (int i=0; i<int(m_stored_motions.size()); i++)
    {
        m_stored_motions[i] = resample( m_stored_motions[i], samples );
    }

    std::ofstream s;
    s.open( filename.c_str() );
    cout << "Opening save file : " << filename << endl;

    Eigen::Vector3d pos;
    Eigen::Transform3d T;

    const bool save_cartesian=false;

    for (int i=0; i<int(m_stored_motions.size()); i++)
    {
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

//                s << (*m_stored_motions[i][j].second)[27] << ","; // lShoulderX
//                s << (*m_stored_motions[i][j].second)[28] << ","; // lShoulderZ
//                s << (*m_stored_motions[i][j].second)[29] << ","; // lShoulderY
//                s << (*m_stored_motions[i][j].second)[30] << ","; // lArmTrans
//                s << (*m_stored_motions[i][j].second)[31] << endl; // lElbowZ
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

    Eigen::Vector3d pos;

    for (int i=0; i<int(motion.size()); i++)
    {
        m_robot->setAndUpdate( *motion[i].second );

        Eigen::Transform3d T( m_robot->getJoint("Pelvis")->getMatrixPos().inverse() );

        pos = T*m_robot->getJoint("rWristX")->getVectorPos();
        for (int j=0; j<int(pos.size()); j++)
            s << pos[j] << ",";

        pos = T*m_robot->getJoint("rElbowZ")->getVectorPos();
        for (int j=0; j<int(pos.size()); j++)
            s << pos[j] << ",";

        s << endl;
    }

    s << endl;
    //cout << "Closing save file" << endl;
    s.close();
}

static const double transX = 0.10;
static const double transY = 0.50;
static const double transZ = 0.15;
static const double transT = 0.00;

void RecordMotion::translateStoredMotions()
{
    cout << "Translate all stored motions" << endl;

    for(int i=0;i<int(m_stored_motions.size());i++)
    {
        for(int j=0;j<int(m_stored_motions[i].size());j++)
        {
            confPtr_t q = m_stored_motions[i][j].second;
            (*q)[6]  += transX; // X
            (*q)[7]  += transY; // Y
            (*q)[8]  += transZ; // Z
            (*q)[11] += transT; // theta
        }
    }
}

motion_t RecordMotion::invertTranslation( const motion_t& motion )
{
    motion_t motion_trans = motion;

    for(int j=0;j<int(motion_trans.size());j++)
    {
        confPtr_t q = motion_trans[j].second->copy();
        (*q)[6]  -= transX; // X
        (*q)[7]  -= transY; // Y
        (*q)[8]  -= transZ; // Z
        (*q)[11] -= transT; // theta
        motion_trans[j].second = q;
    }
    return motion_trans;
}

bool  RecordMotion::loadRegressedFromCSV()
{
    string foldername = "/home/jmainpri/Dropbox/workspace/gesture-recognition/gmm/gmm-gmr-gesture-recognition/";

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

confPtr_t RecordMotion::getConfigOpenRave( const std::vector<std::string>& config )
{
    std::vector<double> tmp(config.size());

    for(int i=0;i<int(config.size());i++)
    {
        convert_text_to_num<double>( tmp[i], config[i], std::dec );
    }

    confPtr_t q = m_robot->getCurrentPos();

    for( std::map<std::string,int>::iterator it_map=or_map.begin(); it_map!=or_map.end(); it_map++ )
    {
        (*q)[ move3d_map[it_map->first] ] = tmp[ it_map->second ];
    }

    return q;
}

confPtr_t RecordMotion::getConfigTwelveDoF( const std::vector<std::string>& config )
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

motion_t RecordMotion::loadFromCSV( const std::string& filename, bool quiet )
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

        confPtr_t q;

        if( m_use_or_format )
        {
            q= getConfigOpenRave( matrix[i] );
        }
        else
        {
            q = getConfigTwelveDoF( matrix[i] );
        }

        // This is because the data was recorded near limits
        q->adaptCircularJointsLimits();
        //q->print();

        motion.push_back( std::make_pair(0.02,q) );
     }

    return motion;
}
