#include "HRICS_RecordMotion.hpp"

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

RecordMotion* global_motionRecorder=NULL;

RecordMotion::RecordMotion()
{
    m_id_motion = 0;
    m_robot = NULL;
    reset();
}

RecordMotion::RecordMotion(const std::string& robotname)
{
    m_id_motion = 0;
    setRobot( robotname );
    reset();
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

            vectConfs.push_back( make_pair(time,confPtr_t(new Configuration(m_robot,q,true))));
            //vectConfs.back().second->print();
        }

        cur = cur->next;
    }

    return vectConfs;
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

bool RecordMotion::setConfiguration(int ith)
{
    if( ith < 0 || ith >= int(m_motion.size()) ) {
        cout << "index out of range in " << __func__ << endl;
        return false;
    }

    cout << m_motion[ith].first << endl;
    m_robot->setAndUpdate( *m_motion[ith].second );
    return true;
}

void RecordMotion::showStoredMotion()
{
    for (int i=0; i<int(m_stored_motions.size()); i++)
    {
        cout << "Show motion : " << i << " with " <<  m_stored_motions[i].size() << " frames" << endl;
        showMotion( m_stored_motions[i] );
    }
}

void RecordMotion::showCurrentMotion()
{
    showMotion( m_motion );
}

void RecordMotion::showMotion( const motion_t& motion )
{
    if( motion.empty() ) {
        return;
    }

    int StopRun = false;
    int i=0;
    double tu_last = 0.0;
    double dt = 0.0;

    m_robot->setAndUpdate( *motion[0].second );
    g3d_draw_allwin_active();

    while ( !StopRun )
    {
        timeval tim;
        gettimeofday(&tim, NULL);
        double tu = tim.tv_sec+(tim.tv_usec/1000000.0);
        dt += ( tu - tu_last );
        tu_last = tu;

        if ( dt>=motion[i].first ) {

//            (*motion[i].second)[6] = (*m_init_q)[6];
//            (*motion[i].second)[7] = (*m_init_q)[7];
//            (*motion[i].second)[11] = (*m_init_q)[11];
//            (*motion[i].second)[12] = (*m_init_q)[12];
//            (*motion[i].second)[13] = (*m_init_q)[13];
//            (*motion[i].second)[14] = (*m_init_q)[14];
            m_robot->setAndUpdate( *motion[i].second );
//            cout << "dt : " << dt << " , m_motion[i].first : " << motion[i].first << endl;
//            motion[i].second->print();
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

void RecordMotion::saveStoredToCSV( const std::string &filename )
{
    cout << "down sampling" << endl;
    for (int i=0; i<int(m_stored_motions.size()); i++)
    {
        int samples = 100;
        double inc = double(m_stored_motions[i].size())/double(samples);
        double k =0; motion_t motion;

        for (int j=0; j<samples; j++)
        {
            motion.push_back( m_stored_motions[i][floor(k)] );
            k += inc;
        }
        cout << "m_stored_motions[i].size() : " << m_stored_motions[i].size() << endl;
        cout << "k : " << k << endl;
        m_stored_motions[i] = motion;
    }

    std::ofstream s;
    s.open( filename.c_str() );
    //cout << "Opening save file : " << oss.str() << endl;

    Eigen::Vector3d pos;

    for (int i=0; i<int(m_stored_motions.size()); i++)
    {
        for (int j=0; j<int(m_stored_motions[i].size()); j++)
        {
            m_robot->setAndUpdate( *m_stored_motions[i][j].second );

            Eigen::Transform3d T( m_robot->getJoint("Pelvis")->getMatrixPos().inverse() );

            s << j << ";";

            pos = T*m_robot->getJoint("rWristX")->getVectorPos();
            for (int k=0; k<int(pos.size()); k++)
                s << pos[k] << ";";

            pos = T*m_robot->getJoint("rElbowZ")->getVectorPos();
            for (int k=0; k<int(pos.size()); k++)
                s << pos[k] << ";";

            s << endl;
        }
    }

    s << endl;
    //cout << "Closing save file" << endl;
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
            s << pos[j] << ";";

        pos = T*m_robot->getJoint("rElbowZ")->getVectorPos();
        for (int j=0; j<int(pos.size()); j++)
            s << pos[j] << ";";

        s << endl;
    }

    s << endl;
    //cout << "Closing save file" << endl;
    s.close();
}
