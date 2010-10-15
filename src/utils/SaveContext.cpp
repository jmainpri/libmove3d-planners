/*
 * SaveContext.cpp
 *
 *  Created on: Sep 11, 2009
 *      Author: jmainpri
 */

#include "SaveContext.hpp"
#include <iostream>
#include <iterator>

#ifdef QT_LIBRARY
#include "qtLibrary.hpp"
#endif

using namespace std;

SaveContext storedContext;

SaveParameterEnv<
PlanParam::boolParameter,
PlanParam::intParameter,
PlanParam::doubleParameter,
PlanParam::stringParameter,
PlanParam::vectorParameter>* storedPlannerContext = NULL;

SaveContext::SaveContext()
{
	//	clear();
	//	saveCurrentEnvToStack();
	clear();
	_Time.clear();
}

SaveContext::~SaveContext()
{
	clear();
}

void SaveContext::clear()
{
	cout << "Deleting " << _MapBool.size() << " context" << endl;
	
	_MapBool.clear();
	_MapInt.clear();
	_MapDouble.clear();
	_MapString.clear();
	
	for(unsigned int i=0;i<_Time.size();i++)
	{
		_Time.at(i).clear();
	}
	
	_Time.clear();
}

unsigned int SaveContext::getNumberStored()
{
	return _MapBool.size();
}

unsigned int SaveContext::saveCurrentEnvToStack()
{
	
	_MapBool.push_back( copyMap(ENV.getBoolMap()) );
	_MapInt.push_back( copyMap(ENV.getIntMap()) );
	_MapDouble.push_back( copyMap(ENV.getDoubleMap()) );
	_MapString.push_back( copyMap(ENV.getStringMap()) );
	
	return getNumberStored();
}

void SaveContext::printData(unsigned int i)
{
	printVariables(i);
	
	cout << "_Map["<<i<<"]"<<endl;
}


void SaveContext::printVariables(unsigned int id)
{
	cout << "Bool -------------------------------" << endl;
	
	map<Env::boolParameter, boolContainer*>::iterator iter1 = _MapBool[id].begin();
	
	for(; iter1 != _MapBool[id].end(); iter1++)
	{
		cout << iter1->second->get() << endl;
	}
	
	
	cout << "Integers ---------------------------" << endl;
	map<Env::intParameter, intContainer*>::iterator iter2 = _MapInt[id].begin();
	//	cout << "size " << d->getIntMap().size() << endl;
	
	for(; iter2 != _MapInt[id].end(); iter2++)
	{
		cout << iter2->second->get() << endl;
	}
	
	cout << "Doubles ----------------------------" << endl;
	map<Env::doubleParameter, doubleContainer*>::iterator iter3 = _MapDouble[id].begin();
	
	for(; iter3 != _MapDouble[id].end(); iter3++)
	{
		cout << iter3->second->get() << endl;
	}
	
#ifdef QT_LIBRARY
	cout << "String -----------------------------" << endl;
	map<Env::stringParameter, stringContainer*>::iterator iter4 = _MapString[id].begin();
	
	for(; iter4 != _MapString[id].end(); iter4++)
	{
		cout << iter4->second->get().toStdString() << endl;
	}
#endif
	cout << "------------ end --------------" << endl;
}


void SaveContext::switchCurrentEnvTo(unsigned int i)
{
	cout << "Switching to " <<  i << " context" << endl;
	cout << "       " <<  getNumberStored() << " are stored" << endl;
	
	for(map<Env::boolParameter, boolContainer*>::iterator iter = _MapBool[i].begin();
			iter != _MapBool[i].end();
			iter++)
	{
		ENV.setBool(iter->first,iter->second->get());
	}
	
	for(map<Env::intParameter, intContainer*>::iterator iter = _MapInt[i].begin();
			iter != _MapInt[i].end();
			iter++)
	{
		ENV.setInt(iter->first,iter->second->get());
	}
	
	for(map<Env::doubleParameter, doubleContainer*>::iterator iter = _MapDouble[i].begin();
			iter != _MapDouble[i].end();
			iter++)
	{
		ENV.setDouble(iter->first,iter->second->get());
	}
	
#ifdef QT_LIBRARY
	for(map<Env::stringParameter, stringContainer*>::iterator iter = _MapString[i].begin();
			iter != _MapString[i].end();
			iter++)
	{
		ENV.setString(iter->first,iter->second->get());
	}
#endif
}

void SaveContext::deleteEnv(unsigned int i)
{
	cout << "Switching to " <<  i << " context" << endl;
	cout << "       " <<  getNumberStored() << " are stored" << endl;
	
	_MapBool.erase (		_MapBool.begin()	+i);
	_MapInt.erase (			_MapInt.begin()			+i);
	_MapDouble.erase (	_MapDouble.begin()	+i);
	_MapString.erase (	_MapString.begin()	+i);
}

map<Env::boolParameter, boolContainer*> SaveContext::copyMap( map<Env::boolParameter, boolContainer*> myMap )
{
	map<Env::boolParameter, boolContainer*>::iterator iter = myMap.begin();
	
	for(; iter != myMap.end(); iter++)
	{
		//		cout << iter->second->get() << endl;
		iter->second = new boolContainer(iter->second->get());
	}
	
	return myMap;
}

map<Env::intParameter, intContainer*> SaveContext::copyMap( map<Env::intParameter, intContainer*> myMap )
{
	map<Env::intParameter, intContainer*>::iterator iter = myMap.begin();
	
	for(; iter != myMap.end(); iter++)
	{
		//		cout << iter->second->get() << endl;
		iter->second = new intContainer(iter->second->get());
	}
	
	return myMap;
}

map<Env::doubleParameter, doubleContainer*> SaveContext::copyMap( map<Env::doubleParameter, doubleContainer*> myMap )
{
	map<Env::doubleParameter, doubleContainer*>::iterator iter = myMap.begin();
	
	for(; iter != myMap.end(); iter++)
	{
		//		cout << iter->second->get() << endl;
		iter->second = new doubleContainer(iter->second->get());
	}
	
	return myMap;
}

map<Env::stringParameter, stringContainer*> SaveContext::copyMap( map<Env::stringParameter, stringContainer*> myMap )
{
	map<Env::stringParameter, stringContainer*>::iterator iter = myMap.begin();
	
	for(; iter != myMap.end(); iter++)
	{
		//		cout << iter->second->get() << endl;
#ifdef QT_LIBRARY
		iter->second = new stringContainer(iter->second->get());
#endif
	}
	
	return myMap;
}

//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
// TODO

//SaveParameterEnv storedContext;

//template <typename enumBool,
//typename enumInt,
//typename enumDouble,
//typename enumString,
//typename enumVector>
//SaveParameterEnv<enumBool,enumInt,enumDouble,enumString,enumVector>::SaveParameterEnv(Parameters<enumBool,enumInt,enumDouble,enumString,enumVector>* env) : m_ParamEnv(env)
//{
//	//	clear();
//	//	saveCurrentEnvToStack();
//	clear();
//	m_Time.clear();
//}
//
//template <typename enumBool,
//typename enumInt,
//typename enumDouble,
//typename enumString,
//typename enumVector>
//unsigned int SaveParameterEnv<enumBool,enumInt,enumDouble,enumString,enumVector>::getNumberStored()
//{
//	return m_MapBool.size();
//}

//template <typename enumBool,
//typename enumInt,
//typename enumDouble,
//typename enumString,
//typename enumVector>
//void SaveParameterEnv<enumBool,enumInt,enumDouble,enumString,enumVector>::clear()
//{
//	cout << "Deleting " << m_MapBool.size() << " context" << endl;
//	
//	m_MapBool.clear();
//	m_MapInt.clear();
//	m_MapDouble.clear();
//	m_MapString.clear();
//	
//	for(unsigned int i=0;i<m_Time.size();i++)
//	{
//		m_Time.at(i).clear();
//	}
//	
//	m_Time.clear();
//}
//
//template <typename enumBool,
//typename enumInt,
//typename enumDouble,
//typename enumString,
//typename enumVector>
//unsigned int SaveParameterEnv<enumBool,enumInt,enumDouble,enumString,enumVector>::saveCurrentEnvToStack()
//{
//	
//	m_MapBool.push_back(		copyMap(m_ParamEnv->getBoolMap()) );
//	m_MapInt.push_back(			copyMap(m_ParamEnv->getIntMap()) );
//	m_MapDouble.push_back(	copyMap(m_ParamEnv->getDoubleMap()) );
//	m_MapString.push_back(	copyMap(m_ParamEnv->getStringMap()) );
//	
//	return getNumberStored();
//}
//
//template <typename enumBool,
//typename enumInt,
//typename enumDouble,
//typename enumString,
//typename enumVector>
//void SaveParameterEnv<enumBool,enumInt,enumDouble,enumString,enumVector>::printData(unsigned int i)
//{
//	printVariables(i);
//	
//	cout << "m_Map["<<i<<"]"<<endl;
//}
//
//template <typename enumBool,
//typename enumInt,
//typename enumDouble,
//typename enumString,
//typename enumVector>
//void SaveParameterEnv<enumBool,enumInt,enumDouble,enumString,enumVector>::switchCurrentEnvTo(unsigned int i)
//{
//	std::cout << "Switching to " <<  i << " context" <<  std::endl;
//	std::cout << "       " <<  getNumberStored() << " are stored" <<  std::endl;
//	
//	for(typename std::map<enumBool,boolContainer*>::iterator iter = m_MapBool[i].begin();
//			iter != m_MapBool[i].end();
//			iter++)
//	{
//		m_ParamEnv->setBool(iter->first,iter->second->get());
//	}
//	
//	for(typename std::map<enumInt, intContainer*>::iterator iter = m_MapInt[i].begin();
//			iter != m_MapInt[i].end();
//			iter++)
//	{
//		m_ParamEnv->setInt(iter->first,iter->second->get());
//	}
//	
//	for(typename std::map<enumDouble, doubleContainer*>::iterator iter = m_MapDouble[i].begin();
//			iter != m_MapDouble[i].end();
//			iter++)
//	{
//		m_ParamEnv->etDouble(iter->first,iter->second->get());
//	}
//	
//#ifdef QT_LIBRARY
//	for(typename std::map<enumString, stringContainer*>::iterator iter = m_MapString[i].begin();
//			iter != m_MapString[i].end();
//			iter++)
//	{
//		m_ParamEnv->setString(iter->first,iter->second->get());
//	}
//#endif
//}
//
//template <typename enumBool,
//typename enumInt,
//typename enumDouble,
//typename enumString,
//typename enumVector>
//void SaveParameterEnv<enumBool,enumInt,enumDouble,enumString,enumVector>::deleteEnv(unsigned int i)
//{
//
//	m_MapBool.erase (	m_MapBool.begin()	+ i );
//	m_MapInt.erase (	m_MapInt.begin() + i );
//	m_MapDouble.erase (	m_MapDouble.begin()	+ i );
//	m_MapString.erase (	m_MapString.begin()	+ i );
//}
//
//template <typename enumBool,
//typename enumInt,
//typename enumDouble,
//typename enumString,
//typename enumVector>
//void SaveParameterEnv<enumBool,enumInt,enumDouble,enumString,enumVector>::printVariables(unsigned int id)
//{
//	std::cout << "Bool -------------------------------" <<  std::endl;
//	
//	typename std::map<enumBool, boolContainer*>::iterator iter1;
//	iter1 = m_MapBool[id].begin();
//	
//	for(; iter1 != m_MapBool[id].end(); iter1++)
//	{
//		std::cout << iter1->second->get() << std::endl;
//	}
//	
//	
//	std::cout << "Integers ---------------------------" <<  std::endl;
//	typename std::map<enumBool, intContainer*>::iterator iter2 = m_MapInt[id].begin();
//	//	cout << "size " << d->getIntMap().size() << endl;
//	
//	for(; iter2 != m_MapInt[id].end(); iter2++)
//	{
//		std::cout << iter2->second->get() <<  std::endl;
//	}
//	
//	std::cout << "Doubles ----------------------------" <<  std::endl;
//	typename std::map<enumDouble, doubleContainer*>::iterator iter3 = m_MapDouble[id].begin();
//	
//	for(; iter3 != m_MapDouble[id].end(); iter3++)
//	{
//		std::cout << iter3->second->get() <<  std::endl;
//	}
//	
//#ifdef QT_LIBRARY
//	std::cout << "String -----------------------------" << std::endl;
//	typename std::map<enumString, stringContainer*>::iterator iter4 = m_MapString[id].begin();
//	
//	for(; iter4 != m_MapString[id].end(); iter4++)
//	{
//		std::cout << iter4->second->get().toStdString() <<  std::endl;
//	}
//#endif
//	std::cout << "------------ end --------------" <<  std::endl;
//}

//template <typename enumBool,
//typename enumInt,
//typename enumDouble,
//typename enumString,
//typename enumVector>
//std::map<enumBool, boolContainer*> SaveParameterEnv<enumBool,enumInt,enumDouble,enumString,enumVector>::copyMap( map<enumBool, boolContainer*> myMap )
//{
//	typename std::map<enumBool, boolContainer*>::iterator iter = myMap.begin();
//	
//	for(; iter != myMap.end(); iter++)
//	{
//		//		cout << iter->second->get() << endl;
//		iter->second = new boolContainer(iter->second->get());
//	}
//	
//	return myMap;
//}
//
//template <typename enumBool,
//typename enumInt,
//typename enumDouble,
//typename enumString,
//typename enumVector>
//std::map<enumInt, intContainer*> SaveParameterEnv<enumBool,enumInt,enumDouble,enumString,enumVector>::copyMap( map<enumInt, intContainer*> myMap )
//{
//	typename std::map<enumInt, intContainer*>::iterator iter = myMap.begin();
//	
//	for(; iter != myMap.end(); iter++)
//	{
//		//		cout << iter->second->get() << endl;
//		iter->second = new intContainer(iter->second->get());
//	}
//	
//	return myMap;
//}
//
//template <typename enumBool,
//typename enumInt,
//typename enumDouble,
//typename enumString,
//typename enumVector>
//std::map<enumDouble, doubleContainer*> SaveParameterEnv<enumBool,enumInt,enumDouble,enumString,enumVector>::copyMap( map<enumDouble, doubleContainer*> myMap )
//{
//	typename std::map<enumDouble, doubleContainer*>::iterator iter = myMap.begin();
//	
//	for(; iter != myMap.end(); iter++)
//	{
//		//		cout << iter->second->get() << endl;
//		iter->second = new doubleContainer(iter->second->get());
//	}
//	
//	return myMap;
//}
//
//template <typename enumBool,
//typename enumInt,
//typename enumDouble,
//typename enumString,
//typename enumVector>
//std::map<enumString, stringContainer*> SaveParameterEnv<enumBool,enumInt,enumDouble,enumString,enumVector>::copyMap( map<enumString, stringContainer*> myMap )
//{
//	typename std::map<enumString, stringContainer*>::iterator iter = myMap.begin();
//	
//	for(; iter != myMap.end(); iter++)
//	{
//		//		cout << iter->second->get() << endl;
//#ifdef QT_LIBRARY
//		iter->second = new stringContainer(iter->second->get());
//#endif
//	}
//}
