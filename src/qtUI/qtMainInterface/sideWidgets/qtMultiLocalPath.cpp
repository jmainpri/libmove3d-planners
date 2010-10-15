/*
 *  qtMultiLocalPath.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 05/10/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "qtMultiLocalPath.hpp"

#include "P3d-pkg.h"
#include "Localpath-pkg.h"

MultiLocalPathWidget::MultiLocalPathWidget(QWidget *parent)
{
	initMultiLocalPathForm();
}

MultiLocalPathWidget::~MultiLocalPathWidget()
{

}

//initialisation
/****************************************************************************/
/** \brief Create the multilocalpath form.
 */
/****************************************************************************/
void MultiLocalPathWidget::initMultiLocalPathForm()
{
	m_verticalLayout = new QVBoxLayout(this);
	m_verticalLayout->setSpacing(6);
	m_verticalLayout->setContentsMargins(3, 3, 3, 3);
	
	this->createMultiLocalPathList_obj();
	
	m_deactivButton = new QPushButton();
	m_deactivButton->setText("Deactivate");
	
	m_verticalLayout->addWidget( m_deactivButton );
	
	connect( m_deactivButton , SIGNAL(clicked()), this , SLOT(deactivateButton()) );
}

void MultiLocalPathWidget::deactivateButton()
{
  p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); /* current robot */
  p3d_multiLocalPath_disable_all_groupToPlan(r);
	
  for(int i=0; i<r->mlp->nblpGp; i++)
	{
    m_MLP_CheckBoxes[i]->setChecked(false);
  }
}

void MultiLocalPathWidget::createMultiLocalPathList_obj() 
{
	p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); /* current robot */
 	
	QGroupBox* groupBox = new QGroupBox();
	groupBox->setTitle("Groups");
	
	QVBoxLayout* groupLayout = new QVBoxLayout(groupBox);
	groupLayout->setSpacing(6);
	groupLayout->setContentsMargins(11, 11, 11, 11);
	
	for(int i=0; i<r->mlp->nblpGp; i++) 
	{
		m_MLP_CheckBoxes.push_back( new QCheckBox(groupBox) );
		m_MLP_CheckBoxes.back()->setText(r->mlp->mlpJoints[i]->gpName);
		groupLayout->addWidget( m_MLP_CheckBoxes.back() );
		//fl_set_object_color(MGGRAPH_OBJ[i],FL_MCOL,FL_GREEN);
		
		m_MLP_Validator.push_back( new GroupValidator() );
		
		if( p3d_multiLocalPath_get_value_groupToPlan( r, i) == 1) 
		{
			m_MLP_CheckBoxes.back()->setChecked(true);
		} 
		else 
		{
			m_MLP_CheckBoxes.back()->setChecked(false);
		}

		connect(	m_MLP_CheckBoxes.back() , SIGNAL(toggled(bool)), 
							m_MLP_Validator.back() ,	SLOT(multiLocalPathList_obj(bool)) );
	}
	
	m_verticalLayout->addWidget( groupBox );
}

GroupValidator::GroupValidator(QWidget *parent)
{
	
}

GroupValidator::~GroupValidator()
{
	
}

void GroupValidator::multiLocalPathList_obj(bool value) 
{
	int mgID = m_group;
	
	p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); /* current robot */
	//value = fl_get_button(ob);
	//printf("graph %d value %d\n",mgID,	value );
 	p3d_multiLocalPath_set_groupToPlan(r, mgID, value);
}
