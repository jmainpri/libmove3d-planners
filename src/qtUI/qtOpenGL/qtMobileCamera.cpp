/*
 *  qtMobileCamera.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 02/06/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "qtMobileCamera.h"
#include "qtLibrary.hpp"
#include <QRadioButton>

#include "P3d-pkg.h"

#include "Graphic-pkg.h"

qtMobileCamera::qtMobileCamera()
{
	createRadioButtons();
}

void qtMobileCamera::createRadioButtons()
{
	char body_name[8];
	int  i,njnt,ord;
	p3d_rob* r;
	r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	njnt = r->njoints+1;
	
	QGridLayout *mainLayout = new QGridLayout;
	
	mainLayout->addWidget(new QLabel(r->name),0,0);
	
	ord = 0;
	for(i=0;i<njnt;i++) 
	{
		sprintf(body_name,"Body %d",i);

		QRadioButton* ptrRB = new QRadioButton(body_name,this);
		mainLayout->addWidget(ptrRB,i+1,0);
		m_radioButtons.push_back(ptrRB);
		connect(ptrRB,SIGNAL(toggled(bool)),this,SLOT(changeMobileCam()));
		ord=ord+1;
	}
	m_num_bod = njnt;
	setLayout(mainLayout);
}

bool qtMobileCamera::isMatrixChanged()
{
	for (unsigned int i=0; i<m_radioButtons.size(); i++) 
	{
		if(m_radioButtons[i]->isChecked())
		{
			p3d_rob* r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
			p3d_jnt* j = r->joints[i];
			m_transf = &(j->abs_pos);
			return true;
		}
	}
	
	return false;
}

void qtMobileCamera::changeMobileCam()
{
#if defined( CXX_PLANNER )
		G3D_Window *win = qt_get_cur_g3d_win();

		if( isMatrixChanged() ) 
		{
			qt_change_mob_frame(win,m_transf);
		} 
		else 
		{
			qt_reset_mob_frame(win);
		}
#endif
		
//		g3d_draw_win(win);
}
