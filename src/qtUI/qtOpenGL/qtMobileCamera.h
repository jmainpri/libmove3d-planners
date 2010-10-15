/*
 *  qtMobileCamera.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 02/06/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef QT_MOBILE_CAM_H
#define QT_MOBILE_CAM_H

#include <QDialog>
#include "P3d-pkg.h"

class QRadioButton;

class qtMobileCamera : public QDialog
{
	Q_OBJECT
	
public:
	qtMobileCamera();
	
public slots:
	bool isMatrixChanged();
	void changeMobileCam();
	
private:
	void createRadioButtons();
	int m_num_bod;
	std::vector< QRadioButton* > m_radioButtons;
	pp3d_matrix4 m_transf;
};

#endif