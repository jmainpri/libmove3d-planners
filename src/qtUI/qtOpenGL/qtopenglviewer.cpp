#include "qtopenglviewer.hpp"
#include "ui_qtopenglviewer.h"

qtOpenGLViewer::qtOpenGLViewer(QWidget *parent) :
    QWidget(parent),
    m_ui(new Ui::qtOpenGLViewer)
{
    m_ui->setupUi(this);
}

qtOpenGLViewer::~qtOpenGLViewer()
{
    delete m_ui;
}

void qtOpenGLViewer::changeEvent(QEvent *e)
{
    QWidget::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        m_ui->retranslateUi(this);
        break;
    default:
        break;
    }
}
