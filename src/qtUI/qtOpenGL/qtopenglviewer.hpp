#ifndef QTOPENGLVIEWER_H
#define QTOPENGLVIEWER_H

#ifdef CXX_PLANNER
#include "qtLibrary.hpp"
#endif

#ifdef OOMOVE3D_CORE
#include "qtUI/qtLibrary.hpp"
#endif

namespace Ui {
    class qtOpenGLViewer;
}

class qtOpenGLViewer : public QWidget {
    Q_OBJECT
public:
    qtOpenGLViewer(QWidget *parent = 0);
    ~qtOpenGLViewer();

protected:
    void changeEvent(QEvent *e);

private:
    Ui::qtOpenGLViewer *m_ui;
};

#endif // QTOPENGLVIEWER_H
