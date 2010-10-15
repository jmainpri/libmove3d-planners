#ifndef QTLIBRARY_HPP
#define QTLIBRARY_HPP

#include "Graphic-pkg.h"

#undef Status
#undef Bool
#undef Black
#undef CursorShape
#undef None
#undef KeyPress
#undef KeyRelease
#undef FocusIn
#undef FocusOut
#undef FontChange
#undef Unsorted

#include <QtOpenGL/QGLWidget>

#include <QtCore/QObject>

#include <QtCore/QThread>
#include <QtCore/QTimer>
#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtCore/QSemaphore>
#include <QtCore/QDebug>
#include <QtCore/QWaitCondition>
#include <QtCore/QMutex>

//#include <QtGui>
#include <QtGui/QCDEStyle>
#include <QtGui/QCleanlooksStyle>
#include <QtGui/QCommonStyle>
#include <QtGui/QMotifStyle>
#include <QtGui/QPlastiqueStyle>
#include <QtGui/QWindowsStyle>
//#include <QtGui/QMacStyle>
#include <QtGui/QWindowsVistaStyle>

#include <QtGui/QMainWindow>

#include <QtGui/QWidget>
#include <QtGui/QApplication>
#include <QtGui/QFileDialog>
#include <QtGui/QDialog>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QLabel>
#include <QtGui/QLayout>
#include <QtGui/QSlider>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QCheckBox>
#include <QtGui/QPushButton>
#include <QtGui/QComboBox>
#include <QtGui/QStackedWidget>
#include <QtGui/QSpacerItem>
#include <QtGui/QLineEdit>
#include <QtGui/QListWidget>
#include <QtGui/QLabel>
#include <QtGui/QScrollBar>
#include <QtGui/QTextEdit>
#include <QtGui/QKeyEvent>
#include <QtGui/QScrollArea>

#define Bool int
#define Status int
#define True 1
#define False 0
#define Black 0

#endif // QTLIBRARY_HPP
