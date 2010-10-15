#ifndef KCDPROPERTIESWINDOW_HPP
#define KCDPROPERTIESWINDOW_HPP

#if defined( CXX_PLANNER )
#include "qtLibrary.hpp"
#endif

#if defined( WITH_OOMOVE3D )
#include "qtUI/qtLibrary.hpp"
#endif

namespace Ui {
    class KCDpropertiesWindow;
}

class KCDpropertiesWindow : public QDialog {
    Q_OBJECT
    Q_DISABLE_COPY(KCDpropertiesWindow)
public:
    explicit KCDpropertiesWindow(QWidget *parent = 0);
    virtual ~KCDpropertiesWindow();

protected:
    virtual void changeEvent(QEvent *e);

private:
    Ui::KCDpropertiesWindow *m_ui;

private slots:
//    void setLineEditFromScrollBar();
	void setTestTrajMethod(int TrajMethod);
};

#endif // KCDPROPERTIESWINDOW_H
