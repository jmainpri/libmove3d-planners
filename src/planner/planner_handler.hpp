#ifndef PLANNER_HANDLER_HPP_INCLUDED
#define PLANNER_HANDLER_HPP_INCLUDED

#include <QtCore/QObject>
#include <QtCore/QString>

class PlannerHandler : public QObject
{
  Q_OBJECT;

public:
  enum state { running, // A planning algorithm instance is running
	       stopped, // A planning algorithm instance has been created,
	       // but is stopped
	       none }; // There is no instance created
  
  PlannerHandler(int argc, char** argv);

public slots:
  void init();
  void startPlanner(QString plannerName);
  void stopPlanner();
  void resetPlanner();

signals:
  void initIsDone();
  void plannerIsStopped();
  void plannerIsReset();

protected:
  state mState;
  int mArgc;
  char** mArgv;
};

#endif
