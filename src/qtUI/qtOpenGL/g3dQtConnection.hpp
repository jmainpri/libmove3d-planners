/*
 * g3d_qt_connection.hpp
 *
 * Qt connection
 *
 *  Created on: Oct 7, 2009
 *      Author: jmainpri
 */

#ifndef G3D_QT_CONNECTION_HPP_
#define G3D_QT_CONNECTION_HPP_

class GLWidget;

#undef CursorShape

#include <QtCore/QObject>
#include <QtCore/QTimer>

/**
  * @ingroup qtWindow
  * @brief Synchronises the QtWindow with the worker thread
  */
class Move3D2OpenGl : public QObject {
	Q_OBJECT

public:
	Move3D2OpenGl();
	Move3D2OpenGl(GLWidget *glW);

	void update();
	void updatePipe();

	void addCurrentImage();
	void saveImagesToDisk();

	void setIsNotTimeControlled(bool value);

        void reinitGraphics();

private slots:
	void releaseLockIfWating();

signals:
	void activate_qt_gl_window(void);
	void add_current_image_vector(void);
	void save_image_vector_to_disk(void);
        void g3d_reinit_graphics_called(void);

private:
	GLWidget *_glWidget;
	QTimer *_timer;
	bool _isWatingForTimer;
	bool _isNotTimeControlled;
};

#endif /* G3D_QT_CONNECTION_HPP_ */
