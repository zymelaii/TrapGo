#pragma once

#include "World.h"
#include "Shader.h"
#include "Entity.h"

#include <Eigen/Dense>
#include <QWidget>
#include <QOpenGLWidget>
#include <QOpenGLFunctions_4_5_Core>
#include <QTimer>

#include <bitset>
#include <future>

class WorldView final : public QOpenGLWidget, protected QOpenGLFunctions_4_5_Core {
	Q_OBJECT

public:
	explicit WorldView(QWidget* parent = nullptr);
	~WorldView();

private:
	void initializeGL() override;
	void resizeGL(int w, int h) override;
	void paintGL() override;

	void keyPressEvent(QKeyEvent* e) override;
	void keyReleaseEvent(QKeyEvent* e) override;
	void paintEvent(QPaintEvent* e) override;

public slots:
	void onIdle();
	void aboutToRender();

public:
	Shader shader;
	GLuint BOs[3];	 //! VBO, EBO, UBO

	int							 roleTarget = 0;
	std::vector<Eigen::Vector3f> effectVertices;
	std::vector<Eigen::Vector3f> vertices;
	std::vector<Entity>			 entities;

public:
	//! interaction
	std::bitset<128> keyPressed;
	bool			 aboutToSwitch = false;
	int				 switchTarget  = -1;
	//! core
	enum class State {
		Startup,	  //!< 游戏启动中
		Initialize,	  //!< 世界初始化
		Suspend,	  //!< 游戏挂起
		Normal,		  //!< 常规游戏进程
		MindFlash,	  //!< 思维境界
	} state;
	World world;
	//! internal
	QTimer idleTimer;
	QTimer renderTimer;
	//! common
	std::future<void> sharedFuture;
};