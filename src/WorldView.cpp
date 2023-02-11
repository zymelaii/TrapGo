#include "WorldView.h"

#include "Entity.h"

#include <QPainter>
#include <QKeyEvent>
#include <QPaintEvent>

#include <math.h>
#include <limits.h>
#include <utility>
#include <chrono>
#include <random>
#include <vector>

static Eigen::Matrix4f getTranslationTransform(const Eigen::Vector3f& translation) {
	Eigen::Matrix4f transform	= Eigen::Matrix4f::Identity();
	transform.block<3, 1>(0, 3) = translation.transpose();
	return transform;
}

static Eigen::Matrix4f getAxisRotationTransform(float rotation, const Eigen::Vector3f& axis) {
	Eigen::Matrix4f transform	= Eigen::Matrix4f::Identity();
	const auto		angleAxis	= Eigen::AngleAxisf(qDegreesToRadians(rotation), axis);
	transform.block<3, 3>(0, 0) = angleAxis.matrix();
	return transform;
}

static Eigen::Matrix4f getOrthoProjection(float width, float height, float zNear, float zFar) {
	const auto zRange = zFar - zNear;

	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

	transform(0, 0) = .5 / width;
	transform(1, 1) = .5 / height;
	transform(2, 2) = -2. / zRange;
	transform(2, 3) = -(zNear + zFar) / zRange;

	return transform;
}

static Eigen::Matrix4f getPerspectiveProjection(float aspectRatio, float FOV, float zNear,
												float zFar) {
	const auto cotan  = 1. / tan(qDegreesToRadians(FOV) * .5);
	const auto zRange = zFar - zNear;

	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

	transform(0, 0) = cotan / aspectRatio;
	transform(1, 1) = cotan;
	transform(2, 2) = -(zNear + zFar) / zRange;
	transform(2, 3) = -2.0 * zNear * zFar / zRange;
	transform(3, 2) = -1.0;

	return transform;
}

WorldView::WorldView(QWidget* parent)
	: QOpenGLWidget(parent)
	, state(State::Startup) {
	setMinimumSize(640, 480);

	QSurfaceFormat surfaceFormat;
	surfaceFormat.setSamples(4);
	setFormat(surfaceFormat);

	connect(&idleTimer, &QTimer::timeout, this, &WorldView::onIdle);
	connect(&renderTimer, &QTimer::timeout, this, &WorldView::aboutToRender);

	idleTimer.start(0);
	renderTimer.start(10);
}

WorldView::~WorldView() {
	glDeleteBuffers(1, BOs);
}

void WorldView::initializeGL() {
	initializeOpenGLFunctions();

	shader.init();
	shader.addSource(GL_VERTEX_SHADER, R"(#version 450 core
layout (location = 0) in vec3 Position;
uniform mat4 WVP;
void main() { gl_Position = WVP * vec4(Position, 1.0); })");
	shader.addSource(GL_FRAGMENT_SHADER, R"(#version 450 core
out vec4 FragColor;
void main() { FragColor = vec4(1.0, 0.0, 0.0, 1.0); })");
	shader.finalize();

	glGenBuffers(1, BOs);

	auto	   g  = std::mt19937(std::chrono::system_clock::now().time_since_epoch().count());
	auto	   gp = std::uniform_real_distribution<float>(-64.0, 64.0);
	auto	   ga = std::uniform_real_distribution<float>(.0, M_PI * 2.0);
	const auto n  = (g() % 1024) + 1;
	entities.emplace_back(Eigen::Vector2f(.0, .0), .0);
	for (int i = 0; i < n; ++i) {
		entities.emplace_back(Eigen::Vector2f(gp(g), gp(g)), ga(g));
	}
}

void WorldView::resizeGL(int w, int h) {
	glViewport(0, 0, w, h);
}

void WorldView::paintGL() {
	shader.bind();

	glBindBuffer(GL_ARRAY_BUFFER, BOs[0]);
	glBufferData(GL_ARRAY_BUFFER,
				 vertices.size() * sizeof(Eigen::Vector3f),
				 vertices.data(),
				 GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, false, 0, 0);

	glEnableVertexAttribArray(0);
	glDrawArrays(GL_TRIANGLES, 0, vertices.size());
	glDisableVertexAttribArray(0);

	shader.release();
}

void WorldView::keyPressEvent(QKeyEvent* e) {
	if (e->key() <= 0x7f) {
		keyPressed.set(e->key());
	}

	switch (e->key()) {
		case Qt::Key_Tab: {
			if (entities.size() > 1) {
				aboutToSwitch = true;
			}
			e->accept();
		} break;
		default: break;
	}
}

void WorldView::keyReleaseEvent(QKeyEvent* e) {
	if (e->key() <= 0x7f) {
		keyPressed.reset(e->key());
	}

	switch (e->key()) {
		case Qt::Key_F11: {
			setWindowState(windowState() ^ Qt::WindowFullScreen);
			e->accept();
		} break;
		case Qt::Key_Return: {
			if (e->isAutoRepeat()) {
				e->ignore();
			} else if (aboutToSwitch) {
				roleTarget	  = switchTarget;
				aboutToSwitch = false;
				e->accept();
			}
		} break;
		default: break;
	}
}

void WorldView::paintEvent(QPaintEvent* e) {
	QOpenGLWidget::paintEvent(e);

	QPainter painter(this);
	painter.setRenderHint(QPainter::Antialiasing);
	painter.setPen(Qt::white);
	painter.setFont(QFont("得意黑", 16));
	painter.drawText(rect(),
					 QString("%1%3\n%2%4")
						 .arg(tr("当前时间："))
						 .arg(tr("当前坐标："))
						 .arg(world.toDayTime())
						 .arg(QString("(%1, %2)")
								  .arg(entities[roleTarget].position.x())
								  .arg(entities[roleTarget].position.y())));
}

void WorldView::onIdle() {
	switch (state) {
		case State::Startup: {
			show();
			state = State::Initialize;
		} break;
		case State::Initialize: {
			state = State::Normal;
			//! TODO
		} break;
		case State::Suspend: {
			//! TODO
		} break;
		case State::Normal: {
			//! TODO
		} break;
		case State::MindFlash: {
			//! TODO
		} break;
	}
}

void WorldView::aboutToRender() {
	switch (state) {
		case State::Initialize: {
			state = State::Normal;
			//! TODO
		} break;
		case State::Suspend: {
			//! TODO
		} break;
		case State::Normal: {
			Eigen::Vector2f dir = Eigen::Vector2f::Zero();
			if (keyPressed[Qt::Key_W]) dir += Eigen::Vector2f::UnitY();
			if (keyPressed[Qt::Key_S]) dir -= Eigen::Vector2f::UnitY();
			if (keyPressed[Qt::Key_D]) dir += Eigen::Vector2f::UnitX();
			if (keyPressed[Qt::Key_A]) dir -= Eigen::Vector2f::UnitX();
			if (!dir.isZero()) {
				entities[roleTarget].velocity = dir.normalized() * 0.5;
			} else {
				entities[roleTarget].velocity = Eigen::Vector2f::Zero();
			}

			world.stepForward();

			constexpr auto dt = 1;

			std::mt19937 g(std::chrono::high_resolution_clock::now().time_since_epoch().count());
			for (int i = 0; i < entities.size(); ++i) {
				if (i == roleTarget) continue;
				auto& entity = entities[i];
				if (entity.velocity.isZero()) {
					if (g() & 1) {
						const auto dir	 = g() * 1e-3;
						const auto speed = (g() % 100) * 1e-3;
						entity.velocity	 = Eigen::Vector2f(cos(dir), sin(dir)) * speed;
					}
				} else if (g() % 30 == 0) {
					const auto coeff	  = 5e-2;
					const auto currentDir = (signbit(entity.velocity.y()) ? -1.0 : 1.0) *
											acos(entity.velocity.normalized().x());
					const auto dir =
						(1.0 - coeff) * currentDir + coeff * fmod(g() * 1e-3, M_PI * 2.0);
					entity.velocity =
						Eigen::Vector2f(cos(dir), sin(dir)) *
						std::clamp<float>(entity.velocity.norm() *
											  std::uniform_real_distribution<float>(0.8, 1.2)(g),
										  0.01,
										  0.1);
				}
			}

			for (auto& entity : entities) {
				if (entity.velocity.isZero()) {
					entity.direction = Eigen::Vector2f::Zero();
					continue;
				}
				entity.position += entity.velocity * dt;
				if (entity.direction.isZero()) {
					entity.direction = Eigen::Vector2f::UnitX();
				}
				const auto u	 = entity.direction;
				const auto v	 = entity.velocity.normalized();
				auto	   m	 = (signbit(u.y()) ? -1.0 : 1.0) * acos(u.x());
				auto	   n	 = (signbit(v.y()) ? -1.0 : 1.0) * acos(v.x());
				const auto angle = (n - m > M_PI ? (m - n) * .2 : (n - m) * .2) + m;
				entity.direction = Eigen::Vector2f(cos(angle), sin(angle));
			}

			const auto worldToLocal = getTranslationTransform(Eigen::Vector3f(
				-entities[roleTarget].position.x(), -entities[roleTarget].position.y(), .0));
			const auto rotation		= getAxisRotationTransform(-30.0, Eigen::Vector3f::UnitX());
			const auto translation	= getTranslationTransform(-32 * Eigen::Vector3f::UnitZ());
			const auto projection =
				getPerspectiveProjection(1.0 * width() / height(), 60.0, 0.1, 32.0);
			Eigen::Matrix4f transform = projection * translation * rotation * worldToLocal;
			shader.setUniform<4, 4>("WVP", transform.data());

			const auto worldTime = world.toDayTime();
			vertices.clear();
			for (const auto& entity : entities) {
				generateEntityMode(entity, worldTime, vertices);
			}

			if (aboutToSwitch && entities.size() > 1) {
				auto target = std::make_pair(roleTarget, .0);
				for (int i = 0; i < entities.size(); ++i) {
					if (i == roleTarget) continue;
					const auto& u		= entities[roleTarget].position;
					const auto& v		= entities[i].position;
					const auto	negDist = -(u - v).norm();
					if (abs(negDist) < 1e-6) {
						target.second = i;
						break;
					}
					const auto negInvDist = 1.0 / negDist;
					if (negInvDist < target.second) {
						target.first  = i;
						target.second = negInvDist;
					}
				}
				switchTarget  = target.first;
				const auto& u = entities[roleTarget].position;
				const auto& v = entities[switchTarget].position;
				effectVertices.emplace_back(u.x(), u.y(), .0);
				effectVertices.emplace_back(v.x(), v.y(), .0);
				effectVertices.emplace_back(u.x(), u.y(), 1.0);
			}
			vertices.insert(vertices.end(), effectVertices.begin(), effectVertices.end());
			effectVertices.clear();
		} break;
		case State::MindFlash: {
			//! TODO
		} break;
		default: break;
	}

	update();
}