#ifndef BETTERSCENE_H
#define BETTERSCENE_H

#include "../AbstractOpenGLSample.h"
#include "../Shader.h"
#include "share/v1/Role.h"
#include "share/v1/Camera.h"

#include <Eigen/Core>
#include <unordered_map>
#include <vector>

class BetterScene : public AbstractOpenGLSample {
	Q_OBJECT

public:
	explicit BetterScene(QWidget* parent = nullptr);
	~BetterScene();

	virtual void keyPressEvent(QKeyEvent* e) override;
	virtual void keyReleaseEvent(QKeyEvent* e) override;

	virtual void initializeGL() override;
	virtual void resizeGL(int w, int h) override;
	virtual void paintGL() override;

	void initRoleModel();
	void setupScene();
	void initSkybox(const QString& texturePath);
	void renderSkybox();
	void renderFloor();

	enum class ViewMode { FPP, TPP };
	void setViewMode(ViewMode mode);

	void computeNextFrame();
	void updateFPPTransform();
	void updateTPPTransform();
	void updateRoleAnimation();

public slots:
	void onIdle();

private:
	QTimer idleTimer_;

	std::unordered_map<Qt::Key, Eigen::Vector3f> moveControlBindings_;
	Eigen::Vector3f								 moveIndicate_;
	Eigen::Vector3f								 moveAccumulate_;

	std::unordered_map<Qt::Key, Eigen::Vector2f> viewRotateControlBindings_;
	Eigen::Vector2f								 viewRotateIndicate_;
	Eigen::Vector2f								 viewRotateAccumulate_;
	float										 pitch_;
	bool										 lockPitch_;

	const float gravity_;
	float		zVelocity_;
	float		TPPCameraRotation_;
	float		viewJitter_;

	v1::Role   role_;
	v1::Camera camera_;

	Shader worldShader_;
	Shader skyboxShader_;
	Shader floorShader_;

	GLuint VBO_;
	GLuint IBO_;
	GLuint skyboxVBO_;
	GLuint skyboxTexture_;

	std::vector<Eigen::Vector3f> roleVertices_;
	std::vector<Eigen::Vector3f> vertices_;
	std::vector<GLuint>			 indices_;

	ViewMode viewMode_;
};

#endif	 // BETTERSCENE_H
