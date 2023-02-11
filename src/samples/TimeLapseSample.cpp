#include "TimeLapseSample.h"

#include <Eigen/Dense>
#include <iterator>

static inline Eigen::Matrix4f getTranslationTransform(const Eigen::Vector3f& translation) {
	Eigen::Matrix4f transform	= Eigen::Matrix4f::Identity();
	transform.block<3, 1>(0, 3) = translation.transpose();
	return transform;
}

static inline Eigen::Matrix4f getAxisRotationTransform(float				  rotation,
													   const Eigen::Vector3f& axis) {
	Eigen::Matrix4f transform	= Eigen::Matrix4f::Identity();
	const auto		angleAxis	= Eigen::AngleAxisf(qDegreesToRadians(rotation), axis);
	transform.block<3, 3>(0, 0) = angleAxis.matrix();
	return transform;
}

static inline void createCubeObject(const Eigen::Vector3f& position, const Eigen::Vector3f& scale,
									float angle, std::vector<Eigen::Vector3f>& outputVertices,
									std::vector<GLuint>& outputIndices, bool invert = false) {
	Eigen::Vector3f cubeVertices[]{
		{.5, .5, -.5},
		{-.5, .5, -.5},
		{-.5, -.5, -.5},
		{.5, -.5, -.5},
		{.5, .5, .5},
		{-.5, .5, .5},
		{-.5, -.5, .5},
		{.5, -.5, .5},
	};

	GLuint cubeIndices[]{0, 3, 2, 1, 4, 5, 6, 7, 1, 2, 6, 5, 0, 4, 7, 3, 0, 1, 5, 4, 2, 3, 7, 6};
	if (invert) {
		const auto end = cubeIndices + sizeof(cubeIndices) / sizeof(GLuint);
		for (auto it = cubeIndices + 4; it != end; it += 4) {
			std::reverse(it - 4, it);
		}
	}

	const auto offset	= outputVertices.size();
	const auto rotation = Eigen::AngleAxisf(qDegreesToRadians(angle), Eigen::Vector3f::UnitZ());

	auto vit = std::back_inserter(outputVertices);
	for (const auto& cubeVertex : cubeVertices) {
		auto vertex = cubeVertex;
		vertex(0) *= scale(0);
		vertex(1) *= scale(1);
		vertex(2) *= scale(2);
		vertex = rotation * vertex + position;
		vertex.z() += scale.z() * .5;
		*vit++ = vertex;
	}

	auto iit = std::back_inserter(outputIndices);
	for (const auto& cubeIndex : cubeIndices) {
		*iit++ = cubeIndex + offset;
	}
}

TimeLapseSample::TimeLapseSample(QWidget* parent)
	: AbstractOpenGLSample(parent)
	, moveIndicate_(.0, .0, .0)
	, viewRotateIndicate_(.0, .0)
	, moveAccumulate_(.0, .0, .0)
	, viewRotateAccumulate_(.0, .0)
	, pitch_(.0)
	, gravity_(9.8)
	, worldTime_(12.0)
	, zVelocity_(.0)
	, TPPCameraRotation_(.0)
	, viewJitter_(.0) {
	using namespace Eigen;
	role_.position		   = Vector3f::Zero();
	role_.velocity		   = 8.0;
	role_.angularVelocity  = 180.0;
	role_.currentDirection = .0;
	role_.targetDirection  = role_.currentDirection;

	setViewMode(ViewMode::FPP);

	moveControlBindings_.clear();
	moveControlBindings_.emplace(Qt::Key_W, Vector3f::UnitY());
	moveControlBindings_.emplace(Qt::Key_S, -Vector3f::UnitY());
	moveControlBindings_.emplace(Qt::Key_A, -Vector3f::UnitX());
	moveControlBindings_.emplace(Qt::Key_D, Vector3f::UnitX());

	viewRotateControlBindings_.clear();
	viewRotateControlBindings_.emplace(static_cast<Qt::Key>(';'), Vector2f::UnitX());
	viewRotateControlBindings_.emplace(static_cast<Qt::Key>('\''), -Vector2f::UnitX());
	viewRotateControlBindings_.emplace(Qt::Key_Up, Vector2f::UnitY());
	viewRotateControlBindings_.emplace(Qt::Key_Down, -Vector2f::UnitY());

	QSurfaceFormat surfaceFormat;
	surfaceFormat.setSamples(4);
	setFormat(surfaceFormat);

	connect(&idleTimer_, &QTimer::timeout, this, &TimeLapseSample::onIdle);
}

TimeLapseSample::~TimeLapseSample() {
	if (VBO_ != 0) {
		glDeleteBuffers(1, &VBO_);
		VBO_ = 0;
	}

	if (IBO_ != 0) {
		glDeleteBuffers(1, &IBO_);
		IBO_ = 0;
	}

	if (skyboxTexture_ != 0) {
		glDeleteTextures(1, &skyboxTexture_);
		skyboxTexture_ = 0;
	}

	if (skyboxVBO_ != 0) {
		glDeleteBuffers(1, &skyboxVBO_);
		skyboxVBO_ = 0;
	}

	camera_.detach();
}

void TimeLapseSample::keyPressEvent(QKeyEvent* e) {
	const auto key = static_cast<Qt::Key>(e->key());

	if (!e->isAutoRepeat()) {
		if (auto it = moveControlBindings_.find(key); it != moveControlBindings_.end()) {
			moveIndicate_ += it->second;
			e->accept();
		}

		if (auto it = viewRotateControlBindings_.find(key);
			it != viewRotateControlBindings_.end()) {
			viewRotateIndicate_ += it->second;
			e->accept();
		}

		if (key == Qt::Key_Space) {
			if (role_.position.z() == 0.) {
				zVelocity_ += 4.0;
			}
			e->accept();
		}
	}

	if (key == Qt::Key_Shift) {
		lockPitch_ = true;
		e->accept();
	}

	if (!e->isAccepted()) {
		e->ignore();
	}
}

void TimeLapseSample::keyReleaseEvent(QKeyEvent* e) {
	const auto key = static_cast<Qt::Key>(e->key());

	if (!e->isAutoRepeat()) {
		if (auto it = moveControlBindings_.find(key); it != moveControlBindings_.end()) {
			moveIndicate_ -= it->second;
			e->accept();
		}

		if (auto it = viewRotateControlBindings_.find(key);
			it != viewRotateControlBindings_.end()) {
			viewRotateIndicate_ -= it->second;
			e->accept();
		}

		if (key == Qt::Key_Tab) {
			if (viewMode_ == ViewMode::FPP) {
				setViewMode(ViewMode::TPP);
			} else if (viewMode_ == ViewMode::TPP) {
				setViewMode(ViewMode::FPP);
			}
			e->accept();
		}

		if (key == Qt::Key_F11) {
			setWindowState(windowState() ^ Qt::WindowFullScreen);
			e->accept();
		}
	}

	if (key == Qt::Key_Shift) {
		lockPitch_ = false;
		e->accept();
	}

	if (!e->isAccepted()) {
		e->ignore();
	}
}

void TimeLapseSample::initializeGL() {
	AbstractOpenGLSample::initializeGL();

	setupScene();

	worldShader_.init();
	worldShader_.add(GL_VERTEX_SHADER, ":/assets/world.vs");
	worldShader_.add(GL_FRAGMENT_SHADER, ":/assets/world-2.fs");
	worldShader_.finalize();

	skyboxShader_.init();
	skyboxShader_.add(GL_VERTEX_SHADER, ":/assets/skybox.vs");
	skyboxShader_.add(GL_FRAGMENT_SHADER, ":/assets/skybox-2.fs");
	skyboxShader_.finalize();

	floorShader_.init();
	floorShader_.add(GL_VERTEX_SHADER, ":/assets/floor.vs");
	floorShader_.add(GL_FRAGMENT_SHADER, ":/assets/world-2.fs");
	floorShader_.finalize();

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);

	worldShader_.setUniform("SpotLight.cutoff", 10.0);
	worldShader_.setUniform("SpotLight.maxCutoff", 15.0);
	floorShader_.setUniform("SpotLight.cutoff", 10.0);
	floorShader_.setUniform("SpotLight.maxCutoff", 15.0);

	idleTimer_.start(0);
}

void TimeLapseSample::resizeGL(int w, int h) {
	AbstractOpenGLSample::resizeGL(w, h);
	camera_.installPerspectiveProjection(1.0 * w / h, 60.0, .5, 1024.0);
}

void TimeLapseSample::paintGL() {
	const size_t N = roleVertices_.size() * 3;

	if (viewMode_ == ViewMode::FPP) {
		renderSkybox();
	}

	renderFloor();

	GLint cullFaceMode;
	glGetIntegerv(GL_CULL_FACE_MODE, &cullFaceMode);
	glCullFace(GL_BACK);
	worldShader_.bind();
	glBindBuffer(GL_ARRAY_BUFFER, VBO_);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IBO_);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	if (viewMode_ == ViewMode::TPP) {
		glDrawElements(GL_QUADS, indices_.size(), GL_UNSIGNED_INT, 0);
	} else {
		glDrawElements(GL_QUADS, indices_.size() - N, GL_UNSIGNED_INT, (void*)(N * sizeof(GLuint)));
	}
	glDisableVertexAttribArray(0);
	worldShader_.release();
	glCullFace(cullFaceMode);
}

void TimeLapseSample::initRoleModel() {
	const auto headSize = Eigen::Vector3f(.4, .4, .38);
	const auto bodySize = Eigen::Vector3f(.4, .3, .6);
	const auto armSize	= Eigen::Vector3f(.09, .09, .72);
	const auto legSize	= Eigen::Vector3f(.18, .2, .8);

	//! head
	createCubeObject(Eigen::Vector3f(.0, .0, legSize.z() + bodySize.z()),
					 headSize,
					 role_.currentDirection,
					 vertices_,
					 indices_);

	//! body
	createCubeObject(Eigen::Vector3f(.0, .0, legSize.z()),
					 bodySize,
					 role_.currentDirection,
					 vertices_,
					 indices_);

	//! left arm
	createCubeObject(
		Eigen::Vector3f(
			-(bodySize.x() + armSize.x()) * .5, .0, legSize.z() + bodySize.z() - armSize.z()),
		armSize,
		role_.currentDirection,
		vertices_,
		indices_);

	//! right arm
	createCubeObject(
		Eigen::Vector3f(
			+(bodySize.x() + armSize.x()) * .5, .0, legSize.z() + bodySize.z() - armSize.z()),
		armSize,
		role_.currentDirection,
		vertices_,
		indices_);

	//! left leg
	createCubeObject(Eigen::Vector3f(-(bodySize.x() - legSize.x()) * .5, .0, .0),
					 legSize,
					 role_.currentDirection,
					 vertices_,
					 indices_);
	//! right leg
	createCubeObject(Eigen::Vector3f(+(bodySize.x() - legSize.x()) * .5, .0, .0),
					 legSize,
					 role_.currentDirection,
					 vertices_,
					 indices_);

	roleVertices_.assign(vertices_.begin(), vertices_.end());
}

void TimeLapseSample::setupScene() {
	initRoleModel();

	createCubeObject({13, 25, 0}, {8, 8, 4}, 30.0, vertices_, indices_, false);	   //!< Cube 0 - out
	createCubeObject({13, 25, 0}, {8, 8, 4}, 30.0, vertices_, indices_, true);	   //!< Cube 0 - in
	createCubeObject({2.8, -5, 0}, {4, 4, 6}, 75.0, vertices_, indices_, false);   //!< Cube 1 - out
	createCubeObject({2.8, -5, 0}, {4, 4, 6}, 75.0, vertices_, indices_, true);	   //!< Cube 1 - in

	glGenBuffers(1, &VBO_);
	glBindBuffer(GL_ARRAY_BUFFER, VBO_);
	glBufferData(GL_ARRAY_BUFFER,
				 vertices_.size() * sizeof(Eigen::Vector3f),
				 vertices_.data(),
				 GL_DYNAMIC_DRAW);

	glGenBuffers(1, &IBO_);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IBO_);
	glBufferData(
		GL_ELEMENT_ARRAY_BUFFER, indices_.size() * sizeof(GLuint), indices_.data(), GL_STATIC_DRAW);


	initSkybox(":/assets/skybox.jpg");
}

void TimeLapseSample::initSkybox(const QString& texturePath) {
	QImage		 texture(texturePath);
	const size_t size = texture.width() / 4;

	auto up		= texture.copy(size, 0, size, size);
	auto bottom = texture.copy(size, size * 2, size, size);
	auto front	= texture.copy(size, size, size, size);
	auto back	= texture.copy(size * 3, size, size, size);
	auto left	= texture.copy(0, size, size, size);
	auto right	= texture.copy(size * 2, size, size, size);

	QImage* textures[6]{&right, &left, &up, &bottom, &front, &back};

	glGenTextures(1, &skyboxTexture_);
	glBindTexture(GL_TEXTURE_CUBE_MAP, skyboxTexture_);

	for (int i = 0; i < 6; ++i) {
		glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
					 0,
					 GL_RGB,
					 size,
					 size,
					 0,
					 GL_BGRA,
					 GL_UNSIGNED_BYTE,
					 textures[i]->bits());

		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
	}

	Eigen::Vector3f data[]{
		{+1.0, +1.0, -1.0}, {+1.0, +1.0, +1.0}, {+1.0, -1.0, +1.0}, {+1.0, -1.0, -1.0},
		{-1.0, +1.0, -1.0}, {-1.0, -1.0, -1.0}, {-1.0, -1.0, +1.0}, {-1.0, +1.0, +1.0},
		{+1.0, +1.0, -1.0}, {-1.0, +1.0, -1.0}, {-1.0, +1.0, +1.0}, {+1.0, +1.0, +1.0},
		{+1.0, -1.0, -1.0}, {+1.0, -1.0, +1.0}, {-1.0, -1.0, +1.0}, {-1.0, -1.0, -1.0},
		{+1.0, -1.0, +1.0}, {+1.0, +1.0, +1.0}, {-1.0, +1.0, +1.0}, {-1.0, -1.0, +1.0},
		{+1.0, -1.0, -1.0}, {-1.0, -1.0, -1.0}, {-1.0, +1.0, -1.0}, {+1.0, +1.0, -1.0},
	};

	glGenBuffers(1, &skyboxVBO_);
	glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(data), data, GL_STATIC_DRAW);
}

void TimeLapseSample::renderSkybox() {
	GLint cullFaceMode;
	GLint depthFuncMode;
	glGetIntegerv(GL_CULL_FACE_MODE, &cullFaceMode);
	glGetIntegerv(GL_DEPTH_FUNC, &depthFuncMode);
	glCullFace(GL_FRONT);
	glDepthFunc(GL_LEQUAL);

	skyboxShader_.bind();
	skyboxShader_.setUniform("sampleCubeTexture", 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_CUBE_MAP, skyboxTexture_);

	glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO_);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glDrawArrays(GL_QUADS, 0, 24);

	glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
	glDisableVertexAttribArray(0);
	skyboxShader_.release();

	glCullFace(cullFaceMode);
	glDepthFunc(depthFuncMode);
}

void TimeLapseSample::renderFloor() {
	Eigen::Vector2f metaData[4]{
		{1.0, 0.0},
		{1.0, 1.0},
		{0.0, 1.0},
		{0.0, 0.0},
	};

	constexpr auto	Span = 15;
	constexpr auto	N	 = Span * Span * 16;
	Eigen::Vector3f data[N];
	for (int i = 0; i < Span * 2; ++i) {
		for (int j = 0; j < Span * 2; ++j) {
			const auto index  = (i * Span * 2 + j) * 4;
			const auto offset = Eigen::Vector2f(i - Span, j - Span);
			for (int k = 0; k < 4; ++k) {
				data[index + k].block<2, 1>(0, 0) = metaData[k] + offset;
				data[index + k](2)				  = (i & 1) ^ (j & 1) ? 1.0 : 0.0;
			}
		}
	}

	GLuint VBO;
	glGenBuffers(1, &VBO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(data), data, GL_STATIC_DRAW);

	floorShader_.bind();
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Eigen::Vector3f), 0);
	glVertexAttribPointer(
		1, 1, GL_FLOAT, GL_FALSE, sizeof(Eigen::Vector3f), (void*)sizeof(Eigen::Vector2f));
	glDrawArrays(GL_QUADS, 0, N * 3);
	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	floorShader_.release();

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glDeleteBuffers(1, &VBO);
}

void TimeLapseSample::setViewMode(ViewMode mode) {
	using namespace Eigen;

	switch (mode) {
		case ViewMode::FPP: {
			const Matrix4f r2cRotation	  = getAxisRotationTransform(-90.0, Vector3f::UnitX());
			const Matrix4f r2cTranslation = getTranslationTransform(-1.6 * Vector3f::UnitY());
			camera_.attach(&role_, r2cTranslation * r2cRotation);
			viewJitter_ = .0;
		} break;
		case ViewMode::TPP: {
			constexpr auto viewDistance = 12.0;
			const Matrix4f r2cRotation	= getAxisRotationTransform(-45.0, Vector3f::UnitX());
			const Matrix4f r2cTranslation =
				getTranslationTransform(-viewDistance * Vector3f::UnitZ());
			camera_.attach(&role_, r2cTranslation * r2cRotation);
			viewJitter_ = .0;
		} break;
	}

	viewMode_ = mode;
}

void TimeLapseSample::computeNextFrame() {
	switch (viewMode_) {
		case ViewMode::FPP: {
			updateFPPTransform();
		} break;
		case ViewMode::TPP: {
			updateTPPTransform();
		} break;
	}
}

void TimeLapseSample::updateFPPTransform() {
	using namespace Eigen;
	role_.moveForward(moveAccumulate_);
	role_.currentDirection += viewRotateAccumulate_(0);
	role_.targetDirection = role_.currentDirection;
	moveAccumulate_.setZero();

	Matrix4f transform = Matrix4f::Identity();

	if (viewRotateAccumulate_(1) == .0 && !lockPitch_) {
		pitch_ *= 0.8;
	}

	pitch_ = std::clamp<float>(pitch_ + viewRotateAccumulate_(1), -30.0, 90.0);
	viewRotateAccumulate_.setZero();

	auto viewJitter = .0;
	if (!moveIndicate_.isZero()) {
		viewJitter_ = fmod(viewJitter_ + M_PI_2 * .1, M_PI * 2.0);
		viewJitter	= sin(viewJitter_) * .5;
	} else {
		viewJitter_ = .0;
	}

	auto roleToCameraTransform	  = camera_.roleToCameraTransform;
	camera_.roleToCameraTransform = getAxisRotationTransform(viewJitter, Eigen::Vector3f::UnitZ()) *
									getAxisRotationTransform(-pitch_, Eigen::Vector3f::UnitX()) *
									camera_.roleToCameraTransform;

	transform					  = camera_.getWorld2CameraTransform();
	camera_.roleToCameraTransform = roleToCameraTransform;

	skyboxShader_.setUniform<4, 4>("WVP", transform.data());
	skyboxShader_.setUniform("time", worldTime_);

	floorShader_.setUniform<4, 4>("WVP", transform.data());
	floorShader_.setUniform(
		"rolePosition", role_.position.x(), role_.position.y(), role_.position.z());
	floorShader_.setUniform("time", worldTime_);

	worldShader_.setUniform<4, 4>("WVP", transform.data());
	worldShader_.setUniform("time", worldTime_);

	worldShader_.setUniform("SpotLight.pos", role_.position.x(), role_.position.y(), role_.position.z() + 1.6);
	worldShader_.setUniform("SpotLight.dir", cos(role_.currentDirection), sin(role_.currentDirection), .0);
	floorShader_.setUniform("SpotLight.pos", role_.position.x(), role_.position.y(), role_.position.z() + 1.6);
	floorShader_.setUniform("SpotLight.dir", cos(role_.currentDirection), sin(role_.currentDirection), .0);
}

void TimeLapseSample::updateTPPTransform() {
	using namespace Eigen;

	Matrix4f transform = Matrix4f::Identity();

	constexpr auto TPPCameraRotationBase = 90.0;
	TPPCameraRotation_ += viewRotateAccumulate_(0);
	viewRotateAccumulate_.setZero();

	if (!moveIndicate_.isZero()) {
		auto dirNorm = moveIndicate_.normalized();
		auto u		 = Vector3f::UnitX().dot(dirNorm);
		auto v		 = Vector3f::UnitX().cross(dirNorm);
		auto angle	 = qRadiansToDegrees(acos(u));
		auto azimuth = v.z() >= 0. ? angle : 360.0 - angle;
		role_.setTargetDirection(TPPCameraRotation_ + azimuth - TPPCameraRotationBase);
	}

	if (!moveIndicate_.isZero()) {
		viewJitter_ = fmod(viewJitter_ + M_PI_2 * .1, M_PI * 2.0);
	} else {
		viewJitter_ = .0;
	}

	role_.update();

	auto direction		   = role_.currentDirection;
	auto height			   = role_.position.z();
	role_.currentDirection = TPPCameraRotation_;
	role_.position.z()	   = .0;
	role_.moveForward(moveAccumulate_);
	moveAccumulate_.setZero();
	transform			   = camera_.getWorld2CameraTransform();
	role_.currentDirection = direction;
	role_.position.z()	   = height;

	skyboxShader_.setUniform<4, 4>("WVP", transform.data());
	skyboxShader_.setUniform("time", worldTime_);

	floorShader_.setUniform<4, 4>("WVP", transform.data());
	floorShader_.setUniform(
		"rolePosition", role_.position.x(), role_.position.y(), role_.position.z());
	floorShader_.setUniform("time", worldTime_);

	worldShader_.setUniform<4, 4>("WVP", transform.data());
	worldShader_.setUniform("time", worldTime_);

	updateRoleAnimation();
}

void TimeLapseSample::updateRoleAnimation() {
	using namespace Eigen;

	Matrix4f transform = Matrix4f::Identity();

	std::copy(roleVertices_.begin(), roleVertices_.end(), vertices_.begin());

	const auto headSize			 = Vector3f(.4, .4, .38);
	const auto bodySize			 = Vector3f(.4, .3, .6);
	const auto armSize			 = Vector3f(.09, .09, .72);
	const auto legSize			 = Vector3f(.18, .2, .8);
	const auto armCentroidHeight = legSize.z() + bodySize.z() - .045;

	//! head
	//! body
	//! left arm
	transform = getTranslationTransform(armCentroidHeight * Vector3f::UnitZ()) *
				getAxisRotationTransform(sin(viewJitter_) * 30.0, Vector3f::UnitX()) *
				getTranslationTransform(-armCentroidHeight * Vector3f::UnitZ());
	for (int i = 2 * 8; i < 3 * 8; ++i) {
		auto vertex				 = Vector4f(.0, .0, .0, 1.0);
		vertex.block<3, 1>(0, 0) = vertices_[i];
		vertices_[i]			 = (transform * vertex).block<3, 1>(0, 0);
	}
	//! right arm
	transform = getTranslationTransform(armCentroidHeight * Vector3f::UnitZ()) *
				getAxisRotationTransform(-sin(viewJitter_) * 30.0, Vector3f::UnitX()) *
				getTranslationTransform(-armCentroidHeight * Vector3f::UnitZ());
	for (int i = 3 * 8; i < 4 * 8; ++i) {
		auto vertex				 = Vector4f(.0, .0, .0, 1.0);
		vertex.block<3, 1>(0, 0) = vertices_[i];
		vertices_[i]			 = (transform * vertex).block<3, 1>(0, 0);
	}
	//! left leg
	transform = getTranslationTransform(legSize.z() * Vector3f::UnitZ()) *
				getAxisRotationTransform(-sin(viewJitter_) * 40.0, Vector3f::UnitX()) *
				getTranslationTransform(-legSize.z() * Vector3f::UnitZ());
	for (int i = 4 * 8; i < 5 * 8; ++i) {
		auto vertex				 = Vector4f(.0, .0, .0, 1.0);
		vertex.block<3, 1>(0, 0) = vertices_[i];
		vertices_[i]			 = (transform * vertex).block<3, 1>(0, 0);
	}
	//! right leg
	transform = getTranslationTransform(legSize.z() * Vector3f::UnitZ()) *
				getAxisRotationTransform(sin(viewJitter_) * 40.0, Vector3f::UnitX()) *
				getTranslationTransform(-legSize.z() * Vector3f::UnitZ());
	for (int i = 5 * 8; i < 6 * 8; ++i) {
		auto vertex				 = Vector4f(.0, .0, .0, 1.0);
		vertex.block<3, 1>(0, 0) = vertices_[i];
		vertices_[i]			 = (transform * vertex).block<3, 1>(0, 0);
	}

	transform = getTranslationTransform(role_.position) *
				getAxisRotationTransform(role_.currentDirection, Vector3f::UnitZ());
	for (int i = 0; i < roleVertices_.size(); ++i) {
		auto vertex				 = Vector4f(.0, .0, .0, 1.0);
		vertex.block<3, 1>(0, 0) = vertices_[i];
		vertices_[i]			 = (transform * vertex).block<3, 1>(0, 0);
	}

	glBindBuffer(GL_ARRAY_BUFFER, VBO_);
	glBufferSubData(GL_ARRAY_BUFFER, 0, roleVertices_.size() * sizeof(Vector3f), vertices_.data());
}

void TimeLapseSample::onIdle() {
	using std::chrono::duration_cast;
	using namespace Eigen;
	static auto idleRec	  = std::chrono::high_resolution_clock::now();
	static auto updateRec = idleRec;

	auto now  = std::chrono::high_resolution_clock::now();
	auto diff = duration_cast<std::chrono::nanoseconds>(now - idleRec).count();
	idleRec	  = now;

	Vector3f moveIndicate = Vector3f::Zero();
	if (!moveIndicate_.isZero()) {
		moveIndicate = moveIndicate_.normalized() * role_.velocity;
	}

	Vector2f viewRotateIndicate = viewRotateIndicate_ * role_.angularVelocity;

	const auto dt = diff * 1e-9;
	moveAccumulate_ += moveIndicate * dt;
	viewRotateAccumulate_ += viewRotateIndicate * dt;

	worldTime_ += dt / (300.0 / 24.0);

	role_.position.z() = std::max(.0, role_.position.z() + zVelocity_ * dt);
	zVelocity_		   = role_.position.z() == 0. ? 0. : zVelocity_ - gravity_ * dt;

	diff = duration_cast<std::chrono::milliseconds>(now - updateRec).count();
	if (diff > 10) {
		updateRec = now;
		computeNextFrame();
		update();
	}
}
