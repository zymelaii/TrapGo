#pragma once

#include <Eigen/Core>

namespace v1 {

struct Role;

struct Camera {
	void setPose(Eigen::Vector3f lootAtDirection, Eigen::Vector3f upDirection);

	void attach(Role* role, Eigen::Matrix4f transform = Eigen::Matrix4f::Identity());
	void detach();

	void installOrthoProjection(float width, float height, float zNear, float zFar);
	void installPerspectiveProjection(float aspectRatio, float FOV, float zNear, float zFar);
	Eigen::Matrix4f getWorld2CameraTransform();

	Eigen::Vector3f position;
	Eigen::Vector3f lookAt;
	Eigen::Vector3f up;
	Eigen::Matrix4f projectionTransform;

	Role*			attachedRole;
	Eigen::Matrix4f roleToCameraTransform;
};

};	 // namespace v1