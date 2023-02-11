#include "Camera.h"

#include "Role.h"

#include <Eigen/Dense>
#include <QtCore>

namespace v1 {

void Camera::attach(Role* role, Eigen::Matrix4f transform) {
	if (attachedRole != nullptr) {
		detach();
	}
	attachedRole		  = role;
	roleToCameraTransform = transform;
}

void Camera::detach() {
	attachedRole		  = nullptr;
	roleToCameraTransform = Eigen::Matrix4f::Identity();
}

void Camera::setPose(Eigen::Vector3f lootAtDirection, Eigen::Vector3f upDirection) {
	lookAt = lootAtDirection.normalized();
	up	   = upDirection.normalized();
}

void Camera::installOrthoProjection(float width, float height, float zNear, float zFar) {
	const auto zRange = zFar - zNear;

	projectionTransform.setIdentity();
	projectionTransform(0, 0) = .5 / width;
	projectionTransform(1, 1) = .5 / height;
	projectionTransform(2, 2) = -2. / zRange;
	projectionTransform(2, 3) = -(zNear + zFar) / zRange;
}

void Camera::installPerspectiveProjection(float aspectRatio, float FOV, float zNear, float zFar) {
	const auto cotan  = 1. / tan(qDegreesToRadians(FOV) * .5);
	const auto zRange = zFar - zNear;

	projectionTransform.setIdentity();
	projectionTransform(0, 0) = cotan / aspectRatio;
	projectionTransform(1, 1) = cotan;
	projectionTransform(2, 2) = -(zNear + zFar) / zRange;
	projectionTransform(2, 3) = -2.0 * zNear * zFar / zRange;
	projectionTransform(3, 2) = -1.0;
}

Eigen::Matrix4f Camera::getWorld2CameraTransform() {
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	if (attachedRole != nullptr) {
		transform = roleToCameraTransform * attachedRole->getWorld2RoleTransform();
	} else {
		transform.block<3, 1>(0, 0) = lookAt.transpose();
		transform.block<3, 1>(0, 2) = up.transpose();
		transform.block<3, 1>(0, 3) = lookAt.cross(up).transpose();
	}
	return projectionTransform * transform;
}

}	// namespace v1