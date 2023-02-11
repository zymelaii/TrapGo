#include "Role.h"

#include <Eigen/Dense>
#include <QtCore>

namespace v1 {

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

void Role::moveForward(const Eigen::Vector3f& movement) {
	using namespace Eigen;
	auto rotation = AngleAxisf(qDegreesToRadians(currentDirection), Vector3f::UnitZ());
	position += rotation * movement;
}

void Role::update() {
	currentDirection += (targetDirection - currentDirection) * 0.2;
}

void Role::setTargetDirection(float direction) {
	targetDirection = direction;
	if (targetDirection - currentDirection > 180.0) {
		targetDirection -= 360.0;
	}
}

Eigen::Matrix4f Role::getWorld2RoleTransform() const {
	using namespace Eigen;
	Matrix4f translation = getTranslationTransform(-position);
	Matrix4f rotation	 = getAxisRotationTransform(-currentDirection, Vector3f::UnitZ());
	return rotation * translation;
}

}	// namespace v1