#pragma once

#include <Eigen/Core>

namespace v1 {

struct Role {
	void moveForward(const Eigen::Vector3f& movement);
	void update();

	void			setTargetDirection(float direction);
	Eigen::Matrix4f getWorld2RoleTransform() const;

	Eigen::Vector3f position;
	float			velocity;
	float			angularVelocity;
	float			currentDirection;
	float			targetDirection;
};

}	// namespace v1