#include "Entity.h"

#include <Eigen/Dense>
#include <QtCore>
#include <vector>
#include <array>
#include <math.h>

Entity::Entity(const Eigen::Vector2f& pos, float dir)
	: position(pos)
	, direction(cos(qDegreesToRadians(dir)), sin(qDegreesToRadians(dir)))
	, velocity(Eigen::Vector2f::Zero())
	, target(Eigen::Vector2f::Zero()) {}

void generateEntityMode(const Entity& entity, float time, std::vector<Eigen::Vector3f>& verticies) {
	auto it = std::back_inserter(verticies);

	Eigen::Vector3f centroid   = Eigen::Vector3f::Zero();
	centroid.block<2, 1>(0, 0) = entity.position;

	constexpr auto degreeStep	  = 15;
	constexpr auto radius		  = 1.0;
	const auto	   floatingHeight = 0.5 + sin(time * 8) * 0.4;
	for (int i = 0; i < 360; i += degreeStep) {
		*it++ = centroid + Eigen::Vector3f::UnitZ() * floatingHeight;
		*it++ = centroid + Eigen::Vector3f(radius * cos(qDegreesToRadians(i)),
										   radius * sin(qDegreesToRadians(i)),
										   floatingHeight);
		*it++ = centroid + Eigen::Vector3f(radius * cos(qDegreesToRadians((i + degreeStep) % 360)),
										   radius * sin(qDegreesToRadians((i + degreeStep) % 360)),
										   floatingHeight);
	}

	constexpr auto spacing	  = 0.2;
	const auto	   arrowShape = Eigen::Vector3f(1.2, 0.5, 0.3);	  //! (width, height, centroid)
	std::array<Eigen::Vector3f, 6> arrowVertices;
	arrowVertices[0] = Eigen::Vector3f(radius + spacing, .0, .0);
	arrowVertices[1] = arrowVertices[0] + Eigen::Vector3f(arrowShape(2), -arrowShape(1) * .5, .0);
	arrowVertices[2] = arrowVertices[0] + Eigen::Vector3f(arrowShape(2), +arrowShape(1) * .5, .0);
	arrowVertices[3] = arrowVertices[2];
	arrowVertices[4] = arrowVertices[1];
	arrowVertices[5] = arrowVertices[0] + Eigen::Vector3f(arrowShape(0), .0, .0);

	if (!entity.direction.isZero()) {
		const auto degree =
			(signbit(entity.direction.y()) ? -1.0 : 1.0) * acos(entity.direction.normalized().x());
		const auto rotation = Eigen::AngleAxisf(degree, Eigen::Vector3f::UnitZ());
		for (const auto& e : arrowVertices) {
			*it++ = rotation * e + centroid;
		}
		return;
	}

	for (int i = 0; i < 4; ++i) {
		const auto degree	= M_PI_2 * i;
		const auto rotation = Eigen::AngleAxisf(degree, Eigen::Vector3f::UnitZ());
		for (const auto& e : arrowVertices) {
			*it++ = rotation * e + centroid;
		}
	}
}