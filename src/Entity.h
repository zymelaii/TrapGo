#pragma once

#include <Eigen/Core>
#include <QtCore>
#include <vector>

struct Entity {
	Eigen::Vector2f position;
	Eigen::Vector2f direction;
	Eigen::Vector2f velocity;
	Eigen::Vector2f target;

	Entity(const Eigen::Vector2f& pos, float dir);
};

void generateEntityMode(const Entity& entity, float time, std::vector<Eigen::Vector3f>& verticies);