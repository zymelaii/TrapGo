#include "World.h"

World::World()
	: tick(0)
	, second(0)
	, minute(0)
	, hour(0)
	, day(0)
	, month(0)
	, year(0) {}

void World::stepForward() {
	int t = 1;

	constexpr int bounds[]{100, 60, 60, 24, 30, 12};

	tick += t;
	t = !(tick ^ bounds[0]);
	tick %= bounds[0];

	second += t;
	t = !(second ^ bounds[1]);
	second %= bounds[1];

	minute += t;
	t = !(minute ^ bounds[2]);
	minute %= bounds[2];

	hour += t;
	t = !(hour ^ bounds[3]);
	hour %= bounds[3];

	day += t;
	t = !(day ^ bounds[4]);
	day %= bounds[4];

	month += t;
	t = !(month ^ bounds[5]);
	month %= bounds[5];

	year += t;
}

float World::toDayTime() {
	float timestamp = hour;
	timestamp		= timestamp * 60.0 + minute;
	timestamp		= timestamp * 60.0 + second;
	timestamp		= timestamp + tick * 0.01;
	return timestamp;
}