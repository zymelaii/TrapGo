#pragma once

#include <stddef.h>
#include <stdint.h>

struct World {
	uint8_t	 tick;
	uint8_t	 second;
	uint8_t	 minute;
	uint8_t	 hour;
	uint8_t	 day;
	uint8_t	 month;
	uint16_t year;

	World();

	void  stepForward();
	float toDayTime();
};