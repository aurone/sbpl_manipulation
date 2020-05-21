#ifndef FRANKA_ALLOWED_COLLISION_PAIRS_H
#define FRANKA_ALLOWED_COLLISION_PAIRS_H

#include <utility>

// Copied from pr2.srdf
const std::pair<const char*, const char*> FrankaAllowedCollisionPairs[] =
{
	{ "panda_link0", "panda_link1" },
	{ "panda_link1", "panda_link2" },
	{ "panda_link2", "panda_link3" },
	{ "panda_link3", "panda_link4" },
	{ "panda_link4", "panda_link5" },
	{ "panda_link5", "panda_link6" },
	{ "panda_link6", "panda_link7" },
	{ "panda_link7", "panda_hand" }
};

#endif
