#pragma once
#include "membership/Types.h"

struct MemberInfo {
    DroneStatus status;
    double lastHeartbeatTime;
    glm::vec3 lastKnownPosition;
	int32_t groupHint;
};
