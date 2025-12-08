#pragma once
#include <glm/glm.hpp>

using Vec3 = glm::vec3;

enum class DroneStatus {
    UP,
    DOWN
};

enum class MembershipStatus {
    MEMBER,
    NOT_MEMBER,
    SUSPECT
};
#pragma once
