module;

export module helios.physics.collision.types:CollisionSnapshot;

import helios.math;

export namespace helios::physics::collision::types {

    struct CollisionSnapshot {
        helios::math::vec3f overlapCenter;
        helios::math::vec3f overlapNormal;
        bool leftDynamic;
        bool rightDynamic;

        explicit CollisionSnapshot(
            helios::math::vec3f overlapC,
            helios::math::vec3f overlapN,
            const bool leftIsDynamic = false,
            const bool rightIsDynamic = false
        ) : overlapCenter(overlapC),
            overlapNormal(overlapN),
            leftDynamic(leftIsDynamic),
            rightDynamic(rightIsDynamic) {
        }
    };

}




