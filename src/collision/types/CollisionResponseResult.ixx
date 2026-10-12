module;

export module helios.physics.collision.types:CollisionResponseResult;

import helios.math;

export namespace helios::physics::collision::types {

    struct CollisionResponseResult {
        helios::math::vec3f leftVelocity;
        helios::math::vec3f rightVelocity;

        explicit CollisionResponseResult(
            helios::math::vec3f leftV,
            helios::math::vec3f rightV
        ) : leftVelocity(leftV),
            rightVelocity(rightV) {
        }
    };

}

