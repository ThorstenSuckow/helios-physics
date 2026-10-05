module;

export module helios.physics.collision.types:CollisionPair;

import helios.math;

export namespace helios::physics::collision::types {

    template<typename THandle>
    struct CollisionPair {
        THandle leftHandle;
        THandle rightHandle;
        helios::math::vec3f leftVelocity;
        helios::math::vec3f rightVelocity;
        helios::math::vec3f overlapCenter;
    };

}