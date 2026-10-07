module;

export module helios.physics.collision.types:CollisionPair;

import helios.math;

export namespace helios::physics::collision::types {

    template<typename THandle>
    struct CollisionPair {
        THandle leftHandle;
        THandle rightHandle;
        helios::math::vec3f overlapCenter;
        helios::math::vec3f overlapNormal;
    };

}