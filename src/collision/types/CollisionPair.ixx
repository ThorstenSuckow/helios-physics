module;

#include <cstdint>
export module helios.physics.collision.types:CollisionPair;

import helios.math;
import :CollisionSnapshot;

export namespace helios::physics::collision::types {

    template<typename THandle>
    struct CollisionPair {

        std::uint64_t key;

        THandle leftHandle;
        THandle rightHandle;
        CollisionSnapshot collisionSnapshot;

        explicit CollisionPair(
            THandle leftH,
            THandle rightH,
            helios::math::vec3f overlapC,
            helios::math::vec3f overlapN) :
            key {(std::uint64_t{leftH.entityId()} << 32) | std::uint64_t{rightH.entityId()}},
            leftHandle(leftH),
            rightHandle(rightH),
            collisionSnapshot(CollisionSnapshot{overlapC, overlapN}) {}

        bool operator==(const CollisionPair& other) const {
            return key == other.key;
        }

        bool operator<(const CollisionPair& other) const {
            return key < other.key;
        }

    };

}