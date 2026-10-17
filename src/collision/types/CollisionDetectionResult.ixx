module;

#include <vector>
#include <span>
export module helios.physics.collision.types:CollisionDetectionResult;

import :CollisionPair;


export namespace helios::physics::collision ::types{

    template<typename THandle>
    class CollisionDetectionResult {


        using CollisionPair = CollisionPair<THandle>;

        std::vector<CollisionPair> collisionPairs_;

    public:


        CollisionDetectionResult() = default;

        explicit CollisionDetectionResult(std::vector<CollisionPair>&& pairs)
        : collisionPairs_(std::move(pairs)){};


        [[nodiscard]] std::span<const CollisionPair> collisionPairs() const {
            return collisionPairs_;
        }
    };


}