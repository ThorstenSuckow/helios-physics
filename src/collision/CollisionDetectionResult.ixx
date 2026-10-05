module;

#include <vector>
#include <span>
export module helios.physics.collision.CollisionDetectionResult;

import  helios.physics.collision.types;


export namespace helios::physics::collision {

    template<typename THandle>
    class CollisionDetectionResult {


        using CollisionPair = types::CollisionPair<THandle>;

        std::vector<CollisionPair> collisionPairs_;

    public:

        explicit CollisionDetectionResult(std::vector<CollisionPair>&& pairs)
        : collisionPairs_(std::move(pairs)){};


        [[nodiscard]] std::span<const CollisionPair> collisionPairs() const {
            return collisionPairs_;
        }
    };


}