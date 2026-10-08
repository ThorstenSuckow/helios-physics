/**
* @file GridCollisionDetectionSystem.ixx
 * @brief Spatial-partitioning based collision detection using a uniform 3D grid.
 */
module;

export module helios.physics.collision.systems:CollisionResponseSystem;

import helios.physics.collision.CollisionDetectionResult;

import helios.physics.collision.components;
import helios.physics.collision.types;
import helios.physics.motion.components;

import helios.engine.core.types;
import helios.engine.runtime;

import helios.ecs.component;
import helios.ecs.entity;
import helios.ecs.common;

import helios.core.log;

import helios.math;

import helios.engine.spatial.components;

#define HELIOS_LOG_SCOPE "helios::physics::systems::CollisionResponseSystem"
export namespace helios::physics::collision::systems {


    template<typename THandle>
    class CollisionResponseSystem {
    public:

        void update(
        const CollisionDetectionResult<THandle>& collisionResult,
            ecs::entity::query::Query<
                THandle,
                ecs::entity::ReadSet<motion::components::Velocity3DComponent<engine::core::types::Local>>,
                ecs::entity::WriteSet<motion::components::Velocity3DComponent<engine::core::types::Local>>
            > query

        ) noexcept {

            const auto& collisions = collisionResult.collisionPairs();

            constexpr float restitution = 1.0f;

            for (const auto& collisionPair : collisions) {

                auto lftResult = query.get(collisionPair.leftHandle);
                auto rgtResult = query.get(collisionPair.rightHandle);

                // staionary collision (most unlikely)
                if (!lftResult && !rgtResult) [[unlikely]] {
                    continue;
                }

                helios::math::vec3f leftVelocity{0.0f, 0.0f, 0.0f};
                helios::math::vec3f rightVelocity{0.0f, 0.0f, 0.0f};

                if (lftResult) {
                    auto [entity, velocity] = *lftResult;
                    leftVelocity = velocity->value();
                }

                if (rgtResult) {
                    auto [entity, velocity] = *rgtResult;
                    rightVelocity = velocity->value();
                }

                // at least one stationary
                const float leftInverseMass  = lftResult ? 1.0f : 0.0f;
                const float rightInverseMass = rgtResult ? 1.0f : 0.0f;

                const auto relativeVelocity =
                    rightVelocity - leftVelocity;

                // overlapNormal: left -> right.
                const float velocityAlongNormal =
                    helios::math::dot(
                        relativeVelocity,
                        collisionPair.overlapNormal
                    );

                // moving away from each other - ignore
                if (velocityAlongNormal >= 0.0f) {
                    continue;
                }

                const float impulseMagnitude =
                    -(1.0f + restitution) *
                    velocityAlongNormal /
                    (leftInverseMass + rightInverseMass);

                const auto impulse = collisionPair.overlapNormal * impulseMagnitude;

                if (lftResult) {
                    auto [entity, velocity] = *lftResult;

                    velocity->setValue(
                        leftVelocity -
                        impulse * leftInverseMass
                    );
                }

                if (rgtResult) {
                    auto [entity, velocity] = *rgtResult;

                    velocity->setValue(
                        rightVelocity +
                        impulse * rightInverseMass
                    );
                }
            }
        }
    };
}