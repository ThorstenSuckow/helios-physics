/**
* @file CollisionImpulseResponse.ixx
 * @brief Collision impulse response system.
 */
module;

#include <optional>
export module helios.physics.collision.strategies:CollisionImpulseResponse;

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

#define HELIOS_LOG_SCOPE "helios::physics::strategies::CollisionImpulseResponse"
export namespace helios::physics::collision::strategies {


    class CollisionImpulseResponse {
    public:


        std::optional<types::CollisionResponseResult> process(
            const types::CollisionSnapshot& collisionSnapshot,
            const helios::math::vec3f& leftVelocity,
            const helios::math::vec3f& rightVelocity
        ) const noexcept {

            constexpr float restitution = 1.0f;

            // staionary collision
            if (!collisionSnapshot.leftDynamic && !collisionSnapshot.rightDynamic) [[unlikely]] {
                return types::CollisionResponseResult{leftVelocity, rightVelocity};
            }
            const helios::math::vec3f leftCurrentVelocity = leftVelocity;
            const helios::math::vec3f rightCurrentVelocity = rightVelocity;

            // at least one stationary
            const float leftInverseMass  = collisionSnapshot.leftDynamic ? 1.0f : 0.0f;
            const float rightInverseMass = collisionSnapshot.rightDynamic ? 1.0f : 0.0f;

            const auto relativeVelocity =
                rightCurrentVelocity - leftCurrentVelocity;

            // overlapNormal: left -> right.
            const float velocityAlongNormal =
                helios::math::dot(
                    relativeVelocity,
                    collisionSnapshot.overlapNormal
                );

            // moving away from each other - ignore
            if (velocityAlongNormal >= 0.0f) {
                return std::nullopt;
            }

            const float impulseMagnitude =
                -(1.0f + restitution) *
                velocityAlongNormal /
                (leftInverseMass + rightInverseMass);

            const auto impulse =
                collisionSnapshot.overlapNormal * impulseMagnitude;

            helios::math::vec3f nextLeftVelocity = leftCurrentVelocity;
            helios::math::vec3f nextRightVelocity = rightCurrentVelocity;

            if (collisionSnapshot.leftDynamic) {
                nextLeftVelocity = leftCurrentVelocity - impulse * leftInverseMass;
            }

            if (collisionSnapshot.rightDynamic) {
                nextRightVelocity = rightCurrentVelocity + impulse * rightInverseMass;
            }

            return types::CollisionResponseResult{nextLeftVelocity, nextRightVelocity};
        }
    };
}