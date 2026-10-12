/**
* @file CollisionResponseSystem.ixx
 * @brief Applies collision response functions sequentially to collision pairs.
 */
module;

#include <utility>

export module helios.physics.collision.systems:CollisionResponseSystem;

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

import helios.physics.collision.commands;

import helios.ecs.command.TypedCommandBuffer;

#define HELIOS_LOG_SCOPE "helios::physics::systems::CollisionResponseSystem"
export namespace helios::physics::collision::systems {


    template<typename THandle, typename TResponseStrategy>
    class CollisionResponseSystem {

        using ResponseStrategy = TResponseStrategy;

        using CollisionCommand = physics::collision::commands::CollisionCommand<THandle>;

        using CommandBuffer = ecs::command::TypedCommandBuffer<CollisionCommand>;

        ResponseStrategy responseStrategy_;

    public:

        explicit CollisionResponseSystem(TResponseStrategy&& responseStrategy)
            : responseStrategy_(std::move(responseStrategy)) {
        }


        void update(
            CommandBuffer& commandBuffer,
            const types::CollisionDetectionResult<THandle>& collisionResult,
            ecs::entity::query::Query<
                THandle,
                ecs::entity::ReadSet<motion::components::Velocity3DComponent<engine::core::types::Local>>,
                ecs::entity::WriteSet<motion::components::Velocity3DComponent<engine::core::types::Local>>
            > query

        ) noexcept {

            const auto& collisions = collisionResult.collisionPairs();

            for (const auto& collisionPair : collisions) {

                auto lftResult = query.get(collisionPair.leftHandle);
                auto rgtResult = query.get(collisionPair.rightHandle);

                // stationary collision (most unlikely)
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

                auto collisionSnapshot = collisionPair.collisionSnapshot;
                collisionSnapshot.leftDynamic = static_cast<bool>(lftResult);
                collisionSnapshot.rightDynamic = static_cast<bool>(rgtResult);

                auto collisionResponse = responseStrategy_.process(
                    collisionSnapshot,
                    leftVelocity,
                    rightVelocity
                );

                if (!collisionResponse) {
                    continue;
                }

                leftVelocity = collisionResponse->leftVelocity;
                rightVelocity = collisionResponse->rightVelocity;

                if (lftResult) {
                    auto [entity, velocity] = *lftResult;

                    velocity->setValue(
                        leftVelocity
                    );
                }

                if (rgtResult) {
                    auto [entity, velocity] = *rgtResult;
                    velocity->setValue(rightVelocity);
                }

                commandBuffer.template add<CollisionCommand>(CollisionCommand{
                    .leftHandle = collisionPair.leftHandle,
                    .rightHandle = collisionPair.rightHandle,
                    .type = commands::CollisionCommandType::Impact,
                    .location = collisionPair.collisionSnapshot.overlapCenter
                });
            }
        }
    };
}