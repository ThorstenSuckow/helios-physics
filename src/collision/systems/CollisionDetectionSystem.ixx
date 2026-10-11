/**
 * @file GridCollisionDetectionSystem.ixx
 * @brief Spatial-partitioning based collision detection using a uniform 3D grid.
 */
module;

#include <algorithm>
#include <cassert>
#include <cmath>
#include <format>
#include <helios-physics-config.h>
#include <unordered_set>
#include <vector>
#include <cstddef>


export module helios.physics.collision.systems:CollisionDetectionSystem;

import helios.physics.collision.components;
import helios.physics.collision.types;
import helios.physics.motion.components;

import helios.engine.core.types;

import helios.ecs.component;
import helios.ecs.entity;
import helios.ecs.common;

import helios.core.log;

import helios.math;

import helios.engine.spatial.components;

#define HELIOS_LOG_SCOPE "helios::physics::systems::CollisionDetectionSystem"
export namespace helios::physics::collision::systems {

    template<
        typename THandle,
        typename TStrategy
    >
    class CollisionDetectionSystem {

        using DetectionStrategy = TStrategy;

        static inline const auto& logger_ = helios::core::log::LogManager::loggerForScope(HELIOS_LOG_SCOPE);

        using CollisionComponent = components::CollisionComponent;
        using WorldBoundsComponent = engine::spatial::components::BoundsComponent<engine::core::types::World>;

        DetectionStrategy strategy_;

    public:



        explicit CollisionDetectionSystem(TStrategy&& strategy)
        : strategy_(std::move(strategy)) {
        }


        types::CollisionDetectionResult<THandle> update(
            ecs::entity::query::Query<
                THandle,
                ecs::entity::ReadSet<
                    CollisionComponent,
                    WorldBoundsComponent
                >
            > query
        ) noexcept {

            strategy_.reset();

            for (auto [entity, cc, wb] : query) {
                strategy_.addCollisionCandidate(entity.handle(), wb->value());
            }

            return strategy_.solveCollisions();
        }



    };


}
