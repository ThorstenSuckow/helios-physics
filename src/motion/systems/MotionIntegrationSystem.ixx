/**
 * @file MotionIntegrationSystem.ixx
 * @brief Integrates local position from local velocity using frame delta time.
 */
module;

#include <iostream>
#include <ostream>


export module helios.physics.motion.systems:MotionIntegrationSystem;

import helios.ecs.component;

import helios.physics.motion.components;



import helios.engine.runtime.gameloop.types;
import helios.ecs.entity.EntityWorld;
import helios.ecs.entity.EntityAccessSet;
import helios.ecs.entity.Query;


import helios.engine.core.types;

import helios.engine.spatial.components;

using namespace helios::ecs::components;
using namespace helios::engine::core::types;
using namespace helios::physics::motion::components;
using namespace helios::engine::spatial::components;


export namespace helios::physics::motion::systems {

    /**
     * @brief Updates local position by applying local velocity each frame.
     *
     * @tparam TMemberHandle ECS handle type.
     */
    template<typename TMemberHandle>
    class MotionIntegrationSystem {

        using EntityWorld = ecs::entity::EntityWorld;
        using UpdateContext = helios::engine::runtime::gameloop::types::UpdateContext;

        template<typename TRead, typename TWrite, typename TFilter = ecs::entity::Filter<ecs::entity::AnyDirty<>>>
        using Query = ecs::entity::Query<TRead, TWrite, TFilter>;

        template<typename ... TReads>
        using Read = ecs::entity::ReadSet<TReads...>;

        template<typename ... TWrites>
        using Write = ecs::entity::WriteSet<TWrites...>;

    public:

        using HandleType = TMemberHandle;



        /**
         * @brief Applies Euler integration to all active entities with position and velocity.
         *
         * @param query Frame-local query over local velocity and local position.
         * @param updateContext Frame update context containing delta time.
         */
        void update(
            Query<
                Read<Velocity3DComponent<TMemberHandle, Local>,
                    Position3DComponent<TMemberHandle, Local>
                >,
                Write<Position3DComponent<TMemberHandle, Local>>,
                ecs::entity::Filter<ecs::entity::IsActive>
            > query,
            const UpdateContext& updateContext
        ) noexcept {

            for (auto[
                entity,
                localVelocity,
                localPosition
            ]: query
            ) {

                entity.setTrackedValue(
                    localPosition,
                    localPosition->value() + localVelocity->value() * updateContext.deltaTime()
                );
            }

        }


    };


}