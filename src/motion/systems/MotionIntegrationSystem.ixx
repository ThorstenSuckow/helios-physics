/**
 * @file MotionIntegrationSystem.ixx
 * @brief Integrates local position from local velocity using frame delta time.
 */
module;

export module helios.physics.motion.systems:MotionIntegrationSystem;

import helios.ecs.component;

import helios.physics.motion.components;



import helios.engine.runtime.gameloop.types;
import helios.ecs.entity.EntityWorld;


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

    public:

        using Handle_type = TMemberHandle;



        /**
         * @brief Applies Euler integration to all active entities with position and velocity.
         *
         * @param ecsWorld Frame ECS world containing view access.
         * @param updateContext Frame update context containing delta time.
         */
        void update(EntityWorld& ecsWorld, const UpdateContext& updateContext) noexcept {

            for (auto[
                entity,
                localVelocity,
                localPosition
            ]: ecsWorld.view<
                TMemberHandle,
                Velocity3DComponent<TMemberHandle, Local>,
                Position3DComponent<TMemberHandle, Local>
            >().withActive()
            ) {

                entity.setTrackedValue(
                    localPosition,
                    localPosition->value() + localVelocity->value() * updateContext.deltaTime()
                );

            }

        }


    };


}