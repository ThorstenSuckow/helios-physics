/**
 * @file MotionIntegrationSystem.ixx
 * @brief Integrates local position from local velocity using frame delta time.
 */
module;

export module helios.physics.motion.systems:MotionIntegrationSystem;

import helios.ecs.component;

import helios.physics.motion.components;



import helios.engine.runtime.world.UpdateContext;
import helios.engine.runtime.world.types;
import helios.engine.runtime.concepts;

import helios.engine.core.types;

import helios.engine.spatial.components;

using namespace helios::ecs::components;
using namespace helios::engine::core::types;
using namespace helios::physics::motion::components;
using namespace helios::engine::spatial::components;
using namespace helios::engine::runtime::world;

export namespace helios::physics::motion::systems {

    /**
     * @brief Updates local position by applying local velocity each frame.
     *
     * @tparam TMemberHandle ECS handle type.
     */
    template<typename TMemberHandle>
    class MotionIntegrationSystem {

    public:

        using Handle_type = TMemberHandle;



        /**
         * @brief Applies Euler integration to all active entities with position and velocity.
         *
         * @param updateContext Frame update context containing view access and delta time.
         */
        void update(UpdateContext& updateContext) noexcept {

            for (auto[
                entity,
                localVelocity,
                localPosition
            ]: updateContext.template view<
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