/**
 * @file MotionIntegrationSystem.ixx
 * @brief Integrates local position from local velocity using frame delta time.
 */
module;

export module helios.physics.motion.systems:MotionIntegrationSystem;

import helios.ecs.component;

import helios.physics.motion.components;

import helios.ecs.system.tags;

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
     * @tparam TUpdateContextType Type of the UpdateContext to use.
     */
    template<typename TMemberHandle,
             typename TUpdateContextType = types::SystemUpdateContext>
    requires engine::runtime::concepts::ProvidesUpdateContext<TUpdateContextType, UpdateContext>
    class MotionIntegrationSystem {

    public:

        using Handle_type = TMemberHandle;
        using UpdateContextType = TUpdateContextType;

        using EcsRoleTag = ecs::system::tags::TypedSystemRole;


        /**
         * @brief Applies Euler integration to all active entities with position and velocity.
         *
         * @param updateCtx Frame update context containing view access and delta time.
         */
        bool update(TUpdateContextType& updateCtx) {

            auto& updateContext = updateCtx.updateContext();

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

            return true;
        }


    };


}