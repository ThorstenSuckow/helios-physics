/**
 * @file MotionIntegrationSystem.ixx
 * @brief Integrates local position from local velocity using frame delta time.
 */
module;

export module helios.physics.motion.systems:MotionIntegrationSystem;

import helios.ecs.components.Active;

import helios.physics.motion.components;

import helios.engine.runtime.world.tags.SystemRole;

import helios.engine.runtime.world.UpdateContext;

import helios.engine.core.types.ComponentTypeTags;

import helios.engine.spatial.components;

using namespace helios::ecs::components;
using namespace helios::engine::core::types;
using namespace helios::physics::motion::components;
using namespace helios::engine::spatial::components;
using namespace helios::engine::runtime::world;
using namespace helios::engine::runtime::world::tags;
export namespace helios::physics::motion::systems {

    /**
     * @brief Updates local position by applying local velocity each frame.
     *
     * @tparam TMemberHandle ECS handle type.
     */
    template<typename TMemberHandle>
    class MotionIntegrationSystem {

    public:


        using EngineRoleTag = TypedSystemRole;


        /**
         * @brief Applies Euler integration to all active entities with position and velocity.
         *
         * @param updateContext Frame update context containing view access and delta time.
         */
        void update(UpdateContext& updateContext) {

            for (auto[
                entity,
                localVelocity,
                localPosition
            ]: updateContext.view<
                TMemberHandle,
                Velocity3DComponent<TMemberHandle, Local>,
                Position3DComponent<TMemberHandle, Local>
            >().withActive()
               .whereAllEnabled()
            ) {

                localPosition->setValue(
                    localPosition->value() + localVelocity->value() * updateContext.deltaTime()
                );
                entity.template markDirty<Position3DComponent<TMemberHandle, Local>>();

            }

        }


    };


}