/**
 * @file Velocity3DComponent.ixx
 * @brief Velocity component alias for local-space 3D motion.
 */
module;

export module helios.physics.motion.components:Velocity3DComponent;

import helios.engine.core.components.Vec3Component;

export namespace helios::physics::motion::components {

    /**
     * @brief Domain marker for velocity component typing.
     */
    struct VelocityComponentDomain{};

    /**
     * @brief Local-space 3D velocity component alias.
     *
     * @tparam TOwnerHandle ECS handle type.
     * @tparam TArgs Optional component tags.
     */
    template<typename TOwnerHandle, typename ... TArgs>
    using Velocity3DComponent = helios::engine::core::components::Vec3Component<TOwnerHandle, float, VelocityComponentDomain, TArgs...>;


}