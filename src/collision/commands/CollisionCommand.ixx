module;


export module helios.physics.collision.commands:CollisionCommand;

import helios.math;

export namespace helios::physics::collision::commands {

    enum class CollisionCommandType {
        Impact
    };

    template<typename THandle>
    struct CollisionCommand {

        THandle leftHandle;
        THandle rightHandle;

        CollisionCommandType type;

        helios::math::vec3f location;

    };

}