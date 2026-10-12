module;

#include <functional>
#include <span>
#include <utility>
#include <vector>

export module helios.physics.collision:CollisionManager;

import helios.physics.collision.commands;
import helios.ecs.command;
import helios.ecs.entity.EntityManager;

export namespace helios::physics::collision {

    template <typename THandle, typename TTargetHandle>
    class CollisionManager {
    public:
        using CollisionCommand = commands::CollisionCommand<THandle>;
        using CollisionCommands = std::span<const CollisionCommand>;
        using ExecuteLambda = std::function<void(
            ecs::entity::EntityManager<THandle>&, ecs::entity::EntityManager<TTargetHandle>&, CollisionCommands
        )>;

    private:
        ExecuteLambda executeLambda_;
        std::vector<CollisionCommand> pendingCommands_;

    public:
        CollisionManager() = delete;

        explicit CollisionManager(ExecuteLambda executeLambda)
            : executeLambda_(std::move(executeLambda)),
              pendingCommands_{} {
        }

        bool init(ecs::command::CommandHandlerRegistry& commandHandlerRegistry) noexcept {
            commandHandlerRegistry.handleCommands<CollisionCommand>(*this);
            return true;
        }

        void reset() noexcept {
            pendingCommands_.clear();
        }

        bool submit(CollisionCommand&& command) noexcept {
            pendingCommands_.push_back(std::move(command));
            return true;
        }

        bool execute(ecs::entity::EntityManager<THandle>& entityManager, ecs::entity::EntityManager<TTargetHandle>& targetEntityManager) noexcept {
            executeLambda_(entityManager, targetEntityManager, pendingCommands_);
            pendingCommands_.clear();
            return true;
        }
    };

}





