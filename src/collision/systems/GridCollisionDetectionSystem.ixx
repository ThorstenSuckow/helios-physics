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


export module helios.physics.collision.systems:GridCollisionDetectionSystem;

import helios.physics.collision.CollisionDetectionResult;

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

#define HELIOS_LOG_SCOPE "helios::physics::systems::GridCollisionDetectionSystem"
export namespace helios::physics::collision::systems {

    template<typename THandle>
    class GridCollisionDetectionSystem {

        using CollisionComponent = components::CollisionComponent;
        using WorldBoundsComponent = engine::spatial::components::BoundsComponent<engine::core::types::World>;
        using LocalVelocityComponent = physics::motion::components::Velocity3DComponent<engine::core::types::Local>;
        using CollisionPair = types::CollisionPair<THandle>;

        struct EntityHandlePairHash {

            std::uint64_t operator()(const std::pair<THandle, THandle>& pair) const {

                auto g1 = std::hash<THandle>{}(pair.first);
                auto g2 = std::hash<THandle>{}(pair.second);

                // compute the hash for the pair - shift g2 one position left, then xor with g1.
                return g1 ^ (g2 << 1);
            };

        };

        struct CollisionCandidate {
            THandle entityHandle;
            helios::math::aabbf bounds;
            helios::math::vec3f velocity;
        };


        struct GridCell {
            std::vector<CollisionCandidate> collisionCandidates;
            void clear() {
                collisionCandidates.clear();
            }
        };

        static inline const auto& logger_ = helios::core::log::LogManager::loggerForScope(HELIOS_LOG_SCOPE);


        std::vector<CollisionPair> collisionPairs_;
        std::unordered_set<std::pair<THandle, THandle>, EntityHandlePairHash> solvedCollisions_;

        /**
         * @brief Size of each grid cell in world units.
         */
        float cellSize_;

        /**
         * @brief World-space bounds defining the spatial region covered by the grid.
         */
        helios::math::aabbf gridBounds_;

        /**
         * @brief Flat storage for all grid cells, indexed as x + y * cellsX + z * (cellsX * cellsY).
         */
        std::vector<GridCell> cells_;

        /**
         * @brief Number of cells along the X axis.
         */
        unsigned int cellsX_;

        /**
         * @brief Number of cells along the Y axis.
         */
        unsigned int cellsY_;

        /**
         * @brief Number of cells along the Z axis.
         */
        unsigned int cellsZ_;

        /**
         * @brief Helper to keep track of updated cells (with potential collisions candidates) in one pass.
         */
        std::vector<size_t> trackedCells_;

        /**
         * @brief Initializes the grid based on world bounds and cell size.
         *
         * Computes the number of cells needed in each dimension and allocates
         * the cell storage.
         */
        void initGrid() {

            helios::math::vec3f size = gridBounds_.size();

            cellsX_ = std::max(1u, static_cast<unsigned int>(std::ceil(size[0] / cellSize_)));
            cellsY_ = std::max(1u, static_cast<unsigned int>(std::ceil(size[1] / cellSize_)));
            cellsZ_ = std::max(1u, static_cast<unsigned int>(std::ceil(size[2] / cellSize_)));

            const size_t cellCount = static_cast<size_t>(cellsX_) * cellsY_ * cellsZ_;

            // this would make 100'000'000 * sizeof(GridCell) Bytes
            // if we have 24 Bytes per GridCell, we end up with 2,4 GB alone for this spatial grid.
            if (cellCount > 100'000'000) {
                logger_.warn(std::format("Spatial Grid requested {0} cells", cellCount));
                throw std::runtime_error("Cell count too high.");
            }

            trackedCells_.reserve(cellCount);
            cells_.resize(cellCount);
        }

        /**
         * @brief Prepares the grid for a new collision detection pass.
         *
         * Clears all collision candidates from every cell and resets the set of
         * already-solved collision pairs.
         */
        inline void prepareCollisionDetection() {
            for (const auto idx : trackedCells_) {
                cells_[idx].clear();
            }

            trackedCells_.clear();
            solvedCollisions_.clear();
            collisionPairs_.clear();
        }




    public:



        explicit GridCollisionDetectionSystem(
            const helios::math::aabbf& bounds,
            const float cellSize
        ) : gridBounds_(bounds),
            cellSize_(cellSize) {
            assert(cellSize_ > 0.0f && "cellSize must not be <= 0.0f");
            initGrid();
        }


        CollisionDetectionResult<THandle> update(
            ecs::entity::query::Query<
                THandle,
                ecs::entity::ReadSet<
                    CollisionComponent,
                    WorldBoundsComponent,
                    LocalVelocityComponent
                >
            > query
        ) noexcept {

            prepareCollisionDetection();

            for (auto [entity, cc, wb, vel] : query) {

                updateCollisionCandidate(
                    entity.handle(),
                    worldBoundsToGridBounds(wb->value()),
                    wb, vel
                );
            }

            for (const auto idx : trackedCells_) {

                if (cells_[idx].collisionCandidates.size() < 2) {
                    continue;
                }

                solveCell(cells_[idx]);
            }

            return CollisionDetectionResult<THandle>(
                std::move(collisionPairs_)
            );
        }


        /**
         * @brief Converts world-space AABB bounds to grid cell indices.
         *
         * Transforms a floating-point AABB in world space to integer cell coordinates,
         * clamped to the valid grid range. Used to determine which cells an entity occupies.
         *
         * @param aabbf The world-space AABB to convert.
         *
         * @return An integer AABB representing the range of grid cell indices the entity spans.
         */
        [[nodiscard]] helios::math::aabbi worldBoundsToGridBounds(const helios::math::aabbf& aabbf) const noexcept {

            helios::math::vec3f min = aabbf.min() - gridBounds_.min();
            helios::math::vec3f max = aabbf.max() - gridBounds_.min();

            int xMin = static_cast<int>(std::floor(min[0] / cellSize_));
            int yMin = static_cast<int>(std::floor(min[1] / cellSize_));
            int zMin = static_cast<int>(std::floor(min[2] / cellSize_));

            int xMax = static_cast<int>(std::floor(max[0] / cellSize_));
            int yMax = static_cast<int>(std::floor(max[1] / cellSize_));
            int zMax = static_cast<int>(std::floor(max[2] / cellSize_));

            return helios::math::aabbi{
                std::clamp(xMin, 0, static_cast<int>(cellsX_ - 1)),
                std::clamp(yMin, 0, static_cast<int>(cellsY_ - 1)),
                std::clamp(zMin, 0, static_cast<int>(cellsZ_ - 1)),
                std::clamp(xMax, 0, static_cast<int>(cellsX_ - 1)),
                std::clamp(yMax, 0, static_cast<int>(cellsY_ - 1)),
                std::clamp(zMax, 0, static_cast<int>(cellsZ_ - 1)),
            };
        }


        /**
         * @brief Inserts a collision candidate into all grid cells it overlaps.
         *
         * Iterates through all cells within the given bounds and adds the candidate
         * to each cell's collision candidate list for subsequent narrow-phase testing.
         *
         * @param entityHandle Handle of the collision candidate.
         * @param bounds Grid cell index bounds (integer AABB) the entity spans.
         * @param worldBoundsComponent Pointer to the entity's world-space bounds component.
         * @param collisionComponent Pointer to the entity's collision component.
         * @param velocityComponent Pointer to the entity's local velocity component.
         */
        inline void updateCollisionCandidate(
            THandle entityHandle,
            const helios::math::aabbi& bounds,
            const WorldBoundsComponent* worldBoundsComponent,
            const LocalVelocityComponent* velocityComponent
        ) {
            const auto xMin = bounds.min()[0];
            const auto xMax = bounds.max()[0];
            const auto yMin = bounds.min()[1];
            const auto yMax = bounds.max()[1];
            const auto zMin = bounds.min()[2];
            const auto zMax = bounds.max()[2];

            for (int x = xMin; x <= xMax; x++) {
                for (int y = yMin; y <= yMax; y++) {
                    for (int z = zMin; z <= zMax; z++) {
                        auto& [collisionCandidates] = cell(x, y, z);

                        collisionCandidates.push_back(
                            CollisionCandidate{
                                entityHandle,
                                worldBoundsComponent->value(),
                                velocityComponent->value()
                            }
                        );

                        // only consider the first added to prevent dups
                        if (collisionCandidates.size() == 1) {
                            const auto idx = cellIndex(x, y, z);
                            trackedCells_.push_back(idx);
                        }
                    }
                }
            }
        }

        /**
         * @brief Performs pairwise AABB overlap detection for all candidates in a cell.
         */
        inline void solveCell(GridCell& cell) {

            auto& candidates = cell.collisionCandidates;

            for (size_t i = 0; i < candidates.size(); i++) {

                CollisionCandidate& candidate = candidates[i];
                const helios::math::aabbf& aabbCandidate = candidate.bounds;

                for (size_t j = i+1; j < candidates.size(); j++) {

                    auto& match = candidates[j];


                    const helios::math::aabbf& aabbMatch = match.bounds;
                    if (!aabbCandidate.intersects(aabbMatch)) {
                        continue;
                    }

                    auto lHandle = candidate.entityHandle;
                    auto rHandle = match.entityHandle;

                    if (lHandle > rHandle) {
                        std::swap(lHandle, rHandle);
                    }

                    // if we have already processed a collision, do not add this collision again.
                    if (solvedCollisions_.contains({lHandle, rHandle})) {
                        continue;
                    }

                    solvedCollisions_.insert({lHandle, rHandle});

                    collisionPairs_.push_back(CollisionPair{
                        lHandle,
                        rHandle,
                        candidate.velocity,
                        match.velocity,
                        helios::math::overlapCenter(aabbCandidate, aabbMatch)
                    });
                }
            }
        }

        /**
         * @brief Accesses a grid cell by its 3D coordinates.
         *
         * @param x X-coordinate of the cell (0 to cellsX - 1).
         * @param y Y-coordinate of the cell (0 to cellsY - 1).
         * @param z Z-coordinate of the cell (0 to cellsZ - 1).
         *
         * @return Reference to the GridCell at the specified coordinates.
         */
        [[nodiscard]] inline GridCell& cell(const unsigned int x, const  unsigned int y, const unsigned int z) noexcept {
            return cells_[cellIndex(x, y, z)];
        }

        /**
         * @brief Computes the 1D index of a cell in a 3D grid based on its x, y, and z coordinates.
         *
         * @param x The x-coordinate of the cell (must be less than the grid's x-dimension).
         * @param y The y-coordinate of the cell (must be less than the grid's y-dimension).
         * @param z The z-coordinate of the cell (must be less than the grid's z-dimension).
         *
         * @return The 1D index of the cell in the grid.
         */
        [[nodiscard]] inline constexpr size_t cellIndex(const unsigned int x, const unsigned int y, const unsigned int z) const noexcept {
            assert (x < cellsX_ && y < cellsY_ && z < cellsZ_ && "Invalid range values");

            return x + y * cellsX_ + (z * cellsX_ * cellsY_);
        }

    };


}
