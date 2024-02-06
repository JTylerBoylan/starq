#ifndef STARQ_SLAM__TERRAIN_MAP_HPP_
#define STARQ_SLAM__TERRAIN_MAP_HPP_

#include <memory>
#include "eigen3/Eigen/Dense"

namespace starq::slam
{
    using namespace Eigen;

    class TerrainMap
    {
    public:
        using Ptr = std::shared_ptr<TerrainMap>;

        /// @brief Get the distance to the nearest obstacle.
        /// @param position Position vector [m] (x, y, z) in the world frame.
        /// @return Distance to the nearest obstacle [m]
        virtual double getDistanceToObstacle(const Vector3f &position) = 0;

    private:
    };

}

#endif