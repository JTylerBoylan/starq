#ifndef STARQ__ROBOT_HPP_
#define STARQ__ROBOT_HPP_

#include "starq/motor_controller.hpp"
#include "starq/leg_dynamics.hpp"
#include "starq/leg_controller.hpp"
#include "starq/leg_command_publisher.hpp"
#include "starq/trajectory_file_reader.hpp"
#include "starq/trajectory_publisher.hpp"
#include "starq/slam/localization.hpp"

namespace starq
{
    class Robot
    {
    public:
        using Ptr = std::shared_ptr<Robot>;

        Robot();

    protected:
        LegDynamics::Ptr dynamics_;
        LegController::Ptr controller_;
        LegCommandPublisher::Ptr publisher_;
        TrajectoryFileReader::Ptr reader_;
        TrajectoryPublisher::Ptr trajectory_publisher_;
        slam::Localization::Ptr localization_;
    };
}

#endif