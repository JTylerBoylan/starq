#ifndef STARQ__LEG_COMMAND_PUBLISHER_HPP_
#define STARQ__LEG_COMMAND_PUBLISHER_HPP_

#include "starq/leg_controller.hpp"
#include "starq/slam/localization.hpp"
#include "starq/system_localization.hpp"
#include "starq/thread_runner.hpp"

#include <memory>
#include <thread>
#include <mutex>
#include <unordered_map>

namespace starq
{

    /// @brief Leg command structure
    struct LegCommand
    {
        using Ptr = std::shared_ptr<LegCommand>;
        milliseconds delay = milliseconds(0);
        uint8_t leg_id = 0;
        uint32_t control_mode = 0;
        uint32_t input_mode = 0x1;
        Vector3 target_position = Vector3::Zero();
        Vector3 target_velocity = Vector3::Zero();
        Vector3 target_force = Vector3::Zero();

        milliseconds release_time;
        void stamp(const milliseconds &clock)
        {
            release_time = clock + delay;
        }
    };

    using LegList = std::vector<LegController::Ptr>;

    /// @brief Handles leg commands from multiple processes and republishes them at a fixed rate
    class LegCommandPublisher : public ThreadRunner
    {
    public:
        using Ptr = std::shared_ptr<LegCommandPublisher>;

        /// @brief Construct a new Leg Command Publisher object
        /// @param leg_controllers Leg controllers to publish commands to.
        /// @param localization Localization (for clock synchronization)
        LegCommandPublisher(const LegList leg_controllers,
                            const slam::Localization::Ptr localization = std::make_shared<SystemLocalization>());

        /// @brief Destroy the Leg Command Publisher object
        ~LegCommandPublisher();

        /// @brief Get the leg controllers.
        /// @return Leg controllers.
        LegList getLegControllers() const { return leg_controllers_; }

        /// @brief Get the localization.
        /// @return Localization.
        slam::Localization::Ptr getLocalization() const { return localization_; }

        /// @brief Send a leg command to the leg controller.
        /// @param leg_command Leg command to send.
        void sendCommand(LegCommand::Ptr leg_command);

        /// @brief Clear the leg command publisher.
        void clear();

        /// @brief Set the stop on fail flag.
        /// @param stop_on_fail Stop on fail flag.
        void setStopOnFail(const bool stop_on_fail) { stop_on_fail_ = stop_on_fail; }

        /// @brief Set the sleep duration between commands.
        /// @param sleep_duration_us Sleep duration in microseconds.
        void setSleepDuration(const time_t sleep_duration_us) { sleep_duration_us_ = sleep_duration_us; }

    private:
        /// @brief Run the leg command publisher. (threaded)
        void run() override;

        LegList leg_controllers_;
        slam::Localization::Ptr localization_;
        std::unordered_map<uint8_t, LegCommand::Ptr> leg_command_map_;

        bool stop_on_fail_;
        time_t sleep_duration_us_;
    };
}

#endif
