#ifndef STARQ_MPC__GAIT_HPP_
#define STARQ_MPC__GAIT_HPP_

#include <memory>
#include <vector>
#include <string>
#include <map>

#include "starq/mpc/mpc_types.hpp"

#define GAIT_RESOLUTION 1000

namespace starq::mpc
{

    enum GaitControlMode
    {
        GAIT_POSITION_CONTROL = 0,
        GAIT_VELOCITY_CONTROL = 1
    };

    /// @brief Gait class
    class Gait
    {
    public:
        using Ptr = std::shared_ptr<Gait>;

        /// @brief Create a new Gait object
        Gait();

        /// @brief Destroy the Gait object
        ~Gait();

        /// @brief Load the gait pattern from a file
        /// @param file_path The file to load the gait pattern from
        /// @return True if the gait pattern was loaded, false otherwise
        bool load(const std::string &file_path);

        /// @brief Set the frequency of the gait pattern
        /// @param frequency The frequency of the gait pattern
        void setFrequency(const double &frequency);

        /// @brief Set the control mode of the gait pattern
        /// @param control_mode The control mode of the gait pattern
        void setControlMode(const GaitControlMode &control_mode);

        /// @brief Set the pose of the gait pattern
        /// @param position The position of the gait pattern
        /// @param orientation The orientation of the gait pattern
        void setPose(const Vector3f &position, const Vector3f &orientation);

        /// @brief Set the max velocity of the gait pattern for position control
        /// @param max_linear_velocity The max linear velocity of the gait pattern
        /// @param max_angular_velocity The max angular velocity of the gait pattern
        void setMaxVelocity(const Vector3f &max_linear_velocity, const Vector3f &max_angular_velocity);

        /// @brief Set the velocity of the gait pattern
        /// @param linear_velocity The linear velocity of the gait pattern
        void setVelocity(const Vector3f &linear_velocity, const Vector3f &angular_velocity);

        /// @brief Set the weights of the gait pattern
        /// @param reference_weights The reference weights of the gait pattern
        /// @param force_weights The force weights of the gait pattern
        void setWeights(const ReferenceWeights &reference_weights, const ForceWeights &force_weights);

        /// @brief Get the duration of the gait pattern
        /// @return The duration of the gait pattern
        milliseconds getDuration() const;

        /// @brief Get the stance duration of the gait pattern
        /// @return The stance duration of the gait pattern
        milliseconds getStanceDuration() const;

        /// @brief Get the swing duration of the gait pattern
        /// @return The swing duration of the gait pattern
        milliseconds getSwingDuration() const;

        /// @brief Get the control mode of the gait pattern
        /// @return The control mode of the gait pattern
        GaitControlMode getControlMode() const;

        /// @brief Get the reference position
        /// @return The reference position
        Vector3f getPosition() const;

        /// @brief Get the reference orientation
        /// @return The reference orientation
        Vector3f getOrientation() const;

        /// @brief Get the max linear velocity of the gait pattern
        /// @return The max linear velocity of the gait pattern
        Vector3f getMaxLinearVelocity() const;

        /// @brief Get the max angular velocity of the gait pattern
        /// @return The max angular velocity of the gait pattern
        Vector3f getMaxAngularVelocity() const;

        /// @brief Get the linear velocity of the gait pattern
        /// @return The linear velocity of the gait pattern
        Vector3f getLinearVelocity() const;

        /// @brief Get the angular velocity of the gait pattern
        /// @return The angular velocity of the gait pattern
        Vector3f getAngularVelocity() const;

        /// @brief Get the stance state of the gait pattern at a given time
        /// @param time The time to get the stance state at in milliseconds between 0 and the duration
        /// @return The stance state of the gait pattern at the given time
        StanceState getStanceState(const milliseconds &time) const;

        /// @brief Get the reference weights of the gait
        /// @return The reference weights of the gait
        ReferenceWeights getReferenceWeights() const;

        /// @brief Get the force weights of the gait
        /// @return The force weights of the gait
        ForceWeights getForceWeights() const;

    private:
        milliseconds duration_;
        double stance_ratio_;

        GaitControlMode control_mode_;
        Vector3f position_;
        Vector3f orientation_;
        Vector3f max_linear_velocity_;
        Vector3f max_angular_velocity_;
        Vector3f linear_velocity_;
        Vector3f angular_velocity_;
        std::map<milliseconds, StanceState> stance_pattern_;
        ReferenceWeights reference_weights_;
        ForceWeights force_weights_;
    };

    using GaitSequence = std::vector<Gait::Ptr>;
}

#endif