#include "starq/mpc/gait.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace starq::mpc
{
    Gait::Gait()
        : duration_(1000),
          control_mode_(GAIT_VELOCITY_CONTROL),
          position_(Vector3::Zero()),
          orientation_(Vector3::Zero()),
          linear_velocity_(Vector3::Zero()),
          angular_velocity_(Vector3::Zero())
    {
    }

    Gait::~Gait()
    {
    }

    bool Gait::load(const std::string &file_path)
    {
        std::ifstream file(file_path);

        if (!file.is_open())
        {
            std::cerr << "Could not open file " << file_path << std::endl;
            return false;
        }

        std::string line;
        std::istringstream iss;

        int stance_count = 0;
        time_t time_last = 0;
        stance_pattern_.clear();
        while (std::getline(file, line))
        {
            iss = std::istringstream(line);

            if (line[0] == '-')
                break;

            time_t time_ms;
            StanceState stance;

            if (!(iss >> time_ms))
            {
                std::cerr << "Error reading line " << line << std::endl;
                return false;
            }

            bool stance_i;
            while (iss >> stance_i)
            {
                if (stance.empty() && stance_i)
                    stance_count += time_ms - time_last;

                stance.push_back(stance_i);
            }

            stance_pattern_[milliseconds(time_ms)] = stance;
            time_last = time_ms;
        }

        stance_ratio_ = static_cast<float>(stance_count) / 1E3f;

        std::string control_mode_str;
        std::getline(file, line);
        iss = std::istringstream(line);
        if (!(iss >> control_mode_str))
        {
            std::cerr << "Error reading control mode " << line << std::endl;
            return false;
        }

        if (control_mode_str == "POSITION")
        {
            control_mode_ = GAIT_POSITION_CONTROL;
        }
        else if (control_mode_str == "VELOCITY")
        {
            control_mode_ = GAIT_VELOCITY_CONTROL;
        }
        else
        {
            std::cerr << "Error reading control mode " << control_mode_str << std::endl;
            return false;
        }

        std::getline(file, line);
        iss = std::istringstream(line);
        if (!(iss >>
                max_linear_velocity_.x() >>
                max_linear_velocity_.y() >>
                max_linear_velocity_.z()))
        {
            std::cerr << "Error reading max linear velocity " << line << std::endl;
            return false;
        }

        std::getline(file, line);
        iss = std::istringstream(line);
        if (!(iss >>
                max_angular_velocity_.x() >>
                max_angular_velocity_.y() >>
                max_angular_velocity_.z()))
        {
            std::cerr << "Error reading max angular velocity " << line << std::endl;
            return false;
        }

        std::getline(file, line);
        iss = std::istringstream(line);
        if (line[0] != '-')
        {
            std::cerr << "Expected a section break here: " << line << std::endl;
            return false;
        }

        std::getline(file, line);
        iss = std::istringstream(line);
        if (!(iss >>
              reference_weights_.linear_velocity.x() >>
              reference_weights_.linear_velocity.y() >>
              reference_weights_.linear_velocity.z()))
        {
            std::cerr << "Error reading velocity weights " << line << std::endl;
            return false;
        }

        std::getline(file, line);
        iss = std::istringstream(line);
        if (!(iss >>
              reference_weights_.angular_velocity.x() >>
              reference_weights_.angular_velocity.y() >>
              reference_weights_.angular_velocity.z()))
        {
            std::cerr << "Error reading angular velocity weights " << line << std::endl;
            return false;
        }

        std::getline(file, line);
        iss = std::istringstream(line);
        if (!(iss >>
              force_weights_.x() >>
              force_weights_.y() >>
              force_weights_.z()))
        {
            std::cerr << "Error reading position force weights " << line << std::endl;
            return false;
        }

        return true;
    }

    void Gait::setFrequency(const double &frequency)
    {
        duration_ = milliseconds(static_cast<int>(1000.0 / frequency));
    }

    void Gait::setControlMode(const GaitControlMode &control_mode)
    {
        control_mode_ = control_mode;
    }

    void Gait::setPose(const Vector3 &position, const Vector3 &orientation)
    {
        position_ = position;
        orientation_ = orientation;
    }

    void Gait::setMaxVelocity(const Vector3 &max_linear_velocity, const Vector3 &max_angular_velocity)
    {
        max_linear_velocity_ = max_linear_velocity;
        max_angular_velocity_ = max_angular_velocity;
    }

    void Gait::setVelocity(const Vector3 &linear_velocity, const Vector3 &angular_velocity)
    {
        linear_velocity_ = linear_velocity;
        angular_velocity_ = angular_velocity;
    }

    void Gait::setWeights(const ReferenceWeights &reference_weights, const ForceWeights &force_weights)
    {
        reference_weights_ = reference_weights;
        force_weights_ = force_weights;
    }

    milliseconds Gait::getDuration() const
    {
        return duration_;
    }

    milliseconds Gait::getStanceDuration() const
    {
        return milliseconds(static_cast<int>(std::round(duration_.count() * stance_ratio_)));
    }

    milliseconds Gait::getSwingDuration() const
    {
        return getDuration() - getStanceDuration();
    }

    GaitControlMode Gait::getControlMode() const
    {
        return control_mode_;
    }

    Vector3 Gait::getPosition() const
    {
        return position_;
    }

    Vector3 Gait::getOrientation() const
    {
        return orientation_;
    }

    Vector3 Gait::getMaxLinearVelocity() const
    {
        return max_linear_velocity_;
    }

    Vector3 Gait::getMaxAngularVelocity() const
    {
        return max_angular_velocity_;
    }

    Vector3 Gait::getLinearVelocity() const
    {
        return linear_velocity_;
    }

    Vector3 Gait::getAngularVelocity() const
    {
        return angular_velocity_;
    }

    StanceState Gait::getStanceState(const milliseconds &time) const
    {
        if (stance_pattern_.empty())
        {
            return StanceState();
        }

        auto time_scaled = milliseconds((GAIT_RESOLUTION * time.count()) / duration_.count());

        auto stance = stance_pattern_.lower_bound(time_scaled);

        if (stance == stance_pattern_.end())
        {
            return StanceState();
        }

        return stance->second;
    }

    ReferenceWeights Gait::getReferenceWeights() const
    {
        return reference_weights_;
    }

    ForceWeights Gait::getForceWeights() const
    {
        return force_weights_;
    }
}