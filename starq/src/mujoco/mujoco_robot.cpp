#include "starq/mujoco/mujoco_robot.hpp"

#include <iostream>

namespace starq::mujoco
{

    MuJoCoRobot::MuJoCoRobot()
        : mujoco_(MuJoCo::getInstance())
    {
    }

    void MuJoCoRobot::setSceneFile(const std::string &scene_file)
    {
        scene_file_ = scene_file;
    }

    void MuJoCoRobot::startSimulation()
    {
        mujoco_->load(scene_file_);
        mujoco_->start();

        while (!mujoco_->isOpen())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    void MuJoCoRobot::waitForSimulation()
    {
        mujoco_->wait();
    }

    bool MuJoCoRobot::isSimulationOpen()
    {
        return mujoco_->isRunning();
    }

    void MuJoCoRobot::setupLocalization()
    {
        localization_ = std::make_shared<MuJoCoLocalization>(mujoco_);
    }

    void MuJoCoRobot::setupLegCommandPublisher()
    {
        publisher_ = std::make_shared<LegCommandPublisher>(legs_, localization_);
    }

}