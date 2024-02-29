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
        simulation_ = std::async(std::launch::async, [this]()
                                 { mujoco_->open(scene_file_); });
        
        while (!mujoco_->isOpen())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void MuJoCoRobot::waitForSimulation()
    {
        if (simulation_.valid())
        {
            simulation_.wait();
        }
    }

    bool MuJoCoRobot::isSimulationOpen()
    {
        return mujoco_->isOpen();
    }

    void MuJoCoRobot::setupLocalization()
    {
        localization_ = std::make_shared<MuJoCoLocalization>(mujoco_);
    }

}