#include <stdio.h>

#include "starq/can/can_socket.hpp"
using namespace starq::can;

#include "starq/odrive/odrive_socket.hpp"
#include "starq/odrive/odrive_controller.hpp"
using namespace starq::odrive;

#include "starq/dynamics/starq_fivebar2d.hpp"
using namespace starq::dynamics;

#include "starq/leg_controller.hpp"
using namespace starq;

#define CAN_ID_A 0
#define CAN_ID_B 1

#define GEAR_RATIO_A 6.0
#define GEAR_RATIO_B 6.0

#define LINK_LENGTH_1 0.05
#define LINK_LENGTH_2 0.15

int main() {

  printf("Hello, World!\n");

  // Connect to CAN Interface
  CANSocket::Ptr can_socket = std::make_shared<CANSocket>("can0");
  if (!can_socket->connect())
  {
      printf("Failed to connect to CAN interface.\n");
      return 1;
  }

  // Connect to ODrive controllers
  ODriveSocket::Ptr odrive_socket = std::make_shared<ODriveSocket>(can_socket);

  // Create a ODrive controllers
  ODriveController::Ptr odrive_A = std::make_shared<ODriveController>(odrive_socket, CAN_ID_A);
  ODriveController::Ptr odrive_B = std::make_shared<ODriveController>(odrive_socket, CAN_ID_B);
  std::vector<MotorController::Ptr> motors = {odrive_A, odrive_B};

  // Set gear ratios
  odrive_A->setGearRatio(GEAR_RATIO_A);
  odrive_B->setGearRatio(GEAR_RATIO_B);

  // Create FiveBar2D Dynamics
  STARQ_FiveBar2D::Ptr fivebar = std::make_shared<STARQ_FiveBar2D>(LINK_LENGTH_1, LINK_LENGTH_2);

  // Create Leg Controller
  LegController::Ptr leg_controller = std::make_shared<LegController>(fivebar, motors);

  // Move foot to center
  Eigen::Vector2f foot_center(0.0, -0.15);
  leg_controller->setState(AxisState::CLOSED_LOOP_CONTROL);
  leg_controller->setControlMode(ControlMode::POSITION);
  leg_controller->setFootPosition(foot_center);
  sleep(1); // Wait 1 second for foot to move

  leg_controller->setState(AxisState::IDLE);

  return 0;
} 