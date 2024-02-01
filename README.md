# STARQ Documentation
*Updated 01/29/2023*

## NVIDIA Jetson AGX Orin 64GB Developer Kit

### NVIDIA Store
https://store.nvidia.com/en-us/jetson/store/

### Developer Guide
https://docs.nvidia.com/jetson/archives/r35.1/DeveloperGuide/index.html

### Login
```
username: nvidia
password: nvidia
```

## Motor Controllers

### ODrive Shop
https://odriverobotics.com/shopfolder

### ODrive Documentation
https://docs.odriverobotics.com/v/latest/guides/getting-started.html

### ODrive Pro Data Sheet
https://docs.odriverobotics.com/v/latest/hardware/pro-datasheet.html

### MJ5208 Moteus Motor Shop
https://mjbots.com/products/mj5208

### NTCLE300E3502SB Thermistor Shop
https://www.mouser.com/ProductDetail/Vishay-BC-Components/NTCLE300E3502SB?qs=%2FWiulJ9oly5IYkswf0Y9eA%3D%3D

### Thermistor Data Sheet
Data Sheet: https://www.vishay.com/doc?29051

### MJ5208 ODrive Auto-Configuration Script
1. Plug in USB from the Jetson to the ODrive controller 
2. Open the Terminal on the the Jetson
3. Go to the `docs` folder: $`cd ~/starq-lib/docs`
4. Run the auto-configuration script: $`python3 configure_odrive.py <CAN_ID>`
5. Let the script complete before unplugging

*Note: The thermistor needs to be plugged in before running this script*

## ODrive CAN Communication

### ODrive CAN Guide
https://docs.odriverobotics.com/v/latest/guides/can-guide.html \
Instructions on how to set up a CAN network.

### ODrive CAN Protocol
https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#can-protocol \
CAN frame IDs and required data for the different ODrive commands.

### ODrive Pinout
https://docs.odriverobotics.com/v/latest/hardware/pro-datasheet.html#pro-pinout \
Locations of the CAN, thermistor, and other IO pins.

## Jetson AGX Orin CAN

### Waveshare SN65HVD230 CAN Board
https://www.amazon.com/SN65HVD230-CAN-Board-Communication-Development/dp/B00KM6XMXO \
Converts CAN frames from GPIO to High/Low signals on the CAN bus.

### Jetson GPIO Pinout
https://jetsonhacks.com/nvidia-jetson-agx-orin-gpio-header-pinout/ \
Useful for determining where the 5V, 3.3V, GND, and CAN pins are on the GPIO array.

### Jetson CAN Documentation
https://docs.nvidia.com/jetson/archives/r35.4.1/DeveloperGuide/text/HR/ControllerAreaNetworkCan.html \
Instructions on how to enable CAN on the Jetson.

### Jetson CAN GPIO Pins

| CAN0 RX | CAN0 TX | CAN1 RX | CAN1 TX | 3.3V |     GND     |
| ------- | ------- | ------- | ------- | ---- | ----------- |
|   29    |   31    |    37   |    33   | 1/17 | 25/30/34/39 |

### Jetson CAN Setup
1. Open the Terminal on the Jetson
2. Go to the `docs` folder: $`cd ~/starq-lib/docs`
3. Run the command: $`sudo ./loadcan_jetson.sh` \
*Note: This needs to be run every time the Jetson is booted.*

## STARQ C++ Library

### Installation
1. Open the Terminal
2. Go to home folder: $`cd ~`
3. Run the command: `git clone https://github.com/JTylerBoylan/starq-lib`
4. Create a build directory: $`cd starq-lib && mkdir build`
5. Build the project: $`cd build && cmake .. && make`
6. Test executables will be generated in the build folder

### Creating an executable
1. Open `~/starq-lib/CMakeLists.txt`
2. Add the lines:
```
add_executable(my_executable examples/my_executable.cpp)
target_include_directories(my_executable PUBLIC include)
target_link_libraries(my_executable PUBLIC stdc++ stdc++fs m starq pthread)
```
- *Replace `my_executable` with your executable name*
3. Create the file: `~/starq-lib/examples/my_executable.cpp`
4. Add the lines:
```
#include <stdio.h>

int main()
{
    printf("Hello World!\n");

    return 0;
}
```

### Compiling and Running an executable

1. Open the Terminal
2. Go to `~/starq-lib/build`
3. Run the command: $`cmake .. && make`
4. Run the executable: $`./my_executable`

### Classes

#### CANSocket

* Read and write frames to a CAN interface
* Source: `~/starq-lib/src/can/can_socket.hpp`
* Include: `#include "starq/can/can_socket.hpp"`
* Namespace `starq::can`
* Functions:
```
// Constructor
CANSocket(const std::string &interface);

bool connect();

bool send(const uint8_t can_id, const uint8_t *data, const uint8_t size);

ssize_t receive(struct can_frame &frame);
```
*Note: The functions returning a boolean indicate if it was successful or not*

#### ODriveSocket

* Convert ODrive commands to CAN frames
* Source: `~/starq-lib/src/odrive/odrive_socket.hpp`
* Include: `#include "starq/odrive/odrive_socket.hpp`
* Namespace: `starq::odrive`
* Constructor:
```
ODriveSocket(const starq::can::CANSocket::Ptr socket);
```

*Note: ODriveSocket contains all the same functions as ODriveController, but requires the CAN ID.*

#### MotorController

* Abstract class for motor controllers. Used as a template for derived classes.
* Include: `#include "starq/motor_controller.hpp`
* Namespace: `starq`
* Functions:
```
bool setGearRatio(const float gear_ratio);

bool setState(const uint32_t state);

bool setControlMode(const uint32_t control_mode, const uint32_t input_mode = 0x1);

bool setPosition(const float pos, const float vel_ff = 0, const float torque_ff = 0);

bool setVelocity(const float vel, const float torque_ff = 0);

bool setTorque(const float torque);

float getPositionEstimate();

float getVelocityEstimate();

float getTorqueEstimate();
```

#### ODriveController

* Implementation of MotorController for ODrive controllers
* Source: `~/starq-lib/src/odrive/odrive_controller.hpp`
* Include: `#include "starq/odrive/odrive_controller.hpp`
* Namespace: `starq::odrive`
* Functions:
  * Same as MotorController, but includes:
```
// Constructor
ODriveController(const ODriveSocket::Ptr socket, const uint8_t can_id);

bool setPosGain(const float pos_gain);

bool setVelGains(const float vel_gain, const float integrator_gain);

bool setLimits(const float velocity_limit, const float current_limit);

uint8_t getCANID() const;

uint32_t getAxisError();

uint8_t getAxisState();

float getIqSetpoint();

float getIqMeasured();

float getFETTemperature();

float getMotorTemperature();

float getBusVoltage();

float getBusCurrent();

bool clearErrors();

void printInfo();

std::string getErrorName();
```

#### LegDynamics

* Abstract class for leg dynamics
* Include: `#include "starq/leg_dynamics.hpp`
* Namespace: `starq`
* Functions:
```
bool getForwardKinematics(const VectorXf &joint_angles, VectorXf &foot_position);

bool getInverseKinematics(const VectorXf &foot_position, VectorXf &joint_angles);

getJacobian(const VectorXf &joint_angles, MatrixXf &jacobian);
```

#### STARQ_FiveBar2D

* Implementation of LegDynamics for the 2D symmetric five-bar leg
* Source: `~/starq-lib/src/dynamics/starq_fivebar2d.hpp`
* Include: `#include "starq/dynamics/starq_fivebar2d.hpp`
* Namespace: `starq::dynamics`
* Functions:
    * Same as LegDynamics
```
// Constructor
STARQ_FiveBar2D(float L1, float L2);
```

#### LegController

* Uses LegDynamics to convert leg commands into motor commands
* Include: `#include "starq/leg_controller.hpp`
* Namespace: `starq`
* Functions:
```
// Constructor
LegController(const starq::LegDynamics::Ptr dynamics, const std::vector<MotorController::Ptr> motor_controllers);

bool setState(const uint32_t state);

bool setControlMode(const uint32_t control_mode, const uint32_t input_mode = 0x1);

bool setFootPosition(const VectorXf &foot_position, const VectorXf &foot_velocity_ff, const VectorXf &foot_torque_ff);

bool setFootVelocity(const VectorXf &foot_velocity, const VectorXf &foot_torque_ff);

bool setFootForce(const VectorXf &foot_force);

bool getFootPositionEstimate(VectorXf &foot_position);

bool getFootVelocityEstimate(VectorXf &foot_velocity);

bool getFootForceEstimate(VectorXf &foot_force);
```

#### LegCommandPublisher

* Handles leg commands from multiple processes and republishes them at a fixed rate
* Include: `#include "starq/leg_command_publisher.hpp`
* Namespace: `starq`
* Functions:
```
// Constructor
LegCommandPublisher(const std::vector<LegController::Ptr> leg_controllers);

void sendCommand(const LegCommand &leg_command);

void clear();

void setStopOnFail(const bool stop_on_fail);

void setSleepDuration(const time_t sleep_duration_us);
```
* Advantages:
  * Ability to use ODrive's timeout function to shut off the motors if no messages are recieved within some time limit.
  * Leg force/velocity commands are constantly recomputed with the latest Jacobian, so they will stay in the correct direction.
  * Multiple processes can send commands to the leg controller without interfering with one another (race conditions)

* LegCommandPublisher also holds the definition of a LegCommand:
```
struct LegCommand
{
    uint8_t leg_id = 0;
    uint32_t control_mode = 0;
    uint32_t input_mode = 0x1;
    VectorXf target_position = VectorXf();
    VectorXf target_velocity = VectorXf();
    VectorXf target_force = VectorXf();
};
```
#### TrajectoryFileReader

* Reads leg trajectories from text files
* Include: `#include "starq/trajectory_file_reader.hpp`
* Namespace: `starq`
* Functions:
```
// Constructor
TrajectoryFileReader();

bool load2D(const std::string &file_path);

bool load3D(const std::string &file_path);

std::vector<LegCommand> getTrajectory();
```

#### TrajectoryPublisher

* Publishes a leg trajectory at a fixed frequency
* Include: `#include "starq/trajectory_publisher.hpp`
* Namespace: `starq`
* Functions:
```
// Constructor
TrajectoryPublisher(LegCommandPublisher::Ptr leg_command_publisher);

bool setTrajectory(const std::vector<LegCommand> &trajectory);

bool setFrequency(int frequency);

void start();

void stop();
```

#### GaitController (TODO)

* Abstract class for gait control
* Functions:
```
void start();

void stop();

void setVelocity(const Vector3f &linear_speed, const Vector3f &angular_speed);
```

#### WalkingGaitController (TODO)

* Implementation of GaitController for walking

#### WalkingGaitPlanner (TODO)

* Plan body of mass trajectory and foot positions

#### BodyControlMPC (TODO)

* Formulate plan into a QP problem to solve for leg forces
* QP Solver: OSQP or qpOASIS
* Reference: \
*Di Carlo, J., Wensing, P. M., Katz, B., Bledt, G., & Kim, S. (2018). Dynamic locomotion in the MIT Cheetah 3 through Convex Model-predictive control. 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). https://doi.org/10.1109/iros.2018.8594448*

#### SwimmingGaitController (TODO)

* Implementation of GaitController for swimming
* Includes FF-SLIP

#### GaitPlanner (TODO)

* Find the optimal trajectory and gait sequence to a user-defined goal.
* Planner: SBMPO

#### Localization (TODO)

* Abstract class for localization methods
* Functions:
```
Vector3f getCurrentPosition();

Vector3f getCurrentOrientation();

Vector3f getCurrentVelocity();

Vector3f getCurrentAngularVelocity();
```

#### TerrainMap (TODO)

* Abstract class for mapping methods
* Functions:
```
double getDistanceToObstacle(const Vector3f &position);
```

### ROS Interface (TODO)
* Library written as a ROS node so it can interface with other ROS packages
* ROS Packages:
  * cuVSLAM (NVIDIA's SLAM)
  * joy (joystick control)
  * rviz (visualization tools)
  * MATLAB ROS Toolbox

### Example Code
```
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
```

### STARQ Test Executables
Executables located in the `build` folder:
* `test_can_connection`: Prints 10 CAN frames to the console.
* `test_odrive_control`: Moves ODrive motors to a few set positions.
* `test_clear_errors`: Clears errors for all connected ODrives.
* `test_fivebar2d_position_control`:  Moves end effector position in a circle of a set radius.
* `test_fivebar2d_force_control`: Applies a constant end effector force in a set direction.
