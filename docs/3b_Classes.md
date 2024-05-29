### Classes

#### CANSocket

* Read and write frames to a CAN interface
* Source: `~/starq_ws/src/starq/src/lib/can/can_socket.hpp`
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
* Source: `~/starq_ws/src/starq/src/lib/odrive/odrive_socket.hpp`
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
* Source: `~/starq_ws/src/starq/src/lib/odrive/odrive_controller.hpp`
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
* Source: `~/starq_ws/src/starq/src/lib/dynamics/starq_fivebar2d.hpp`
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

bool load(const std::string &file_path);

std::vector<LegCommand::Ptr> getTrajectory();
```

#### TrajectoryPublisher

* Publishes a leg trajectory at a fixed frequency
* Include: `#include "starq/trajectory_publisher.hpp`
* Namespace: `starq`
* Functions:
```
// Constructor
TrajectoryPublisher(LegCommandPublisher::Ptr leg_command_publisher);

bool setTrajectory(const std::vector<LegCommand::Ptr> &trajectory);

bool setFrequency(int frequency);

void start();

void stop();
```

#### MPCController (TODO)

* Formulate plan into a QP problem to solve for leg forces
* QP Solver: OSQP or qpOASIS
* Reference: \
*Di Carlo, J., Wensing, P. M., Katz, B., Bledt, G., & Kim, S. (2018). Dynamic locomotion in the MIT Cheetah 3 through Convex Model-predictive control. 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). https://doi.org/10.1109/iros.2018.8594448*

#### Localization

* Abstract class for localization methods
* Functions:
```
Vector3f getCurrentPosition();

Vector3f getCurrentOrientation();

Vector3f getCurrentVelocity();

Vector3f getCurrentAngularVelocity();
```

#### TerrainMap

* Abstract class for mapping methods
* Functions:
```
double getDistanceToObstacle(const Vector3f &position);
```