#include <stdio.h>

#include "starq/mujoco/mujoco.hpp"

using namespace starq::mujoco;

//*************************************
void set_torque_control(const mjModel *m, int actuator_no, int flag)
{

    if (flag == 0)
        m->actuator_gainprm[10 * actuator_no + 0] = 0;
    else
        m->actuator_gainprm[10 * actuator_no + 0] = 1;
}
//***********************************

//*************************************
void set_velocity_servo(const mjModel *m, int actuator_no, double kv)
{
    m->actuator_gainprm[10 * actuator_no + 0] = kv;
    m->actuator_biasprm[10 * actuator_no + 2] = -kv;
}
//***********************************

//*************************************
void set_position_servo(const mjModel *m, int actuator_no, double kp)
{
    m->actuator_gainprm[10 * actuator_no + 0] = kp;
    m->actuator_biasprm[10 * actuator_no + 1] = -kp;
}
//***********************************

void computeControlInput(const mjModel *m, mjData *d)
{
    int i;
    int actuator_no;
    // 0 = torque actuator
    actuator_no = 0;
    int flag = 0;
    set_torque_control(m, actuator_no, flag);
    d->ctrl[0] = -10 * (d->qpos[0] - 0) - 1 * d->qvel[0]; // PD control
    // d->ctrl[0] = -10*(d->sensordata[0]-0)-1*d->sensordata[1];

    // 1=position servo
    actuator_no = 1;
    double kp = 10;
    set_position_servo(m, actuator_no, kp);
    // for (i=0;i<10;i++)
    //  {
    //    //printf("%f \n", m->actuator_gainprm[10*actuator_no+i]);
    //    //printf("%f \n", m->actuator_biasprm[10*actuator_no+i]);
    //  }

    // printf("*********** \n");
    d->ctrl[1] = 0.5;

    // 2= velocity servo
    actuator_no = 2;
    double kv = 1;
    set_velocity_servo(m, actuator_no, kv);
    d->ctrl[2] = 0.2;

    // // PD control
    // actuator_no = 1;
    // double kp2 = 10;
    // set_position_servo(m, actuator_no, kp2);
    // actuator_no = 2;
    // double kv2 = 1;
    // set_velocity_servo(m, actuator_no, kv2);
    // d->ctrl[1] = -0.5;
    // d->ctrl[2] = 0;
}

int main(void)
{

    MuJoCo::Ptr mujoco = MuJoCo::getInstance();

    mujoco->addMotorControlFunction(computeControlInput);

    // mujoco->open("/home/nvidia/starq_ws/src/models/unitree_a1/scene.xml");
    mujoco->open("/home/nvidia/starq_ws/src/models/pendulum2.xml");

    return 0;
}