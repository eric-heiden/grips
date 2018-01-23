#include "DubinsSteering.h"

Trajectory *currentTrajectory;


DubinsSteering::DubinsSteering(double rho) : _rho(rho)
{

}

int updateConfiguration(double q[3], double x, void* user_data)
{
//    printf("%f,%f,%f,%f\n", q[0], q[1], q[2], x);//
    currentTrajectory->addPointEnd(Tpoint(q[0], q[1], q[2]));
    currentTrajectory->addVelocities(1, 1);
    return 0;
}

bool DubinsSteering::Steer(GNode_base *parent_node, GNode_base *successor)
{
    return false;
}

bool DubinsSteering::Steer(const GNode_base *parent_node, const GNode_base *successor,
                           Trajectory *traj)
{
    double q0[] = { parent_node->x_r, parent_node->y_r, parent_node->theta };
    double q1[] = { successor->x_r, successor->y_r, successor->theta };
    DubinsPath path;
    dubins_init(q0, q1, _rho, &path);
    currentTrajectory = traj;
    dubins_path_sample_many(&path, updateConfiguration, 0.1, nullptr);
    return true;
}

void DubinsSteering::initialize()
{
    Steer_base::initialize();
}