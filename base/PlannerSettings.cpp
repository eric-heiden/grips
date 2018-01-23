#include "PlannerSettings.h"
#include "steer_functions/ReedsShepp/ReedsSheppSteering.h"
#include "steer_functions/POSQ/POSQSteering.h"
#include "steer_functions/Linear/LinearSteering.h"
#include "steer_functions/Dubins/DubinsSteering.h"
#include "steer_functions/G1Clothoid/ClothoidSteering.hpp"

Environment *PlannerSettings::environment = nullptr; //Environment::createRandomCorridor(50, 50, 2, 60); //, 1492977396);
//Environment PlannerSettings::environment = Environment::loadFromXml("/ros/src/idm/thetastar_2PBVP/pedsim_ros/pedsim_simulator/scenarios/maze.xml");

int PlannerSettings::numberEdges = 10;

// steering function settings
Steering::SteeringType PlannerSettings::steeringType = Steering::STEER_TYPE_POSQ;
double PlannerSettings::CarTurningRadius = 3.5;

SteerFunction *PlannerSettings::steering = nullptr;
double PlannerSettings::Kv = 2.95;
double PlannerSettings::Krho = 0.2;
double PlannerSettings::Ktheta = 0;
double PlannerSettings::rhoEndcondition = 0.15;

// grid environment settings
double PlannerSettings::gridWidth = Environment::DefaultWidth;
double PlannerSettings::gridHeight = Environment::DefaultHeight;
// TODO support following grid settings
double PlannerSettings::minx = 0;
double PlannerSettings::miny = 0;
double PlannerSettings::cellWidth = 1;
double PlannerSettings::cellHeight = 1;

// collision checker settings
double PlannerSettings::radiusCollision = Environment::RADIUS_COLLISION;
int PlannerSettings::displayInfoCollision = Environment::DISPLAY_INFO_COLLISION;

bool PlannerSettings::VisualizeSmoothing1 = true;
bool PlannerSettings::VisualizeSmoothing2 = true;
bool PlannerSettings::VisualizeSmoothing3 = true;
bool PlannerSettings::VisualizeSmoothing4 = true;

void PlannerSettings::initializeSteering()
{
    if (steeringType == Steering::STEER_TYPE_REEDS_SHEPP)
        PlannerSettings::steering = new ReedsSheppSteering(PlannerSettings::CarTurningRadius);
    else if (steeringType == Steering::STEER_TYPE_POSQ)
        PlannerSettings::steering = new POSQSteering;
    else if (steeringType == Steering::STEER_TYPE_LINEAR)
        PlannerSettings::steering = new LinearSteering;
    else if (steeringType == Steering::STEER_TYPE_DUBINS)
        PlannerSettings::steering = new DubinsSteering(PlannerSettings::CarTurningRadius);
    else if (steeringType == Steering::STEER_TYPE_CLOTHOID)
        PlannerSettings::steering = new ClothoidSteering;
}
