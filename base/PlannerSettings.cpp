#include "PlannerSettings.h"
#include "steer_functions/ReedsShepp/ReedsSheppSteering.h"
#include "steer_functions/POSQ/POSQSteering.h"
#include "steer_functions/Linear/LinearSteering.h"
#include "steer_functions/Dubins/DubinsSteering.h"
#include "steer_functions/G1Clothoid/ClothoidSteering.hpp"

Environment *PlannerSettings::environment = nullptr;

int PlannerSettings::numberEdges = 10;

// steering function settings
Steering::SteeringType PlannerSettings::steeringType = Steering::STEER_TYPE_POSQ;
double PlannerSettings::CarTurningRadius = 3.5;

SteerFunction *PlannerSettings::steering = nullptr;

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
