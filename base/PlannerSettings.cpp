#include "PlannerSettings.h"
#include "steer_functions/ReedsShepp/ReedsSheppSteering.h"
#include "steer_functions/POSQ/POSQSteering.h"
#include "steer_functions/Linear/LinearSteering.h"
#include "steer_functions/Dubins/DubinsSteering.h"
#ifdef G1_AVAILABLE
    #include "steer_functions/G1Clothoid/ClothoidSteering.hpp"
#endif

Environment *PlannerSettings::environment = nullptr;

int PlannerSettings::numberEdges = 10;

// steering function settings
Steering::SteeringType PlannerSettings::steeringType = Steering::STEER_TYPE_REEDS_SHEPP;
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
#ifdef G1_AVAILABLE
    else if (steeringType == Steering::STEER_TYPE_CLOTHOID)
        PlannerSettings::steering = new ClothoidSteering;
#else
    else if (steeringType == Steering::STEER_TYPE_CLOTHOID)
    {
        OMPL_ERROR("G1 Clothoid steering is not available in this release!");
        OMPL_ERROR("Select a steering type other than STEER_TYPE_CLOTHOID in PlannerSettings.");
    }
#endif
}

// GRIPS settings
double PlannerSettings::gripsMinNodeDistance = 3;
double PlannerSettings::gripsEta = 0.5;
double PlannerSettings::gripsEtaDiscount = 0.8;
unsigned int PlannerSettings::gripsGradientDescentRounds = 5;
unsigned int PlannerSettings::gripsMaxPruningRounds = 100;
