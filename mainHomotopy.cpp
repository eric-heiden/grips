#include "gui/QtVisualizer.h"
#include "gui/PathEvaluation.h"
#include "base/PlannerSettings.h"
#include "planners/AStar.hpp"


namespace og = ompl::geometric;

int main(int argc, char **argv)
{
    PathEvaluation::initialize();

    PlannerSettings::steeringType = Steering::STEER_TYPE_REEDS_SHEPP;
    PlannerSettings::CarTurningRadius = 3.5;
    PlannerSettings::initializeSteering();

    PlannerSettings::VisualizeSmoothing1 = false;
    PlannerSettings::VisualizeSmoothing2 = false;
    PlannerSettings::VisualizeSmoothing3 = false;
    PlannerSettings::VisualizeSmoothing4 = false;

    PathStatisticsAggregator statsAggregator{};

    QtVisualizer::initialize();

    PlannerSettings::environment = Environment::createRandom(
            Environment::DefaultWidth, Environment::DefaultHeight, 0.1, 1234);
    PlannerSettings::environment->setStart(Tpoint(2, 12));
    PlannerSettings::environment->setGoal(Tpoint(Environment::DefaultWidth-1, Environment::DefaultHeight-12));

    QtVisualizer::visualize(*PlannerSettings::environment, 0);

    statsAggregator.add(PathEvaluation::add(new ThetaStar, "Theta*", Qt::black));
    statsAggregator.add(PathEvaluation::add(new AStar, "A*", Qt::gray));
    QtVisualizer::show();

    statsAggregator.showSummary();

    return QtVisualizer::exec();
}
