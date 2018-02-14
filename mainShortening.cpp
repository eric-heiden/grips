#include "steer_functions/POSQ/POSQSteering.h"
#include "steer_functions/Linear/LinearSteering.h"
#include "steer_functions/ReedsShepp/ReedsSheppSteering.h"

#include "base/Environment.h"
#include "base/gnode.h"

#include "metrics/PathLengthMetric.h"

#include "gui/PathEvaluation.h"
#include "gui/QtVisualizer.h"

#include "planners/AStar.hpp"
#include "planners/ThetaStar.h"
#include "planners/OMPLPlanner.hpp"
#include "planners/OMPLSimplifier.hpp"

#include "PostSmoothing.h"


namespace og = ompl::geometric;

int main(int argc, char **argv)
{
    PathEvaluation::initialize();

    PlannerSettings::steeringType = Steering::STEER_TYPE_REEDS_SHEPP;
    PlannerSettings::CarTurningRadius = 5;
    PlannerSettings::initializeSteering();

    QtVisualizer::initialize();
    QtVisualizer::showStartGoal(false);

    std::vector<Rectangle> obstacles;
    PlannerSettings::environment = Environment::createFromObstacles(obstacles);

    QtVisualizer::visualize(*PlannerSettings::environment, 0);

    std::vector<GNode> path {
            GNode( 5, 25, 0),
            GNode(15, 10, 0),
            GNode(25, 35, 0),
            GNode(35, 10, 0),
            GNode(45, 25, 0)
    };

    QtVisualizer::drawNodes(path, true);
    QtVisualizer::drawPath(PlannerUtils::toSteeredTrajectoryPoints(path), Qt::blue, 4);
    QtVisualizer::addLegendEntry(LegendEntry("Original path", QPen(Qt::blue, 4.)));

    PostSmoothing ourSmoothing;
    std::vector<GNode> ourSmoothed(path);
    ourSmoothing.smooth(ourSmoothed);
    QtVisualizer::drawPath(PlannerUtils::toSteeredTrajectoryPoints(ourSmoothed), Qt::black, 4);
    QtVisualizer::addLegendEntry(LegendEntry("GRIPS", QPen(Qt::black, 4.)));

    OMPLSimplifier simplifier(path);

    QPen bpen(QColor(255, 150, 0), 2., Qt::DashDotLine);
    QtVisualizer::drawPath(simplifier.smoothBSpline().trajectory, bpen);
    QtVisualizer::addLegendEntry(LegendEntry("B-Spline", bpen));

    QPen spen(Qt::magenta, 2., Qt::DotLine);
    QtVisualizer::drawPath(simplifier.shortcutPath().trajectory, spen);
    QtVisualizer::addLegendEntry(LegendEntry("Shortcut", spen));

    QPen dpen(Qt::cyan, 2., Qt::DotLine);
    QtVisualizer::drawPath(simplifier.simplifyMax().trajectory, dpen);
    QtVisualizer::addLegendEntry(LegendEntry("SimplifyMax", dpen));
    QtVisualizer::show();

    return QtVisualizer::exec();
}