#define DEBUG // TODO activate DEBUG in PlannerSettings.h

#include "base/PlannerSettings.h"

#include "steer_functions/POSQ/POSQSteering.h"

#include "metrics/PathLengthMetric.h"

#include "planners/OMPLPlanner.hpp"
#include "planners/ThetaStar.h"

#include "gui/PathEvaluation.h"

#include "PostSmoothing.h"
#include "planners/OMPLSimplifier.hpp"


namespace og = ompl::geometric;

int main(int argc, char **argv)
{
    PlannerSettings::steeringType = Steering::STEER_TYPE_REEDS_SHEPP;
    PlannerSettings::initializeSteering();

    QtVisualizer::initialize();
    QtVisualizer::showStartGoal(false);

    PlannerSettings::environment = Environment::createRandomCorridor(50, 50, 3, 30, 123);

    QtVisualizer::visualize(*PlannerSettings::environment, 0);


    auto *planner = new RRTPlanner;
    if (!planner->run())
        return EXIT_FAILURE;

    std::vector<GNode> path = planner->solutionTrajectory();
    std::vector<Tpoint> trajectory = planner->solutionPath();

    QtVisualizer::drawPath(PlannerUtils::toSteeredTrajectoryPoints(path), Qt::black, 4);
    QtVisualizer::addLegendEntry(LegendEntry("Original path", QPen(Qt::black, 4.)));

    PostSmoothing ourSmoothing;
    std::vector<GNode> ourSmoothed(path);
    ourSmoothing.smooth(ourSmoothed);
    QColor qc("#36F");
    QtVisualizer::drawPath(PlannerUtils::toSteeredTrajectoryPoints(ourSmoothed), qc, 4);
    QtVisualizer::addLegendEntry(LegendEntry("GRIPS", QPen(qc, 4.)));

    OMPLSimplifier simplifier(path);

    QPen bpen(QColor("#F70"), 3., Qt::DashDotLine);
    QtVisualizer::drawPath(simplifier.smoothBSpline().trajectory, bpen);
    QtVisualizer::addLegendEntry(LegendEntry("B-Spline", bpen));

    QPen spen(QColor("#BF5"), 3., Qt::DotLine);
    QtVisualizer::drawPath(simplifier.shortcutPath().trajectory, spen);
    QtVisualizer::addLegendEntry(LegendEntry("Shortcut", spen));

    QPen dpen(Qt::cyan, 3., Qt::DotLine);
    QtVisualizer::drawPath(simplifier.simplifyMax().trajectory, dpen);
    QtVisualizer::addLegendEntry(LegendEntry("SimplifyMax", dpen));

    QPen apen(Qt::darkRed, 3., Qt::DotLine);
    QtVisualizer::drawPath(planner->anytimePathShortening().trajectory, apen);
    QtVisualizer::addLegendEntry(LegendEntry("AnytimePS", apen));

    QtVisualizer::show();

    return QtVisualizer::exec();
}