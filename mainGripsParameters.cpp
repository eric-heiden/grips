#include <planners/ThetaStar.h>
#include "base/PlannerSettings.h"

#include "steer_functions/POSQ/POSQSteering.h"

#include "metrics/PathLengthMetric.h"

#include "planners/OMPLPlanner.hpp"

#include "gui/PathEvaluation.h"

#include "PostSmoothing.h"


namespace og = ompl::geometric;

int main(int argc, char **argv)
{
    PlannerSettings::steeringType = Steering::STEER_TYPE_POSQ;
    PlannerSettings::initializeSteering();

    QtVisualizer::initialize();
    QtVisualizer::showStartGoal(false);

    PlannerSettings::environment = Environment::createRandomCorridor(50, 50, 3, 30, 123);

    QtVisualizer::visualize(*PlannerSettings::environment, 0);

    auto *planner = new ThetaStar;
    if (!planner->run())
        return EXIT_FAILURE;

    std::vector<GNode> path = planner->solutionTrajectory();
    std::vector<Tpoint> trajectory = planner->solutionPath();

    QtVisualizer::drawPath(PlannerUtils::toSteeredTrajectoryPoints(path), Qt::black, 4);
    QtVisualizer::addLegendEntry(LegendEntry("Original path", QPen(Qt::black, 4.)));

    QColor qc("#36F");
    qc.setAlpha(100);
    QtVisualizer::addLegendEntry(LegendEntry("GRIPS", QPen(qc, 4.)));

    ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_NONE);

    Stopwatch stopwatch;

    for (auto eta : {0.25, 0.5, 0.75})
    {
        for (auto etaDiscount : {0.5, 0.8, 0.95})
        {
            for (auto gdRounds : {3u, 5u, 10u})
            {
                PlannerSettings::gripsEta = eta;
                PlannerSettings::gripsEtaDiscount = etaDiscount;
                PlannerSettings::gripsGradientDescentRounds = gdRounds;

                std::cout << "$ " << eta
                          << " $ & $ " << etaDiscount
                          << " $ & " << gdRounds << " & ";

                PostSmoothing ourSmoothing;
                std::vector<GNode> ourSmoothed(path);
                stopwatch.start();
                ourSmoothing.smooth(ourSmoothed);
                double time = stopwatch.stop();
                auto steered = PlannerUtils::toSteeredTrajectoryPoints(ourSmoothed);
                QtVisualizer::drawPath(steered, qc, 4);

                //std::cout << PlannerUtils::collides(steered) << " & $";

                auto *smoothedTraj = new Trajectory(steered);
                double pathLength = PathLengthMetric::evaluate(smoothedTraj);
                std::cout << std::setprecision(4) << pathLength << " & $";
                double curvature = CurvatureMetric::evaluateMetric(smoothedTraj, 0, false);
                std::cout << std::setprecision(3) << curvature << "$ & $";

                std::cout << ourSmoothed.size() << "$ & ";

                auto smoothedDistances = PathEvaluation::PathEvaluation::computeObstacleDistances(steered);
                double mean = 0;
                for (auto d : smoothedDistances)
                    mean += d;
                mean /= smoothedDistances.size();
                double stddev = 0;
                for (auto d : smoothedDistances)
                    stddev += std::pow(d - mean, 2.);
                stddev = std::sqrt(stddev / (smoothedDistances.size() - 1));
                std::cout << " \\SI{" << mean << " \\pm " << stddev;
                std::cout << "}{} & ";
                std::cout << (int)(time * 1000) << " \\\\" << std::endl;
            }
        }
    }

    QtVisualizer::show();

    return QtVisualizer::exec();
}