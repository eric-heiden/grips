#pragma once

#include <algorithm>
#include <ompl/geometric/PathGeometric.h>

#include "planners/stl_thetastar.h"
#include "base/Trajectory.h"

#include "base/Environment.h"
#include "AbstractPlanner.hpp"


namespace ob = ompl::base;
namespace og = ompl::geometric;


class ThetaStar : public AbstractPlanner, public ob::Planner
{
public:
    ThetaStar();
    virtual ~ThetaStar();

    bool initialize();

    void setProblemDefinition(const ob::ProblemDefinitionPtr &pdef) override;
    ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override;

    ob::PlannerStatus run() override;

    std::vector<GNode> solutionTrajectory() const override;
    std::vector<Tpoint> solutionPath() const override;
    og::PathGeometric geometricPath() const override;

    bool hasReachedGoalExactly() const override;
    double planningTime() const override;

private:

//    Trajectory *trajectory_;
//
//    size_t traj_step_;

//    void updateGridCells();
//
//    void min_cost_flow(std::vector<GNode> &path);
//
//    double standard_deviation(std::vector<double> data);
//
//    double computeJerkMetric(Trajectory *traj, double t);
//    double computeSpeedArcLengthMetric(Trajectory *traj, double t);
//    double computePeaksMetric(Trajectory *traj, double t);
//    double computePathLenghtMetric(Trajectory *traj);
//    double computeCostMetric(Trajectory *traj);

//    double diff_angle_unwrap(double alpha1, double alpha2);

//    double set_angle_to_range(double alpha, double min);

//    bool initializeSteer();

//    static int getNumRand();
//    static int getNumBest();

    Trajectory *curr_traj;

    bool COST_SEARCH;
    bool USE_ASTAR;
    bool USE_GRANDPARENT;

    double _planningTime;

    std::vector<std::vector<GNode> > global_paths;

//    std::vector<double> peaks_v;
//    std::vector<double> cost_v;
//    std::vector<double> jerk_v;
//    std::vector<double> spal_v;
//    std::vector<double> pathlength_v;
//    std::vector<double> timesol_v;

    bool search(std::vector<std::vector<GNode> > &paths, GNode start, GNode goal);

protected:
    explicit ThetaStar(bool astar, std::string name);

    inline ob::Planner *omplPlanner() override
    {
        return this;
    }
};
