#pragma once

#include <vector>
#include <cmath>

/**
 * Simple point in a trajectory.
 */
typedef struct TrajectoryPoint
{
    double x;
    double y;
    double z;

    TrajectoryPoint()
    {
        x = 0.0;
        y = 0.0;
        z = 0.0;
    }

    TrajectoryPoint(double xx, double yy) : x(xx), y(yy)
    { z = 0.0; }

    TrajectoryPoint(double xx, double yy, double zz) : x(xx), y(yy), z(zz)
    {}

    explicit TrajectoryPoint(const double *p)
    {
        x = p[0];
        y = p[1];
        z = p[2];
    }

    TrajectoryPoint &operator=(TrajectoryPoint const &copy) = default;

    double distance(const TrajectoryPoint &p) const
    {
        return std::sqrt(std::pow(p.x-x, 2.)
                         + std::pow(p.y-y, 2.)
                         + std::pow(p.z-z, 2.));
    }

    double distance(double _x, double _y) const
    {
        return std::sqrt(std::pow(_x-x, 2.) + std::pow(_y-y, 2.));
    }

    double distanceSquared(const TrajectoryPoint &p) const
    {
        return std::pow(p.x-x, 2.) + std::pow(p.y-y, 2.) + std::pow(p.z-z, 2.);
    }

    double distanceSquared(double _x, double _y) const
    {
        return std::pow(_x-x, 2.) + std::pow(_y-y, 2.);
    }
} Tpoint;



/**
 * Trajectory defined as a list of points for the robot to follow.
 */
class Trajectory
{
public:
    Trajectory();

    explicit Trajectory(const std::vector<Tpoint> &points)
    : path_(points), drive_path_(points)
    {}

    ~Trajectory()
    { path_.clear(); }


    void addPointEnd(Tpoint p);

    void addVelocities(double v_, double w_);

    unsigned long length()
    { return path_.size(); }

    void reset()
    {
        path_.clear();
        v.clear();
        w.clear();
    }

    std::vector<Tpoint> getPath() const
    { return path_; }

    std::vector<double> getV() const
    { return v; }

    std::vector<double> getW() const
    { return w; }

    std::vector<double> v;
    std::vector<double> w;

private:

    std::vector<Tpoint> path_;
    std::vector<Tpoint> drive_path_;
};
