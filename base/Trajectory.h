#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include <cmath>

/// \brief simple point in a trajectory
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

    TrajectoryPoint(double *p)
    {
        x = p[0];
        y = p[1];
        z = p[2];
    }

    TrajectoryPoint &operator=(TrajectoryPoint const &copy)
    {
        x = copy.x;
        y = copy.y;
        z = copy.z;
        return *this;
    }

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


/// \class Trajectory
/// \brief A list of point for the robot agent to follow
class Trajectory
{
public:
    Trajectory();
    Trajectory(const std::vector<Tpoint> &points)
    : path_(points), drive_path_(points)
    {}

    ~Trajectory()
    { path_.clear(); }


    void addPointEnd(Tpoint p);

    void addVelocities(double v_, double w_);

    void addPointBegin(Tpoint p);

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

    Tpoint getNextPoint(Tpoint p);

private:

    std::vector<Tpoint> path_;
    std::vector<Tpoint> drive_path_;
};

#endif // TRAJECTORY_H