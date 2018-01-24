#include <iostream>
#include <cmath>

#include "Trajectory.h"

Trajectory::Trajectory()
{
    path_.clear();
    drive_path_.clear();
    v.clear();
    w.clear();
}

void Trajectory::addVelocities(double v_, double w_)
{
    v.push_back(v_);
    w.push_back(w_);
}

void Trajectory::addPointEnd(Tpoint p)
{
    path_.push_back(p);
    drive_path_.push_back(p);
}
