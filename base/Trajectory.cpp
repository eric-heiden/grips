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

void Trajectory::addPointBegin(Tpoint p)
{
    // TODO implement?
    //path_.push_front(p);
}

Tpoint Trajectory::getNextPoint(Tpoint p)
{
    auto it = drive_path_.begin();
    Tpoint n = path_[0];

    int erase_point = 0;
    for (; it != drive_path_.end(); it++)
    {
        Tpoint q = (*it);
        double dn = sqrt((p.x - n.x) * (p.x - n.x) + (p.y - n.y) * (p.y - n.y));
        double dq = sqrt((p.x - q.x) * (p.x - q.x) + (p.y - q.y) * (p.y - q.y));

        if (dq < dn)
        {
            n = q;
            erase_point++;
        }
    }

    std::cout << "Erasing: " << erase_point << std::endl;

    //remove it from the store
    drive_path_.erase(drive_path_.begin() + erase_point);

    return n;
}
