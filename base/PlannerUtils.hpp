#pragma once

#include <cmath>

#include "base/Trajectory.h"
#include "base/gnode.h"
#include "gui/QtVisualizer.h"
#include "PlannerSettings.h"


class PlannerUtils
{
public:
    static double slope(double x1, double y1, double x2, double y2)
    {
        double dy = y2 - y1;
        double dx = x2 - x1;
        double x = std::atan2(dy, dx);
        return x;
    }

    static double slope(const Tpoint &a, const Tpoint &b)
    {
        double dy = b.y - a.y;
        double dx = b.x - a.x;
        double x = std::atan2(dy, dx);
        return x;
    }

    static double slope(const GNode &a, const GNode &b)
    {
        double dy = b.y_r - a.y_r;
        double dx = b.x_r - a.x_r;
        double x = std::atan2(dy, dx);
        return x;
    }

    static bool collides(const std::vector<Tpoint> &path)
    {
        for (unsigned int i = 0; i < path.size(); ++i)
        {
            if (GNode_base::isblock(path[i].x, path[i].y))
            {
#ifdef DEBUG
                //QtVisualizer::drawPath(path, QColor(255, 100, 0, 170));
#endif
                return true;
            }

            // check intermediary points
            if (i < path.size()-1)
            {
                double dx = (path[i+1].x - path[i].x);
                double dy = (path[i+1].y - path[i].y);
                double size = std::sqrt(dx*dx + dy*dy);
                const double scale = 1.5;
                dx = dx / size * scale;
                dy = dy / size * scale;

                auto steps = (int)(size / std::sqrt(dx*dx + dy*dy));

                for (int j = 1; j <= steps ; ++j)
                {
                    if (GNode_base::isblock(path[i].x + dx * j, path[i].y + dy * j))
                        //  || GNode_base::isblock(path[i].x + dx * j + .5, path[i].y + dy * j + .5))
                    {
//                        QtVisualizer::drawNode(path[i].x + dx * j, path[i].y + dy * j,
//                                               QColor(255*.8, 255*0, 255*.9), 0.3);
#ifdef DEBUG
                        //QtVisualizer::drawPath(path, QColor(250, 0, 0, 70));
#endif
                        return true;
                    }
                }
            }
        }
#ifdef DEBUG
        //QtVisualizer::drawPath(path, QColor(70, 150, 0, 120));
#endif
        return false;
    }

    static bool collides(const GNode &a, const GNode &b)
    {
        auto *traj = new Trajectory();
        PlannerSettings::steering->Steer(&a, &b, traj);
        auto path = traj->getPath();
        delete traj;
        bool c = collides(path);
        return c;
    }

    static bool collides(const std::vector<Tpoint> &path, std::vector<Tpoint> &collisions)
    {
        collisions.clear();
        for (unsigned int i = 1; i < path.size(); ++i)
        {
            if (GNode_base::isblock(path[i].x, path[i].y))
            {
#ifdef DEBUG
                //QtVisualizer::drawPath(path, QColor(255, 100, 0, 170));
#endif
                collisions.emplace_back(path[i]);
                continue;
            }

            // check intermediary points
            if (i < path.size()-1)
            {
                double dx = (path[i+1].x - path[i].x);
                double dy = (path[i+1].y - path[i].y);
                double size = std::sqrt(dx*dx + dy*dy);
                const double scale = 0.15; //1.5;
                dx = dx / size * scale;
                dy = dy / size * scale;

                auto steps = (int)(size / std::sqrt(dx*dx + dy*dy));

                for (int j = 1; j <= steps ; ++j)
                {
                    if (GNode_base::isblock(path[i].x + dx * j, path[i].y + dy * j))
                        //  || GNode_base::isblock(path[i].x + dx * j + .5, path[i].y + dy * j + .5))
                    {
//                        QtVisualizer::drawNode(path[i].x + dx * j, path[i].y + dy * j,
//                                               QColor(255*.8, 255*0, 255*.9), 0.3);
#ifdef DEBUG
                        //QtVisualizer::drawPath(path, QColor(250, 0, 0, 70));
#endif
                        collisions.emplace_back(path[j]);
                        continue;
                    }
                }
            }
        }
#ifdef DEBUG
        //QtVisualizer::drawPath(path, QColor(150, 200, 0, 70));
#endif
        return !collisions.empty();
    }

    static bool collides(const GNode &a, const GNode &b, std::vector<Tpoint> &collisions)
    {
        auto *traj = new Trajectory();
        PlannerSettings::steering->Steer(&a, &b, traj);
        auto path = traj->getPath();
        delete traj;
        return collides(path, collisions);
    }

    static void updateAngles(std::vector<GNode> &path, bool AverageAngles = true,
                             bool preventCollisions = true)
    {
        if (path.size() < 2)
            return;

        double theta_old = path[0].theta;
        path[0].theta = slope(path[0], path[1]);
        if (preventCollisions && collides(path[0], path[1]))
            path[0].theta = theta_old; // revert setting
        for (int i = 1; i < path.size() - 1; ++i)
        {
            theta_old = path[i].theta;
            if (AverageAngles)
            {
                double l = slope(path[i - 1], path[i]);
                double r = slope(path[i], path[i + 1]);
                if (std::abs(l - r) >= M_PI)
                {
                    if (l > r)
                        l += 2. * M_PI;
                    else
                        r += 2. * M_PI;
                }
                path[i].theta = (l + r) * 0.5;
            }
            else
                path[i].theta = slope(path[i - 1], path[i]);

            if (preventCollisions && (collides(path[i-1], path[i]) || collides(path[i], path[i+1])))
                path[i].theta = theta_old; // revert setting
        }
        theta_old = path[path.size() - 1].theta;
        path[path.size() - 1].theta = slope(path[path.size() - 2], path[path.size() - 1]);
        if (preventCollisions && collides(path[path.size() - 1], path[path.size() - 2]))
            path[path.size() - 1].theta = theta_old; // revert setting
    }

    static std::vector<Tpoint> linearInterpolate(const Tpoint &a, const Tpoint &b, double dt = 0.1)
    {
        std::vector<Tpoint> points;
        double dx = (b.x - a.x);
        double dy = (b.y - a.y);
        double size = std::sqrt(dx*dx + dy*dy);
        if (size == 0)
            return std::vector<Tpoint>{a};
        dx = dx / size * dt;
        dy = dy / size * dt;
        auto steps = (int)(size / std::sqrt(dx*dx + dy*dy));

        for (int j = 1; j <= steps ; ++j)
            points.emplace_back(a.x + dx * j, a.y + dy * j);

        if (points.empty())
            points.emplace_back(a);

        return points;
    }

    static std::vector<Tpoint> linearInterpolate(const GNode &a, const GNode &b, double dt = 0.1)
    {
        return linearInterpolate(Tpoint(a.x_r, a.y_r), Tpoint(b.x_r, b.y_r), dt);
    }

    static std::vector<Tpoint> toTrajectoryPoints(const std::vector<GNode> &path)
    {
        std::vector<Tpoint> points;
        for (auto &gnode : path)
            points.emplace_back(Tpoint(gnode.x_r, gnode.y_r));
        return points;
    }

    static std::vector<Tpoint> toSteeredTrajectoryPoints(const std::vector<GNode> &path)
    {
        std::vector<Tpoint> points;
        for (unsigned int i = 0; i < path.size()-1; ++i)
        {
            points.emplace_back(path[i].x_r, path[i].y_r);
            auto *traj = new Trajectory();
            PlannerSettings::steering->Steer(&path[i], &path[i + 1], traj);
            auto tpath = traj->getPath();
            delete traj;
            points.insert(points.end(), tpath.begin(), tpath.end());
            points.emplace_back(path[i+1].x_r, path[i+1].y_r);
        }
        return points;
    }

    static std::vector<Tpoint> toSteeredTrajectoryPoints(const GNode &a, const GNode &b)
    {
        auto *traj = new Trajectory();
        PlannerSettings::steering->Steer(&a, &b, traj);
        auto tpath = traj->getPath();
        delete traj;
        return tpath;
    }

    static Trajectory toSteeredTrajectory(const std::vector<GNode> &path)
    {
        return Trajectory(toSteeredTrajectoryPoints(path));
    }

    static Tpoint centroid(const std::vector<Tpoint> &points)
    {
        double x = 0, y = 0;
        for (auto &p : points)
        {
            x += p.x;
            y += p.y;
        }
        return Tpoint(x/points.size(), y/points.size());
    }

    static GNode closestPoint(Tpoint x, const std::vector<Tpoint> &points)
    {
        if (points.size() == 1)
            return GNode(points[0].x, points[0].y);
        unsigned int closest = 0;
        double dist = points[closest].distanceSquared(x);
        for (unsigned int i = 1; i < points.size()-1; ++i)
        {
            if (PlannerSettings::environment->occupied(points[i].x, points[i].y))
                continue;
            double d = points[i].distanceSquared(x);
            if (d < dist)
            {
                dist = d;
                closest = i;
            }
        }
        double theta;
        if (closest == 0)
            theta = slope(points[0], points[1]);
        else
            theta = slope(points[closest-1], points[closest+1]);
        return GNode(points[closest].x, points[closest].y, theta);
    }
};
