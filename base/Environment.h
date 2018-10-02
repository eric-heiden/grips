#pragma once

#include <ctime>
#include <iostream>

#include "Trajectory.h"

#define ROS_SUPPORT 0
#define XML_SUPPORT 0

#if ROS_SUPPORT
    #include <ros/ros.h>
#endif

struct Rectangle
{
    double x1{0}, y1{0};
    double x2{0}, y2{0};

    Rectangle() = default;
    Rectangle(double x1, double y1, double x2, double y2)
            : x1(x1), y1(y1), x2(x2), y2(y2)
    {}

    inline double x() const
    {
        return std::min(x1, x2);
    }

    inline double y() const
    {
        return std::min(y1, y2);
    }

    inline double width() const
    {
        return std::abs(x1 - x2);
    }

    inline double height() const
    {
        return std::abs(y1 - y2);
    }
};


class Environment
{
public:
    Environment() = default;
    Environment(const Environment &environment);

    virtual ~Environment();

    static const unsigned int DefaultWidth = 50;
    static const unsigned int DefaultHeight = 50;

    unsigned int seed() const
    {
        return _seed;
    }

    void setStart(Tpoint point)
    {
        _start = point;
    }
    Tpoint start() const
    {
        return _start;
    }

    void setGoal(Tpoint point)
    {
        _goal = point;
    }
    Tpoint goal() const
    {
        return _goal;
    }

    bool empty() const
    {
        return _empty;
    }

    unsigned int width() const
    {
        return _width;
    }

    unsigned int height() const
    {
        return _height;
    }

    inline bool occupied(unsigned int index) const
    {
        return _grid[index];
    }

    inline bool occupied(double x, double y, bool fast = false)
    {
        if (x < 0 || y < 0 || x > _width || y > _height)
            return true;
        if (!fast)
            return bilinearDistance(x, y) <= 0.05;
        return _grid[coord2key(x, y)]
               || _grid[coord2key(x+.15, y)]
               || _grid[coord2key(x, y+.15)] || _grid[coord2key(x+.15, y+.15)]
               || _grid[coord2key(x-.15, y)] || _grid[coord2key(x, y-.15)]
               || _grid[coord2key(x-.15, y-.15)];
    }

    inline bool occupiedCell(unsigned int xi, unsigned int yi) const
    {
        return _grid[yi * _width + xi];
    }

    /**
     * Computes distances field if necessary, and returns the distance
     * to the nearest obstacle.
     */
    inline double distance(double x, double y)
    {
        if (_distances == nullptr)
            computeDistances();
        return _distances[coord2key(x, y)];
    }

    /**
     * Computes distances field if necessary, and returns the distance
     * to the nearest obstacle.
     */
    inline double distance(unsigned int index)
    {
        if (_distances == nullptr)
            computeDistances();
        return _distances[index];
    }

    /**
     * Computes distances field if necessary, and returns the distance
     * to the nearest obstacle.
     */
    inline double distance(unsigned int xi, unsigned int yi)
    {
        if (_distances == nullptr)
            computeDistances();
        return _distances[yi * _width + xi];
    }

    /**
     * Bilinear filtering of occupancy.
     */
    double bilinearOccupancy(double x, double y) const
    {
        auto xi = (unsigned int) x;
        auto yi = (unsigned int) y;
        double u_ratio = x - xi;
        double v_ratio = y - yi;
        double u_opposite = 1. - u_ratio;
        double v_opposite = 1. - v_ratio;
        xi = std::max(std::min(_width, xi), (unsigned int) 0);
        yi = std::max(std::min(_height, yi), (unsigned int) 0); // repeat voxels at edge
        unsigned int xp = std::max(std::min(_width, xi + 1), (unsigned int) 0);
        unsigned int yp = std::max(std::min(_height, yi + 1), (unsigned int) 0);
        double tl = occupiedCell(xi, yi), tr = occupiedCell(xp, yi);
        double bl = occupiedCell(xi, yp), br = occupiedCell(xp, yp);
        return   ((1-tl) * u_opposite + (1-tr) * u_ratio) * v_opposite
               + ((1-bl) * u_opposite + (1-br) * u_ratio) * v_ratio;
    }

    /**
     * Bilinear filtering of distance.
     */
    double bilinearDistance(double x, double y)
    {
        auto xi = (unsigned int) x;
        auto yi = (unsigned int) y;
        double u_ratio = x - xi;
        double v_ratio = y - yi;
        double u_opposite = 1. - u_ratio;
        double v_opposite = 1. - v_ratio;
        xi = std::max(std::min(_width, xi), (unsigned int) 0);
        yi = std::max(std::min(_height, yi), (unsigned int) 0); // repeat voxels at edge
        unsigned int xp = std::max(std::min(_width, xi + 1), (unsigned int) 0);
        unsigned int yp = std::max(std::min(_height, yi + 1), (unsigned int) 0);
        double tl = distance(xi, yi), tr = distance(xp, yi);
        double bl = distance(xi, yp), br = distance(xp, yp);
        return   (tl * u_opposite + tr * u_ratio) * v_opposite
               + (bl * u_opposite + br * u_ratio) * v_ratio;
    }

    double bilinearDistance(const Tpoint &point)
    {
        return bilinearDistance(point.x, point.y);
    }


    /**
     * Computes gradient of distance field at position x,y.
     * @param x Position coordinate x.
     * @param y Position coordinate y.
     * @param dx Resulting gradient coordinate x.
     * @param dy Resulting gradient coordinate y.
     * @param p Sampling precision.
     * @return True, if x and y are within grid boundaries.
     */
    bool distanceGradient(double x, double y, double &dx, double &dy, double p = 0.1)
    {
        if (x < 0 || y < 0 || x > _width || y > _height)
            return false;

        // compute distances left, right, top, bottom
        double dl = bilinearDistance(x-p, y), dr = bilinearDistance(x+p, y);
        double db = bilinearDistance(x, y-p), dt = bilinearDistance(x, y+p);

        dx = (dl - dr) / (p * 2.);
        dy = (dt - db) / (p * 2.);

        return true;
    }

    void print(std::ostream &stream = std::cout) const
    {
        for (unsigned int y = 0; y <= _height; ++y)
        {
            for (unsigned int x = 0; x <= _width; ++x)
            {
                if (std::round(_goal.x) == x && std::round(_goal.y) == y)
                    stream << "G";
                else if (std::round(_start.x) == x && std::round(_start.y) == y)
                    stream << "S";
                else if (occupiedCell(x, y))
                    stream << "#";
                else
                    stream << " ";
            }
            stream << std::endl;
        }
    }

    std::vector<Rectangle> obstacles() const;
    std::vector<Rectangle> obstacles(double x1, double y1, double x2, double y2) const;

    bool collides(double x, double y);
    bool collides(const Trajectory &trajectory);

    static Environment *createRandom(unsigned int width = DefaultWidth,
                                     unsigned int height = DefaultHeight,
                                     double obsRatio = 0.3,
                                     unsigned int seed = (unsigned int) time(nullptr),
                                     int borderSize = 1);
    static Environment *createRandomCorridor(unsigned int width = DefaultWidth,
                                             unsigned int height = DefaultHeight,
                                             int radius = 2,
                                             int branches = 30,
                                             unsigned int seed = (unsigned int) time(nullptr),
                                             int borderSize = 1);
    static Environment *createFromObstacles(const std::vector<Rectangle> &obstacles,
                                            unsigned int width = DefaultWidth,
                                            unsigned int height = DefaultHeight,
                                            int borderSize = 1);
    static Environment *createSimple();

#if XML_SUPPORT
    static Environment *loadFromXml(std::string filename);
#endif
#if ROS_SUPPORT
    void publish(ros::NodeHandle &nodeHandle) const;
#endif

    double obstacleRatio() const;
    double corridorRadius() const;
    std::string generatorType() const;

protected:
    Environment(unsigned int seed, unsigned int width, unsigned int height);

    inline unsigned int coord2key(double x, double y) const
    {
        return (unsigned int)std::max(0., std::floor(y) * _width + std::floor(x));
    }
    void fill(double x, double y, bool value);
    void fill(Rectangle r, bool value);
    void fillBorder(bool value, int size = 1);

    /**
     * Brute-force, quadratic in the number of cells, algorithm
     * to compute the distance field, i.e. distance to the nearest
     * obstacle for every voxel.
     */
    void computeDistances();

private:
    Tpoint _start;
    Tpoint _goal;
    bool *_grid{nullptr};
    double *_distances{nullptr};
    unsigned int _width{0}, _height{0};
//    CollisionChecker *_checker{nullptr};
    bool _empty{true};
    unsigned int _seed{0};
    std::string _type{"undefined"};
    double _corridorRadius{3.};
};
