#include <ompl/util/Console.h>

#include "Environment.h"
#include "PlannerSettings.h"
#include "gui/QtVisualizer.h"

#if XML_SUPPORT
    #include <pugixml/pugixml.hpp>
#endif

#if ROS_SUPPORT
    #include <nav_msgs/GridCells.h>
    #include <visualization_msgs/Marker.h>
    #include <visualization_msgs/MarkerArray.h>
#endif


Environment::Environment(unsigned int seed, unsigned int width, unsigned int height)
: _width(width), _height(height), _empty(false), _seed(seed),
  _distances(nullptr), _corridorRadius(-1), _type("unknown")
{
    _grid = new bool[(width+1) * (height+1)];
    bool *g = _grid;
    for (unsigned int i = 0; i < (width+1) * (height+1); ++i)
        *g++ = false;
}

Environment::Environment(const Environment &environment) : _distances(nullptr)
{
    _grid = new bool[(environment._width+1) * (environment._height+1)];
    bool *g = _grid;
    for (unsigned int i = 0; i < (environment._width+1) * (environment._height+1); ++i)
        *g++ = environment._grid[i];

    _seed = environment._seed;
    _width = environment._width;
    _height = environment._height;
    _empty = environment._empty;
    _corridorRadius = environment._corridorRadius;
    _type = environment._type;
}

Environment::~Environment()
{
    delete [] _grid;
    delete [] _distances;
}

void Environment::fill(double x, double y, bool value)
{
    _grid[coord2key(x, y)] = value;
}

void Environment::fill(Rectangle r, bool value)
{
    for (int x = (int) std::round(std::max(0., std::min(r.x1, r.x2)));
         x <= std::round(std::min((double)_width, std::max(r.x1, r.x2))); ++x)
    {
        for (int y = (int) std::round(std::max(0., std::min(r.y1, r.y2)));
             y <= std::round(std::min((double)_height, std::max(r.y1, r.y2))); ++y)
        {
            _grid[coord2key(x, y)] = value;
        }
    }
}

void Environment::fillBorder(bool value, int size)
{
    if (size < 0)
        return;
    fill(Rectangle(0, 0, _width, size-1), value);
    fill(Rectangle(0, _height-size+1, _width, _height), value);
    fill(Rectangle(0, 0, size-1, _height), value);
    fill(Rectangle(_width-size+1, 0, _width, _height), value);
}

Environment *Environment::createRandomCorridor(unsigned int width, unsigned int height,
                                              int radius, int branches, unsigned int seed, int borderSize)
{
    OMPL_INFORM("Generating environment with seed %i", seed);
    srand(seed);
    auto *environment = new Environment(seed, width, height);
    environment->_type = "corridor";
    environment->_corridorRadius = radius;

    for (unsigned int i = 0; i <= (width+1) * (height+1); ++i)
    {
        environment->_grid[i] = true;
    }

    std::vector<Tpoint> positions({Tpoint(width/2, height/2)});
    for (int k = 0; k < branches; ++k)
    {
        int x = 2 + rand() % (width - 2);
        int y = 2 + rand() % (height - 2);

        // find closest vertex
        double minDistance = std::numeric_limits<double>::max();
        Tpoint closest;
        for (auto &pos : positions)
        {
            double d = pos.distance(x, y);
            if (d < minDistance)
            {
                minDistance = d;
                closest = pos;
            }
        }

        if (std::abs(x - closest.x) < std::abs(y - closest.y))
        {
            // connect vertically
            environment->fill(Rectangle(closest.x - radius,
                                       std::min(closest.y, (double)y) - radius,
                                       closest.x + radius,
                                       std::max(closest.y, (double)y) + radius), false);
            positions.emplace_back(closest.x, y);
        }
        else
        {
            // connect horizontally
            environment->fill(
                    Rectangle(std::min(closest.x, (double)x) - radius,
                              closest.y - radius,
                              std::max(closest.x, (double)x) + radius,
                              closest.y + radius), false);
            positions.emplace_back(x, closest.y);
        }
    }
    environment->fillBorder(true, borderSize);

    environment->computeDistances();

    // find start / goal positions
    double max_dist = 0;
    for (auto &p : positions)
    {
        if (environment->occupied(p.x, p.y))
        {
#ifdef DEBUG
            QtVisualizer::drawNode(p.x, p.y);
#endif
            OMPL_WARN("(%f %f) is occupied.", p.x, p.y);
            continue;
        }
        for (auto &q : positions)
        {
            if (environment->occupied(q.x, q.y))
                continue;
            double dist = p.distance(q);
            if (dist > max_dist)
            {
                max_dist = dist;
                environment->_start = p;
                environment->_goal = q;
            }
        }
    }

    environment->computeDistances();
    environment->print();
    return environment;
}

Environment *Environment::createRandom(unsigned int width, unsigned int height,
                                      double obsRatio, unsigned int seed, int borderSize)
{
    OMPL_INFORM("Generating environment with seed %i", seed);
    srand(seed);
    auto *environment = new Environment(seed, width, height);
    environment->_type = "random";
    // make borders occupied
    environment->fillBorder(true, borderSize);
    double x, y;
    for (int i = 0; i < std::round((width-1.)*(height-1.) * obsRatio); ++i)
    {
        x = rand() * 1. / RAND_MAX * (width-1.);
        y = rand() * 1. / RAND_MAX * (height-1.);
        environment->fill(x, y, true);
//        environment->_checker->add_obstacle(Tobstacle(x, y, 0, 1, 1));
    }

    do
    {
        x = rand() * 1. / RAND_MAX * (width/8.);
        y = rand() * 1. / RAND_MAX * (height/8.);
        environment->_start = Tpoint(x, y);
    }
    while (environment->occupied(x, y));

    do
    {
        x = width*7./8. + rand() * 1. / RAND_MAX * (width/8.);
        y = height*7./8. + rand() * 1. / RAND_MAX * (height/8.);
        environment->_goal = Tpoint(x, y);
    }
    while (environment->occupied(x, y));

    return environment;
}

Environment *Environment::createFromObstacles(const std::vector<Rectangle> &obstacles,
                                             unsigned int width, unsigned int height, int borderSize)
{
    auto *environment = new Environment(0, width, height);
    for (auto &obs : obstacles)
        environment->fill(obs, true);
    environment->fillBorder(true, borderSize);
    environment->_type = "obstacles - " + std::to_string(obstacles.size());
    return environment;
}

#if XML_SUPPORT
Environment *Environment::loadFromXml(std::string filename)
{
    pugi::xml_document doc;
    doc.load_file(filename.c_str());

    double maxx = std::numeric_limits<double>::min();
    double maxy = std::numeric_limits<double>::min();
    std::vector<Rectangle> obstacles;
    for (auto &node : doc.select_nodes("/scenario/obstacle"))
    {
        double x1 = node.node().attribute("x1").as_double();
        double x2 = node.node().attribute("x2").as_double();
        double y1 = node.node().attribute("y1").as_double();
        double y2 = node.node().attribute("y2").as_double();
        maxx = std::max(maxx, std::max(x1, x2));
        maxy = std::max(maxy, std::max(y1, y2));
        obstacles.push_back(Rectangle(x1, y1, x2, y2));
    }

    auto environment = createFromObstacles(obstacles,
                                           (unsigned int) std::round(maxx),
                                           (unsigned int) std::round(maxy));
    environment->_start = Tpoint(3, 3, 0);
    environment->_goal = Tpoint(44.5, 36.50, 0);
    environment->_type = "XML - " + filename;

    return environment;
}
#endif

#if ROS_SUPPORT
void Environment::publish(ros::NodeHandle &nodeHandle) const
{
    ros::Rate loop_rate(5);
    auto pub_obstacles_ = nodeHandle.advertise<nav_msgs::GridCells>("static_obstacles", 0);
    auto pub_obstacle_cells_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("static_obstacle_cells", 0);
    auto pub_start_goal_ = nodeHandle.advertise<visualization_msgs::Marker>("start_goal", 0);
    ros::spinOnce();
    loop_rate.sleep();

    nav_msgs::GridCells obstacles;
    obstacles.header.frame_id = "world";
    obstacles.cell_width = 1;
    obstacles.cell_height = 1;

    visualization_msgs::MarkerArray cells;
    visualization_msgs::Marker removeAllCells;
    removeAllCells.action = 3;
    removeAllCells.id = 0;
    removeAllCells.header.frame_id = "world";
    cells.markers.push_back(removeAllCells);
    for (int x = 0; x <= _width; ++x)
    {
        for (int y = 0; y <= _height; ++y)
        {
            if (!_grid[coord2key(x, y)])
                continue;

            geometry_msgs::Point p;
            p.x = x + 0.5;
            p.y = y + 0.5;
            p.z = 0.0;
            obstacles.cells.push_back(p);

            visualization_msgs::Marker cell;
            cell.action = 0;
            cell.id = _width * (y+1) + x;
            cell.header.frame_id = "world";
            cell.type = visualization_msgs::Marker::CUBE;
            cell.pose.position.x = x + 0.5;
            cell.pose.position.y = y + 0.5;
            cell.pose.position.z = 0;
            cell.scale.x = 1;
            cell.scale.y = 1;
            cell.scale.z = 1;
            cell.color.a = 1;
            cell.color.r = .4;
            cell.color.g = .4;
            cell.color.b = .4;
            cells.markers.push_back(cell);
        }
    }

    visualization_msgs::Marker s, g;
    s.header.frame_id = "world";
    s.id = 0;
    s.type = visualization_msgs::Marker::SPHERE;
    s.color.a = 1;
    s.color.r = 0.10;
    s.color.g = 0.70;
    s.color.b = 0.10;
    s.scale.x = 1;
    s.scale.y = 1;
    s.scale.z = 1;
    s.action = 0;  // add or modify
    s.pose.position.x = _start.x;
    s.pose.position.y = _start.y;
    s.pose.position.z = 0;

    g.header.frame_id = "world";
    g.id = 1;
    g.type = visualization_msgs::Marker::SPHERE;
    g.color.a = 1;
    g.color.r = 0.20;
    g.color.g = 0.40;
    g.color.b = 0.70;
    g.scale.x = 1;
    g.scale.y = 1;
    g.scale.z = 1;
    g.action = 0;  // add or modify
    g.pose.position.x = _goal.x;
    g.pose.position.y = _goal.y;
    g.pose.position.z = 0;

    ROS_INFO("Start: %f %f", _start.x, _start.y);
    ROS_INFO("Goal:  %f %f", _goal.x, _goal.y);

    for (int i = 0; i < 2; ++i)
    {
        pub_obstacles_.publish(obstacles);
        pub_obstacle_cells_.publish(cells);

        pub_start_goal_.publish(s);
        pub_start_goal_.publish(g);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
#endif

bool Environment::collides(const Trajectory &trajectory)
{
    for (auto &p : trajectory.getPath())
    {
        if (occupied(p.x, p.y))
        {
#ifdef DEBUG
            QtVisualizer::drawNode(p.x, p.y, QColor(255, 255, 0, 150), .5);
#endif
            return true;
        }
    }

    return false;
}

bool Environment::collides(double x, double y)
{
    return occupied(x, y);// || _checker->check_collision_state_circle(x, y, 0) == 0;
}

std::vector<Rectangle> Environment::obstacles() const
{
    std::vector<Rectangle> obs;
    for (unsigned int x = 0; x <= _width; ++x)
    {
        for (unsigned int y = 0; y <= _height; ++y)
        {
            if (!occupiedCell(x, y))
                continue;

            obs.emplace_back(x, y, x+1, y+1);
        }
    }
    return obs;
}

std::vector<Rectangle> Environment::obstacles(double x1, double y1, double x2, double y2) const
{
    std::vector<Rectangle> obs;
    for (auto x = (unsigned int) std::round(x1); x <= std::round(x2); ++x)
    {
        for (auto y = (unsigned int) std::round(y1); y <= std::round(y2); ++y)
        {
            if (!occupiedCell(x, y))
                continue;

            obs.emplace_back(x, y, x+1, y+1);
        }
    }
    return obs;
}

void Environment::computeDistances()
{
    _distances = new double[(_width+1) * (_height+1)];
    for (int x = 0; x <= _width; ++x)
    {
        for (int y = 0; y <= _height; ++y)
        {
            double minDistance = std::numeric_limits<double>::max();
            if (_grid[coord2key(x, y)])
                minDistance = 0;

            for (int dx = 0; dx <= _width; ++dx)
            {
                for (int dy = 0; dy <= _height; ++dy)
                {
                    if (occupied((double)dx, (double)dy, true))
                    {
                        double d = std::sqrt(std::pow(dx - x, 2) + std::pow(dy - y, 2));
                        minDistance = std::min(d, minDistance);
                    }
                }
            }
            _distances[coord2key(x, y)] = minDistance;
        }
    }
}

Environment *Environment::createSimple()
{
    auto *environment = new Environment(0, DefaultWidth, DefaultHeight);
    environment->fill(Rectangle(18, 18, 34, 34), true);
    environment->_start = Tpoint(18, 45, 0);
    environment->_goal = Tpoint(45, 18, 0);
    environment->_type = "simple";
    return environment;
}

double Environment::obstacleRatio() const
{
    int occ = 0;
    for (unsigned int x = 0; x <= _width; ++x)
    {
        for (unsigned int y = 0; y <= _height; ++y)
            occ += (int) occupiedCell(x, y);

    }
    return (double) (occ) / (double) ((_width + 1) * (_height + 1));
}

double Environment::corridorRadius() const
{
    return _corridorRadius;
}

std::string Environment::generatorType() const
{
    return _type;
}
