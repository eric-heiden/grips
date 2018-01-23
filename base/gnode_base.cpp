#include "gnode_base.h"

#include "base/PlannerSettings.h"

bool GNode_base::isblock(double x, double y)
{
    return PlannerSettings::environment->collides(x, y);
}

bool GNode_base::line(double x0, double y0, double y1, double x1)
{
    double dx = (x1 - x0);
    double dy = (y1 - y0);
    double size = std::sqrt(dx*dx + dy*dy);
    const double scale = 0.01;
    dx = dx / size * scale;
    dy = dy / size * scale;

    auto steps = (int)(size / std::sqrt(dx*dx + dy*dy));

    for (int j = 0; j <= steps ; ++j)
    {
        if (GNode_base::isblock(x0 + dx * j, y0 + dy * j))
        {
//            QtVisualizer::drawPath(std::vector<Tpoint>({Tpoint(x0, y0), Tpoint(x1, y1)}), Qt::red);
//            QtVisualizer::drawNode(x0 + dx * j, y0 + dy * j, Qt::red);
            return false;
        }
    }

//    QtVisualizer::drawPath(std::vector<Tpoint>({Tpoint(x0, y0), Tpoint(x1, y1)}), Qt::darkGreen);

    return true;
//    double unit;
//    unit = .5;
//    bool res = true;
//    double dt = 0.1;
//    // parent_node->theta=atan2((y0-y1)/dt,(x0-x1)/dt);
//
//    double dx, dy, f, sy, sx;
//
//    dy = (y1 - y0);
//    dx = (x1 - x0);
//    f = 0;
//
//    if (dy < 0)
//    {
//        dy = -dy;
//        sy = -unit;
//    }
//    else
//    {
//        sy = unit;
//    }
//
//    if (dx < 0)
//    {
//        dx = -dx;
//        sx = -unit;
//    }
//    else
//    {
//        sx = unit;
//    }
//
//
//    if (dx >= dy)
//    {
//        while (x0 != x1)
//        {
//            f = f + dy;
//
//            if (f >= dx)
//            {
//                if (isblock(x0 + ((sx - unit) / 2), y0 + ((sy - unit) / 2)))
//                {
//                    return false;
//                }
//                y0 = y0 + sy;
//                f = f - dx;
//            }
//
//
//            if (f != 0 && isblock(x0 + ((sx - unit) / 2), y0 + ((sy - unit) / 2)))
//            {
//                return false;
//            }
//
//            if (dy == 0 && isblock(x0 + ((sx - unit) / 2), y0) && isblock(x0 + ((sx - unit) / 2), y0 - unit))
//            {
//                return false;
//            }
//
//            x0 = x0 + sx;
//        }
//    }
//    else
//    {
//        while (y0 != y1)
//        {
//            f = f + dx;
//
//            if (f >= dy)
//            {
//                if (isblock(x0 + ((sx - unit) / 2), y0 + ((sy - unit) / 2)))
//                {
//                    return false;
//                }
//                x0 = x0 + sx;
//                f = f - dy;
//            }
//
//            if (f != 0 && isblock(x0 + ((sx - unit) / 2), y0 + ((sy - unit) / 2)))
//            {
//                return false;
//            }
//
//            if (dx == 0 && isblock(x0, y0 + ((sy - unit) / 2)) && isblock(x0 - unit, y0 + ((sy - unit) / 2)))
//            {
//                return false;
//            }
//
//            y0 = y0 + sy;
//        }
//    }
//
//    return res;
}

bool GNode_base::line(const GNode_base *successor, const GNode_base *parent_node)
{
    double x0, y0, y1, x1;
    x0 = parent_node->x;
    y0 = parent_node->y;
    x1 = successor->x;
    y1 = successor->y;
    return line(x0, y0, y1, x1);
}
