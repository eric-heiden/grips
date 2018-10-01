#pragma once

#include <QtCore/Qt>
#include <QtGui/QColor>

#include <utility>
#include <vector>
#include <cmath>

struct PathStatistics
{
    double planningTime{0};
    double ourSmoothingTime{-1};
    double omplSmoothing1Time{-1};
    double omplSmoothing2Time{-1};
    double omplSmoothing3Time{-1};
    double omplSmoothing4Time{-1};
    bool pathFound{false};
    bool omplSmoothing4Found{false};
    bool pathCollides{true};
    bool ourSmoothingCollides{true};
    bool omplSmoothing1Collides{true};
    bool omplSmoothing2Collides{true};
    bool omplSmoothing3Collides{true};
    bool omplSmoothing4Collides{true};
    bool exactGoalPath{true};
    bool exactGoalSmoothedPath{true};
    double pathLength{0};
    double ourSmoothingPathLength{-1};
    double omplSmoothing1PathLength{-1};
    double omplSmoothing2PathLength{-1};
    double omplSmoothing3PathLength{-1};
    double omplSmoothing4PathLength{-1};
    double curvature{0};
    double ourSmoothingCurvature{-1};
    double omplSmoothing1Curvature{-1};
    double omplSmoothing2Curvature{-1};
    double omplSmoothing3Curvature{-1};
    double omplSmoothing4Curvature{-1};
    std::string planner;
    QColor color;

    explicit PathStatistics(
                std::string planner = "UNKNOWN",
                QColor color = Qt::white)
            : planner(std::move(planner)), color(std::move(color))
    {}
};

namespace stat
{
    inline double median(std::vector<double> values)
    {
        std::sort(values.begin(), values.end());
        size_t count = values.size();
        if (count % 2 == 1)
            return values[count/2];

        double right = values[count/2];
        double left = values[count/2 - 1];
        return (right + left) / 2.0;
    }

    inline double mean(const std::vector<double> &values)
    {
        double avg = 0;
        for (double v : values)
            avg += v;
        return avg / values.size();
    }

    inline double min(const std::vector<double> &values)
    {
        double m = std::numeric_limits<double>::max();
        for (double v : values)
            m = std::min(m, v);
        return m;
    }

    inline double max(const std::vector<double> &values)
    {
        double m = std::numeric_limits<double>::min();
        for (double v : values)
            m = std::max(m, v);
        return m;
    }

    inline double std(const std::vector<double> &values)
    {
        double s = 0;
        double m = mean(values);
        for (double v : values)
            s += std::pow(v - m, 2.);
        return std::sqrt(1./values.size() * s);
    }
}
