#pragma once

#include <base/gnode.h>
#include <unordered_map>

#include <QtCharts>
#include <QtWidgets/QMainWindow>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QBoxPlotSeries>
#include <QtCharts/QBoxSet>
#include <QtGui/QColor>
#include <metrics/CurvatureMetric.h>
#include "planners/OMPLPlanner.hpp"
#include "planners/OMPLAnytimePathShortening.hpp"
#include "PostSmoothing.h"
#include "base/PathStatistics.hpp"
#include "utils/Log.h"


using namespace QtCharts;

class PathStatisticsAggregator
{
public:
    inline void add(const PathStatistics &stats)
    {
        _map[stats.planner].emplace_back(stats);
        _planners.emplace(stats.planner);
        Log::log(stats);
    }

    void showSummary();

private:
    std::unordered_map<std::string, std::vector<PathStatistics> > _map;
    std::unordered_set<std::string> _planners;
};

class PathEvaluation
{
public:
    static void initialize();
    static void show();

    static PathStatistics add(AbstractPlanner *planner, std::string label, QColor color);

    static std::vector<double> computeObstacleDistances(const std::vector<Tpoint> &path);

private:
    static QGridLayout *_layout;
    static QWidget *_window;
    static QChart *_distances;
    static QChart *_boxes;
    static QChart *_pathLengths;
    static QChart *_curvatures;
    static QChart *_times;
    static QBarSeries *_pathLengthSeries;
    static QBarSeries *_curvatureSeries;
    static QBarSeries *_timeSeries;
    static QBoxPlotSeries *_boxSeries;
    static QStringList _labels;

    PathEvaluation() = default;

    static double _maxPathLength;
    static double _maxCurvature;
    static double _maxTime;

    static double findMedian(const std::vector<double> &distances, size_t l, size_t r);

    static QBoxSet *computeBox(std::vector<double> &distances, std::string label);


    static QBarSet *_originalPathLengthSet;
    static QBarSet *_smoothedPathLengthSet;
    static QBarSet *_originalCurvatureSet;
    static QBarSet *_smoothedCurvatureSet;
    static QBarSet *_originalTimeSet;
    static QBarSet *_smoothedTimeSet;

    static QBarSet *_omplSmoothed1PathLengthSet;
    static QBarSet *_omplSmoothed1CurvatureSet;
    static QBarSet *_omplSmoothed1TimeSet;
    static QBarSet *_omplSmoothed2PathLengthSet;
    static QBarSet *_omplSmoothed2CurvatureSet;
    static QBarSet *_omplSmoothed2TimeSet;
    static QBarSet *_omplSmoothed3PathLengthSet;
    static QBarSet *_omplSmoothed3CurvatureSet;
    static QBarSet *_omplSmoothed3TimeSet;
    static QBarSet *_omplSmoothed4PathLengthSet;
    static QBarSet *_omplSmoothed4CurvatureSet;
    static QBarSet *_omplSmoothed4TimeSet;

    static bool _smoothCollides;
};
