#include <QtWidgets/QGridLayout>

#include "base/PlannerUtils.hpp"
#include "PathEvaluation.h"
#include "Table.hpp"

QGridLayout *PathEvaluation::_layout = nullptr;
QWidget *PathEvaluation::_window = nullptr;
QChart *PathEvaluation::_distances = nullptr;
QStringList PathEvaluation::_labels = QStringList();
QChart *PathEvaluation::_pathLengths = nullptr;
QBarSeries *PathEvaluation::_pathLengthSeries = nullptr;
QChart *PathEvaluation::_curvatures = nullptr;
QBarSeries *PathEvaluation::_curvatureSeries = nullptr;
QChart *PathEvaluation::_times = nullptr;
QBarSeries *PathEvaluation::_timeSeries = nullptr;
QChart *PathEvaluation::_boxes = nullptr;
QBoxPlotSeries *PathEvaluation::_boxSeries = nullptr;

QBarSet *PathEvaluation::_originalPathLengthSet = nullptr;
QBarSet *PathEvaluation::_smoothedPathLengthSet = nullptr;
QBarSet *PathEvaluation::_originalCurvatureSet = nullptr;
QBarSet *PathEvaluation::_smoothedCurvatureSet = nullptr;
QBarSet *PathEvaluation::_originalTimeSet = nullptr;
QBarSet *PathEvaluation::_smoothedTimeSet = nullptr;

QBarSet *PathEvaluation::_omplSmoothed1PathLengthSet = nullptr;
QBarSet *PathEvaluation::_omplSmoothed1CurvatureSet = nullptr;
QBarSet *PathEvaluation::_omplSmoothed1TimeSet = nullptr;
QBarSet *PathEvaluation::_omplSmoothed2PathLengthSet = nullptr;
QBarSet *PathEvaluation::_omplSmoothed2CurvatureSet = nullptr;
QBarSet *PathEvaluation::_omplSmoothed2TimeSet = nullptr;
QBarSet *PathEvaluation::_omplSmoothed3PathLengthSet = nullptr;
QBarSet *PathEvaluation::_omplSmoothed3CurvatureSet = nullptr;
QBarSet *PathEvaluation::_omplSmoothed3TimeSet = nullptr;
QBarSet *PathEvaluation::_omplSmoothed4PathLengthSet = nullptr;
QBarSet *PathEvaluation::_omplSmoothed4CurvatureSet = nullptr;
QBarSet *PathEvaluation::_omplSmoothed4TimeSet = nullptr;

double PathEvaluation::_maxPathLength = 0;
double PathEvaluation::_maxCurvature = 0;
double PathEvaluation::_maxTime = 0;

bool PathEvaluation::_smoothCollides = false;

double PathEvaluation::findMedian(const std::vector<double> &distances, size_t l, size_t r)
{
    size_t count = r - l;
    if (count % 2 == 1)
        return distances[count/2 + l];

    double right = distances[count/2 + l];
    double left = distances[count/2 - 1 + l];
    return (right + left) / 2.0;
}

std::vector<double> PathEvaluation::computeObstacleDistances(const std::vector<Tpoint> &path)
{
    std::vector<double> distances;
    for (unsigned int i = 0; i < path.size()-1; ++i)
    {
        for (auto &point : PlannerUtils::linearInterpolate(path[i], path[i+1]))
        {
            double distance = PlannerSettings::environment->bilinearDistance(point);
            distances.push_back(distance);
        }
    }
    return distances;
}

QBoxSet *PathEvaluation::computeBox(std::vector<double> &distances, std::string label)
{
    std::sort(distances.begin(), distances.end());
    QBoxSet *box = new QBoxSet(QString::fromStdString(label));
    if (distances.empty())
        return box;
    box->setValue(QBoxSet::LowerExtreme, distances.front());
    box->setValue(QBoxSet::UpperExtreme, distances.back());
    box->setValue(QBoxSet::Median, findMedian(distances, 0, distances.size()));
    box->setValue(QBoxSet::LowerQuartile, findMedian(distances, 0, distances.size() / 2));
    box->setValue(QBoxSet::UpperQuartile,
                  findMedian(distances,
                             distances.size() / 2 + (distances.size() % 2),
                             distances.size()));
    return box;
}

void PathEvaluation::initialize()
{
    _maxPathLength = 0;
    _maxCurvature = 0;
    _maxTime = 0;
    _smoothCollides = false;

    _layout = new QGridLayout;
    _window = new QWidget;
    _window->setLayout(_layout);
    _window->setWindowTitle("Path Evaluation");

    _distances = new QChart;
    _distances->setTitle("Obstacle Distance");
    _distances->setMinimumSize(800, 400);
    auto *distancesView = new QChartView(_distances);
    distancesView->setRenderHint(QPainter::Antialiasing);
    _layout->addWidget(distancesView, 0, 0, 1, 2);

    _boxSeries = new QBoxPlotSeries;
    _boxes = new QChart;
    _boxes->setTitle("Obstacle Distance");
    _boxes->setMinimumSize(400, 400);
    _boxes->addSeries(_boxSeries);
    auto *boxesView = new QChartView(_boxes);
    boxesView->setRenderHint(QPainter::Antialiasing);
    _layout->addWidget(boxesView, 0, 2);

    _pathLengthSeries = new QBarSeries;
    _pathLengths = new QChart;
    _pathLengths->setTitle("Path Length");
    _pathLengths->setMinimumSize(400, 400);
    _pathLengths->addSeries(_pathLengthSeries);
    auto *pathLengthsView = new QChartView(_pathLengths);
    pathLengthsView->setRenderHint(QPainter::Antialiasing);
    _layout->addWidget(pathLengthsView, 1, 0);

    _curvatureSeries = new QBarSeries;
    _curvatures = new QChart;
    _curvatures->setTitle("Maximum Curvature");
    _curvatures->setMinimumSize(400, 400);
    _curvatures->addSeries(_curvatureSeries);
    auto *curvaturesView = new QChartView(_curvatures);
    curvaturesView->setRenderHint(QPainter::Antialiasing);
    _layout->addWidget(curvaturesView, 1, 1);

    _timeSeries = new QBarSeries;
    _times = new QChart;
    _times->setTitle("Planning Time");
    _times->setMinimumSize(400, 400);
    _times->addSeries(_timeSeries);
    auto *timesView = new QChartView(_times);
    timesView->setRenderHint(QPainter::Antialiasing);
    _layout->addWidget(timesView, 1, 2);

    _originalPathLengthSet = new QBarSet(QString("Original"));
    _smoothedPathLengthSet = new QBarSet(QString("Smoothed"));
    _originalCurvatureSet = new QBarSet(QString("Original"));
    _smoothedCurvatureSet = new QBarSet(QString("Smoothed"));
    _originalTimeSet = new QBarSet(QString("Original"));
    _smoothedTimeSet = new QBarSet(QString("Smoothed"));

    _omplSmoothed1PathLengthSet = new QBarSet(QString("BSpline"));
    _omplSmoothed1CurvatureSet = new QBarSet(QString("BSpline"));
    _omplSmoothed1TimeSet = new QBarSet(QString("BSpline"));
    _omplSmoothed2PathLengthSet = new QBarSet(QString("SimplifyMax"));
    _omplSmoothed2CurvatureSet = new QBarSet(QString("SimplifyMax"));
    _omplSmoothed2TimeSet = new QBarSet(QString("SimplifyMax"));
    _omplSmoothed3PathLengthSet = new QBarSet(QString("Shortcut"));
    _omplSmoothed3CurvatureSet = new QBarSet(QString("Shortcut"));
    _omplSmoothed3TimeSet = new QBarSet(QString("Shortcut"));
    _omplSmoothed4PathLengthSet = new QBarSet(QString("AnytimePS"));
    _omplSmoothed4CurvatureSet = new QBarSet(QString("AnytimePS"));
    _omplSmoothed4TimeSet = new QBarSet(QString("AnytimePS"));
}

QValueAxis *createAxis(qreal min, qreal max)
{
    auto *axis = new QValueAxis;
    axis->setMin(min);
    axis->setMax(max);
    return axis;
}

double findMaximumAxis(double max)
{
    double a = 1.;
    while (a < max)
    {
        if (a * 1.5 >= max)
            return a * 1.5;
        a *= 2.;
    }

    while (a/2. > max)
        a /= 2.;
    return a;
}

void PathEvaluation::show()
{
    _distances->createDefaultAxes();
    _distances->setAxisY(createAxis(0, 10), _distances->series().first());
    for (int i = 1; i < _distances->series().size(); i+=2)
        _distances->legend()->markers(_distances->series()[i])[0]->setVisible(false);

    _boxes->createDefaultAxes();
    _boxes->setAxisY(createAxis(0, 10), _boxes->series().first());
    _boxes->axisX()->hide();
    _boxes->legend()->hide();

//    auto *bs = new QBarSet(QString("smooothed"));
//    bs->append(24);
//    bs->append(60);
//    _pathLengthSeries->append(bs);

    _pathLengthSeries->append(_originalPathLengthSet);
    _pathLengthSeries->append(_smoothedPathLengthSet);
    _pathLengthSeries->append(_omplSmoothed1PathLengthSet);
    _pathLengthSeries->append(_omplSmoothed2PathLengthSet);
    _pathLengthSeries->append(_omplSmoothed3PathLengthSet);
    _pathLengthSeries->append(_omplSmoothed4PathLengthSet);
    _pathLengths->createDefaultAxes();
    OMPL_DEBUG("Maximum path length: %.2f", _maxPathLength);
    auto *pathLengthLabelsBarAxis = new QBarCategoryAxis();
    pathLengthLabelsBarAxis->append(_labels);
    _pathLengths->setAxisX(pathLengthLabelsBarAxis, _pathLengthSeries);
    _pathLengths->setAxisY(createAxis(0, findMaximumAxis(_maxPathLength)), _pathLengthSeries);
//    _pathLengths->legend()->markers(_pathLengthSeries)[1]->setVisible(false);
//    _pathLengths->legend()->markers(_pathLengthSeries)[3]->setVisible(false);

    _curvatureSeries->append(_originalCurvatureSet);
    _curvatureSeries->append(_smoothedCurvatureSet);
    _curvatureSeries->append(_omplSmoothed1CurvatureSet);
    _curvatureSeries->append(_omplSmoothed2CurvatureSet);
    _curvatureSeries->append(_omplSmoothed3CurvatureSet);
    _curvatureSeries->append(_omplSmoothed4CurvatureSet);
    _curvatures->createDefaultAxes();
    auto *curvatureLabelsBarAxis = new QBarCategoryAxis();
    curvatureLabelsBarAxis->append(_labels);
    _curvatures->setAxisX(curvatureLabelsBarAxis, _curvatureSeries);
    _curvatures->setAxisY(createAxis(0, findMaximumAxis(_maxCurvature)), _curvatureSeries);

    _timeSeries->append(_originalTimeSet);
    _timeSeries->append(_smoothedTimeSet);
    _timeSeries->append(_omplSmoothed1TimeSet);
    _timeSeries->append(_omplSmoothed2TimeSet);
    _timeSeries->append(_omplSmoothed3TimeSet);
    _timeSeries->append(_omplSmoothed4TimeSet);
    _times->createDefaultAxes();
    auto *timeLabelsBarAxis = new QBarCategoryAxis();
    timeLabelsBarAxis->append(_labels);
    _times->setAxisX(timeLabelsBarAxis, _timeSeries);
    _times->setAxisY(createAxis(0, findMaximumAxis(_maxTime)), _timeSeries);

    _window->update();
    if (_smoothCollides)
        _window->setStyleSheet("background-color:red");
    _window->showMaximized();
}

PathStatistics PathEvaluation::add(AbstractPlanner *planner, std::string label, QColor color)
{
//    OMPLPlanner<PLANNER> planner;
    PathStatistics stats(label, color);
    if (!planner->run())
    {
        stats.pathFound = false;
        OMPL_ERROR("Planner %s couldn't find a solution.", label.c_str());
        return stats;
    }
    stats.pathFound = true;
    stats.exactGoalPath = planner->hasReachedGoalExactly();
    std::vector<Tpoint> path = planner->solutionPath();
    std::vector<GNode> trajectory = planner->solutionTrajectory();

    double planningTime = planner->planningTime();
    stats.planningTime = planningTime;
//    auto begin_time = ros::WallTime::now().toSec();
    std::vector<GNode> smoothed(trajectory);
    OMPL_INFORM("Running our smoothing method on %s ...", label.c_str());
    PostSmoothing::smooth(smoothed, path);
    auto postsmoothing_time = planningTime + PostSmoothing::smoothingTime;
    auto smoothedTrajPoints = PlannerUtils::toSteeredTrajectoryPoints(smoothed);
    auto *smoothedTraj = new Trajectory(smoothedTrajPoints);
    double smoothedPathLength = PathLengthMetric::evaluate(smoothedTraj);
    double smoothedCurvature = CurvatureMetric::evaluateMetric(smoothedTraj, 0, false);
//        QtVisualizer::drawNodes(smoothedTrajPoints, QColor(255, 150, 0), .03f);
//        QtVisualizer::drawNodes(smoothed, false, Qt::red, .03f);
//        QtVisualizer::drawNodes(smoothed, false, Qt::red, .03f);

    QPen oPen(color, 1., Qt::PenStyle::DashLine);
    QtVisualizer::addLegendEntry(LegendEntry(label, oPen));
//        QtVisualizer::drawNodes(path, color, .03f);
    QtVisualizer::drawPath(path, oPen);

    QPen ourPen(color, 1.5f);

    auto *lines = new QLineSeries;
    lines->setColor(color);
    lines->setName(QString::fromStdString(label));
    lines->setPen(oPen);

    _labels << QString::fromStdString(label);

    std::vector<double> distances = computeObstacleDistances(path);
    stats.pathCollides = false;
    for (unsigned int i = 0; i < distances.size(); ++i)
    {
        lines->append((i + 1.f) / distances.size(), distances[i]);
        if (distances[i] <= 0.0f)
            stats.pathCollides = true;
    }

    _distances->addSeries(lines);

    auto *smoothedLines = new QLineSeries;
    smoothedLines->setName(QString::fromStdString(label) + " smoothed");
    smoothedLines->setColor(color);

    std::vector<double> smoothedDistances = computeObstacleDistances(smoothedTrajPoints);
    stats.ourSmoothingCollides = false;
    for (unsigned int i = 0; i < smoothedDistances.size(); ++i)
    {
        smoothedLines->append((i + 1.f) / smoothedDistances.size(), smoothedDistances[i]);
        if (smoothedDistances[i] <= 0.f)
        {
            _smoothCollides = true;
            stats.ourSmoothingCollides = true;
        }
    }
    stats.ourSmoothingCollides = PlannerUtils::collides(smoothedTrajPoints);
    if (stats.ourSmoothingCollides)
        OMPL_WARN("%s collides with our smoothing!", label.c_str());

    if (stats.exactGoalPath && !stats.ourSmoothingCollides)
    {
        stats.ourSmoothingTime = postsmoothing_time;
        stats.ourSmoothingPathLength = smoothedPathLength;
        stats.ourSmoothingCurvature = smoothedCurvature;
        QtVisualizer::addLegendEntry(LegendEntry(label + " - Our Smoothing", ourPen));
        QtVisualizer::drawPath(smoothedTrajPoints, ourPen);
    }

    _distances->addSeries(smoothedLines);

    QBoxSet *box = computeBox(distances, label);
    box->setBrush(QBrush(color, Qt::BrushStyle::BDiagPattern));
    _boxSeries->append(box);

    QBoxSet *boxSmoothed = computeBox(smoothedDistances, label + " smoothed");
    boxSmoothed->setBrush(QBrush(color));
    _boxSeries->append(boxSmoothed);

    auto *traj = new Trajectory(path);
    double pathLength = PathLengthMetric::evaluate(traj);
    double curvature = CurvatureMetric::evaluate(traj);

    if (pathLength > _maxPathLength)
        _maxPathLength = pathLength;
    if (curvature > _maxCurvature)
        _maxCurvature = curvature;
    if (postsmoothing_time > _maxTime)
        _maxTime = postsmoothing_time;

//    QBarSet *pathLengthBarSet = new QBarSet(QString::fromStdString(label));
//    pathLengthBarSet->setColor(color);
//    *pathLengthBarSet << pathLength;
//    *pathLengthBarSet << smoothedPathLength;
//    std::cout << "Path length of " << label << " path: "
//              << pathLength << std::endl;
//    _pathLengthSeries->append(pathLengthBarSet);

    stats.curvature = curvature;
    stats.pathLength = pathLength;

    *_originalPathLengthSet << pathLength;
    *_smoothedPathLengthSet << smoothedPathLength;

//    QBarSet *curvatureBarSet = new QBarSet(QString::fromStdString(label));
//    curvatureBarSet->setColor(color);
//    *curvatureBarSet << curvature;
//    std::cout << "Curvature of " << label << " path: "
//              << curvature << std::endl;
//    _curvatureSeries->append(curvatureBarSet);
    *_originalCurvatureSet << curvature;
    *_smoothedCurvatureSet << smoothedCurvature;
//
//    QBarSet *timeBarSet = new QBarSet(QString::fromStdString(label));
//    timeBarSet->setColor(color);
//    *timeBarSet << planningTime;
//    std::cout << "Time of " << label << " path: "
//              << planningTime << "s" << std::endl;
//    _timeSeries->append(timeBarSet);
    *_originalTimeSet << planningTime;
    *_smoothedTimeSet << postsmoothing_time;

    OMPL_INFORM("Running B-Spline smoothing method on %s...", label.c_str());
    auto smoothed1 = planner->smoothBSpline();
    if (!smoothed1.trajectory.empty())
    {
        auto omplSmoothedTraj = new Trajectory(smoothed1.trajectory);
        stats.omplSmoothing1Collides = PlannerSettings::environment->collides(*omplSmoothedTraj);
        if (stats.omplSmoothing1Collides)
            OMPL_WARN("%s collides with smoothed B-spline", label.c_str());
        else if (stats.exactGoalPath)
        {
            stats.omplSmoothing1Time = planningTime + smoothed1.time;
            stats.omplSmoothing1PathLength = PathLengthMetric::evaluate(omplSmoothedTraj);
            stats.omplSmoothing1Curvature = CurvatureMetric::evaluateMetric(omplSmoothedTraj, 0, false);
            *_omplSmoothed1PathLengthSet << stats.omplSmoothing1PathLength;
            *_omplSmoothed1CurvatureSet << stats.omplSmoothing1Curvature;
            *_omplSmoothed1TimeSet << stats.omplSmoothing1Time;
            if (stats.omplSmoothing1PathLength > _maxPathLength)
                _maxPathLength = stats.omplSmoothing1PathLength;
            if (stats.omplSmoothing1Curvature > _maxCurvature)
                _maxCurvature = stats.omplSmoothing1Curvature;
            if (stats.omplSmoothing1Time > _maxTime)
                _maxTime = stats.omplSmoothing1Time;
            if (PlannerSettings::VisualizeSmoothing1)
            {
                int h, s, l, a;
                color.getHsl(&h, &s, &l, &a);
                QPen sPen(QColor::fromHsl((h - 40 + 360) % 360, s, l * 0.8, a), 1.5f, Qt::PenStyle::DashLine);
                QtVisualizer::addLegendEntry(LegendEntry(label + " - B-spline", sPen));
//            QtVisualizer::drawNodes(smoothed1.trajectory, QColor::fromHsl((h-40+255)%255, s, l, a), .03);
                QtVisualizer::drawPath(smoothed1.trajectory, sPen);
            }
        }
        delete omplSmoothedTraj;
    }

    OMPL_INFORM("Running SimplifyMax smoothing method on %s...", label.c_str());
    auto smoothed2 = planner->simplifyMax();
    if (!smoothed2.trajectory.empty())
    {
        auto omplSmoothedTraj = new Trajectory(smoothed2.trajectory);
        stats.omplSmoothing2Collides = PlannerSettings::environment->collides(*omplSmoothedTraj);
        if (stats.omplSmoothing2Collides)
            OMPL_WARN("%s collides with simplify max", label.c_str());
        else if (stats.exactGoalPath)
        {
            stats.omplSmoothing2Time = planningTime + smoothed2.time;
            stats.omplSmoothing2PathLength = PathLengthMetric::evaluate(omplSmoothedTraj);
            stats.omplSmoothing2Curvature = CurvatureMetric::evaluateMetric(omplSmoothedTraj, 0, false);
            *_omplSmoothed2PathLengthSet << stats.omplSmoothing2PathLength;
            *_omplSmoothed2CurvatureSet << stats.omplSmoothing2Curvature;
            *_omplSmoothed2TimeSet << stats.omplSmoothing2Time;
            if (stats.omplSmoothing2PathLength > _maxPathLength)
                _maxPathLength = stats.omplSmoothing2PathLength;
            if (stats.omplSmoothing2Curvature > _maxCurvature)
                _maxCurvature = stats.omplSmoothing2Curvature;
            if (stats.omplSmoothing2Time > _maxTime)
                _maxTime = stats.omplSmoothing2Time;
            if (PlannerSettings::VisualizeSmoothing2)
            {
                int h, s, l, a;
                color.getHsl(&h, &s, &l, &a);

                QPen sPen(QColor::fromHsl((h - 40 + 360) % 360, s, l), 1.5f, Qt::PenStyle::DashDotLine);
                QtVisualizer::addLegendEntry(LegendEntry(label + " - SimplifyMax", sPen));
//            QtVisualizer::drawNodes(smoothed1.trajectory, QColor::fromHsl((h-40+255)%255, s, l, a), .03);
                QtVisualizer::drawPath(smoothed2.trajectory, sPen);
//            QtVisualizer::drawNodes(smoothed2.trajectory, Qt::red, 0.3f); //1.5f, Qt::PenStyle::DashDotLine);

//                auto geoPath = planner->geometricPath();
//                for (auto *state : geoPath.getStates())
//                {
////            const auto *state2D = state->as<ob::RealVectorStateSpace::StateType>();
////            // Extract the robot's (x,y) position from its state
////            double x = state2D->values[0];
////            double y = state2D->values[1];
//                    const auto *s = state->as<ob::SE2StateSpace::StateType>();
//                    double x=s->getX(), y=s->getY();
//                    QtVisualizer::drawNode(x, y, Qt::darkGreen);
//                }
            }
        }
        delete omplSmoothedTraj;
    }

    OMPL_INFORM("Running shortcut smoothing method on %s...", label.c_str());
    auto smoothed3 = planner->shortcutPath();
    if (!smoothed3.trajectory.empty())
    {
        auto omplSmoothedTraj = new Trajectory(smoothed3.trajectory);
        stats.omplSmoothing3Collides = PlannerSettings::environment->collides(*omplSmoothedTraj);
        if (stats.omplSmoothing3Collides)
            OMPL_WARN("%s collides with shortcut", label.c_str());
        else if (stats.exactGoalPath)
        {
            stats.omplSmoothing3Time = planningTime + smoothed3.time;
            stats.omplSmoothing3PathLength = PathLengthMetric::evaluate(omplSmoothedTraj);
            stats.omplSmoothing3Curvature = CurvatureMetric::evaluateMetric(omplSmoothedTraj, 0, false);
            *_omplSmoothed3PathLengthSet << stats.omplSmoothing3PathLength;
            *_omplSmoothed3CurvatureSet << stats.omplSmoothing3Curvature;
            *_omplSmoothed3TimeSet << stats.omplSmoothing3Time;
            if (stats.omplSmoothing3PathLength > _maxPathLength)
                _maxPathLength = stats.omplSmoothing3PathLength;
            if (stats.omplSmoothing3Curvature > _maxCurvature)
                _maxCurvature = stats.omplSmoothing3Curvature;
            if (stats.omplSmoothing3Time > _maxTime)
                _maxTime = stats.omplSmoothing3Time;
            if (PlannerSettings::VisualizeSmoothing3)
            {
                int h, s, l, a;
                color.getHsl(&h, &s, &l, &a);
                QPen sPen(QColor::fromHsl((h + 30) % 360, s, l * 0.8), 1.5f, Qt::PenStyle::DashDotDotLine);
                QtVisualizer::addLegendEntry(LegendEntry(label + " - ShortcutPath", sPen));
                QtVisualizer::drawPath(smoothed3.trajectory, sPen);
//            QtVisualizer::drawNodes(smoothed3.trajectory, QColor::fromHsl((h+30+255)%255, s, l, a), 0.03f); //1.5f, Qt::PenStyle::DashDotLine);
            }
        }
        delete omplSmoothedTraj;
    }

    OMPL_INFORM("Running Anytime Path Shortening on %s...", label.c_str());
    auto smoothed4 = planner->anytimePathShortening();
    if (!smoothed4.trajectory.empty())
    {
        stats.omplSmoothing4Found = true;
        stats.exactGoalSmoothedPath = (smoothed4.status == ob::PlannerStatus::EXACT_SOLUTION);
        auto omplSmoothedTraj = new Trajectory(smoothed4.trajectory);
        stats.omplSmoothing4Collides = PlannerSettings::environment->collides(*omplSmoothedTraj);
        if (stats.omplSmoothing4Collides)
            OMPL_WARN("%s collides with Anytime PS", label.c_str());
        else if (stats.exactGoalSmoothedPath)
        {
            stats.omplSmoothing4Time = planningTime + smoothed4.time;
            stats.omplSmoothing4PathLength = PathLengthMetric::evaluate(omplSmoothedTraj);
            stats.omplSmoothing4Curvature = CurvatureMetric::evaluateMetric(omplSmoothedTraj, 0, false);
            *_omplSmoothed4PathLengthSet << stats.omplSmoothing4PathLength;
            *_omplSmoothed4CurvatureSet << stats.omplSmoothing4Curvature;
            *_omplSmoothed4TimeSet << stats.omplSmoothing4Time;
            if (stats.omplSmoothing4PathLength > _maxPathLength)
                _maxPathLength = stats.omplSmoothing4PathLength;
            if (stats.omplSmoothing4Curvature > _maxCurvature)
                _maxCurvature = stats.omplSmoothing4Curvature;
            if (stats.omplSmoothing4Time > _maxTime)
                _maxTime = stats.omplSmoothing4Time;
            if (PlannerSettings::VisualizeSmoothing4)
            {
                int h, s, l, a;
                color.getHsl(&h, &s, &l, &a);
                QPen sPen(QColor::fromHsl((h - 40 + 360) % 360, s, l * 0.8), 1.5f, Qt::PenStyle::DashDotDotLine);
                QtVisualizer::addLegendEntry(LegendEntry(label + " - AnytimePS", sPen));
                QtVisualizer::drawPath(smoothed4.trajectory, sPen);
//            QtVisualizer::drawNodes(smoothed4.trajectory, QColor::fromHsl((h+40+255)%255, s, l, a), 0.04f); //1.5f, Qt::PenStyle::DashDotLine);
            }
        }
        delete omplSmoothedTraj;
    }
    else
        stats.omplSmoothing4Found = false;

    QtVisualizer::drawStats(stats);

    delete smoothedTraj;
    delete traj;
    return stats;
}

bool approxEqual(double a, double b)
{
    return std::abs(a-b) < 1e-4;
}

void PathStatisticsAggregator::showSummary()
{
    auto *_layout = new QGridLayout;
    auto *_window = new QWidget;
    _window->setLayout(_layout);
    _window->setWindowTitle("Evaluation over Multiple Environments");

    auto *pathLengths = new QChart;
    pathLengths->setTitle("Path Length");
    pathLengths->setMinimumSize(400, 300);
    pathLengths->createDefaultAxes();

    auto *curvatures = new QChart;
    curvatures->setTitle("Maximum Curvature");
    curvatures->setMinimumSize(400, 300);
    curvatures->createDefaultAxes();

    auto *times = new QChart;
    times->setTitle("Planning Time");
    times->setMinimumSize(400, 300);
    times->createDefaultAxes();

    auto *results = new QChart;
    results->setTitle("Planning Status");
    results->setMinimumSize(400, 300);
    results->createDefaultAxes();

    // first compute maxima to get the axes right
    std::vector<double> maxPathLengths, maxCurvatures, maxTimes;

    int ourSmoothingCollisions = 0;
    int omplSmoothing1Collisions = 0;
    int omplSmoothing2Collisions = 0;
    int omplSmoothing3Collisions = 0;
    int omplSmoothing4Collisions = 0;
    int pathNotExactGoal = 0;
    int pathNotFound = 0;
    int anytimeNotFound = 0;
    int anytimeNotExactGoal = 0;
    int maxplots = 0;
    std::vector<double> ourCurvatures;
    std::vector<double> omplSmoothing1Curvatures;
    std::vector<double> omplSmoothing2Curvatures;
    std::vector<double> omplSmoothing3Curvatures;
    std::vector<double> omplSmoothing4Curvatures;
    std::vector<double> ourPathLengths;
    std::vector<double> omplSmoothing1PathLengths;
    std::vector<double> omplSmoothing2PathLengths;
    std::vector<double> omplSmoothing3PathLengths;
    std::vector<double> omplSmoothing4PathLengths;
    std::vector<double> ourTimes;
    std::vector<double> omplSmoothing1Times;
    std::vector<double> omplSmoothing2Times;
    std::vector<double> omplSmoothing3Times;
    std::vector<double> omplSmoothing4Times;

    auto psize = _map[*(_planners.begin())].size();
    std::vector<double> minCurvature(psize, std::numeric_limits<double>::max());
    std::vector<double> minPathLength(psize, std::numeric_limits<double>::max());
    std::vector<std::string> minCurvatureName(psize, "Unknown");
    std::vector<std::string> minPathLengthName(psize, "Unknown");

    for (const auto &planner : _planners)
    {
        auto statsList = _map[planner];
        int pi = 0;
        for (auto stats : statsList)
        {
            maxplots++;
            if (!stats.pathFound)
                pathNotFound++;
            else
            {
                pathNotExactGoal += !stats.exactGoalPath ? 1 : 0;
                ourSmoothingCollisions += stats.ourSmoothingCollides ? 1 : 0;
                omplSmoothing1Collisions += stats.omplSmoothing1Collides ? 1 : 0;
                omplSmoothing2Collisions += stats.omplSmoothing2Collides ? 1 : 0;
                omplSmoothing3Collisions += stats.omplSmoothing3Collides ? 1 : 0;

                maxPathLengths.push_back(stats.pathLength);
                if (stats.ourSmoothingPathLength >= 0.)
                {
                    maxPathLengths.push_back(stats.ourSmoothingPathLength);
                    ourPathLengths.push_back((stats.ourSmoothingPathLength));
                    if (stats.ourSmoothingPathLength < minPathLength[pi]
                        || (planner == "Theta*" && approxEqual(stats.ourSmoothingPathLength, minPathLength[pi])))
                    {
                        minPathLength[pi] = stats.ourSmoothingPathLength;
                        minPathLengthName[pi] = planner + " (Ours)";
                    }
                }
                if (stats.omplSmoothing1PathLength >= 0.)
                {
                    maxPathLengths.push_back(stats.omplSmoothing1PathLength);
                    omplSmoothing1PathLengths.push_back(stats.omplSmoothing1PathLength);
                    if (stats.omplSmoothing1PathLength < minPathLength[pi]
                        || (planner == "Theta*" && approxEqual(stats.omplSmoothing1PathLength, minPathLength[pi])))
                    {
                        minPathLength[pi] = stats.omplSmoothing1PathLength;
                        minPathLengthName[pi] = planner + " (B-Spline)";
                    }
                }
                if (stats.omplSmoothing2PathLength >= 0.)
                {
                    maxPathLengths.push_back(stats.omplSmoothing2PathLength);
                    omplSmoothing2PathLengths.push_back(stats.omplSmoothing2PathLength);
                    if (stats.omplSmoothing2PathLength < minPathLength[pi]
                        || (planner == "Theta*" && approxEqual(stats.omplSmoothing2PathLength, minPathLength[pi])))
                    {
                        minPathLength[pi] = stats.omplSmoothing2PathLength;
                        minPathLengthName[pi] = planner + " (SimplifyMax)";
                    }
                }
                if (stats.omplSmoothing3PathLength >= 0.)
                {
                    maxPathLengths.push_back(stats.omplSmoothing3PathLength);
                    omplSmoothing3PathLengths.push_back(stats.omplSmoothing3PathLength);
                    if (stats.omplSmoothing3PathLength < minPathLength[pi]
                        || (planner == "Theta*" && approxEqual(stats.omplSmoothing3PathLength, minPathLength[pi])))
                    {
                        minPathLength[pi] = stats.omplSmoothing3PathLength;
                        minPathLengthName[pi] = planner + " (Shortcut)";
                    }
                }
                maxCurvatures.push_back(stats.curvature);
                if (stats.ourSmoothingCurvature >= 0.)
                {
                    maxCurvatures.push_back(stats.ourSmoothingCurvature);
                    ourCurvatures.push_back((stats.ourSmoothingCurvature));
                    if (stats.ourSmoothingCurvature < minCurvature[pi]
                        || (planner == "Theta*" && approxEqual(stats.ourSmoothingCurvature, minCurvature[pi])))
                    {
                        minCurvature[pi] = stats.ourSmoothingCurvature;
                        minCurvatureName[pi] = planner + " (Ours)";
                    }
                }
                if (stats.omplSmoothing1Curvature >= 0.)
                {
                    maxCurvatures.push_back(stats.omplSmoothing1Curvature);
                    omplSmoothing1Curvatures.push_back(stats.omplSmoothing1Curvature);
                    if (stats.omplSmoothing1Curvature < minCurvature[pi]
                        || (planner == "Theta*" && approxEqual(stats.omplSmoothing1Curvature, minCurvature[pi])))
                    {
                        minCurvature[pi] = stats.omplSmoothing1Curvature;
                        minCurvatureName[pi] = planner + " (B-Spline)";
                    }
                }
                if (stats.omplSmoothing2Curvature >= 0.)
                {
                    maxCurvatures.push_back(stats.omplSmoothing2Curvature);
                    omplSmoothing2Curvatures.push_back(stats.omplSmoothing2Curvature);
                    if (stats.omplSmoothing2Curvature < minCurvature[pi]
                        || (planner == "Theta*" && approxEqual(stats.omplSmoothing2Curvature, minCurvature[pi])))
                    {
                        minCurvature[pi] = stats.omplSmoothing2Curvature;
                        minCurvatureName[pi] = planner + " (SimplifyMax)";
                    }
                }
                if (stats.omplSmoothing3Curvature >= 0.)
                {
                    maxCurvatures.push_back(stats.omplSmoothing3Curvature);
                    omplSmoothing3Curvatures.push_back(stats.omplSmoothing3Curvature);
                    if (stats.omplSmoothing3Curvature < minCurvature[pi]
                        || (planner == "Theta*" && approxEqual(stats.omplSmoothing3Curvature, minCurvature[pi])))
                    {
                        minCurvature[pi] = stats.omplSmoothing3Curvature;
                        minCurvatureName[pi] = planner + " (Shortcut)";
                    }
                }
                maxTimes.push_back(stats.planningTime);
                if (stats.ourSmoothingTime >= 0.)
                {
                    maxTimes.push_back(stats.ourSmoothingTime);
                    ourTimes.push_back((stats.ourSmoothingTime));
                }
                if (stats.omplSmoothing1Time >= 0.)
                {
                    maxTimes.push_back(stats.omplSmoothing1Time);
                    omplSmoothing1Times.push_back(stats.omplSmoothing1Time);
                }
                if (stats.omplSmoothing2Time >= 0.)
                {
                    maxTimes.push_back(stats.omplSmoothing2Time);
                    omplSmoothing2Times.push_back(stats.omplSmoothing2Time);
                }
                if (stats.omplSmoothing3Time >= 0.)
                {
                    maxTimes.push_back(stats.omplSmoothing3Time);
                    omplSmoothing3Times.push_back(stats.omplSmoothing3Time);
                }
            }
            if (!stats.omplSmoothing4Found)
                anytimeNotFound++;
            else
            {
                anytimeNotExactGoal += !stats.exactGoalSmoothedPath ? 1 : 0;
                omplSmoothing4Collisions += stats.omplSmoothing4Collides ? 1 : 0;
                if (stats.omplSmoothing4PathLength >= 0.)
                {
                    maxPathLengths.push_back(stats.omplSmoothing4PathLength);
                    omplSmoothing4PathLengths.push_back(stats.omplSmoothing4PathLength);
                    if (stats.omplSmoothing4PathLength < minPathLength[pi]
                        || (planner == "Theta*" && approxEqual(stats.omplSmoothing4PathLength, minPathLength[pi])))
                    {
                        minPathLength[pi] = stats.omplSmoothing4PathLength;
                        minPathLengthName[pi] = planner + " (Anytime PS)";
                    }
                }
                if (stats.omplSmoothing4Curvature >= 0.)
                {
                    maxCurvatures.push_back(stats.omplSmoothing4Curvature);
                    omplSmoothing4Curvatures.push_back(stats.omplSmoothing4Curvature);
                    if (stats.omplSmoothing4Curvature < minCurvature[pi]
                        || (planner == "Theta*" && approxEqual(stats.omplSmoothing4Curvature, minCurvature[pi])))
                    {
                        minCurvature[pi] = stats.omplSmoothing4Curvature;
                        minCurvatureName[pi] = planner + " (Anytime PS)";
                    }
                }
                if (stats.omplSmoothing4Time >= 0.)
                {
                    maxTimes.push_back(stats.omplSmoothing4Time);
                    omplSmoothing4Times.push_back(stats.omplSmoothing4Time);
                }
            }
            ++pi;
        }
    }

    auto *pathNotFoundSeries = new QBarSet("Path not found");
    auto *missedExactSeries = new QBarSet("Missed exact goal");
    auto *noCollisionSeries = new QBarSet("Collided");
    auto *noProblemSeries = new QBarSet("Correct");
    noProblemSeries->setBrush(QBrush(Qt::black, Qt::BDiagPattern));
    *pathNotFoundSeries << pathNotFound
                        << pathNotFound
                        << pathNotFound
                        << pathNotFound
                        << anytimeNotFound;
    *missedExactSeries  << pathNotExactGoal
                        << pathNotExactGoal
                        << pathNotExactGoal
                        << pathNotExactGoal
                        << anytimeNotExactGoal;
    *noCollisionSeries  << ourSmoothingCollisions
                        << omplSmoothing1Collisions
                        << omplSmoothing2Collisions
                        << omplSmoothing3Collisions
                        << omplSmoothing4Collisions;
    *noProblemSeries    << maxplots-pathNotFound-pathNotExactGoal-ourSmoothingCollisions
                        << maxplots-pathNotFound-pathNotExactGoal-omplSmoothing1Collisions
                        << maxplots-pathNotFound-pathNotExactGoal-omplSmoothing2Collisions
                        << maxplots-pathNotFound-pathNotExactGoal-omplSmoothing3Collisions
                        << maxplots-anytimeNotFound-anytimeNotExactGoal-omplSmoothing4Collisions;
    auto *resultSeries = new QStackedBarSeries;
    resultSeries->setName("Planning Status");
    resultSeries->append(pathNotFoundSeries);
    resultSeries->append(missedExactSeries);
    resultSeries->append(noCollisionSeries);
    resultSeries->append(noProblemSeries);
    results->addSeries(resultSeries);
    results->createDefaultAxes();
    auto *axisResultsX = new QBarCategoryAxis();
    axisResultsX->append("Ours");
    axisResultsX->append("B-Spline");
    axisResultsX->append("SimplifyMax");
    axisResultsX->append("Shortcut");
    axisResultsX->append("Anytime PS");
//    axisResultsX->setRange(0, 4);
    results->setAxisX(axisResultsX, resultSeries);

    Table resultsTable;
    resultsTable.header = std::vector<std::string>{"Ours", "B-Spline", "SimplifyMax", "Shortcut", "Anytime PS"};
    resultsTable.rows.push_back(std::make_pair<std::string, std::vector<double> >(
            "Path not found", std::vector<double>{(double) pathNotFound,
                                                  (double) pathNotFound,
                                                  (double) pathNotFound,
                                                  (double) pathNotFound,
                                                  (double) anytimeNotFound}));
    resultsTable.rows.push_back(std::make_pair<std::string, std::vector<double> >(
            "Missed exact goal", std::vector<double>{(double) pathNotExactGoal,
                                                     (double) pathNotExactGoal,
                                                     (double) pathNotExactGoal,
                                                     (double) pathNotExactGoal,
                                                     (double) anytimeNotExactGoal}));
    resultsTable.rows.push_back(std::make_pair<std::string, std::vector<double> >(
            "Collided", std::vector<double>{(double) ourSmoothingCollisions,
                                            (double) omplSmoothing1Collisions,
                                            (double) omplSmoothing2Collisions,
                                            (double) omplSmoothing3Collisions,
                                            (double) omplSmoothing4Collisions}));
    resultsTable.rows.push_back(std::make_pair<std::string, std::vector<double> >(
            "Correct", std::vector<double>{(double) (maxplots-pathNotFound-pathNotExactGoal-ourSmoothingCollisions),
                                           (double) (maxplots-pathNotFound-pathNotExactGoal-omplSmoothing1Collisions),
                                           (double) (maxplots-pathNotFound-pathNotExactGoal-omplSmoothing2Collisions),
                                           (double) (maxplots-pathNotFound-pathNotExactGoal-omplSmoothing3Collisions),
                                           (double) (maxplots-anytimeNotFound-anytimeNotExactGoal-omplSmoothing4Collisions)}));

    resultsTable.addStatsRows("Curvature", ourCurvatures,
                 omplSmoothing1Curvatures, omplSmoothing2Curvatures,
                 omplSmoothing3Curvatures, omplSmoothing4Curvatures);
    resultsTable.addStatsRows("Path Length", ourPathLengths,
                 omplSmoothing1PathLengths, omplSmoothing2PathLengths,
                 omplSmoothing3PathLengths, omplSmoothing4PathLengths);
    resultsTable.addStatsRows("Time (s)", ourTimes,
                 omplSmoothing1Times, omplSmoothing2Times,
                 omplSmoothing3Times, omplSmoothing4Times);

    std::cout << resultsTable.latex() << std::endl;

    std::cout << "Best path length:" << std::endl;
    for (size_t i = 0; i < psize; ++i)
    {
        std::cout << "\t" << std::left << std::setw(20) << minPathLengthName[i] << ":\t" <<minPathLength[i] << std::endl;
    }
    std::cout << "Best curvature:" << std::endl;
    for (size_t i = 0; i < psize; ++i)
    {
        std::cout << "\t" << std::left << std::setw(20) << minCurvatureName[i] << ":\t" <<minCurvature[i] << std::endl;
    }

    if (maxPathLengths.empty() || maxCurvatures.empty() || maxTimes.empty())
        return;

    std::sort(maxPathLengths.begin(), maxPathLengths.end());
    std::sort(maxCurvatures.begin(), maxCurvatures.end());
    std::sort(maxTimes.begin(), maxTimes.end());
    double maxPathLength = maxPathLengths[(int)(maxPathLengths.size()*0.9)] * 1.2;
    double maxTime = maxTimes[(int)(maxTimes.size()*0.9)] * 1.2;
    double maxCurvature = maxCurvatures[(int)(maxCurvatures.size()*0.9)] * 1.2;

    auto *axisPathLengthX = new QCategoryAxis();
    axisPathLengthX->append("Ours", 1);
    axisPathLengthX->append("B-Spline", 2);
    axisPathLengthX->append("SimplifyMax", 3);
    axisPathLengthX->append("Shortcut", 4);
    axisPathLengthX->append("Anytime PS", 5);
    axisPathLengthX->setRange(0, 5);
    auto *axisCurvatureX = new QCategoryAxis();
    axisCurvatureX->append("Ours", 1);
    axisCurvatureX->append("B-Spline", 2);
    axisCurvatureX->append("SimplifyMax", 3);
    axisCurvatureX->append("Shortcut", 4);
    axisCurvatureX->append("Anytime PS", 5);
    axisCurvatureX->setRange(0, 5);
    auto *axisTimeX = new QCategoryAxis();
    axisTimeX->append("Ours", 1);
    axisTimeX->append("B-Spline", 2);
    axisTimeX->append("SimplifyMax", 3);
    axisTimeX->append("Shortcut", 4);
    axisTimeX->append("Anytime PS", 5);
    axisTimeX->setRange(0, 5);
    auto *axisPathLengthY = createAxis(0, findMaximumAxis(maxPathLength));
    auto *axisCurvatureY = createAxis(0, findMaximumAxis(maxCurvature));
    auto *axisTimeY = createAxis(0, findMaximumAxis(maxTime));

    int shift = 0;
    int x = 0;
    auto planner = _planners.begin();
    for (unsigned int i = 0; i < _planners.size(); ++i, ++planner)
    {
        x++;
        auto statsList = _map[*planner];
        auto *scatterPathLengthsExact = new QScatterSeries();
        scatterPathLengthsExact->setName(QString::fromStdString(*planner));
        scatterPathLengthsExact->setMarkerSize(3);
//        auto *scatterPathLengthsApprox = new QScatterSeries();
//        scatterPathLengthsApprox->setName(QString::fromStdString(*planner + " (approx)"));
//        scatterPathLengthsApprox->setMarkerShape(QScatterSeries::MarkerShapeRectangle);
//        scatterPathLengthsApprox->setMarkerSize(3);
        auto *scatterCurvaturesExact = new QScatterSeries();
        scatterCurvaturesExact->setName(QString::fromStdString(*planner));
        scatterCurvaturesExact->setMarkerSize(3);
//        auto *scatterCurvaturesApprox = new QScatterSeries();
//        scatterCurvaturesApprox->setName(QString::fromStdString(*planner + " (approx)"));
//        scatterCurvaturesApprox->setMarkerShape(QScatterSeries::MarkerShapeRectangle);
//        scatterCurvaturesApprox->setMarkerSize(3);
        auto *scatterTimesExact = new QScatterSeries();
        scatterTimesExact->setName(QString::fromStdString(*planner));
        scatterTimesExact->setMarkerSize(3);
//        auto *scatterTimesApprox = new QScatterSeries();
//        scatterTimesApprox->setName(QString::fromStdString(*planner + " (approx)"));
//        scatterTimesApprox->setMarkerShape(QScatterSeries::MarkerShapeRectangle);
//        scatterTimesApprox->setMarkerSize(3);
        for (auto &stats : statsList)
        {
            ++shift;
            scatterPathLengthsExact->setColor(stats.color);
            scatterPathLengthsExact->setPen(stats.color);
//            scatterPathLengthsApprox->setColor(stats.color);
            scatterCurvaturesExact->setColor(stats.color);
            scatterCurvaturesExact->setPen(stats.color);
//            scatterCurvaturesApprox->setColor(stats.color);
            scatterTimesExact->setColor(stats.color);
            scatterTimesExact->setPen(stats.color);
//            scatterTimesApprox->setColor(stats.color);
            if (!stats.pathFound)
                continue;

            // let scatter points have some offset to be easier distinguishable
            double offset = (shift - maxplots / 2.) / maxplots * 0.4;
//            if (stats.exactGoalPath)
//            {
                scatterPathLengthsExact->append(0.5 + offset, stats.ourSmoothingPathLength);
                scatterCurvaturesExact->append(0.5 + offset, stats.ourSmoothingCurvature);
                scatterTimesExact->append(0.5 + offset, stats.ourSmoothingTime);
//                if (*planner != "Theta*")
//                {
                    if (stats.omplSmoothing1PathLength >= 0
                        && stats.omplSmoothing1Time >= 0
                        && stats.omplSmoothing1Curvature >= 0)
                    {
                        scatterPathLengthsExact->append(1.5 + offset, stats.omplSmoothing1PathLength);
                        scatterCurvaturesExact->append(1.5 + offset, stats.omplSmoothing1Curvature);
                        scatterTimesExact->append(1.5 + offset, stats.omplSmoothing1Time);
                    }
                    if (stats.omplSmoothing2PathLength >= 0
                        && stats.omplSmoothing2Time >= 0
                        && stats.omplSmoothing2Curvature >= 0)
                    {
                        scatterPathLengthsExact->append(2.5 + offset, stats.omplSmoothing2PathLength);
                        scatterCurvaturesExact->append(2.5 + offset, stats.omplSmoothing2Curvature);
                        scatterTimesExact->append(2.5 + offset, stats.omplSmoothing2Time);
                    }
                    if (stats.omplSmoothing3PathLength >= 0
                        && stats.omplSmoothing3Time >= 0
                        && stats.omplSmoothing3Curvature >= 0)
                    {
                        scatterPathLengthsExact->append(3.5 + offset, stats.omplSmoothing3PathLength);
                        scatterCurvaturesExact->append(3.5 + offset, stats.omplSmoothing3Curvature);
                        scatterTimesExact->append(3.5 + offset, stats.omplSmoothing3Time);
                    }
//            if (stats.omplSmoothing4PathLength >= 0
//                && stats.omplSmoothing4Time >= 0
//                && stats.omplSmoothing4Curvature >= 0)
//            {
//                scatterPathLengthsExact->append(4.5 + offset, stats.omplSmoothing4PathLength);
//                scatterCurvaturesExact->append(4.5 + offset, stats.omplSmoothing4Curvature);
//                scatterTimesExact->append(4.5 + offset, stats.omplSmoothing4Time);
//            }
//                }
//            }
//            else
//            {
//                scatterPathLengthsApprox->append(0.5 + offset, stats.ourSmoothingPathLength);
//                if (*planner != "Theta*")
//                {
//                    scatterPathLengthsApprox->append(1.5 + offset, stats.omplSmoothing1PathLength);
//                    scatterPathLengthsApprox->append(2.5 + offset, stats.omplSmoothing2PathLength);
//                }
//            }
//            if (*planner != "Theta*")
//            {
//                if (stats.exactGoalSmoothedPath)
//                {
                    scatterPathLengthsExact->append(4.5 + offset, stats.omplSmoothing4PathLength);
                    scatterCurvaturesExact->append(4.5 + offset, stats.omplSmoothing4Curvature);
                    scatterTimesExact->append(4.5 + offset, stats.omplSmoothing4Time);
//                }
//                else
//                    scatterPathLengthsApprox->append(4.5 + offset, stats.omplSmoothing4PathLength);
//            }
        }
        pathLengths->addSeries(scatterPathLengthsExact);
//        pathLengths->addSeries(scatterPathLengthsApprox);
        pathLengths->setAxisX(axisPathLengthX, scatterPathLengthsExact);
//        pathLengths->setAxisX(axisPathLengthX, scatterPathLengthsApprox);
        pathLengths->setAxisY(axisPathLengthY, scatterPathLengthsExact);
//        pathLengths->setAxisY(axisPathLengthY, scatterPathLengthsApprox);
        curvatures->addSeries(scatterCurvaturesExact);
//        curvatures->addSeries(scatterCurvaturesApprox);
        curvatures->setAxisX(axisCurvatureX, scatterCurvaturesExact);
//        curvatures->setAxisX(axisPathLengthX, scatterCurvaturesApprox);
        curvatures->setAxisY(axisCurvatureY, scatterCurvaturesExact);
//        curvatures->setAxisY(axisCurvatureY, scatterCurvaturesApprox);
        times->addSeries(scatterTimesExact);
//        times->addSeries(scatterTimesApprox);
        times->setAxisX(axisTimeX, scatterTimesExact);
//        times->setAxisX(axisPathLengthX, scatterTimesApprox);
        times->setAxisY(axisTimeY, scatterTimesExact);
//        times->setAxisY(axisTimeY, scatterTimesApprox);
    }

    auto *pathLengthsView = new QChartView(pathLengths);
    pathLengthsView->setRenderHint(QPainter::Antialiasing);
    auto *curvaturesView = new QChartView(curvatures);
    curvaturesView->setRenderHint(QPainter::Antialiasing);
    auto *timesView = new QChartView(times);
    timesView->setRenderHint(QPainter::Antialiasing);
    auto *resultsView = new QChartView(results);
    resultsView->setRenderHint(QPainter::Antialiasing);
    _layout->addWidget(pathLengthsView, 0, 0);
    _layout->addWidget(curvaturesView, 0, 1);
    _layout->addWidget(timesView, 0, 2);
    _layout->addWidget(resultsView, 1, 0, 1, 2);
    _layout->addWidget(resultsTable.createWidget(), 1, 2);

    _window->show();
}
