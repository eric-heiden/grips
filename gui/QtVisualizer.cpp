#include <QtWidgets/QGraphicsTextItem>
#include <QtWidgets/QGraphicsProxyWidget>
#include <QtGui/QTextBlockFormat>
#include <QtGui/QTextCursor>
#include <QtWidgets/QLabel>
#include <QtCore/QTimeLine>
#include <utility>
#include <QtWidgets/QMenu>
#include <QtCharts/QtCharts>
#include <QtSvg/QGraphicsSvgItem>
#include <QtSvg/QSvgGenerator>
#include <QtSvg/QSvgRenderer>
#include <QFile>
#include <QByteArray>

#include "base/PlannerSettings.h"
#include "QtVisualizer.h"

int argc = 0;

QApplication *QtVisualizer::_app = new QApplication(argc, nullptr);
QMainWindow *QtVisualizer::_window = nullptr;
VisualizationView *QtVisualizer::_view = nullptr;
QGraphicsScene *QtVisualizer::_scene = nullptr;

std::vector<LegendEntry> QtVisualizer::_legend;

int QtVisualizer::_statsTextTop = -2;

bool QtVisualizer::_showStartGoal = true;

void QtVisualizer::initialize()
{
    _statsTextTop = 0;
    _window = new QMainWindow;
    _scene = new QGraphicsScene;
    _view = new VisualizationView(_scene);
    _window->setCentralWidget(_view);

    _scene->setItemIndexMethod(QGraphicsScene::NoIndex);
    _view->setScene(_scene);
    _view->setCacheMode(QGraphicsView::CacheBackground);
    _view->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
    _view->setRenderHint(QPainter::Antialiasing);
    _view->setMinimumSize(500, 500);

//    _window->setWindowFlags(Qt::WindowStaysOnTopHint);
    _window->setWindowTitle(QString("Theta* Trajectory Planning"));
}

QtVisualizer::~QtVisualizer()
{
    delete _view;
    delete _app;
    delete _window;
    delete _scene;
}

int QtVisualizer::exec()
{
    return _app->exec();
}

void QtVisualizer::visualize(Environment &environment, int run, bool renderDistances)
{
    _window->setWindowTitle(QString("Theta* Trajectory Planning (run %1)").arg(run));
    _scene->setSceneRect(0, 0, environment.width()+1, environment.height()+1);
    float scale = (float)1080*.9f/(float)environment.width();
    _view->scale(scale, scale);
    QPen pen(QColor(128, 128, 128, 20));
    pen.setWidthF(0.01f);
    pen.setCapStyle(Qt::PenCapStyle::FlatCap);
    pen.setJoinStyle(Qt::PenJoinStyle::BevelJoin);
    for (unsigned x = 0; x <= std::max(environment.width(), environment.height()); ++x)
    {
//        _scene->addLine(x, 0, x, environment.height(), pen);
//        _scene->addLine(0, x, environment.width(), x, pen);
        if (x % 5 == 0)
        {
            QPen blackPen(Qt::black);
            blackPen.setWidthF(0.01f);

            if (x <= environment.width())
            {
                QLabel *htext = new QLabel(QString::fromStdString(std::to_string(x)));
                htext->setFont(QFont("Consolas", 1));
                htext->setAlignment(Qt::AlignLeft);
                htext->setAttribute(Qt::WA_TranslucentBackground);
                _scene->addWidget(htext)->setGeometry(QRectF(x + 0.15, -1.2, 2, 2));
                _scene->addLine(x, -2, x, 0, blackPen);
            }

            if (x <= environment.height())
            {
                QLabel *vtext = new QLabel(QString::fromStdString(std::to_string(x)));
                vtext->setFont(QFont("Consolas", 1));
                vtext->setAlignment(Qt::AlignRight);
                vtext->setAttribute(Qt::WA_TranslucentBackground);
                _scene->addWidget(vtext)->setGeometry(QRectF(-2.2, x, 2, 2));
                _scene->addLine(-2, x, 0, x, blackPen);
            }
        }

        for (unsigned y = 0; y <= environment.height(); ++y)
        {
            if (environment.occupiedCell(x, y))
                _scene->addRect(x, y, 1, 1, pen, QColor(128, 128, 128));
//            _scene->addEllipse(x-.25, y-.25, 0.4, 0.4, pen,
//                            QColor::fromHslF(std::max(0., std::min(.65, environment.distance(x, y) * .1)), 1., .6));
            else if (renderDistances)
            {
                _scene->addRect(x, y, 0.5, 0.5, pen,
                                QColor::fromHslF(std::min(.65, environment.bilinearDistance(x+.25, y+.25) * .1), 1.,
                                                 .6));
                _scene->addRect(x + 0.5, y, 0.5, 0.5, pen,
                                QColor::fromHslF(std::min(.65, environment.bilinearDistance(x+.75, y+.25) * .1), 1.,
                                                 .6));
                _scene->addRect(x, y + 0.5, 0.5, 0.5, pen,
                                QColor::fromHslF(std::min(.65, environment.bilinearDistance(x+.25, y+.75) * .1), 1.,
                                                 .6));
                _scene->addRect(x + 0.5, y + 0.5, 0.5, 0.5, pen,
                                QColor::fromHslF(std::min(.65, environment.bilinearDistance(x+.75, y+.75) * .1), 1.,
                                                 .6));
            }
        }
    }

    if (_showStartGoal)
    {
        drawNode(environment.start(), QColor(200, 60, 0));
        drawNode(environment.goal(), QColor(0, 80, 200));
    }
}

void QtVisualizer::drawNode(const GNode &node, QColor color, double radius,
                            bool drawArrow)
{
    if (drawArrow)
    {
        QPen pen(color);
        pen.setWidthF(radius * 0.5);
        _scene->addLine(node.x_r,
                        node.y_r,
                        node.x_r + std::cos(node.theta) * radius * 5.,
                        node.y_r + std::sin(node.theta) * radius * 5.,
                        pen);
    }
    drawNode(node.x_r, node.y_r, color, radius);
}

void QtVisualizer::drawNode(const Tpoint &point, QColor color, double radius)
{
    drawNode(point.x, point.y, std::move(color), radius);
}

void QtVisualizer::drawNode(double x, double y, QColor color, double radius)
{
    QPen pen(color);
    pen.setWidthF(0.);
    _scene->addEllipse(x - radius, y - radius, 2*radius, 2*radius, pen, QBrush(color));
}

void QtVisualizer::drawTrajectory(std::vector<GNode> nodes, const QColor &color,
                                  float penWidth, Qt::PenStyle penStyle)
{
    std::vector<Tpoint> path;
    for (unsigned int i = 1; i < nodes.size(); ++i)
    {
        auto *traj = new Trajectory();
        PlannerSettings::steering->Steer(&nodes[i - 1], &nodes[i], traj);
        auto tpath = traj->getPath();
        path.insert(path.end(), tpath.begin(), tpath.end());
        delete traj;
    }
    drawPath(path, color, penWidth, penStyle);
}

void QtVisualizer::drawPath(std::vector<GNode> nodes, const QColor &color,
                            float penWidth, Qt::PenStyle penStyle)
{
    if (nodes.empty())
        return;
    QPen pen(color);
    pen.setWidthF(0.05f * penWidth);
    pen.setStyle(penStyle);
    QPainterPath pp;
    pp.moveTo(nodes[0].x_r, nodes[0].y_r);
    for (unsigned int i = 1; i < nodes.size(); ++i)
        pp.lineTo(nodes[i].x_r, nodes[i].y_r);
    _scene->addPath(pp, pen);
}

void QtVisualizer::drawPath(std::vector<Tpoint> nodes, QColor color,
                            float penWidth, Qt::PenStyle penStyle)
{
    if (nodes.empty())
        return;
    QPen pen(color);
    pen.setWidthF(0.05f * penWidth);
    pen.setStyle(penStyle);
    QPainterPath pp;
    pp.moveTo(nodes[0].x, nodes[0].y);
    for (unsigned int i = 1; i < nodes.size(); ++i)
        pp.lineTo(nodes[i].x, nodes[i].y);
    _scene->addPath(pp, pen);
}

void QtVisualizer::drawPath(std::vector<Tpoint> nodes, QPen pen)
{
    if (nodes.empty())
        return;
    pen.setWidthF(pen.widthF() * 0.05f);
    QPainterPath pp;
    pp.moveTo(nodes[0].x, nodes[0].y);
    for (unsigned int i = 1; i < nodes.size(); ++i)
        pp.lineTo(nodes[i].x, nodes[i].y);
    _scene->addPath(pp, pen);
}

void QtVisualizer::drawNodes(std::vector<GNode> nodes, bool drawArrows,
                             QColor color, double radius)
{
    for (auto &node : nodes)
        drawNode(node, color, radius, drawArrows);
}

void QtVisualizer::drawLabel(const std::string &text, double x, double y, QColor color, float size)
{
    QFont font("Consolas", 12);
    auto *textItem = _scene->addText(QString::fromStdString(text), font);
    textItem->setPos(x, y);
    textItem->setScale(size * 0.05);
    textItem->setDefaultTextColor(color);
}

void QtVisualizer::drawStats(const PathStatistics &stats)
{
    if (_statsTextTop == 0)
        _scene->addRect(-14.4, 1.3, 12, 48, QPen(Qt::black, 0.01), QBrush(QColor(255, 255, 255, 180)));
    QFont font("Consolas", 10);
    auto *textItem = _scene->addText(QString("Ours collides (%1) ? %2")
                                             .arg(QString::fromStdString(stats.planner))
                                             .arg(stats.ourSmoothingCollides), font);
    textItem->setPos(-14, _statsTextTop+1.5);
    _statsTextTop += 1;
    textItem->setScale(0.05);
    textItem->setDefaultTextColor(Qt::black);
    textItem = _scene->addText(QString("Curvature: %1")
                                       .arg(stats.ourSmoothingCurvature), font);
    textItem->setPos(-14, _statsTextTop+1.5);
    _statsTextTop += 1;
    textItem->setScale(0.05);
    textItem->setDefaultTextColor(Qt::black);
    textItem = _scene->addText(QString("B-Spline collides (%1) ? %2").arg(
            QString::fromStdString(stats.planner)).arg(stats.omplSmoothing1Collides), font);
    textItem->setPos(-14, _statsTextTop+1.5);
    _statsTextTop += 1;
    textItem->setScale(0.05);
    textItem->setDefaultTextColor(Qt::black);
    textItem = _scene->addText(QString("Curvature: %1")
                                       .arg(stats.omplSmoothing1Curvature), font);
    textItem->setPos(-14, _statsTextTop+1.5);
    _statsTextTop += 1;
    textItem->setScale(0.05);
    textItem->setDefaultTextColor(Qt::black);
    textItem = _scene->addText(QString("SimplifyMax collides (%1) ? %2").arg(
            QString::fromStdString(stats.planner)).arg(stats.omplSmoothing2Collides), font);
    textItem->setPos(-14, _statsTextTop+1.5);
    _statsTextTop += 1;
    textItem->setScale(0.05);
    textItem->setDefaultTextColor(Qt::black);
    textItem = _scene->addText(QString("Curvature: %1")
                                       .arg(stats.omplSmoothing2Curvature), font);
    textItem->setPos(-14, _statsTextTop+1.5);
    _statsTextTop += 1;
    textItem->setScale(0.05);
    textItem->setDefaultTextColor(Qt::black);
    textItem = _scene->addText(QString("ShortcutPath collides (%1) ? %2").arg(
            QString::fromStdString(stats.planner)).arg(stats.omplSmoothing3Collides), font);
    textItem->setPos(-14, _statsTextTop+1.5);
    _statsTextTop += 1;
    textItem->setScale(0.05);
    textItem->setDefaultTextColor(Qt::black);
    textItem = _scene->addText(QString("Curvature: %1")
                                       .arg(stats.omplSmoothing3Curvature), font);
    textItem->setPos(-14, _statsTextTop+1.5);
    _statsTextTop += 1;
    textItem->setScale(0.05);
    textItem->setDefaultTextColor(Qt::black);
    textItem = _scene->addText(QString("Anytime PS collides (%1) ? %2").arg(
            QString::fromStdString(stats.planner)).arg(stats.omplSmoothing4Collides), font);
    textItem->setPos(-14, _statsTextTop+1.5);
    _statsTextTop += 1;
    textItem->setScale(0.05);
    textItem->setDefaultTextColor(Qt::black);
    textItem = _scene->addText(QString("Curvature: %1")
                                       .arg(stats.omplSmoothing4Curvature), font);
    textItem->setPos(-14, _statsTextTop+1.5);
    _statsTextTop += 1;
    textItem->setScale(0.05);
    textItem->setDefaultTextColor(Qt::black);
    _statsTextTop += 0.5;
}

void QtVisualizer::drawNodes(std::vector<Tpoint> nodes, QColor color, double radius)
{
    for (auto &node : nodes)
        drawNode(node, color, radius);
}

void QtVisualizer::addLegendEntry(LegendEntry entry)
{
    entry.pen.setWidthF(entry.pen.widthF() * 0.05f);
    _legend.push_back(entry);
}

void QtVisualizer::drawLegend()
{
    double x = PlannerSettings::environment->width()+3;
    double y = 0;
    _scene->addRect(x, y, 10, (_legend.size()+(_showStartGoal?2:0))*1.+0.2,
                    QPen(Qt::black, 0.01), QBrush(QColor(255, 255, 255, 220)));
    y += 0.2;
    QFont font("Arial", 10);
    for (auto &entry : _legend)
    {
        _scene->addLine(x+.3, y+.4, x+1.5, y+.4, entry.pen);
        auto *textItem = _scene->addText(QString::fromStdString(entry.label), font);
        textItem->setScale(0.05);
        textItem->setPos(x+1.7, y-0.2);
        textItem->setDefaultTextColor(Qt::black);
        y += 1;
    }

    if (_showStartGoal)
    {
        drawNode(x + 0.9, y + 0.37, QColor(200, 60, 0));
        auto *sti = _scene->addText(QString("Start"), font);
        sti->setScale(0.05);
        sti->setPos(x + 1.7, y - 0.2);
        sti->setDefaultTextColor(Qt::black);
        y += 1;
        drawNode(x + 0.9, y + 0.37, QColor(0, 80, 200));
        auto *gti = _scene->addText(QString("Goal"), font);
        gti->setScale(0.05);
        gti->setPos(x + 1.7, y - 0.2);
        gti->setDefaultTextColor(Qt::black);
    }
}

void QtVisualizer::showStartGoal(bool show)
{
    _showStartGoal = show;
}

void QtVisualizer::savePng(const QString &fileName)
{
    QRectF newSceneRect;
    for (auto *item: _scene->items())
    {
        newSceneRect |= item->mapToScene(item->boundingRect()).boundingRect();
    }
    _scene->setSceneRect(newSceneRect);
    _scene->clearSelection();

    QImage image(_scene->sceneRect().size().toSize().scaled(2000, 2000, Qt::KeepAspectRatio),
                 QImage::Format_ARGB32);
    image.fill(Qt::transparent);

    QPainter painter(&image);
    _scene->render(&painter);
    image.save(fileName);

    std::cout << "Saved PNG at " << fileName.toStdString() << std::endl;
}

void QtVisualizer::saveSvg(const QString &fileName)
{
    QRectF newSceneRect;
    for (auto *item: _scene->items())
    {
        newSceneRect |= item->mapToScene(item->boundingRect()).boundingRect();
    }
    _scene->setSceneRect(newSceneRect);
    _scene->clearSelection();
    QSize sceneSize = newSceneRect.size().toSize();
    sceneSize.setWidth(sceneSize.width() * 10);
    sceneSize.setHeight(sceneSize.height() * 10);

    QSvgGenerator generator;
    generator.setFileName(fileName);
    generator.setSize(sceneSize);
    generator.setViewBox(QRect(0, 0, sceneSize.width(), sceneSize.height()));
    generator.setDescription(QObject::tr("Post-smoothing Trajectories"));
    generator.setTitle(fileName);
    QPainter painter;
    painter.begin(&generator);
    _scene->render(&painter);
    painter.end();

    std::cout << "Saved SVG at " << fileName.toStdString() << std::endl;
}

VisualizationView::VisualizationView(QGraphicsScene *scene, QWidget *parent)
    : QGraphicsView(parent), _numScheduledScalings(1), _scene(scene)
{
    setDragMode(QGraphicsView::ScrollHandDrag);
    setContextMenuPolicy(Qt::CustomContextMenu);

    connect(this, SIGNAL(customContextMenuRequested(const QPoint &)),
            this, SLOT(showContextMenu(const QPoint &)));
}

VisualizationView::~VisualizationView()
{
    delete _scene;
}

void VisualizationView::wheelEvent(QWheelEvent *event)
{
    if (event->delta() < 0)
        scale(0.9, 0.9);
    else
        scale(1.1, 1.1);
}

void VisualizationView::showContextMenu(const QPoint &pos)
{
    QMenu contextMenu("Context menu", this);

    QAction action1("Save as SVG", this);
    connect(&action1, SIGNAL(triggered()), this, SLOT(saveSvg()));
    contextMenu.addAction(&action1);

    QAction action2("Save as PNG", this);
    connect(&action2, SIGNAL(triggered()), this, SLOT(savePng()));
    contextMenu.addAction(&action2);

    contextMenu.exec(mapToGlobal(pos));
}

void VisualizationView::saveSvg()
{
    QString filename = QFileDialog::getSaveFileName(this,
                                                    "Save SVG", "",
                                                    "Scalable Vector Graphics (*.svg)");
    if (filename.isNull())
        return;

    QtVisualizer::saveSvg(filename);
}

void VisualizationView::savePng()
{
    QString filename = QFileDialog::getSaveFileName(this,
                                                    "Save PNG", "",
                                                    "Portable Network Graphics (*.png)");
    if (filename.isNull())
        return;

    QtVisualizer::savePng(filename);
}
