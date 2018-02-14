#include "Log.h"
#include "PostSmoothing.h"

nlohmann::json Log::_json = {{"runs", nlohmann::json::array()}};
nlohmann::json Log::_currentRun;

void Log::instantiateRun()
{
    auto time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    char mbstr[100];
    std::strftime(mbstr, sizeof(mbstr), "%F %T", std::localtime(&time));
    std::string tstr(mbstr);
    tstr = tstr.substr(0, tstr.length()-1);
    _currentRun = {
            {"globals",  {
                                 {"time", tstr},
                         }},
            {"settings", {
                                 {"steering", Steering::to_string(PlannerSettings::steeringType)},
                                 {"carTurningRadius", PlannerSettings::CarTurningRadius},
                                 {"ourSmoothing", {
                                      {"minNodeDistance", PlannerSettings::gripsMinNodeDistance},
                                      {"eta", PlannerSettings::gripsEta},
                                      {"etaDiscount", PlannerSettings::gripsEtaDiscount},
                                      {"gdRounds", PlannerSettings::gripsGradientDescentRounds},
                                      {"pruningRounds", PlannerSettings::gripsMaxPruningRounds},
                                      {"minimizePathLength", PostSmoothing::MINIMIZE_PATHLENGTH},
                                      {"fixCollisions", PostSmoothing::FIX_COLLISIONS}
                                 }}
                         }},
            {"environment",
                         {
                                 {"width", PlannerSettings::environment->width()},
                                 {"height", PlannerSettings::environment->height()},
                                 {"generator", PlannerSettings::environment->generatorType()},
                                 {"corridorRadius", PlannerSettings::environment->corridorRadius()},
                                 {"seed", PlannerSettings::environment->seed()},
                                 {"obstacleRatio", PlannerSettings::environment->obstacleRatio()}
                         }},
            {"runs",     nlohmann::json::array()}
    };
}

void Log::log(const PathStatistics &stats)
{
    auto runStats = nlohmann::json({
            {"planningTime",                      stats.planningTime},
            {"ourSmoothingTime",                  stats.ourSmoothingTime},
            {"omplSmoothing1Time",                stats.omplSmoothing1Time},
            {"omplSmoothing2Time",                stats.omplSmoothing2Time},
            {"omplSmoothing3Time",                stats.omplSmoothing3Time},
            {"omplSmoothing4Time",                stats.omplSmoothing4Time},
            {"pathFound",                         stats.pathFound},
            {"omplSmoothing4Found",               stats.omplSmoothing4Found},
            {"pathCollides",                      stats.pathCollides},
            {"ourSmoothingCollides",              stats.ourSmoothingCollides},
            {"omplSmoothing1Collides",            stats.omplSmoothing1Collides},
            {"omplSmoothing2Collides",            stats.omplSmoothing2Collides},
            {"omplSmoothing3Collides",            stats.omplSmoothing3Collides},
            {"omplSmoothing4Collides",            stats.omplSmoothing4Collides},
            {"exactGoalPath",                     stats.exactGoalPath},
            {"exactGoalSmoothedPath",             stats.exactGoalSmoothedPath},
            {"pathLength",                        stats.pathLength},
            {"ourSmoothingPathLength",            stats.ourSmoothingPathLength},
            {"omplSmoothing1PathLength",          stats.omplSmoothing1PathLength},
            {"omplSmoothing2PathLength",          stats.omplSmoothing2PathLength},
            {"omplSmoothing3PathLength",          stats.omplSmoothing3PathLength},
            {"omplSmoothing4PathLength",          stats.omplSmoothing4PathLength},
            {"curvature",                         stats.curvature},
            {"ourSmoothingCurvature",             stats.ourSmoothingCurvature},
            {"omplSmoothing1Curvature",           stats.omplSmoothing1Curvature},
            {"omplSmoothing2Curvature",           stats.omplSmoothing2Curvature},
            {"omplSmoothing3Curvature",           stats.omplSmoothing3Curvature},
            {"omplSmoothing4Curvature",           stats.omplSmoothing4Curvature},
            {"planner",                           stats.planner},
            {"color",                             stats.color.name().toStdString()},
            {"ps_insertedNodes",                  PostSmoothing::insertedNodes},
            {"ps_pruningRounds",                  PostSmoothing::pruningRounds},
            {"ps_collisionFixAttempts",           PostSmoothing::collisionFixAttempts},
            {"ps_roundsWithCollisionFixAttempts", PostSmoothing::roundsWithCollisionFixAttempts},
            {"ps_nodesPerRound",                  nlohmann::json(PostSmoothing::nodesPerRound)},
            {"ps_roundStats",  nlohmann::json::array()}
    });
    for (auto &s : PostSmoothing::statsPerRound)
    {
        runStats["ps_roundStats"].push_back({
               {"pathLength",                 s.pathLength},
               {"maxCurvature",               s.maxCurvature},
               {"time",                       s.time},
               {"nodes",                      s.nodes},
               {"medianNodeObstacleDistance", s.medianNodeObstacleDistance},
               {"meanNodeObstacleDistance",   s.meanNodeObstacleDistance},
               {"minNodeObstacleDistance",    s.minNodeObstacleDistance},
               {"maxNodeObstacleDistance",    s.maxNodeObstacleDistance},
               {"stdNodeObstacleDistance",    s.stdNodeObstacleDistance},
               {"medianTrajObstacleDistance", s.medianTrajObstacleDistance},
               {"meanTrajObstacleDistance",   s.meanTrajObstacleDistance},
               {"minTrajObstacleDistance",    s.minTrajObstacleDistance},
               {"maxTrajObstacleDistance",    s.maxTrajObstacleDistance},
               {"stdTrajObstacleDistance",    s.stdTrajObstacleDistance},
               {"type",                       s.typeName()}
        });
    }
    _currentRun["runs"].push_back(runStats);
}

void Log::save(std::string filename, std::string path)
{
    if (filename.empty())
        filename = _json["runs"][0]["settings"]["steering"].get<std::string>() + " "
                   + std::to_string(_json["runs"][0]["environment"]["width"].get<unsigned int>()) + "x"
                   + std::to_string(_json["runs"][0]["environment"]["height"].get<unsigned int>()) + " "
                   + _json["runs"][0]["environment"]["generator"].get<std::string>() + " "
                   + std::to_string(_json["runs"][0]["environment"]["seed"].get<unsigned int>()) + " "
                   + _currentRun["globals"]["time"].get<std::string>()
                   + (std::string)".json";
    std::ofstream o(path + filename);
    o << std::setw(4) << _json << std::endl;
    OMPL_INFORM("Saved path statistics log file at %s.", (path + filename).c_str());
}

void Log::storeRun()
{
    _json["runs"].push_back(_currentRun);
}
