#include <chrono>

#include "base/PathStatistics.hpp"
#include "base/PlannerSettings.h"

#include "json.hpp"
#include "PostSmoothing.h"


class Log
{
public:
    static void instantiateRun();

    static void log(const PathStatistics &stats);

    static void storeRun();

    static void save(std::string filename = "",
                     std::string path = "/home/wal/catkin_ws/src/thetastar_2PBVP/thetastar_2PBVP/log/");

private:
    static nlohmann::json _json;
    static nlohmann::json _currentRun;

};
