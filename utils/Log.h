#include <chrono>

#include "base/PathStatistics.hpp"

#include "json.hpp"


class Log
{
public:
    static void instantiateRun();

    static void log(const PathStatistics &stats);

    static void storeRun();

    static void save(std::string filename = "",
                     std::string path = "log/");

private:
    static nlohmann::json _json;
    static nlohmann::json _currentRun;

};
