#pragma once

#include <chrono>


class Stopwatch
{
public:
    virtual void start()
    {
        _begin = std::chrono::steady_clock::now();
        time = 0;
    }

    virtual double stop()
    {
        auto _end = std::chrono::steady_clock::now();
        time += std::chrono::duration_cast<std::chrono::milliseconds>(_end - _begin).count() / 1000.;
    }

    virtual double pause()
    {
        auto _end = std::chrono::steady_clock::now();
        time += std::chrono::duration_cast<std::chrono::milliseconds>(_end - _begin).count() / 1000.;
    }

    virtual void resume()
    {
        _begin = std::chrono::steady_clock::now();
    }

    double time{0};

private:
    std::chrono::time_point<std::chrono::steady_clock> _begin;
};
