#pragma once

#include <chrono>


class Stopwatch
{
public:
    typedef std::chrono::high_resolution_clock clock_t;
    virtual void start()
    {
        _begin = clock_t::now();
        time = 0;
    }

    virtual double stop()
    {
        auto _end = clock_t::now();
        time += std::chrono::duration_cast<std::chrono::milliseconds>(_end - _begin).count() / 1e3;
        return time;
    }

    virtual double pause()
    {
        auto _end = clock_t::now();
        time += std::chrono::duration_cast<std::chrono::milliseconds>(_end - _begin).count() / 1e3;
        return time;
    }

    virtual void resume()
    {
        _begin = clock_t::now();
    }

    double time{0};

private:
    std::chrono::time_point<clock_t> _begin;
};
