#pragma once
#include <cstdlib>
#include <iostream>

class Cell
{
public:
    //TODO: use QRectF?
    Cell(double x, double y, double w, double h, bool occupied = false);

    const double x;
    const double y;
    const double width;
    const double height;

    virtual ~Cell();

    bool hasAgent;
    bool occupied;
    double value;
};