#include "cell.h"


Cell::Cell(double x, double y, double w, double h, bool occupied)
        : x(x), y(y), width(w*1.7), height(h*1.7), occupied(occupied) // TODO cell size changed
{
    hasAgent = false;
}

Cell::~Cell()
{
    // clean up
}
