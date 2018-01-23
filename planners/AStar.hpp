#pragma once

#include "ThetaStar.h"

class AStar : public ThetaStar
{
public:
    AStar() : ThetaStar(true, "A*")
    {}
};
