#ifndef PRIMITIVESHAPE_HPP
#define PRIMITIVESHAPE_HPP

#include <memory>

#include "vertexManager.hpp"
#include "elementManager.hpp"

void initPrimitives();

elementManager* getCubeshape();
elementManager* getPlaneshape();


#endif
