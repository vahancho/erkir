#define _USE_MATH_DEFINES

#include <math.h>

#include "coordinate.h"

constexpr double radiansInDegree = M_PI / 180.0;

namespace geodesy
{

Coordinate::Coordinate(double degree)
  :
    m_degree(degree)
{}

double Coordinate::degree() const
{
  return m_degree;
}

double Coordinate::radian() const
{
  return m_degree * radiansInDegree;
}

}

