#define _USE_MATH_DEFINES

#include <math.h>
#include <stdexcept>

#include "coordinate.h"

constexpr double radiansInDegree = M_PI / 180.0;

namespace geodesy
{

Coordinate::Coordinate(double degrees)
  :
    m_degrees(degrees)
{}

double Coordinate::degrees() const
{
  return m_degrees;
}

double Coordinate::radians() const
{
  return m_degrees * radiansInDegree;
}

////////////////////////////////////////////////////////////////////////////////

Latitude::Latitude(double degree)
  :
    Coordinate(degree)
{
  if (degree > 90.0 || degree < -90.0)
  {
    throw std::out_of_range("Latitude measurements range from 0° to (+/–)90°.");
  }
}

////////////////////////////////////////////////////////////////////////////////

Longitude::Longitude(double degree)
  :
    Coordinate(degree)
{
  if (degree > 180.0 || degree < -180.0)
  {
    throw std::out_of_range("Longitude measurements range from 0° to (+/–)180°.");
  }
}

}

