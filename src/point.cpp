#include "point.h"

namespace geodesy
{

Point::Point(const Latitude &latitude, const Longitude &longitude)
  :
    m_latitude(latitude),
    m_longitude(longitude)
{
}

const Latitude &Point::latitude() const
{
  return m_latitude;
}

const Longitude &Point::longitude() const
{
  return m_longitude;
}

}

