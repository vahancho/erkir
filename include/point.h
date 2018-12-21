#ifndef POINT_H
#define POINT_H

#include "coordinate.h"

namespace geodesy
{

class Point
{
public:
  Point(const Latitude &latitude, const Longitude &longitude);

  const Latitude &latitude() const;
  const Longitude &longitude() const;

private:
  Latitude m_latitude;
  Longitude m_longitude;
};

}

#endif // POINT_H
