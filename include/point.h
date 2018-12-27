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

  /// Returns the distance from this point to destination point(using haversine formula).
  /*!
    This function uses calculations on the basis of a spherical earth
    (ignoring ellipsoidal effects) – which is accurate enough for most purposes.

    \param point Latitude / longitude of destination point.
    \param radius (Mean)radius of earth(defaults to radius in 6371.0 metres).
    \returns Distance between this point and destination point, in same units as radius.

    @example
      Point p1{ 52.205, 0.119 };
      Point p2{ 48.857, 2.351 };
      auto d = p1.sphericalDistanceTo(p2); // 404.3 km
  */
  double sphericalDistanceTo(const Point &point, double radius = 6371.0) const;

  /*!
    Returns the(initial) bearing from 'this' point to destination point.

    \param point Latitude / longitude of destination point.
    \returns Initial bearing in degrees from north.

    @example
      Point p1{ 52.205, 0.119 };
      Point p2{ 48.857, 2.351 };
      auto b1 = p1.sphericalBearingTo(p2); // 156.2°
  */
  double sphericalBearingTo(const Point &point) const;

  /*!
    Returns final bearing arriving at destination destination point from 'this' point; the final bearing
    will differ from the initial bearing by varying degrees according to distance and latitude.

    \param   point - Latitude/longitude of destination point.
    \returns Final bearing in degrees from north.

    \example
      Point p1{52.205, 0.119};
      Point p2{48.857, 2.351};
      auto b2 = p1.sphericalFinalBearingTo(p2); // 157.9°
  */
  double sphericalFinalBearingTo(const Point &point) const;

  /*!
    Returns the midpoint between 'this' point and the supplied point.

    \param   point - Latitude/longitude of destination point.
    \returns Midpoint between this point and the supplied point.

    \example
      Point p1{52.205, 0.119};
      Point p2{48.857, 2.351};
      auto pMid = p1.sphericalMidpointTo(p2); // 50.5363°N, 001.2746°E
  */
  Point sphericalMidpointTo(const Point &point) const;

private:
  Latitude m_latitude;
  Longitude m_longitude;
};

}

#endif // POINT_H

