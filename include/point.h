/**********************************************************************************
*  MIT License                                                                    *
*                                                                                 *
*  Copyright (c) 2018 Vahan Aghajanyan <vahancho@gmail.com>                       *
*                                                                                 *
*  Permission is hereby granted, free of charge, to any person obtaining a copy   *
*  of this software and associated documentation files (the "Software"), to deal  *
*  in the Software without restriction, including without limitation the rights   *
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell      *
*  copies of the Software, and to permit persons to whom the Software is          *
*  furnished to do so, subject to the following conditions:                       *
*                                                                                 *
*  The above copyright notice and this permission notice shall be included in all *
*  copies or substantial portions of the Software.                                *
*                                                                                 *
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR     *
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,       *
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE    *
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER         *
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,  *
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE  *
*  SOFTWARE.                                                                      *
***********************************************************************************/

#ifndef POINT_H
#define POINT_H

#include "coordinate.h"

namespace geodesy
{

//! Implements the geographical point.
class Point
{
public:
  //! Constructs a point with the given \p latitude and \p longitude.
  Point(const Latitude &latitude, const Longitude &longitude);

  //! Returns the latitude of this point.
  const Latitude &latitude() const;

  //! Returns the longitude of this point.
  const Longitude &longitude() const;

  /// Returns the distance from this point to destination point(using haversine formula).
  /*!
    This function uses calculations on the basis of a spherical earth
    (ignoring ellipsoidal effects) – which is accurate enough for most purposes.

    \param point Latitude / longitude of destination point.
    \param radius (Mean)radius of earth(defaults to radius in 6371e3 metres).
    \returns Distance between this point and destination point, in same units as radius.

    @example
      Point p1{ 52.205, 0.119 };
      Point p2{ 48.857, 2.351 };
      auto d = p1.sphericalDistanceTo(p2); // 404.3 km
  */
  double sphericalDistanceTo(const Point &point, double radius = 6371e3) const;

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

  /*!
    Returns the point at given fraction between 'this' point and specified point.

   \param   point Latitude/longitude of destination point.
   \param   fraction Fraction between the two points (0 = this point, 1.0 = specified point).
   \returns Intermediate point between this point and destination point.

   \example
    Point p1{52.205, 0.119};
    Point p2{48.857, 2.351};
    auto pMid = p1.sphericalIntermediatePointTo(p2, 0.25); // 51.3721°N, 000.7073°E
  */
  Point sphericalIntermediatePointTo(const Point &point, double fraction) const;

  /*!
    Returns the destination point from ‘this’ point having travelled the given distance on the
    given initial bearing (bearing normally varies around path followed).

    \param   distance Distance travelled, in same units as earth radius (default: metres).
    \param   bearing Initial bearing in degrees from north.
    \param   radius (Mean) radius of earth (defaults to radius in 6371e3 metres).
    \returns {LatLon} Destination point.

    \example
      Point p1{51.4778, -0.0015};
      Point p2 = p1.sphericalDestinationPoint(7794, 300.7); // 51.5135°N, 000.0983°W
  */
  Point sphericalDestinationPoint(double distance, double bearing, double radius = 6371e3) const;

private:
  Latitude m_latitude;
  Longitude m_longitude;
};

}

#endif // POINT_H

