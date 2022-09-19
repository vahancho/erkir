/**********************************************************************************
*  MIT License                                                                    *
*                                                                                 *
*  Copyright (c) 2018-2020 Vahan Aghajanyan <vahancho@gmail.com>                  *
*                                                                                 *
*  Geodesy tools for conversions between (historical) datums                      *
*  (c) Chris Veness 2005-2019                                                     *
*  www.movable-type.co.uk/scripts/latlong-convert-coords.html                     *
*  www.movable-type.co.uk/scripts/geodesy-library.html#latlon-ellipsoidal-datum   *
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

#ifndef ELLIPSOIDAL_POINT_H
#define ELLIPSOIDAL_POINT_H

#include <memory>

#include "point.h"
#include "datum.h"


namespace erkir
{

namespace cartesian
{
  class Point;
}

namespace ellipsoidal
{

//! Implements geodetic point based on ellipsoidal earth model.
/*!
  Includes ellipsoid parameters and datums for different coordinate systems, and methods for
  converting between them and to Cartesian coordinates.
*/
class ERKIR_EXPORT Point : public erkir::Point
{
public:
  //! Constructs a point with the given \p latitude, \p longitude \p height above ellipsoid in metres and \p datum.
  Point(const Latitude &latitude, const Longitude &longitude, double height = 0.0,
        const Datum &datum = {Datum::Type::WGS84});

  /// Return the datum.
  Datum datum() const;

  /// Returns height above the ellipsoid.
  double height() const;

  /// Converts 'this' point's coordinate system to new one.
  /*!
    \param   toDatum Datum this coordinate is to be converted to.
    \returns Reference to this point converted to new datum.

    \example
      Point pWGS84(51.47788, -0.00147, Datum::Type::WGS84);
      auto pOSGB = pWGS84.toDatum(Datum::Type::OSGB36); // 51.4773°N, 000.0001°E
  */
  Point &toDatum(Datum::Type toDatum);

  /// Converts 'this' point from (geodetic) coordinates to (geocentric) Cartesian (x/y/z) coordinates.
  /*!
    \returns Cartesian point equivalent to lat/lon point, with x, y, z in metres from earth centre.
  */
  std::unique_ptr<cartesian::Point> toCartesianPoint();

  /*!
    Returns the distance between 'this' point and destination point along a geodesic on the
    surface of the ellipsoid, using Vincenty inverse solution.

    \param   point Latitude/longitude of destination point.
    \returns Distance in metres between points or NaN if failed to converge.
    \example
      auto p1 = Point(50.06632, -5.71475);
      auto p2 = Point(58.64402, -3.07009);
      auto d = p1.distanceTo(p2); // 969,954.166 m
  */
  double distanceTo(const Point &point) const;

  /*!
    Returns the destination point having travelled the given distance along a geodesic given by
    initial bearing from 'this' point, using Vincenty direct solution.

     \param   distance       Distance travelled along the geodesic in metres.
     \param   initialBearing Initial bearing in degrees from north.
     \returns Destination point.
     \example
      auto p1 = Point(-37.95103, 144.42487);
      auto p2 = p1.destinationPoint(54972.271, 306.86816); // 37.6528°S, 143.9265°E
  */
  Point destinationPoint(double distance, double initialBearing) const;

  /*!
    Returns the initial bearing (forward azimuth) to travel along a geodesic from ‘this’ point to
    the given point, using Vincenty inverse solution.

    \param   point   Latitude/longitude of destination point.
    \returns Initial bearing in degrees from north (0°..360°) or NaN if failed to converge.
    \example
      auto p1 = Point(50.06632, -5.71475);
      auto p2 = Point(58.64402, -3.07009);
      auto b1 = p1.initialBearingTo(p2); // 9.1419°
  */
  double initialBearingTo(const Point &point) const;

  /*!
    Returns the final bearing (reverse azimuth) having travelled along a geodesic from 'this'
    point to the given point, using Vincenty inverse solution.

    \param   point Latitude/longitude of destination point.
    \returns Final bearing in degrees from north (0°..360°) or NaN if failed to converge.

    \example
      auto p1 = Point(50.06632, -5.71475);
      auto p2 = Point(58.64402, -3.07009);
      auto b2 = p1.finalBearingTo(p2); // 11.2972°
  */
  double finalBearingTo(const Point &point) const;

  /*!
    Returns the final bearing (reverse azimuth) having travelled along a geodesic given by initial
    bearing for a given distance from 'this' point, using Vincenty direct solution.

    \param   distance       Distance travelled along the geodesic in metres.
    \param   initialBearing Initial bearing in degrees from north.
    \returns Final bearing in degrees from north (0°..360°).

    \example
      auto p1 = Point(-37.95103, 144.42487);
      auto b2 = p1.finalBearingOn(54972.271, 306.86816); // 307.1736°
  */
  double finalBearingOn(double distance, double initialBearing) const;

private:
  enum class DirectField
  {
    Point,
    FinalBearing
  };

  //!Vincenty direct calculation.
  /*!
    Ellipsoid parameters are taken from datum of 'this' point. Height is ignored.

    \param   distance Distance along bearing in metres.
    \param   initialBearing Initial bearing in degrees from north.
    \returns Object including point (destination point), finalBearing.
    \throws  Formula failed to converge.
  */
  std::tuple<Point, double> direct(double distance, double initialBearing) const;

  enum class InverseField
  {
    Distance,
    InitialBearing,
    FinalBearing
  };

  //! Vincenty inverse calculation.
  /*!
    Ellipsoid parameters are taken from datum of 'this' point. Height is ignored.

    \param   point Latitude/longitude of destination point.
    \returns Object including distance, initialBearing, finalBearing.
    \throws  Invalid point.
    \throws  Points must be on surface of ellipsoid.
    \throws  Formula failed to converge.
  */
  std::tuple<double, double, double> inverse(const Point &point) const;

  double m_height{ 0.0 };
  Datum m_datum;
};

} // ellipsoidal

} // erkir

#endif // ELLIPSOIDAL_POINT_H

