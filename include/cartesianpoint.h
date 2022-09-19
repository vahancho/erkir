/**********************************************************************************
*  MIT License                                                                    *
*                                                                                 *
*  Copyright (c) 2020 Vahan Aghajanyan <vahancho@gmail.com>                       *
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

#ifndef CARTESIAN_POINT_H
#define CARTESIAN_POINT_H

#include "datum.h"
#include "vector3d.h"

#include <memory>

namespace erkir
{

namespace ellipsoidal
{
  class Point;
}

namespace cartesian
{

/// ECEF (earth-centered earth-fixed) geocentric Cartesian coordinates.
class ERKIR_EXPORT Point : public Vector3d
{
public:
  /// Creates Cartesian coordinate representing ECEF (earth-centric earth-fixed) point.
  /*!
    \param x X coordinate in metres (=> 0°N,0°E).
    \param y Y coordinate in metres (=> 0°N,90°E).
    \param z Z coordinate in metres (=> 90°N).

    \example auto coord = Cartesian(3980581.210, -111.159, 4966824.522);
  */
  Point(double x, double y, double z, const ellipsoidal::Datum &datum = {ellipsoidal::Datum::Type::WGS84});

  /// Converts 'this' (geocentric) cartesian (x/y/z) coordinate to (geodetic) latitude/longitude
  /// point( based on the same datum, or WGS84 if unset ).
  /*!
    \returns {LatLon} Latitude/longitude point defined by cartesian coordinates.

    \example
      auto c = cartesian::Point{4027893.924, 307041.993, 4919474.294};
      auto p = c.toGeoPoint(); // 50.7978°N, 004.3592°E
  */
  std::unique_ptr<ellipsoidal::Point> toGeoPoint() const;

  /// Converts this point to the \p targetDatum.
  Point &toDatum(ellipsoidal::Datum::Type targetDatum);

private:
  ellipsoidal::Datum m_datum;
 };

} // cartesian

} // erkir

#endif // CARTESIAN_POINT_H

