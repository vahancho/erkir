// clang-format off
/**
 * MIT License
 *
 * Copyright (c) 2018-2023 Vahan Aghajanyan
 * Copyright (c) 2002-2018 Chris Veness (Latitude/Longitude spherical geodesy tools | https://www.movable-type.co.uk/scripts/latlong.html)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/
// clang-format on

#ifndef ERKIR_SPHERICALPOINT_H_
#define ERKIR_SPHERICALPOINT_H_

#include <vector>

#include "erkir/point.h"

namespace erkir {
namespace spherical {

/**
 * @brief Implements the geographical point.
 * All formulae in this class are for calculations on the basis of a spherical
 * earth (ignoring ellipsoidal effects).
 */
class ERKIR_EXPORT Point : public erkir::Point {
 public:
  /**
   * @brief Constructs an invalid point object.
   */
  Point();

  /**
   * @brief Constructs a point with the given @p latitude and @p longitude.
   *
   * @param latitude Latitude.
   * @param longitude Longitude.
   */
  Point(const Latitude &latitude, const Longitude &longitude);

  /**
   * @brief Returns the distance from this point to destination point(using
   * haversine formula).
   * This function uses calculations on the basis of a spherical earth (ignoring
   * ellipsoidal effects) – which is accurate enough for most purposes.
   * It uses the 'haversine' formula to calculate the great-circle distance
   * between two points – that is, the shortest distance over the earth's
   * surface – giving an 'as-the-crow-flies' distance between the points.
   *
   * @param point Latitude/longitude of destination point.
   * @param radius (Mean)radius of earth(defaults to radius in 6371e3 metres).
   * @return Distance between this point and destination point, in same units as
   * radius.
   *
   * @example
   * erkir::spherical::Point p1{ 52.205, 0.119 };
   * erkir::spherical::Point p2{ 48.857, 2.351 };
   * auto d = p1.distanceTo(p2); // 404.3 km
   */
  double distanceTo(const Point &point, double radius = 6371e3) const;

  /**
   * @brief Returns the (initial) bearing from 'this' point to destination
   * point.
   *
   * @param point Latitude/longitude of destination point.
   * @return Initial bearing in degrees from north.
   *
   * @example
   * erkir::spherical::Point p1{ 52.205, 0.119 };
   * erkir::spherical::Point p2{ 48.857, 2.351 };
   * auto b1 = p1.bearingTo(p2); // 156.2°
   */
  double bearingTo(const Point &point) const;

  /**
   * @brief Returns final bearing arriving at destination point from this point.
   *
   * @param point Latitude/longitude of destination point.
   * @return Final bearing in degrees from north.
   *
   * @example
   * erkir::spherical::Point p1{52.205, 0.119};
   * erkir::spherical::Point p2{48.857, 2.351};
   * auto b2 = p1.finalBearingTo(p2); // 157.9°
   */
  double finalBearingTo(const Point &point) const;

  /**
   * @brief Returns the midpoint between this point and the supplied point.
   *
   * @param point Latitude/longitude of destination point.
   * @return Midpoint between this point and the supplied point.
   *
   * @example
   * erkir::spherical::Point p1{52.205, 0.119};
   * erkir::spherical::Point p2{48.857, 2.351};
   * auto pMid = p1.midpointTo(p2); // 50.5363°N, 001.2746°E
   */
  Point midpointTo(const Point &point) const;

  /**
   * @brief  Returns the point at given fraction between this point and
   * specified point.
   *
   * @param point Latitude/longitude of destination point.
   * @param fraction Fraction between the two points (0 = this point, 1.0 =
   * specified point).
   * @return Intermediate point between this point and destination point.
   *
   * @param
   * erkir::spherical::Point p1{52.205, 0.119};
   * erkir::spherical::Point p2{48.857, 2.351};
   * auto pMid = p1.intermediatePointTo(p2, 0.25); // 51.3721°N, 000.7073°E
   */
  Point intermediatePointTo(const Point &point, double fraction) const;

  /**
   * @brief Returns the destination point from this point.
   * Returns the destination point from 'this' point having travelled the given
   * distance on the given initial bearing (bearing normally autoies around path
   * followed).
   *
   * @param distance Distance travelled, in same units as earth radius (default:
   * metres).
   * @param bearing Initial bearing in degrees from north.
   * @param radius (Mean) radius of earth (defaults to radius in 6371e3 metres).
   * @return Destination point.
   *
   * @example
   * erkir::spherical::Point p1{51.4778, -0.0015};
   * erkir::spherical::Point p2 = p1.destinationPoint(7794, 300.7);
   * // 51.5135°N, 000.0983°W
   */
  Point destinationPoint(double distance, double bearing,
                         double radius = 6371e3) const;

  /**
   * @brief Returns the point of intersection of two paths defined by point and
   * bearing.
   *
   * @param p1 First point.
   * @param brng1 Initial bearing from first point in degrees.
   * @param p2 Second point.
   * @param brng2 Initial bearing from second point in degrees.
   * @return Destination point (an invalid point if no unique intersection
   * defined).
   *
   * @example
   * erkir::spherical::Point p1{51.8853, 0.2545}
   * auto brng1 = 108.547;
   * erkir::spherical::Point p2{49.0034, 2.5735}
   * auto brng2 = 32.435;
   * auto pInt = intersection(p1, brng1, p2, brng2); // 50.9078°N, 004.5084°E
   */
  static Point intersection(const Point &p1, double brng1, const Point &p2,
                            double brng2);

  /**
   * @brief Returns (signed) distance from this point to great circle defined by
   * start - point and end - point.
   *
   * @param pathStart Start point of great circle path.
   * @param pathEnd End point of great circle path.
   * @param radius (Mean) radius of earth (defaults to radius in 6371e3 metres).
   * @return Distance to great circle (negative if to left, positive if to right
   * of path).
   *
   * @example
   * erkir::spherical::Point pCurrent{53.2611, -0.7972};
   * erkir::spherical::Point p1{53.3206, -1.7297};
   * erkir::spherical::Point p2{53.1887,  0.1334};
   * auto d = pCurrent.crossTrackDistanceTo(p1, p2);  // -307.5 m
   */
  double crossTrackDistanceTo(const Point &pathStart, const Point &pathEnd,
                              double radius = 6371e3) const;

  /**
   * @brief Returns how far this point is along a path from start-point, heading
   * towards end-point. That is, if a perpendicular is drawn from this point to
   * the (great circle) path, the along-track distance is the distance from the
   * start point to where the perpendicular crosses the path.
   *
   * @param pathStart Start point of great circle path.
   * @param pathEnd End point of great circle path.
   * @param radius (Mean) radius of earth (defaults to radius in 6371e3 meters).
   * @return Distance along great circle to point nearest this point.
   *
   * @example
   * erkir::spherical::Point pCurrent{53.2611, -0.7972};
   * erkir::spherical::Point p1{53.3206, -1.7297};
   * erkir::spherical::Point p2{53.1887,  0.1334};
   * auto d = pCurrent.alongTrackDistanceTo(p1, p2);  // 62.331 km
   */
  double alongTrackDistanceTo(const Point &pathStart, const Point &pathEnd,
                              double radius = 6371e3) const;

  /**
   * @brief Returns maximum latitude reached when travelling on a great circle.
   * Returns maximum latitude reached when travelling on a great circle on given
   * bearing from this point ('Clairaut's formula'). Negate the result for the
   * minimum latitude (in the Southern hemisphere).
   *
   * The maximum latitude is independent of longitude; it will be the same for
   * all points on a given latitude.
   *
   * @param bearing Initial bearing.
   * @return Maximum latitude.
   */
  double maxLatitude(double bearing) const;

  /**
   * @brief Returns the distance travelling from this point to destination point
   * along a rhumb line. A 'rhumb line' (or loxodrome) is a path of constant
   * bearing, which crosses all meridians at the same angle. Sailors used to
   * (and sometimes still) navigate along rhumb lines since it is easier to
   * follow a constant compass bearing than to be continually adjusting the
   * bearing, as is needed to follow a great circle. Rhumb lines are straight
   * lines on a Mercator Projec­tion map (also helpful for naviga­tion). Rhumb
   * lines are generally longer than great-circle (orthodrome) routes.
   *
   * @param point Latitude/longitude of destination point.
   * @param radius (Mean) radius of earth (defaults to radius in 6371e3 meters).
   * @return Distance between this point and destination point (same units as
   * radius).
   *
   * @example
   * erkir::spherical::Point p1{51.127, 1.338};
   * erkir::spherical::Point p2{50.964, 1.853};
   * auto d = p1.rhumbDistanceTo(p2); // 40.31 km
   */
  double rhumbDistanceTo(const Point &point, double radius = 6371e3) const;

  /**
   * @brief Returns the bearing from this point to destination point along a
   * rhumb line.
   *
   * @param point Latitude/longitude of destination point.
   * @return Bearing in degrees from north.
   *
   * @example
   * erkir::spherical::Point p1{51.127, 1.338};
   * erkir::spherical::Point p2{50.964, 1.853};
   * auto d = p1.rhumbBearingTo(p2); // 116.7
   */
  double rhumbBearingTo(const Point &point) const;

  /**
   * @brief Returns the destination point having travelled along a rhumb line
   * from this point the given distance on the given bearing.
   *
   * @param distance Distance travelled, in same units as earth radius (default:
   * metres).
   * @param bearing Bearing in degrees from north.
   * @param radius (Mean)radius of earth(defaults to radius in 6371e3 metres).
   * @return Destination point.
   *
   * @example
   * erkir::spherical::Point p1{51.127, 1.338};
   * auto p2 = p1.rhumbDestinationPoint(40300, 116.7); // 50.9642°N, 001.8530°E
   */
  Point rhumbDestinationPoint(double distance, double bearing,
                              double radius = 6371e3) const;

  /**
   * @brief Returns the loxodromic midpoint (along a rhumb line) between this
   * point and second point.
   *
   * @param point Latitude/longitude of second point.
   * @return Midpoint between this point and second point.
   *
   * @example
   * erkir::spherical::Point p1{51.127, 1.338};
   * erkir::spherical::Point p2{50.964, 1.853};
   * auto pMid = p1.rhumbMidpointTo(p2); // 51.0455°N, 001.5957°E
   */
  Point rhumbMidpointTo(const Point &point) const;

  /**
   * @brief Calculates the area of a spherical polygon where the sides of the
   * polygon are great circle arcs joining the vertices.
   *
   * @param polygon Array of points defining vertices of the polygon.
   * @param radius (Mean)radius of earth(defaults to radius in 6371e3 metres).
   * @return Area of the polygon, in the same units as radius.
   *
   * @example
   * std::vector<erkir::spherical::Point> polygon = {{0,0}, {1,0}, {0,1}};
   * auto area = erkir::spherical::Point::areaOf(polygon); // 6.18e9 mІ
   */
  static double areaOf(const std::vector<Point> &polygon,
                       double radius = 6371e3);
};

}  // namespace spherical
}  // namespace erkir

#endif  // ERKIR_SPHERICALPOINT_H_
