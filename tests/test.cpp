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

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "erkir/erkir.h"

#define ERKIR_TESTS_EPSILON (0.001)

//
// Coordinate
//

TEST(Coordinate, Wrap360) {
  ASSERT_NEAR(359.0, erkir::Coordinate::wrap360(-1.0), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(1.0, erkir::Coordinate::wrap360(361.0), ERKIR_TESTS_EPSILON);
}

//
// Point
//

TEST(Point, Empty) {
  const erkir::Point p;
  ASSERT_FALSE(p.isValid());
  ASSERT_NEAR(0.0, p.latitude().degrees(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(0.0, p.longitude().degrees(), ERKIR_TESTS_EPSILON);
}

TEST(Point, NonEmpty) {
  const erkir::Point p(52.205, 0.119);
  ASSERT_TRUE(p.isValid());
  ASSERT_NEAR(52.205, p.latitude().degrees(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(0.119, p.longitude().degrees(), ERKIR_TESTS_EPSILON);
}

//
// spherical::Point
//

TEST(SphericalPoint, Empty) {
  const erkir::spherical::Point p;
  ASSERT_FALSE(p.isValid());
  ASSERT_NEAR(0.0, p.latitude().degrees(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(0.0, p.longitude().degrees(), ERKIR_TESTS_EPSILON);
}

TEST(SphericalPoint, NonEmpty) {
  const erkir::spherical::Point p(52.205, 0.119);
  ASSERT_TRUE(p.isValid());
  ASSERT_NEAR(52.205, p.latitude().degrees(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(0.119, p.longitude().degrees(), ERKIR_TESTS_EPSILON);
}

TEST(SphericalPoint, Distance) {
  const erkir::spherical::Point p1(52.205, 0.119);
  const erkir::spherical::Point p2(48.857, 2.351);
  ASSERT_TRUE(p1.isValid());
  ASSERT_TRUE(p2.isValid());
  ASSERT_NEAR(404279.164, p1.distanceTo(p2), ERKIR_TESTS_EPSILON);  // 404.3 km
}

TEST(SphericalPoint, Bearing) {
  const erkir::spherical::Point p1(52.205, 0.119);
  const erkir::spherical::Point p2(48.857, 2.351);
  ASSERT_TRUE(p1.isValid());
  ASSERT_TRUE(p2.isValid());
  ASSERT_NEAR(156.167, p1.bearingTo(p2), ERKIR_TESTS_EPSILON);  // 156.2°
}

TEST(SphericalPoint, FinalBearing) {
  const erkir::spherical::Point p1(52.205, 0.119);
  const erkir::spherical::Point p2(48.857, 2.351);
  ASSERT_TRUE(p1.isValid());
  ASSERT_TRUE(p2.isValid());
  ASSERT_NEAR(157.890, p1.finalBearingTo(p2), ERKIR_TESTS_EPSILON);  // 157.9°
}

TEST(SphericalPoint, MidPoint) {
  const erkir::spherical::Point p1(52.205, 0.119);
  const erkir::spherical::Point p2(48.857, 2.351);
  const erkir::spherical::Point mp =
      p1.midpointTo(p2);  // 50.5363°N, 001.2746°E
  ASSERT_TRUE(p1.isValid());
  ASSERT_TRUE(p2.isValid());
  ASSERT_TRUE(mp.isValid());
  ASSERT_EQ(erkir::spherical::Point(50.5363, 1.2746), mp);
}

TEST(SphericalPoint, IntermediatePoint) {
  const erkir::spherical::Point p1(52.205, 0.119);
  const erkir::spherical::Point p2(48.857, 2.351);
  const erkir::spherical::Point ip =
      p1.intermediatePointTo(p2, 0.25);  // 51.3721°N, 000.7073°E
  ASSERT_TRUE(p1.isValid());
  ASSERT_TRUE(p2.isValid());
  ASSERT_TRUE(ip.isValid());
  ASSERT_EQ(erkir::spherical::Point(51.3721, 0.7073), ip);
}

TEST(SphericalPoint, DestinationPoint) {
  const erkir::spherical::Point p(51.4778, -0.0015);
  const erkir::spherical::Point dp =
      p.destinationPoint(7794.0, 300.7);  // 51.5135°N, 000.0983°W
  ASSERT_TRUE(p.isValid());
  ASSERT_TRUE(dp.isValid());
  ASSERT_EQ(erkir::spherical::Point(51.5135, -0.0983), dp);
}

TEST(SphericalPoint, Intersection) {
  const erkir::spherical::Point p = erkir::spherical::Point::intersection(
      {51.8853, 0.2545}, 108.547, {49.0034, 2.5735},
      32.435);  // 50.9078°N, 004.5084°
  ASSERT_TRUE(p.isValid());
  ASSERT_EQ(erkir::spherical::Point(50.9078, 4.5084), p);
}

TEST(SphericalPoint, CrossTrackDistance) {
  const erkir::spherical::Point p(53.2611, -0.7972);
  const double distance = p.crossTrackDistanceTo(
      {53.3206, -1.7297}, {53.1887, 0.1334});  // -307.5 m
  ASSERT_TRUE(p.isValid());
  ASSERT_NEAR(-307.550, distance, ERKIR_TESTS_EPSILON);
}

TEST(SphericalPoint, AlongTrackDistance) {
  const erkir::spherical::Point p1(53.2611, -0.7972);
  const erkir::spherical::Point p2(53.2611, -0.7972);
  ASSERT_TRUE(p1.isValid());
  ASSERT_TRUE(p2.isValid());
  ASSERT_NEAR(62331.493,
              p1.alongTrackDistanceTo({53.3206, -1.7297}, {53.1887, 0.1334}),
              ERKIR_TESTS_EPSILON);  // 62.331 km
  ASSERT_NEAR(0.0,
              p2.alongTrackDistanceTo({53.2611, -0.7972}, {53.1887, 0.1334}),
              ERKIR_TESTS_EPSILON);  // 0.0 km
}

TEST(SphericalPoint, RhumbDistance) {
  const erkir::spherical::Point p1(51.127, 1.338);
  const erkir::spherical::Point p2(50.964, 1.853);
  ASSERT_TRUE(p1.isValid());
  ASSERT_TRUE(p2.isValid());
  ASSERT_NEAR(40307.745, p1.rhumbDistanceTo(p2),
              ERKIR_TESTS_EPSILON);  // 40.31 km
}

TEST(SphericalPoint, RhumbBearing) {
  const erkir::spherical::Point p1(51.127, 1.338);
  const erkir::spherical::Point p2(50.964, 1.853);
  ASSERT_TRUE(p1.isValid());
  ASSERT_TRUE(p2.isValid());
  ASSERT_NEAR(116.722, p1.rhumbBearingTo(p2), ERKIR_TESTS_EPSILON);  // 116.7°
}

TEST(SphericalPoint, RhumbDestinationPoint) {
  const erkir::spherical::Point p(51.127, 1.338);
  const erkir::spherical::Point dp =
      p.rhumbDestinationPoint(40300, 116.7);  // 50.9642°N, 001.8530°E
  ASSERT_TRUE(p.isValid());
  ASSERT_TRUE(dp.isValid());
  ASSERT_EQ(erkir::spherical::Point(50.9642, 001.8530), dp);
}

TEST(SphericalPoint, RhumbMidPoint) {
  const erkir::spherical::Point p1(51.127, 1.338);
  const erkir::spherical::Point p2(50.964, 1.853);
  const erkir::spherical::Point mp =
      p1.rhumbMidpointTo(p2);  // 51.0455°N, 001.5957°E
  ASSERT_TRUE(p1.isValid());
  ASSERT_TRUE(p2.isValid());
  ASSERT_TRUE(mp.isValid());
  ASSERT_EQ(erkir::spherical::Point(51.0455, 001.5957), mp);
}

TEST(SphericalPoint, Area) {
  const std::vector<erkir::spherical::Point> polygon = {{0, 0}, {1, 0}, {0, 1}};
  for (const erkir::spherical::Point& p : polygon) {
    ASSERT_TRUE(p.isValid());
  }
  ASSERT_NEAR(6182469722.731, erkir::spherical::Point::areaOf(polygon),
              ERKIR_TESTS_EPSILON);  // 6.18e9 mІ
}

//
// ellipsoidal::Point
//

TEST(EllipsoidalPoint, NonEmpty) {
  const erkir::ellipsoidal::Point p(-37.95103, 144.42487);
  ASSERT_TRUE(p.isValid());
  ASSERT_NEAR(-37.95103, p.latitude().degrees(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(144.42487, p.longitude().degrees(), ERKIR_TESTS_EPSILON);
}

TEST(EllipsoidalPoint, Distance) {
  const erkir::ellipsoidal::Point p1(50.06632, -5.71475);
  const erkir::ellipsoidal::Point p2(58.64402, -3.07009);
  const double distance = p1.distanceTo(p2);  // 969,954.166 m
  ASSERT_TRUE(p1.isValid());
  ASSERT_TRUE(p2.isValid());
  ASSERT_NEAR(969954.169, distance, ERKIR_TESTS_EPSILON);
}

TEST(EllipsoidalPoint, InitialBearing) {
  const erkir::ellipsoidal::Point p1(50.06632, -5.71475);
  const erkir::ellipsoidal::Point p2(58.64402, -3.07009);
  ASSERT_TRUE(p1.isValid());
  ASSERT_TRUE(p2.isValid());
  ASSERT_NEAR(9.1419, p1.initialBearingTo(p2), ERKIR_TESTS_EPSILON);  // 9.1419°
}

TEST(EllipsoidalPoint, FinalBearingTo) {
  const erkir::ellipsoidal::Point p1(50.06632, -5.71475);
  const erkir::ellipsoidal::Point p2(58.64402, -3.07009);
  ASSERT_TRUE(p1.isValid());
  ASSERT_TRUE(p2.isValid());
  ASSERT_NEAR(11.2972, p1.finalBearingTo(p2), ERKIR_TESTS_EPSILON);  // 11.2972°
}

TEST(EllipsoidalPoint, FinalBearingOn) {
  const erkir::ellipsoidal::Point p(-37.95103, 144.42487);
  ASSERT_TRUE(p.isValid());
  ASSERT_NEAR(307.1736, p.finalBearingOn(54972.271, 306.86816),
              ERKIR_TESTS_EPSILON);  // 307.1736°
}

TEST(EllipsoidalPoint, DestinationPoint) {
  const erkir::ellipsoidal::Point p(-37.95103, 144.42487);
  const erkir::ellipsoidal::Point dp =
      p.destinationPoint(54972.271, 306.86816);  // 37.6528°S, 143.9265°E
  ASSERT_TRUE(p.isValid());
  ASSERT_TRUE(dp.isValid());
  ASSERT_EQ(erkir::ellipsoidal::Point(-37.6528, 143.9265), dp);
}

TEST(EllipsoidalPoint, Antipodal) {
  const double circMeridional = 40007862.918;
  // TODO(vahancho) Diverges from other results a little bit
  ASSERT_NEAR(19936287.420,
              erkir::ellipsoidal::Point(0.0, 0.0).distanceTo({0.5, 179.5}),
              ERKIR_TESTS_EPSILON);
  ASSERT_TRUE(
      std::isnan(erkir::ellipsoidal::Point(0.0, 0.0).distanceTo({0.5, 179.7})));
  ASSERT_TRUE(std::isnan(
      erkir::ellipsoidal::Point(0.0, 0.0).initialBearingTo({0.5, 179.7})));
  ASSERT_TRUE(std::isnan(
      erkir::ellipsoidal::Point(0.0, 0.0).finalBearingTo({0.5, 179.7})));
  ASSERT_NEAR(circMeridional / 2.0,
              erkir::ellipsoidal::Point(0.0, 0.0).distanceTo({0.0, 180.0}),
              ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(
      0.0, erkir::ellipsoidal::Point(0.0, 0.0).initialBearingTo({0.0, 180.0}),
      ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(circMeridional / 2.0,
              erkir::ellipsoidal::Point(90.0, 0.0).distanceTo({-90.0, 0.0}),
              ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(
      0.0, erkir::ellipsoidal::Point(90.0, 0.0).initialBearingTo({-90.0, 0.0}),
      ERKIR_TESTS_EPSILON);
}

TEST(EllipsoidalPoint, Coincident) {
  const erkir::ellipsoidal::Point p(50.06632, -5.71475);
  ASSERT_TRUE(p.isValid());
  ASSERT_NEAR(0.0, p.distanceTo(p), ERKIR_TESTS_EPSILON);
  ASSERT_TRUE(std::isnan(p.initialBearingTo(p)));
  ASSERT_TRUE(std::isnan(p.finalBearingTo(p)));
  // Inverse equatorial distance
  ASSERT_NEAR(111319.491,
              erkir::ellipsoidal::Point(0.0, 0.0).distanceTo({0.0, 1.0}),
              ERKIR_TESTS_EPSILON);
  // Direct coincident destination
  ASSERT_EQ(p.destinationPoint(0.0, 0.0), p);
}

TEST(EllipsoidalPoint, Datum) {
  erkir::ellipsoidal::Point greenwichWGS84(
      51.47788, -0.00147, 0.0, erkir::ellipsoidal::Datum::Type::WGS84);
  erkir::ellipsoidal::Point& greenwichOSGB36 = greenwichWGS84.toDatum(
      erkir::ellipsoidal::Datum::Type::OSGB36);  // 51.4773°N, 000.0000°E
  ASSERT_TRUE(greenwichWGS84.isValid());
  ASSERT_TRUE(greenwichOSGB36.isValid());
  // TODO(vahancho) Should be 0°E? Out by c. 10 metres / 0.5"
  // TODO(vahancho) This result corresponds to Chris Veness' tests
  ASSERT_EQ(erkir::ellipsoidal::Point(51.4773, 0.0001), greenwichOSGB36);
  greenwichWGS84 =
      greenwichOSGB36.toDatum(erkir::ellipsoidal::Datum::Type::WGS84);
  ASSERT_TRUE(greenwichWGS84.isValid());
  ASSERT_EQ(erkir::ellipsoidal::Point(51.478, 0.0001), greenwichWGS84);
}

TEST(EllipsoidalPoint, DatumFromWGS84ToOSGB36) {
  erkir::ellipsoidal::Point pWGS84(53.0, 1.0, 50.0);
  const erkir::ellipsoidal::Point& pOSGB36 =
      pWGS84.toDatum(erkir::ellipsoidal::Datum::Type::OSGB36);
  ASSERT_TRUE(pWGS84.isValid());
  ASSERT_TRUE(pOSGB36.isValid());
  ASSERT_EQ(erkir::ellipsoidal::Point(52.9996, 1.0018), pOSGB36);
  ASSERT_NEAR(3.987, pOSGB36.height(), ERKIR_TESTS_EPSILON);
}

TEST(EllipsoidalPoint, DatumFromWGS84ToED50) {
  erkir::ellipsoidal::Point pWGS84(53.0, 1.0, 50.0);
  const erkir::ellipsoidal::Point& pED50 =
      pWGS84.toDatum(erkir::ellipsoidal::Datum::Type::ED50);
  ASSERT_TRUE(pWGS84.isValid());
  ASSERT_TRUE(pED50.isValid());
  ASSERT_EQ(erkir::ellipsoidal::Point(53.0008, 1.0014), pED50);
  ASSERT_NEAR(2.721, pED50.height(), ERKIR_TESTS_EPSILON);
}

//
// cartesian::Point
//

TEST(CartesianPoint, CartesianPointToEllipsoidalPoint) {
  const erkir::cartesian::Point cp(3194419.0, 3194419.0, 4487348.0);
  const std::unique_ptr<erkir::ellipsoidal::Point> p = cp.toGeoPoint();
  ASSERT_TRUE(p->isValid());
  ASSERT_TRUE(cp.isValid());
  ASSERT_EQ(erkir::ellipsoidal::Point(45.0, 45.0), *p);
}

TEST(CartesianPoint, EllipsoidalPointToCartesianPoint) {
  erkir::ellipsoidal::Point p(45.0, 45.0);
  const std::unique_ptr<erkir::cartesian::Point> cp = p.toCartesianPoint();
  ASSERT_TRUE(p.isValid());
  ASSERT_TRUE(cp->isValid());
  ASSERT_NEAR(3194419.145, cp->x(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(3194419.145, cp->y(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(4487348.409, cp->z(), ERKIR_TESTS_EPSILON);
}

//
// Vector3d
//

TEST(Vector3d, Empty) {
  const erkir::Vector3d v;
  ASSERT_FALSE(v.isValid());
}

TEST(Vector3d, Length) {
  const erkir::Vector3d v1{3.0, 4.0, 0.0};
  const erkir::Vector3d v2{0.0, 0.0, 0.0};
  ASSERT_TRUE(v1.isValid());
  ASSERT_TRUE(v2.isValid());
  ASSERT_NEAR(5.0, v1.length(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(0.0, v2.length(), ERKIR_TESTS_EPSILON);
}

TEST(Vector3d, Dot) {
  const erkir::Vector3d v1{0.0, 0.0, 0.0};
  const erkir::Vector3d v2{1.0, 2.0, 3.0};
  const erkir::Vector3d v3{2.0, 2.0, 2.0};
  const erkir::Vector3d v4{2.0, 2.0, 2.0};
  ASSERT_TRUE(v1.isValid());
  ASSERT_TRUE(v2.isValid());
  ASSERT_TRUE(v3.isValid());
  ASSERT_TRUE(v4.isValid());
  ASSERT_NEAR(0.0, v1.dot(v2), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(12.0, v3.dot(v4), ERKIR_TESTS_EPSILON);
}

TEST(Vector3d, Cross) {
  const erkir::Vector3d v1{1.0, 0.0, 0.0};
  const erkir::Vector3d v2{0.0, 1.0, 0.0};
  const erkir::Vector3d v3{3.0, 0.0, 0.0};
  const erkir::Vector3d v4{0.0, 4.0, 0.0};
  const erkir::Vector3d v5{1.0, 2.0, 3.0};
  const erkir::Vector3d v6{4.0, 5.0, 6.0};
  const erkir::Vector3d cv1 = v1.cross(v2);
  const erkir::Vector3d cv2 = v3.cross(v4);
  const erkir::Vector3d cv3 = v5.cross(v6);
  ASSERT_TRUE(v1.isValid());
  ASSERT_TRUE(v2.isValid());
  ASSERT_TRUE(v3.isValid());
  ASSERT_TRUE(v4.isValid());
  ASSERT_TRUE(v5.isValid());
  ASSERT_TRUE(v6.isValid());
  ASSERT_TRUE(cv1.isValid());
  ASSERT_TRUE(cv2.isValid());
  ASSERT_TRUE(cv3.isValid());
  ASSERT_NEAR(0.0, cv1.x(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(0.0, cv1.y(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(1.0, cv1.z(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(0.0, cv2.x(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(0.0, cv2.y(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(12.0, cv2.z(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(-3.0, cv3.x(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(6.0, cv3.y(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(-3.0, cv3.z(), ERKIR_TESTS_EPSILON);
}

TEST(Vector3d, Addition) {
  const erkir::Vector3d v1{1.0, 2.0, 3.0};
  const erkir::Vector3d v2{3.0, 2.0, 1.0};
  const erkir::Vector3d addition = v1 + v2;
  ASSERT_TRUE(v1.isValid());
  ASSERT_TRUE(v2.isValid());
  ASSERT_TRUE(addition.isValid());
  ASSERT_NEAR(4.0, addition.x(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(4.0, addition.y(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(4.0, addition.z(), ERKIR_TESTS_EPSILON);
}

TEST(Vector3d, Subtraction) {
  const erkir::Vector3d v1{1.0, 2.0, 3.0};
  const erkir::Vector3d v2{3.0, 2.0, 1.0};
  const erkir::Vector3d subtraction = v1 - v2;
  ASSERT_TRUE(v1.isValid());
  ASSERT_TRUE(v2.isValid());
  ASSERT_TRUE(subtraction.isValid());
  ASSERT_NEAR(-2.0, subtraction.x(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(0.0, subtraction.y(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(2.0, subtraction.z(), ERKIR_TESTS_EPSILON);
}

TEST(Vector3d, Multiplication) {
  const erkir::Vector3d v{1.0, 2.0, 3.0};
  const erkir::Vector3d multiplication = v * 2.0;
  ASSERT_TRUE(v.isValid());
  ASSERT_TRUE(multiplication.isValid());
  ASSERT_NEAR(2.0, multiplication.x(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(4.0, multiplication.y(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(6.0, multiplication.z(), ERKIR_TESTS_EPSILON);
}

TEST(Vector3d, Division) {
  const erkir::Vector3d v{1.0, 2.0, 3.0};
  const erkir::Vector3d division = v / 2.0;
  ASSERT_TRUE(v.isValid());
  ASSERT_TRUE(division.isValid());
  ASSERT_NEAR(0.5, division.x(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(1.0, division.y(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(1.5, division.z(), ERKIR_TESTS_EPSILON);
}

TEST(Vector3d, Other) {
  const erkir::Vector3d v1{1.0, 2.0, 3.0};
  const erkir::Vector3d v2{3.0, 2.0, 1.0};
  const erkir::Vector3d cross = v1.cross(v2);
  const erkir::Vector3d negate = -v1;
  const erkir::Vector3d unit = v1.unit();
  const erkir::Vector3d rotate = v1.rotateAround({0.0, 0.0, 1.0}, 90.0);
  ASSERT_TRUE(v1.isValid());
  ASSERT_TRUE(v2.isValid());
  ASSERT_TRUE(cross.isValid());
  ASSERT_TRUE(negate.isValid());
  ASSERT_TRUE(unit.isValid());
  ASSERT_TRUE(rotate.isValid());
  ASSERT_NEAR(10.0, v1.dot(v2), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(3.7416573867739413, v1.length(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(-4.0, cross.x(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(8.0, cross.y(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(-4.0, cross.z(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(-1.0, negate.x(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(-2.0, negate.y(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(-3.0, negate.z(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(0.267, unit.x(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(0.535, unit.y(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(0.802, unit.z(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(-0.535, rotate.x(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(0.267, rotate.y(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(0.802, rotate.z(), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(erkir::Coordinate::toRadians(44.415), v1.angleTo(v2),
              ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(erkir::Coordinate::toRadians(44.415),
              v1.angleTo(v2, v1.cross(v2)), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(erkir::Coordinate::toRadians(-44.415),
              v1.angleTo(v2, v2.cross(v1)), ERKIR_TESTS_EPSILON);
  ASSERT_NEAR(erkir::Coordinate::toRadians(44.415), v1.angleTo(v2, v1),
              ERKIR_TESTS_EPSILON);
}
