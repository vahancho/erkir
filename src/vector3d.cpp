/**********************************************************************************
*  MIT License                                                                    *
*                                                                                 *
*  Copyright (c) 2018-2019 Vahan Aghajanyan <vahancho@gmail.com>                  *
*                                                                                 *
*  Vector handling functions               (c) Chris Veness 2011-2016             *
*  www.movable-type.co.uk/scripts/geodesy/docs/module-vector3d.html               *
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

#include <vector>
#include <cmath>
#include "vector3d.h"
#include "coordinate.h"

namespace erkir
{

Vector3d::Vector3d()
  :
    m_isValid(false)
{}

Vector3d::Vector3d(double x, double y, double z)
  :
    m_x(x),
    m_y(y),
    m_z(z),
    m_isValid(true)
{}

double Vector3d::x() const
{
  return m_x;
}

double Vector3d::y() const
{
  return m_y;
}

double Vector3d::z() const
{
  return m_z;
}

bool Vector3d::isValid() const
{
  return m_isValid;
}

double Vector3d::dot(const Vector3d &v) const
{
  return x() * v.x() + y() * v.y() + z() * v.z();
}

Vector3d Vector3d::cross(const Vector3d &v) const
{
  return Vector3d(y() * v.z() - z() * v.y(),
                  z() * v.x() - x() * v.z(),
                  x() * v.y() - y() * v.x());
}

double Vector3d::length() const
{
  return std::sqrt(x() * x() + y() * y() + z() * z());
}

Vector3d Vector3d::unit() const
{
  auto norm = length();
  if (norm == 1.0 || norm == 0.0) {
    return *this;
  }

  return Vector3d(x() / norm, y() / norm, z() / norm);
}

double Vector3d::angleTo(const Vector3d &v, const Vector3d &n) const
{
  auto sign = !n.isValid() ? 1.0 : std::copysign(1.0, cross(v).dot(n));
  auto sin = cross(v).length() * sign;
  auto cos = dot(v);

  return std::atan2(sin, cos);
}

Vector3d Vector3d::rotateAround(const Vector3d &axis, double theta) const
{
  // en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
  // en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Quaternion-derived_rotation_matrix
  auto p1 = unit();
  std::vector<double> p = { p1.x(), p1.y(), p1.z() }; // the point being rotated
  auto a = axis.unit(); // the axis being rotated around

  const auto thetaRad = Coordinate::toRadians(theta);
  const auto s = std::sin(thetaRad);
  const auto c = std::cos(thetaRad);

  // Quaternion-derived rotation matrix
  std::vector<std::vector<double>> q =
  {
    {
      a.x() * a.x() * (1.0 - c) + c,
      a.x() * a.y() * (1.0 - c) - a.z() * s,
      a.x() * a.z() * (1.0 - c) + a.y() * s
    },
    {
      a.y() * a.x() * (1.0 - c) + a.z() * s,
      a.y() * a.y() * (1.0 - c) + c,
      a.y() * a.z() * (1.0 - c) - a.x() * s
    },
    {
      a.z() * a.x() * (1.0 - c) - a.y() * s,
      a.z() * a.y() * (1.0 - c) + a.x() * s,
      a.z() * a.z() * (1.0 - c) + c
    }
  };

  // Multiply q Ч p
  auto qp = std::vector<double>(3, 0.0);
  for (auto i = 0; i < 3; i++) {
    for (auto j = 0; j < 3; j++) {
      qp[i] += q[i][j] * p[j];
    }
  }
  return Vector3d(qp[0], qp[1], qp[2]);
  // qv en.wikipedia.org/wiki/Rodrigues'_rotation_formula...
}

} // erkir

