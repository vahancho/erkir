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

#ifndef ERKIR_VECTOR3D_H_
#define ERKIR_VECTOR3D_H_

#include "erkir/export.h"

namespace erkir {

/**
 * @brief Implements 3-d vector manipulation routines.
 * In a geodesy context, these vectors may be used to represent:
 * - n-vector representing a normal to point on Earth's surface
 * - earth-centered, earth fixed vector (= Gade's 'p-vector')
 * - great circle normal to vector (on spherical earth model)
 * - motion vector on Earth's surface
 * - etc
 *
 * Functions return vectors as return results, so that operations can be
 * chained.
 *
 * @example
 * auto v = v1.cross(v2).dot(v3) // equivalent to v1 × v2 . v3
 */
class ERKIR_EXPORT Vector3d {
 public:
  /**
   * @brief Creates an invalid 3-d vector.
   */
  Vector3d();

  /**
   * @brief Creates a 3-d vector.
   * The vector may be normalised, or use x/y/z values for eg height relative to
   * the sphere or ellipsoid, distance from earth centre, etc.
   *
   * @param x X component of vector.
   * @param y Y component of vector.
   * @param z Z component of vector.
   */
  Vector3d(double x, double y, double z);

  double x() const;

  double y() const;

  double z() const;

  bool isValid() const;

  /**
   * @brief Dot (scalar) product of this vector and v vector.
   *
   * @param v Other vector.
   * @return Dot product.
   */
  double dot(const Vector3d &v) const;

  /**
   * @brief Multiplies vector by the supplied vector using cross (vector)
   * product.
   *
   * @param v Other vector.
   * @return Cross product.
   */
  Vector3d cross(const Vector3d &v) const;

  /**
   * @brief Length (magnitude or norm) of this vector.
   *
   * @return Magnitude of this vector.
   */
  double length() const;

  /**
   * @brief Normalizes a vector to its unit vector.
   * If the vector is already unit or is zero magnitude, this is a no-op.
   *
   * @return Normalised version of this vector.
   */
  Vector3d unit() const;

  /**
   * @brief Calculates the angle between this vector and supplied vector.
   *
   * @param v Supplied vector
   * @param n Plane normal: if supplied, angle is -PI..+PI, signed +ve if
   * this->v is clockwise looking along n, -ve in opposite direction (if not
   * supplied, angle is always 0..PI).
   * @return Angle (in radians) between this vector and supplied vector.
   */
  double angleTo(const Vector3d &v, const Vector3d &n = Vector3d()) const;

  /**
   * @brief Rotates this point around an axis by a specified angle.
   *
   * @param axis The axis being rotated around.
   * @param theta The angle of rotation (in radians).
   * @return Rotated point.
   */
  Vector3d rotateAround(const Vector3d &axis, double theta) const;

 private:
  double m_x{0.0};
  double m_y{0.0};
  double m_z{0.0};

  bool m_isValid;
};

/**
 * @brief Vector addition.
 *
 * @param v1 First vector.
 * @param v2 Second vector.
 * @return Addition.
 */
inline Vector3d operator+(const Vector3d &v1, const Vector3d &v2) {
  return Vector3d(v1.x() + v2.x(), v1.y() + v2.y(), v1.z() + v2.z());
}

/**
 * @brief Vector subtraction.
 *
 * @param v1 First vector.
 * @param v2 Second vector.
 * @return Subtraction.
 */
inline Vector3d operator-(const Vector3d &v1, const Vector3d &v2) {
  return Vector3d(v1.x() - v2.x(), v1.y() - v2.y(), v1.z() - v2.z());
}

/**
 * @brief Unary negation of a vector.
 * Negates a vector to point in the opposite direction.
 *
 * @param v1 Vector.
 * @return Negated vector.
 */
inline Vector3d operator-(const Vector3d &v1) {
  return Vector3d(-v1.x(), -v1.y(), -v1.z());
}

/**
 * @brief Multiplication of vector and scalar.
 *
 * @param v1 Vector.
 * @param s Scalar.
 * @return Multiplication.
 */
inline Vector3d operator*(const Vector3d &v1, double s) {
  return Vector3d(v1.x() * s, v1.y() * s, v1.z() * s);
}

/**
 * @brief Division of vector and scalar.
 *
 * @param v1 Vector.
 * @param s Scalar.
 * @return Division.
 */
inline Vector3d operator/(const Vector3d &v1, double s) {
  return Vector3d(v1.x() / s, v1.y() / s, v1.z() / s);
}

/**
 * @brief Multiplication of scalar and vector.
 *
 * @param s Scalar.
 * @param v1 Vector.
 * @return Multiplication.
 */
inline Vector3d operator*(double s, const Vector3d &v1) { return v1 * s; }

}  // namespace erkir

#endif  // ERKIR_VECTOR3D_H_
