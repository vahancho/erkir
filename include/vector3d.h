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

#ifndef VECTOR3D_H
#define VECTOR3D_H

namespace erkir
{

/// Implements 3-d vector manipulation routines.
/*!
  In a geodesy context, these vectors may be used to represent:
  - n-vector representing a normal to point on Earth's surface
  - earth-centered, earth fixed vector (= Gade's 'p-vector')
  - great circle normal to vector (on spherical earth model)
  - motion vector on Earth's surface
  - etc

  Functions return vectors as return results, so that operations can be chained.
  \example auto v = v1.cross(v2).dot(v3) // equivalent to v1 x v2 . v3
*/
class Vector3d
{
public:
  /// Creates a 3-d vector.
  /*!
    The vector may be normalised, or use x/y/z values for eg height relative to the sphere or
    ellipsoid, distance from earth centre, etc.

    \param x X component of vector.
    \param y Y component of vector.
    \param z Z component of vector.
 */
  Vector3d(double x, double y, double z);

private:
  double m_x;
  double m_y;
  double m_z;
};

} // erkir

#endif // VECTOR3D_H

