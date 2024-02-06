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

#ifndef COORDINATE_H
#define COORDINATE_H

#include "export.h"

#include <string>

namespace erkir
{

//! Implements the geographical coordinate abstraction.
class ERKIR_EXPORT Coordinate
{
public:
  /// The coordinates human readable format types.
  enum class Format
  {
    DMS, ///< Degrees Minutes Seconds (D° M' S")
    DDM, ///< Decimal Minutes (D° M.M')
    DD   ///< Decimal Degrees (D.D°)
  };

  //! Constructs a coordinate by the given decimal degrees.
  Coordinate(double degrees);

  /// Returns string representation of the coordinate in the specified \p format.
  virtual std::string toString(Format format, int precision) const = 0;

  //! Returns this coordinate's value in decimal degrees.
  double degrees() const;

  //! Returns this coordinate's value in radians.
  double radians() const;

  /// A helper function to convert radians to degrees.
  static double toDegrees(double radians);

  /// A helper function to convert degrees to radians.
  static double toRadians(double degrees);

  static double pi();

  /// Constrain degrees to range 0..360.0 (e.g. for bearings); -1 => 359, 361 => 1.
  static double wrap360(double degrees);

protected:
  std::string toBaseString(Format format, int precision) const;

private:
  double m_degrees;
};

//! Implements the latitude - geographic coordinate that specifies the north–south
//! position of a point on the Earth's surface.
class ERKIR_EXPORT Latitude : public Coordinate
{
public:
  //! Constructs a latitude object.
  /*!
    \param degree Decimal degrees in the range from 0° to (+/–)90°
    \throws std::out_of_range
  */
  Latitude(double degree);

  std::string toString(Format format, int precision = 2) const override;

  /// Returns the latitude from the human readable coordinates (formatted).
  /*!
    Latitude/Longitude coordinates in three formats: Degrees Minutes Seconds (D° M' S"),
    Decimal Minutes (D° M.M'), and Decimal Degrees (D.D°). Each of these formats
    can represent the same geographic location, but expressed differently.

    For example: 45° 46' 52" N 108° 30' 14" W as displayed in Degrees Minutes Seconds (D° M' S").
    This same location, in displayed Decimal Minutes (D° M.M'), is: 45° 46.8666' N 108° 30.2333' W.
    In Decimal Degrees (D.D°), this same location is: 45.7811111° N 108.5038888° W

    About Sign and North, South, East West
    ---------------------------------------------------------------------------
    Latitude/Longitude is followed by an indication of hemisphere.
    For example 45° 46' 52" N indicates the Northern Hemisphere (North of the equator.)
    108° 30' 14" W indicates an area West of the Prime Meridian. When noting this numerically
    (especially in Decimal Degrees), positive and negative values are sometimes used.
    A positive value for North and East, a negative value for South and West.
    Thus, in our example, when noting 45° 46' 52" N 108° 30' 14" W in Decimal Degrees,
    it may appear as 45.7811111 -108.5038888 when represented numerically.
  */
  static Latitude fromString(const std::string &coord);
};

//! Implements the longitude - the measurement east or west of the prime meridian.
class ERKIR_EXPORT Longitude : public Coordinate
{
public:
  //! Constructs a longitude object.
  /*!
    \param degree Decimal degrees in the range from 0° to (+/–)180°
    \throws std::out_of_range
  */
  Longitude(double degree);

  std::string toString(Format format, int precision = 2) const override;

  /// Returns the longitude from the human readable coordinates (formatted).
  /*!
    Latitude/Longitude coordinates in three formats: Degrees Minutes Seconds (D° M' S"),
    Decimal Minutes (D° M.M'), and Decimal Degrees (D.D°). Each of these formats
    can represent the same geographic location, but expressed differently.

    For example: 45° 46' 52" N 108° 30' 14" W as displayed in Degrees Minutes Seconds (D° M' S").
    This same location, in displayed Decimal Minutes (D° M.M'), is: 45° 46.8666' N 108° 30.2333' W.
    In Decimal Degrees (D.D°), this same location is: 45.7811111° N 108.5038888° W

    About Sign and North, South, East West
    ---------------------------------------------------------------------------
    Latitude/Longitude is followed by an indication of hemisphere.
    For example 45° 46' 52" N indicates the Northern Hemisphere (North of the equator.)
    108° 30' 14" W indicates an area West of the Prime Meridian. When noting this numerically
    (especially in Decimal Degrees), positive and negative values are sometimes used.
    A positive value for North and East, a negative value for South and West.
    Thus, in our example, when noting 45° 46' 52" N 108° 30' 14" W in Decimal Degrees,
    it may appear as 45.7811111 -108.5038888 when represented numerically.
 */
  static Longitude fromString(const std::string &coord);
};

}

#endif // COORDINATE_H

