#ifndef COORDINATE_H
#define COORDINATE_H

namespace geodesy
{

class Coordinate
{
public:
  Coordinate(double degrees);

  double degrees() const;
  double radians() const;

private:
  double m_degrees;
};

class Latitude : public Coordinate
{
public:
  Latitude(double degree);
};

class Longitude : public Coordinate
{
public:
  Longitude(double degree);
};

}

#endif // COORDINATE_H
