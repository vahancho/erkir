#ifndef COORDINATE_H
#define COORDINATE_H

namespace geodesy
{

class Coordinate
{
public:
  Coordinate(double degree);

  double degree() const;
  double radian() const;

private:
  double m_degree;
};

}

#endif // COORDINATE_H
