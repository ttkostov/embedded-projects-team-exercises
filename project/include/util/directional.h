#pragma once
#include <Arduino.h>
#include <math.h>

struct Angle
{
public:
  explicit Angle(float deg = 0)
      : degrees_(wrap(deg)), isAbsolute_(true) {}

  static Angle absolute(float deg) { return Angle(deg, true); }
  static Angle relative(float deg) { return Angle(deg, false); }

  float value() const { return degrees_; }
  bool isAbsolute() const { return isAbsolute_; }

  Angle differenceTo(const Angle &other) const
  {
    float a = wrap(degrees_);
    float b = wrap(other.degrees_);

    float d = b - a;

    if (d > 180)
      d -= 360;
    if (d < -180)
      d += 360;

    return Angle::relative(d);
  }

  Angle operator+(float deg) const { return Angle(degrees_ + deg, isAbsolute_); }
  Angle operator-(float deg) const { return *this + (-deg); }

  Angle operator+(const Angle &other) const { return Angle(degrees_ + other.degrees_, isAbsolute_ || other.isAbsolute_); }
  Angle operator-(const Angle &other) const { return *this + Angle(-other.degrees_, other.isAbsolute_); }

private:
  float degrees_;
  bool isAbsolute_;

  Angle(float deg, bool isAbs)
      : degrees_(isAbs ? wrap(deg) : deg), isAbsolute_(isAbs) {}

  static float wrap(float d)
  {
    d = fmodf(d, 360.0f);
    return (d < 0 ? d + 360.0f : d);
  }
};

struct Direction
{
public:
  enum class Compass
  {
    North,
    NorthEast,
    East,
    SouthEast,
    South,
    SouthWest,
    West,
    NorthWest,
  };

  explicit Direction(Compass v = Compass::North) : value(v) {}

  static Direction fromAngle(const Angle &angle)
  {
    float deg = angle.value();
    int index = static_cast<int>(floor((deg + 22.5f) / 45.0f)) % 8;

    return Direction(directions_[index]);
  }

  const Angle toAngle()
  {
    return Angle(static_cast<int>(value) * 45.0f);
  }

  const char *toString() const
  {
    switch (value)
    {
    case Compass::North:
      return "N";
    case Compass::NorthEast:
      return "NE";
    case Compass::East:
      return "E";
    case Compass::SouthEast:
      return "SE";
    case Compass::South:
      return "S";
    case Compass::SouthWest:
      return "SW";
    case Compass::West:
      return "W";
    case Compass::NorthWest:
      return "NW";
    default:
      return "?";
    }
  }

  bool operator==(const Direction &other) const { return value == other.value; }
  bool operator!=(const Direction &other) const { return !(*this == other); }

private:
  Compass value;

  // ordered in clockwise direction, beginning from north
  static constexpr Compass directions_[8] = {
      Compass::North, Compass::NorthEast, Compass::East, Compass::SouthEast,
      Compass::South, Compass::SouthWest, Compass::West, Compass::NorthWest};
};
