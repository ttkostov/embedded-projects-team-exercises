#pragma once
#include <Arduino.h>
#include <math.h>

struct Angle
{
public:
  explicit Angle(float deg = 0.0f)
      : degrees_(wrapAngle(deg)) {}

  float value() const { return degrees_; }

  Angle operator-(const Angle &other) const { return Angle(degrees_ - other.degrees_); }
  Angle operator+(const Angle &other) const { return Angle(degrees_ + other.degrees_); }
  Angle operator-(float delta) const { return Angle(degrees_ - delta); }
  Angle operator+(float delta) const { return Angle(degrees_ + delta); }

  bool operator==(const Angle &other) const { return degrees_ == other.degrees_; }
  bool operator!=(const Angle &other) const { return !(*this == other); }

  Angle operator*(const float factor) const { return Angle(degrees_ * factor); }

private:
  float degrees_;

  static float wrapAngle(float angle)
  {
    angle = fmodf(angle, 360.0f);
    if (angle < 0)
      angle += 360.0f;
    return angle;
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
