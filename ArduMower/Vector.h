/*
 * Vector.h
 *
 *  Created on: May 5, 2020
 *      Author: ove
 */

#pragma once

#include <math.h>

template <typename T>
struct Vector
{
  T x {};
  T y {};
  T z {};

  Vector() = default;
  Vector(T t) : x(t), y(t), z(t) {};
  Vector(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {};
  Vector(const Vector<T>& v) : x(v.x), y(v.y), z(v.z) {};

  static void cross(const Vector<T>& lhs, const Vector<T>& rhs, Vector<T>& out);

  static T dot(const Vector<T>& lhs, const Vector<T>& rhs);

  static void normalize(Vector<float>& lhs);

  bool isZero()
  {
    return x == 0 && y == 0 && z == 0;
  }

  Vector<T>& operator= (const Vector<T>& v)
  {
    if (this != &v)
    {
      x = v.x;
      y = v.y;
      z = v.z;
    }
    return *this;
  }

  bool operator== (const Vector<T>& v)
  {
    return x == v.x && y == v.y && z == v.z;
  }

  bool operator!= (const Vector<T>& v)
  {
    return !(*this == v);
  }

  Vector<T>& operator+= (const Vector<T>& rhs)
  {
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
  }

  Vector<T>& operator-= (const Vector<T>& rhs)
  {
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
    return *this;
  }

  Vector<T>& operator*= (const T rhs)
  {
    x *= rhs;
    y *= rhs;
    z *= rhs;
    return *this;
  }

  Vector<T>& operator/= (const T rhs)
  {
    x /= rhs;
    y /= rhs;
    z /= rhs;
    return *this;
  }

  friend Vector<T> operator+ (Vector<T> lhs, const Vector<T>& rhs)
  {
    lhs += rhs;
    return lhs;
  }

  friend Vector<T> operator- (Vector<T> lhs, const Vector<T>& rhs)
  {
    lhs -= rhs;
    return lhs;
  }

  friend Vector<T> operator* (Vector<T> lhs, const T rhs)
  {
    lhs *= rhs;
    return lhs;
  }

  friend Vector<T> operator/ (Vector<T> lhs, const T rhs)
  {
    lhs /= rhs;
    return lhs;
  }

  friend Vector<T> operator- (Vector<T> lhs)
  {
    lhs.x = -lhs.x;
    lhs.y = -lhs.y;
    lhs.z = -lhs.z;
    return lhs;
  }

};

template <typename T>
void cross(const Vector<T>& lhs, const Vector<T>& rhs, Vector<T>& out)
{
  out.x = (lhs.y * rhs.z) - (lhs.z * rhs.y);
  out.y = (lhs.z * rhs.x) - (lhs.x * rhs.z);
  out.z = (lhs.x * rhs.y) - (lhs.y * rhs.x);
}

template <typename T>
T dot(const Vector<T>& lhs, const Vector<T>& rhs)
{
  return (lhs.x * rhs.x) + (lhs.y * rhs.y) + (lhs.z * rhs.z);
}

template <typename T>
void normalize(Vector<T>& lhs)
{
  float mag = sqrt(dot(lhs, lhs));
  lhs.x /= mag;
  lhs.y /= mag;
  lhs.z /= mag;
}
