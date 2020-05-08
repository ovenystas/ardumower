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
  union
  {
    T data[3];
    struct
    {
      T x;
      T y;
      T z;
    };
  };

  Vector() : x(0), y(0), z(0) {};
  Vector(T t) : x(t), y(t), z(t) {};
  Vector(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {};
  Vector(const Vector<int32_t>& v) :
    x(static_cast<T>(v.x)),
    y(static_cast<T>(v.y)),
    z(static_cast<T>(v.z)) {};
  Vector(const Vector<int16_t>& v) :
    x(static_cast<T>(v.x)),
    y(static_cast<T>(v.y)),
    z(static_cast<T>(v.z)) {};
  Vector(const Vector<float>& v) :
    x(static_cast<T>(v.x)),
    y(static_cast<T>(v.y)),
    z(static_cast<T>(v.z)) {};

  bool isZero()
  {
    return x == 0 && y == 0 && z == 0;
  }

  static Vector<T> cross(Vector<T> lhs, const Vector<T>& rhs)
  {
    Vector<T> tmp;
    tmp.x = (lhs.y * rhs.z) - (lhs.z * rhs.y);
    tmp.y = (lhs.z * rhs.x) - (lhs.x * rhs.z);
    tmp.z = (lhs.x * rhs.y) - (lhs.y * rhs.x);
    lhs = tmp;
    return lhs;
  }

  static T dot(const Vector<T>& v1, const Vector<T>& v2)
  {
    return (v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z);
  }

  static void normalize(Vector<T>& v)
  {
    float mag = sqrt(dot(v, v));
    v.x /= mag;
    v.y /= mag;
    v.z /= mag;
  }

  Vector<T>& operator= (const Vector<T>& rhs)
  {
    if (this != &rhs)
    {
      x = rhs.x;
      y = rhs.y;
      z = rhs.z;
    }
    return *this;
  }

  bool operator== (const Vector<T>& rhs)
  {
    return x == rhs.x && y == rhs.y && z == rhs.z;
  }

  bool operator!= (const Vector<T>& rhs)
  {
    return !(*this == rhs);
  }

  Vector<T>& operator+= (const Vector<T>& rhs)
  {
    x = static_cast<T>(x + rhs.x);
    y = static_cast<T>(y + rhs.y);
    z = static_cast<T>(z + rhs.z);
    return *this;
  }

  Vector<T>& operator-= (const Vector<T>& rhs)
  {
    x = static_cast<T>(x - rhs.x);
    y = static_cast<T>(y - rhs.y);
    z = static_cast<T>(z - rhs.z);
    return *this;
  }

  Vector<T>& operator*= (const T rhs)
  {
    x = static_cast<T>(x * rhs);
    y = static_cast<T>(y * rhs);
    z = static_cast<T>(z * rhs);
    return *this;
  }

  Vector<T>& operator*= (const Vector<T> rhs)
  {
    x = static_cast<T>(x * rhs.x);
    y = static_cast<T>(y * rhs.y);
    z = static_cast<T>(z * rhs.z);
    return *this;
  }

  Vector<T>& operator/= (const T rhs)
  {
    x = static_cast<T>(x / rhs);
    y = static_cast<T>(y / rhs);
    z = static_cast<T>(z / rhs);
    return *this;
  }

  Vector<T>& operator/= (const Vector<T> rhs)
  {
    x = static_cast<T>(x / rhs.x);
    y = static_cast<T>(y / rhs.y);
    z = static_cast<T>(z / rhs.z);
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

  friend Vector<T> operator* (Vector<T> lhs, const Vector<T> rhs)
  {
    lhs *= rhs;
    return lhs;
  }

  friend Vector<T> operator/ (Vector<T> lhs, const T rhs)
  {
    lhs /= rhs;
    return lhs;
  }

  friend Vector<T> operator/ (Vector<T> lhs, const Vector<T> rhs)
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
