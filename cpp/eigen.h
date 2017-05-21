#ifndef MCPT_EIGEN_H_
#define MCPT_EIGEN_H_

#include <cmath>
#include <cassert>
#include <cstddef>
#include <initializer_list>

template <typename T, size_t D>
struct Vector {
  static_assert(D > 0, "Vectors should have dimension greater than 0!");

  Vector() : v{0} {}

  template <typename... Args>
  Vector(Args... args) : Vector{T(args)...} {}

  template<size_t E>
  Vector(const Vector<T, E>& other) : v{0} {
    for (size_t i = 0; i < (D < E ? D : E); ++i)
      v[i] = other[i];
  }

  Vector(const std::initializer_list<T>& args) : v{0} {
    size_t i = 0;
    for (auto e : args) {
      if (i < D)
        v[i++] = e;
      else
        break;
    }
  }

  size_t Rows() const { return D; }

  T operator[](size_t i) const { return v[i]; }
  T& operator[](size_t i) { return v[i]; }

  Vector operator+() const {
    return (*this);
  }

  Vector operator-() const {
    Vector result;
    for (size_t i = 0; i < D; ++i)
      result[i] = -v[i];
    return result;
  }

  Vector operator+(const Vector& other) const {
    Vector result;
    for (size_t i = 0; i < D; ++i)
      result[i] = v[i] + other[i];
    return result;
  }

  Vector operator-(const Vector& other) const {
    Vector result;
    for (size_t i = 0; i < D; ++i)
      result[i] = v[i] - other[i];
    return result;
  }

  T operator*(const Vector& other) const {
    T product = 0;
    for (size_t i = 0; i < D; ++i)
      product += v[i] * other[i];
    return product;
  }

  Vector operator*(T scalar) const {
    Vector result;
    for (size_t i = 0; i < D; ++i)
      result[i] = v[i] * scalar;
    return result;
  }

  Vector operator/(T scalar) const {
    Vector result;
    for (size_t i = 0; i < D; ++i)
      result[i] = v[i] / scalar;
    return result;
  }

  Vector& operator+=(T scalar) {
    for (size_t i = 0; i < D; ++i)
      v[i] += scalar;
    return *this;
  }

  Vector& operator-=(T scalar) {
    for (size_t i = 0; i < D; ++i)
      v[i] -= scalar;
    return *this;
  }

  Vector& operator*=(T scalar) {
    for (size_t i = 0; i < D; ++i)
      v[i] *= scalar;
    return *this;
  }

  Vector& operator/=(T scalar) {
    for (size_t i = 0; i < D; ++i)
      v[i] /= scalar;
    return *this;
  }

  T Cos(const Vector& other) const {
    return (*this) * other / (L2() * other.L2());
  }

  Vector Cross(const Vector& other) const {
    assert(D == 3 && "Cross product is only supported by 3-D Vectors!");
    Vector result;
    result.x() = x() * other.z() - z() * other.y();
    result.y() = z() * other.x() - x() * other.z();
    result.z() = x() * other.y() - y() * other.x();
    return result;
  }

  int L0() const {
    int result = 0;
    for (size_t i = 0; i < D; ++i) {
      if (v[i] != 0)
        ++result;
    }
    return result;
  }

  T L1() const {
    T result = 0;
    for (size_t i = 0; i < D; ++i)
      result += std::abs(v[i]);
    return result;
  }

  T L2() const {
    T result = 0;
    for (size_t i = 0; i < D; ++i)
      result += v[i] * v[i];
    return std::sqrt<T>(result);
  }

  void SetAll(T scalar) {
    for (size_t i = 0; i < D; ++i)
      v[i] = scalar;
  }

  void SetZero() {
    for (size_t i = 0; i < D; ++i)
      v[i] = 0;
  }

  void SetOnes() {
    for (size_t i = 0; i < D; ++i)
      v[i] = 1;
  }

  void NormalizeInPlace() {
    T norm = L2();
    for (size_t i = 0; i < D; ++i)
      v[i] /= norm;
  }

  Vector Normalize() const {
    Vector result = (*this);
    return result.NormalizeInPlace();
  }

  T x() const { return assert(D > 0), v[0]; }
  T y() const { return assert(D > 1 && "y() can only be called by Vectors greater than 1-D!"), v[1]; }
  T z() const { return assert(D > 2 && "z() can only be called by Vectors greater than 2-D!"), v[2]; }
  T w() const { return assert(D > 3 && "w() can only be called by Vectors greater than 3-D!"), v[3]; }
  T& x() { return assert(D > 0), v[0]; }
  T& y() { return assert(D > 1 && "y() can only be called by Vectors greater than 1-D!"), v[1]; }
  T& z() { return assert(D > 2 && "z() can only be called by Vectors greater than 2-D!"), v[2]; }
  T& w() { return assert(D > 3 && "w() can only be called by Vectors greater than 3-D!"), v[3]; }
  T r() const { return assert(D > 0), v[0]; }
  T g() const { return assert(D > 1 && "g() can only be called by Vectors greater than 1-D!"), v[1]; }
  T b() const { return assert(D > 2 && "b() can only be called by Vectors greater than 2-D!"), v[2]; }
  T a() const { return assert(D > 3 && "a() can only be called by Vectors greater than 3-D!"), v[3]; }
  T& r() { return assert(D > 0), v[0]; }
  T& g() { return assert(D > 1 && "g() can only be called by Vectors greater than 1-D!"), v[1]; }
  T& b() { return assert(D > 2 && "b() can only be called by Vectors greater than 2-D!"), v[2]; }
  T& a() { return assert(D > 3 && "a() can only be called by Vectors greater than 3-D!"), v[3]; }

  static Vector Zero() {
    static const Vector zero;
    return zero;
  }

  static Vector Ones() {
    static const Vector ones = Vector() + 1;
    return ones;
  }

  T v[D];
};

template <typename T, size_t D>
Vector<T, D> operator*(T scalar, const Vector<T, D>& vec) {
  Vector<T, D> result;
  for (size_t i = 0; i < D; ++i)
    result[i] = scalar * vec[i];
  return result;
}

template <typename T, size_t D>
Vector<T, D> Cos(const Vector<T, D>& v1, const Vector<T, D>& v2) {
  return v1 * v2 / (v1.L2() * v2.L2());
}

template <typename T, size_t D>
Vector<T, D> Cross(const Vector<T, D>& v1, const Vector<T, D>& v2) {
  assert(D == 3 && "Cross product is only supported by 3-D Vectors!");
  Vector<T, D> result;
  result.x() = v1.x() * v2.z() - v1.z() * v2.y();
  result.y() = v1.z() * v2.x() - v1.x() * v2.z();
  result.z() = v1.x() * v2.y() - v1.y() * v2.x();
  return result;
}

typedef Vector<float, 2> Vector2f;
typedef Vector<float, 3> Vector3f;
typedef Vector<float, 4> Vector4f;
typedef Vector<int, 2> Vector2i;
typedef Vector<int, 3> Vector3i;
typedef Vector<int, 4> Vector4i;

#endif  /* MCPT_EIGEN_H_ */
