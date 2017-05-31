#ifndef MCPT_EIGEN_H_
#define MCPT_EIGEN_H_

#include <cassert>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <iomanip>
#include <ostream>

template <typename T, size_t D>
struct Vector {
  static_assert(D > 0, "Vectors should have dimension greater than 0!");

  Vector() : v{0} {}

  template <typename... Args>
  explicit Vector(Args... args) : Vector{T(args)...} {}

  template<size_t E>
  explicit Vector(const Vector<T, E>& other) : v{0} {
    for (size_t i = 0; i < (D < E ? D : E); ++i)
      v[i] = other[i];
  }

  explicit Vector(const std::initializer_list<T>& args) : v{0} {
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

  Vector& operator+=(const Vector& other) {
    for (size_t i = 0; i < D; ++i)
      v[i] += other[i];
    return *this;
  }

  Vector& operator-=(const Vector& other) {
    for (size_t i = 0; i < D; ++i)
      v[i] += other[i];
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
    assert(D == 3 && "Cross product is only support for 3-D vectors!");
    Vector result;
    result.x() = y() * other.z() - z() * other.y();
    result.y() = z() * other.x() - x() * other.z();
    result.z() = x() * other.y() - y() * other.x();
    return result;
  }

  T Cross2D(const Vector& other) const {
    assert(D == 2 && "2-D Cross product is only support for 2-D vectors!");
    return x() * other.y() - y() * other.x();
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
    return std::sqrt(result);
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
    result.NormalizeInPlace();
    return result;
  }

  T x() const { return v[0]; }
  T y() const { return assert(D > 1 && "y() can only be called by vectors greater than 1-D!"), v[1]; }
  T z() const { return assert(D > 2 && "z() can only be called by vectors greater than 2-D!"), v[2]; }
  T w() const { return assert(D > 3 && "w() can only be called by vectors greater than 3-D!"), v[3]; }
  T& x() { return v[0]; }
  T& y() { return assert(D > 1 && "y() can only be called by vectors greater than 1-D!"), v[1]; }
  T& z() { return assert(D > 2 && "z() can only be called by vectors greater than 2-D!"), v[2]; }
  T& w() { return assert(D > 3 && "w() can only be called by vectors greater than 3-D!"), v[3]; }
  T r() const { return v[0]; }
  T g() const { return assert(D > 1 && "g() can only be called by vectors greater than 1-D!"), v[1]; }
  T b() const { return assert(D > 2 && "b() can only be called by vectors greater than 2-D!"), v[2]; }
  T a() const { return assert(D > 3 && "a() can only be called by vectors greater than 3-D!"), v[3]; }
  T& r() { return v[0]; }
  T& g() { return assert(D > 1 && "g() can only be called by vectors greater than 1-D!"), v[1]; }
  T& b() { return assert(D > 2 && "b() can only be called by vectors greater than 2-D!"), v[2]; }
  T& a() { return assert(D > 3 && "a() can only be called by vectors greater than 3-D!"), v[3]; }

  static Vector All(T scalar) {
    Vector all;
    all.SetAll(scalar);
    return all;
  }

  static const Vector& Zero() {
    static const Vector zero;
    return zero;
  }

  static const Vector& Ones() {
    static const Vector ones = All(1);
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
T Cos(const Vector<T, D>& v1, const Vector<T, D>& v2) {
  return v1 * v2 / (v1.L2() * v2.L2());
}

template <typename T, size_t D>
Vector<T, D> Cross(const Vector<T, D>& v1, const Vector<T, D>& v2) {
  assert(D == 3 && "Cross product is only support for 3-D vectors!");
  Vector<T, D> result;
  result.x() = v1.y() * v2.z() - v1.z() * v2.y();
  result.y() = v1.z() * v2.x() - v1.x() * v2.z();
  result.z() = v1.x() * v2.y() - v1.y() * v2.x();
  return result;
}

template <typename T, size_t D>
T Cross2D(const Vector<T, D> v1, const Vector<T, D>& v2) {
  assert(D == 2 && "2-D Cross product is only support for 2-D vectors!");
  return v1.x() * v2.y() - v1.y() * v2.x();
}

template <typename T, size_t D>
std::ostream& operator<<(std::ostream& os, const Vector<T, D>& v) {
  os << "[ ";
  for (size_t i = 0; i < D; ++i)
    os << std::setw(5) << v[i] << " ";
  os << "]";
  return os;
}

typedef Vector<float, 1> Vector1f;
typedef Vector<float, 2> Vector2f;
typedef Vector<float, 3> Vector3f;
typedef Vector<float, 4> Vector4f;
typedef Vector<int, 1> Vector1i;
typedef Vector<int, 2> Vector2i;
typedef Vector<int, 3> Vector3i;
typedef Vector<int, 4> Vector4i;

template <typename T, size_t D>
struct Matrix {
  Matrix() {
    for (size_t i = 0; i < D; ++i)
      m[i].SetZero();
  }

  explicit Matrix(const std::initializer_list<T>& args) {
    size_t i = 0;
    for (auto e : args) {
      if (i < D * D) {
        m[i % D][i / D] = e;
        ++i;
      } else {
        break;
      }
    }
  }

  template <typename... Args>
  explicit Matrix(Args... args) : Matrix{T(args)...} {}

  Vector<T, D> operator*(const Vector<T, D>& vec) const {
    Vector<T, D> result;
    for (size_t i = 0; i < D; ++i)
      result += vec[i] * m[i];
    return result;
  }

  const T operator()(size_t row, size_t col) const {
    return m[col][row];
  }

  T& operator()(size_t row, size_t col) {
    return m[col][row];
  }

  static Matrix Diagonal(const Vector<T, D>& diag) {
    Matrix result;
    for (size_t i = 0; i < D; ++i)
      result(i, i) = diag[i];
    return result;
  }

  static const Matrix& Identity() {
    static Matrix identity = Matrix::Diagonal(Vector<T, D>::Ones());
    return identity;
  }

  Vector<T, D> m[D];
};

typedef Matrix<float, 1> Matrix1f;
typedef Matrix<float, 2> Matrix2f;
typedef Matrix<float, 3> Matrix3f;
typedef Matrix<float, 4> Matrix4f;

#endif  /* MCPT_EIGEN_H_ */
