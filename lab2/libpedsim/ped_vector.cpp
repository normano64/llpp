//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//

#include "ped_vector.h"

#include <cmath>
#include <string>
#include <sstream>
#include <x86intrin.h>
/// Default constructor, which makes sure that all the values are set to 0.
/// \date    2012-01-16
Ped::Tvector::Tvector()
{
  xyz[0] = .0f;
  xyz[1] = .0f;
  xyz[2] = .0f;
  xyz[3] = .0f;
}


std::string Ped::Tvector::to_string() const {
  float x_ = xyz[0];
  float y_ = xyz[1];
  float z_ = xyz[2];
  std::ostringstream strs;
  strs << x_ << "/" << y_ << "/" << z_;
  return strs.str();
  //return std::to_string((const long double) x) + "/" + std::to_string((const long double) y_) + "/" + std::to_string((const long double) z_);
}


/// Returns the length of the vector.
/// \return the length
float Ped::Tvector::length() const {
  if ((xyz[0] == 0) && (xyz[1] == 0) && (xyz[2] == 0)) return 0;  
  return sqrt(lengthSquared());
}


/// Returns the length of the vector squared. This is faster than the real length.
/// \return the length squared
float Ped::Tvector::lengthSquared() const {
  if(Ped::Tvector::VEC)
  {
	__m128 vec = _mm_load_ps(xyz);
	vec = _mm_mul_ps(vec, vec);
    float ret[4] __attribute__((aligned(16)));
		__m128 t = _mm_add_ps(vec, _mm_movehl_ps(vec, vec));
	vec = _mm_add_ss(t, _mm_shuffle_ps(t, t, 1));
	_mm_store_ps(ret, vec);
	return ret[0];
/*	_mm_store_ps(ret, vec);
	return ret[0] + ret[1] + ret[2];*/
  }
  return xyz[0]*xyz[0] + xyz[1]*xyz[1] + xyz[2]*xyz[2];
}


/// Normalizes the vector to a length of 1.
/// \date    2010-02-12
void Ped::Tvector::normalize() {
  float len = length();

  // null vectors cannot be normalized
  if (len == 0)
	return;
  if(Ped::Tvector::VEC)
  {
	__m128 vec = _mm_load_ps(xyz);
	float ln[4] __attribute__((aligned(16))) = {len, len, len, len};
	__m128 lnv  = _mm_load_ps(ln);
	vec = _mm_div_ps(vec, lnv);
	_mm_store_ps(xyz, vec);
  }
  else
  {
	xyz[0] /= len;
	xyz[1] /= len;
	xyz[2] /= len;
  }
/*  x /= len;
    y /= len;
    z /= len;*/
}

/// Normalizes the vector to a length of 1.
/// \date    2013-08-02
Ped::Tvector Ped::Tvector::normalized() const {
  float len = length();

  // null vectors cannot be normalized
  if (len == 0)
	return Ped::Tvector();;	
  if(Ped::Tvector::VEC)
  {
	__m128 vec = _mm_load_ps(xyz);
	float ln[4] __attribute__((aligned(16))) = {len, len, len, len};
	__m128 lnv  = _mm_load_ps(ln);
	float ret[4] __attribute__((aligned(16)));
	vec = _mm_div_ps(vec, lnv);
	_mm_store_ps(ret, vec);
	return Ped::Tvector(ret[0], ret[1], ret[2]);
  }
  return Ped::Tvector(xyz[0]/len, xyz[1]/len, xyz[2]/len);
}


/// Vector scalar product helper: calculates the scalar product of two vectors.
/// \date    2012-01-14
/// \return  The scalar product.
/// \param   &a The first vector
/// \param   &b The second vector
float Ped::Tvector::scalar(const Ped::Tvector &a, const Ped::Tvector &b) {
  return acos( Ped::Tvector::dotProduct(a, b) / ( a.length() * b.length() ) );
}


/// Vector scalar product helper: calculates the scalar product of two vectors.
/// \date    2012-01-14
/// \return  The scalar product.
/// \param   &a The first vector
/// \param   &b The second vector
float Ped::Tvector::dotProduct(const Ped::Tvector &a, const Ped::Tvector &b) {
  if(Ped::Tvector::VEC)
  {
	__m128 av = _mm_load_ps(a.xyz);
	__m128 bv = _mm_load_ps(b.xyz);
	av = _mm_mul_ps(av, bv);
	float ret[4] __attribute((aligned(16)));
	__m128 t = _mm_add_ps(av, _mm_movehl_ps(av, av));
	av = _mm_add_ss(t, _mm_shuffle_ps(t, t, 1));
	_mm_store_ps(ret, av);
	return ret[0];
/*	_mm_store_ps(ret, av);
	return (ret[0]);//et[1] + ret[2]);*/
  }
  return (a.xyz[0]*b.xyz[0] + a.xyz[1]*b.xyz[1] + a.xyz[2]*b.xyz[2]);
}


/// Calculates the cross product of two vectors.
/// \date    2010-02-12
/// \param   &a The first vector
/// \param   &b The second vector
Ped::Tvector Ped::Tvector::crossProduct(const Ped::Tvector &a, const Ped::Tvector &b) {
  if(Ped::Tvector::VEC)
  {
	float axyz1[4] __attribute__((aligned(16)))   = {a.xyz[1], a.xyz[2], a.xyz[0], .0f};
	float axyz2[4] __attribute__((aligned(16)))   = {a.xyz[2], a.xyz[0], a.xyz[1], .0f};
	float bxyz1[4] __attribute__((aligned(16)))   = {b.xyz[2], b.xyz[0], b.xyz[1], .0f};
	float bxyz2[4] __attribute__((aligned(16)))   = {b.xyz[1], b.xyz[2], b.xyz[0], .0f};
	float result1[4] __attribute__((aligned(16))) = {.0f,   .0f,  .0f, .0f};
	float result2[4] __attribute__((aligned(16))) = {.0f,   .0f,  .0f, .0f};

	__m128 av = _mm_load_ps(axyz1);
	__m128 bv = _mm_load_ps(bxyz1);
	av = _mm_mul_ps(av, bv);
	_mm_store_ps(result1, av);

	av = _mm_load_ps(axyz2);
	bv = _mm_load_ps(bxyz2);
	av = _mm_mul_ps(av, bv);
	_mm_store_ps(result2, av);
  
	return Ped::Tvector(
	  result1[0]-result2[0],
	  result1[1]-result2[1],
	  result1[2]-result2[2]);
  }
  return Ped::Tvector(
	a.xyz[1]*b.xyz[2] - a.xyz[2]*b.xyz[1],
	a.xyz[2]*b.xyz[0] - a.xyz[0]*b.xyz[2],
	a.xyz[0]*b.xyz[1] - a.xyz[1]*b.xyz[0]);
}


/// Scales this vector by a given factor in each dimension.
/// \date    2013-08-02
/// \param   factor The scalar value to multiply with.
void Ped::Tvector::scale(float factor) {
  if(Ped::Tvector::VEC)
  {
	float fact[2] __attribute__((aligned(16))) = {factor, factor};
	__m128 factv = _mm_load_ps(fact);
	__m128 xyzv = _mm_load_ps(xyz);
	xyzv = _mm_mul_ps(xyzv, factv);
	_mm_store_ps(xyz, xyzv);
  }
  else
  {
    xyz[0] *= factor;
    xyz[1] *= factor;
    xyz[2] *= factor;
  }
}


/// Returns a copy of this vector which is multiplied in each dimension by a given factor.
/// \date    2013-07-16
/// \return  The scaled vector.
/// \param   factor The scalar value to multiply with.
Ped::Tvector Ped::Tvector::scaled(float factor) const {
  if(Ped::Tvector::VEC)
  {
	float fact[2] __attribute__((aligned(16))) = {factor, factor};
	__m128 factv = _mm_load_ps(fact);
	__m128 xyzv = _mm_load_ps(xyz);
	float ret[4] __attribute__((aligned(16)));
	xyzv = _mm_mul_ps(xyzv, factv);
	_mm_store_ps(ret, xyzv);
  
	return Ped::Tvector(ret[0], ret[1], ret[2]);
  }
  return Ped::Tvector(xyz[0]*factor, xyz[1]*factor, xyz[2]*factor);
}

Ped::Tvector Ped::Tvector::leftNormalVector() const {
  return Ped::Tvector(-xyz[1], xyz[0]);
}

Ped::Tvector Ped::Tvector::rightNormalVector() const {
  return Ped::Tvector(xyz[1], -xyz[0]);
}

float Ped::Tvector::polarRadius() const {
  return length();
}

float Ped::Tvector::polarAngle() const {
  return atan2(xyz[1], xyz[0]);
}

float Ped::Tvector::angleTo(const Tvector &other) const {
  float angleThis = polarAngle();
  float angleOther = other.polarAngle();

  // compute angle
  float diffAngle = angleOther - angleThis;
  // → normalize angle
  if(diffAngle > M_PI)
	diffAngle -= 2*M_PI;
  else if(diffAngle <= -M_PI)
	diffAngle += 2*M_PI;

  return diffAngle;
}

Ped::Tvector Ped::Tvector::operator+(const Tvector& other) const {
  if(Ped::Tvector::VEC)
  {
	__m128 xyzv = _mm_load_ps(xyz);
	__m128 xyz2v = _mm_load_ps(other.xyz);
	float ret[4] __attribute__((aligned(16)));
	xyzv = _mm_add_ps(xyzv, xyz2v);
	_mm_store_ps(ret, xyzv);
	return Ped::Tvector(
	  ret[0],
	  ret[1],
	  ret[2]);
  }
  return Ped::Tvector(
	xyz[0] + other.xyz[0],
	xyz[1] + other.xyz[1],
	xyz[2] + other.xyz[2]);
}

Ped::Tvector Ped::Tvector::operator-(const Tvector& other) const {
  if(Ped::Tvector::VEC)
  {
	__m128 xyzv = _mm_load_ps(xyz);
	__m128 xyz2v = _mm_load_ps(other.xyz);
	float ret[4] __attribute__((aligned(16)));
	xyzv = _mm_sub_ps(xyzv, xyz2v);
	_mm_store_ps(ret, xyzv);
	return Ped::Tvector(
	  ret[0],
	  ret[1],
	  ret[2]);
  }
  return Ped::Tvector(
	xyz[0] - other.xyz[0],
	xyz[1] - other.xyz[1],
	xyz[2] - other.xyz[2]);
}

Ped::Tvector Ped::Tvector::operator*(float factor) const {
  return scaled(factor);
}

Ped::Tvector Ped::Tvector::operator/(float divisor) const {
  return scaled(1/divisor);
}

Ped::Tvector& Ped::Tvector::operator+=(const Tvector& vectorIn) {
  if(Ped::Tvector::VEC)
  {
	__m128 xyzv = _mm_load_ps(xyz);
	__m128 xyz2v = _mm_load_ps(vectorIn.xyz);
	xyzv = _mm_add_ps(xyzv, xyz2v);
	_mm_store_ps(xyz, xyzv);
  }
  else
  {
	xyz[0] += vectorIn.xyz[0];
    xyz[1] += vectorIn.xyz[1];
    xyz[2] += vectorIn.xyz[2];
  }
  return *this;
}

Ped::Tvector& Ped::Tvector::operator-=(const Tvector& vectorIn) {
  if(Ped::Tvector::VEC)
  {
	__m128 xyzv = _mm_load_ps(xyz);
	__m128 xyz2v = _mm_load_ps(vectorIn.xyz);
	xyzv = _mm_sub_ps(xyzv, xyz2v);
	_mm_store_ps(xyz, xyzv);
  }
  else
  {
    xyz[0] -= vectorIn.xyz[0];
    xyz[1] -= vectorIn.xyz[1];
    xyz[2] -= vectorIn.xyz[2];
  }
  return *this;
}

Ped::Tvector& Ped::Tvector::operator*=(float factor) {
  scale(factor);
  return *this;
}

Ped::Tvector& Ped::Tvector::operator*=(const Tvector& vectorIn) {
  if(Ped::Tvector::VEC)
  {
	__m128 xyzv = _mm_load_ps(xyz);
	__m128 xyz2v = _mm_load_ps(vectorIn.xyz);
	xyzv = _mm_mul_ps(xyzv, xyz2v);
	_mm_store_ps(xyz, xyzv);
  }
  else
  {
    xyz[0] *= vectorIn.xyz[0];
    xyz[1] *= vectorIn.xyz[1];
    xyz[2] *= vectorIn.xyz[2];
  }
  return *this;
}

Ped::Tvector& Ped::Tvector::operator/=(float divisor) {
  scale(1/divisor);
  return *this;
}

bool operator==(const Ped::Tvector& vector1In, const Ped::Tvector& vector2In) {
  return (vector1In.xyz[0] == vector2In.xyz[0])
	&& (vector1In.xyz[1] == vector2In.xyz[1])
	&& (vector1In.xyz[2] == vector2In.xyz[2]);
}

bool operator!=(const Ped::Tvector& vector1In, const Ped::Tvector& vector2In) {
  return (vector1In.xyz[0] != vector2In.xyz[0])
	|| (vector1In.xyz[1] != vector2In.xyz[1])
	|| (vector1In.xyz[2] != vector2In.xyz[2]);
}

Ped::Tvector operator+(const Ped::Tvector& vector1In, const Ped::Tvector& vector2In) {
  if(Ped::Tvector::VEC)
  {
	__m128 xyzv = _mm_load_ps(vector1In.xyz);
	__m128 xyz2v = _mm_load_ps(vector2In.xyz);
	float ret[4] __attribute__((aligned(16)));
	xyzv = _mm_add_ps(xyzv, xyz2v);
	_mm_store_ps(ret, xyzv);
	return Ped::Tvector(
	  ret[0],
	  ret[1],
	  ret[2]);
  }
  return Ped::Tvector(
	vector1In.xyz[0] + vector2In.xyz[0],
	vector1In.xyz[1] + vector2In.xyz[1],
	vector1In.xyz[2] + vector2In.xyz[2]);
}

Ped::Tvector operator-(const Ped::Tvector& vector1In, const Ped::Tvector& vector2In) {
  if(Ped::Tvector::VEC)
  {
	__m128 xyzv = _mm_load_ps(vector1In.xyz);
	__m128 xyz2v = _mm_load_ps(vector2In.xyz);
	float ret[4] __attribute__((aligned(16)));
	xyzv = _mm_sub_ps(xyzv, xyz2v);
	_mm_store_ps(ret, xyzv);
	return Ped::Tvector(
	  ret[0],
	  ret[1],
	  ret[2]);
  }
  return Ped::Tvector(
	vector1In.xyz[0] - vector2In.xyz[0],
	vector1In.xyz[1] - vector2In.xyz[1],
	vector1In.xyz[2] - vector2In.xyz[2]);
}

Ped::Tvector operator-(const Ped::Tvector& vectorIn) {
  return Ped::Tvector(
	-vectorIn.xyz[0],
	-vectorIn.xyz[1],
	-vectorIn.xyz[2]);
}

Ped::Tvector operator*(float factor, const Ped::Tvector& vector) {
  return vector.scaled(factor);
}

bool Ped::Tvector::VEC = false;
