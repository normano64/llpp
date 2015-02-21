//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//

#include "ped_vector.h"

#include <cmath>
#include <string>
#include <sstream>
#include <x86intrin.h>
#include <iostream>
/// Default constructor, which makes sure that all the values are set to 0.
/// \date    2012-01-16
Ped::Tvector::Tvector()
{
  data.xyz[0] = .0f;
  data.xyz[1] = .0f;
  data.xyz[2] = .0f;
  data.xyz[3] = .0f;
}

std::string Ped::Tvector::to_string() const {
  float x_ = data.xyz[0];
  float y_ = data.xyz[1];
  float z_ = data.xyz[2];
  std::ostringstream strs;
  strs << x_ << "/" << y_ << "/" << z_;
  return strs.str();
  //return std::to_string((const long float) x) + "/" + std::to_string((const long float) y_) + "/" + std::to_string((const long float) z_);
}

/// Returns the length of the vector.
/// \return the length
// float Ped::Tvector::length() const {
//     if ((data.xyz[0] == 0) && (data.xyz[1] == 0) && (data.xyz[2] == 0)) return 0;
//     return sqrt(lengthSquared());
// }

// /// Returns the length of the vector squared. This is faster than the real length.
// /// \return the length squared
float Ped::Tvector::lengthSquared() const {
  if(Ped::Tvector::VEC)
  {
	__m128 ret = _mm_mul_ps(data.vector, data.vector);
	return ret[0]+ret[1];
  }
  return data.xyz[0]*data.xyz[0] + data.xyz[1]*data.xyz[1];
}

float Ped::Tvector::length() const {
  if ((data.xyz[0] == 0) && (data.xyz[1] == 0)) return 0;
  return sqrt(lengthSquared());
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
	__m128 lnv = _mm_load_ps1(&len);
	data.vector = _mm_div_ps(data.vector, lnv);
  }
  else
  {
	data.xyz[0] /= len;
	data.xyz[1] /= len;
  }
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
	__m128 lnv  = _mm_load_ps1(&len);
	__m128 ret  = _mm_div_ps(data.vector, lnv);
	return Ped::Tvector(ret[0], ret[1]);
  }
  return Ped::Tvector(data.xyz[0]/len, data.xyz[1]/len);
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
    __m128 ret = _mm_mul_ps(a.data.vector, a.data.vector);
	return (ret[0] + ret[1]);
  }
  return (a.data.xyz[0]*b.data.xyz[0] + a.data.xyz[1]*b.data.xyz[1]);
}

/// Scales this vector by a given factor in each dimension.
/// \date    2013-08-02
/// \param   factor The scalar value to multiply with.
void Ped::Tvector::scale(float factor) {
  if(Ped::Tvector::VEC)
  {
	// float fact[2] __attribute__((aligned(16))) = {factor, factor};
	__m128 factv = _mm_load_ps1(&factor);
	data.vector = _mm_mul_ps(data.vector, factv);
  }
  else
  {
	data.xyz[0] *= factor;
	data.xyz[1] *= factor;
  }
}

/// Returns a copy of this vector which is multiplied in each dimension by a given factor.
/// \date    2013-07-16
/// \return  The scaled vector.
/// \param   factor The scalar value to multiply with.
Ped::Tvector Ped::Tvector::scaled(float factor) const {
  if(Ped::Tvector::VEC)
  {
	// float fact[2] __attribute__((aligned(16))) = {factor, factor};
	__m128 factv = _mm_load_ps(&factor);
	__m128 ret = _mm_mul_ps(data.vector, factv);
  
	return Ped::Tvector(ret[0], ret[1]);
  }
  return Ped::Tvector(data.xyz[0]*factor, data.xyz[1]*factor);
}

Ped::Tvector Ped::Tvector::leftNormalVector() const {
  return Ped::Tvector(-data.xyz[1], data.xyz[0]);
}

Ped::Tvector Ped::Tvector::rightNormalVector() const {
  return Ped::Tvector(data.xyz[1], -data.xyz[0]);
}

float Ped::Tvector::polarRadius() const {
  return length();
}

float Ped::Tvector::polarAngle() const {
  return atan2(data.xyz[1], data.xyz[0]);
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
	__m128 ret = _mm_add_ps(data.vector, other.data.vector);
	return Ped::Tvector(
	  ret[0],
	  ret[1]);
  }
  return Ped::Tvector(
	data.xyz[0] + other.data.xyz[0],
	data.xyz[1] + other.data.xyz[1]);
}

Ped::Tvector Ped::Tvector::operator-(const Tvector& other) const {
  if(Ped::Tvector::VEC)
  {
	__m128 ret = _mm_sub_ps(data.vector, other.data.vector);
	return Ped::Tvector(
	  ret[0],
	  ret[1]);
  }
  return Ped::Tvector(
	data.xyz[0] - other.data.xyz[0],
	data.xyz[1] - other.data.xyz[1]);
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
	data.vector = _mm_add_ps(data.vector, vectorIn.data.vector);
  }
  else
  {
	data.xyz[0] += vectorIn.data.xyz[0];
	data.xyz[1] += vectorIn.data.xyz[1];
  }
  return *this;
}

Ped::Tvector& Ped::Tvector::operator-=(const Tvector& vectorIn) {
  if(Ped::Tvector::VEC)
  {
	data.vector = _mm_sub_ps(data.vector, vectorIn.data.vector);
  }
  else
  {
	data.xyz[0] -= vectorIn.data.xyz[0];
	data.xyz[1] -= vectorIn.data.xyz[1];
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
	data.vector = _mm_mul_ps(data.vector, vectorIn.data.vector);
  }
  else
  {
	data.xyz[0] *= vectorIn.data.xyz[0];
	data.xyz[1] *= vectorIn.data.xyz[1];
  }
  return *this;
}

Ped::Tvector& Ped::Tvector::operator/=(float divisor) {
  scale(1/divisor);
  return *this;
}

bool operator==(const Ped::Tvector& vector1In, const Ped::Tvector& vector2In) {
  return (vector1In.getX() == vector2In.getX())
	&& (vector1In.getY() == vector2In.getY());
}

bool operator!=(const Ped::Tvector& vector1In, const Ped::Tvector& vector2In) {
  return (vector1In.getX() != vector2In.getX())
	|| (vector1In.getY()!= vector2In.getY());
}

Ped::Tvector operator+(const Ped::Tvector& vector1In, const Ped::Tvector& vector2In) {
  if(Ped::Tvector::VEC)
  {
	__m128 ret = _mm_add_ps(vector1In,  vector2In);
	return Ped::Tvector(
	  ret[0],
	  ret[1]);
  }
  return Ped::Tvector(
	vector1In.getX() + vector2In.getX(),
	vector1In.getY() + vector2In.getY());
}

Ped::Tvector operator-(const Ped::Tvector& vector1In, const Ped::Tvector& vector2In) {
  if(Ped::Tvector::VEC)
  {
	__m128 ret = _mm_sub_ps(vector1In, vector2In);
	return Ped::Tvector(
	  ret[0],
	  ret[1]);
  }
  return Ped::Tvector(
	vector1In.getX() - vector2In.getX(),
	vector1In.getY() - vector2In.getY());
}


Ped::Tvector operator-(const Ped::Tvector& vectorIn) {
  return Ped::Tvector(
	-vectorIn.getX(),
	-vectorIn.getY());
}

Ped::Tvector operator*(float factor, const Ped::Tvector& vector) {
  return vector.scaled(factor);
}

Ped::Tvector Ped::Tvector::rounded()
{
  return Ped::Tvector(round(data.xyz[0]), round(data.xyz[1]));
}

bool Ped::Tvector::VEC = false;
