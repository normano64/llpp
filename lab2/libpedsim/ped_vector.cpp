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
    xyz[0] = .0;
    xyz[1] = .0;
    xyz[2] = .0;
}

std::string Ped::Tvector::to_string() const {
    double x_ = xyz[0];
    double y_ = xyz[1];
    double z_ = xyz[2];
    std::ostringstream strs;
    strs << x_ << "/" << y_ << "/" << z_;
    return strs.str();
    //return std::to_string((const long double) x) + "/" + std::to_string((const long double) y_) + "/" + std::to_string((const long double) z_);
}

/// Returns the length of the vector.
/// \return the length
// double Ped::Tvector::length() const {
//     if ((xyz[0] == 0) && (xyz[1] == 0) && (xyz[2] == 0)) return 0;
//     return sqrt(lengthSquared());
// }

// /// Returns the length of the vector squared. This is faster than the real length.
// /// \return the length squared
double Ped::Tvector::lengthSquared() const {
    // if(Ped::Tvector::VEC)
    // {
    //       __m128d vec1 = _mm_load_pd(xyz);
    //       double ret[2] __attribute__((aligned(16)));
    //       vec1 = _mm_mul_pd(vec1, vec1);
    //       _mm_store_pd(ret, vec1);
    //       return ret[0]+ret[1]+xyz[2]*xyz[2];
    // }
    return xyz[0]*xyz[0] + xyz[1]*xyz[1] + xyz[2]*xyz[2];
}

double Ped::Tvector::length() const {
    if ((xyz[0] == 0) && (xyz[1] == 0) && (xyz[2] == 0)) return 0;
    return sqrt(xyz[0]*xyz[0] + xyz[1]*xyz[1] + xyz[2]*xyz[2]);
}

/// Normalizes the vector to a length of 1.
/// \date    2010-02-12
void Ped::Tvector::normalize() {
    double len = length();

    // null vectors cannot be normalized
    if (len == 0)
        return;
    if(Ped::Tvector::VEC)
        {
            // double ln __attribute__((aligned(16))) = len;
            __m128d vec1 = _mm_load_pd(xyz);
            __m128d lnv  = _mm_load_pd1(&len);
            vec1 = _mm_div_pd(vec1, lnv);
            _mm_store_pd(xyz, vec1);
            xyz[2] /= len;
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
    double len = length();

    // null vectors cannot be normalized
    if (len == 0)
	return Ped::Tvector();;	
    if(Ped::Tvector::VEC)
        {
            __m128d vec1 = _mm_load_pd(xyz);
            // double ln __attribute__((aligned(16))) = len;
            __m128d lnv  = _mm_load_pd1(&len);
            double ret[2] __attribute__((aligned(16)));
            vec1 = _mm_div_pd(vec1, lnv);
            _mm_store_pd(ret, vec1);
            return Ped::Tvector(ret[0], ret[1], xyz[2]/len);
        }
    return Ped::Tvector(xyz[0]/len, xyz[1]/len, xyz[2]/len);
}

/// Vector scalar product helper: calculates the scalar product of two vectors.
/// \date    2012-01-14
/// \return  The scalar product.
/// \param   &a The first vector
/// \param   &b The second vector
double Ped::Tvector::scalar(const Ped::Tvector &a, const Ped::Tvector &b) {
    return acos( Ped::Tvector::dotProduct(a, b) / ( a.length() * b.length() ) );
}

/// Vector scalar product helper: calculates the scalar product of two vectors.
/// \date    2012-01-14
/// \return  The scalar product.
/// \param   &a The first vector
/// \param   &b The second vector
double Ped::Tvector::dotProduct(const Ped::Tvector &a, const Ped::Tvector &b) {
    if(Ped::Tvector::VEC)
        {
            __m128d av1 = _mm_load_pd(a.xyz);
            __m128d av2 = _mm_load_pd(&(a.xyz[2]));
            __m128d bv1 = _mm_load_pd(b.xyz);
            __m128d bv2 = _mm_load_pd(&(b.xyz[2]));
            av1 = _mm_mul_pd(av1, bv1);
            av2 = _mm_mul_pd(av2, bv2);
            double ret[4] __attribute((aligned(16)));
            _mm_store_pd(ret, av1);
            _mm_store_pd(&(ret[2]), av2);
            return (ret[0] + ret[1] + ret[2]);
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
            double axyz1[4] __attribute__((aligned(16)))   = {a.xyz[1], a.xyz[2], a.xyz[0], .0};
            double axyz2[4] __attribute__((aligned(16)))   = {a.xyz[2], a.xyz[0], a.xyz[1], .0};
            double bxyz1[4] __attribute__((aligned(16)))   = {b.xyz[2], b.xyz[0], b.xyz[1], .0};
            double bxyz2[4] __attribute__((aligned(16)))   = {b.xyz[1], b.xyz[2], b.xyz[0], .0};
            double result1[4] __attribute__((aligned(16))) = {.0,   .0,  .0, .0};
            double result2[4] __attribute__((aligned(16))) = {.0,   .0,  .0, .0};

            __m128d av = _mm_load_pd(axyz1);
            __m128d bv = _mm_load_pd(bxyz1);
            av = _mm_mul_pd(av, bv);
            _mm_store_pd(result1, av);
  
            av = _mm_load_pd(&(axyz1[2]));
            bv = _mm_load_pd(&(bxyz1[2]));
            av = _mm_mul_pd(av, bv);
            _mm_store_pd(&(result1[2]), av);

            av = _mm_load_pd(axyz2);
            bv = _mm_load_pd(bxyz2);
            av = _mm_mul_pd(av, bv);
            _mm_store_pd(result2, av);

            av = _mm_load_pd(&(axyz2[2]));
            bv = _mm_load_pd(&(bxyz2[2]));
            _mm_store_pd(&(result2[2]), av);
  
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
void Ped::Tvector::scale(double factor) {
    if(Ped::Tvector::VEC)
        {
            // double fact[2] __attribute__((aligned(16))) = {factor, factor};
            __m128d factv = _mm_load_pd1(&factor);
            __m128d xyzv = _mm_load_pd(xyz);
            xyzv = _mm_mul_pd(xyzv, factv);
            _mm_store_pd(xyz, xyzv);
            xyz[2] *= factor;
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
Ped::Tvector Ped::Tvector::scaled(double factor) const {
    if(Ped::Tvector::VEC)
        {
            // double fact[2] __attribute__((aligned(16))) = {factor, factor};
            __m128d factv = _mm_load_pd(&factor);
            __m128d xyzv = _mm_load_pd(xyz);
            double ret[4] __attribute__((aligned(16)));
            xyzv = _mm_mul_pd(xyzv, factv);
            _mm_store_pd(ret, xyzv);
            xyzv = _mm_load_pd(&(xyz[2]));
            xyzv = _mm_mul_pd(xyzv, factv);
            _mm_store_pd(&(ret[2]), xyzv);
  
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

double Ped::Tvector::polarRadius() const {
    return length();
}

double Ped::Tvector::polarAngle() const {
    return atan2(xyz[1], xyz[0]);
}

double Ped::Tvector::angleTo(const Tvector &other) const {
    double angleThis = polarAngle();
    double angleOther = other.polarAngle();

    // compute angle
    double diffAngle = angleOther - angleThis;
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
            __m128d xyzv = _mm_load_pd(xyz);
            __m128d xyz2v = _mm_load_pd(other.xyz);
            double ret[2] __attribute__((aligned(16)));
            xyzv = _mm_add_pd(xyzv, xyz2v);
            _mm_store_pd(ret, xyzv);
            return Ped::Tvector(
                                ret[0],
                                ret[1],
                                xyz[2] + other.xyz[2]);
        }
    return Ped::Tvector(
                        xyz[0] + other.xyz[0],
                        xyz[1] + other.xyz[1],
                        xyz[2] + other.xyz[2]);
}

Ped::Tvector Ped::Tvector::operator-(const Tvector& other) const {
    if(Ped::Tvector::VEC)
        {
            __m128d xyzv = _mm_load_pd(xyz);
            __m128d xyz2v = _mm_load_pd(other.xyz);
            double ret[2] __attribute__((aligned(16)));
            xyzv = _mm_sub_pd(xyzv, xyz2v);
            _mm_store_pd(ret, xyzv);
            return Ped::Tvector(
                                ret[0],
                                ret[1],
                                xyz[2] - other.xyz[2]);
        }
    return Ped::Tvector(
                        xyz[0] - other.xyz[0],
                        xyz[1] - other.xyz[1],
                        xyz[2] - other.xyz[2]);
}

Ped::Tvector Ped::Tvector::operator*(double factor) const {
    return scaled(factor);
}

Ped::Tvector Ped::Tvector::operator/(double divisor) const {
    return scaled(1/divisor);
}

Ped::Tvector& Ped::Tvector::operator+=(const Tvector& vectorIn) {
    if(Ped::Tvector::VEC)
        {
            __m128d xyzv = _mm_load_pd(xyz);
            __m128d xyz2v = _mm_load_pd(vectorIn.xyz);
            xyzv = _mm_add_pd(xyzv, xyz2v);
            _mm_store_pd(xyz, xyzv);
            xyzv = _mm_load_pd(&(xyz[2]));
            xyz2v = _mm_load_pd(&(vectorIn.xyz[2]));
            xyzv = _mm_add_pd(xyzv, xyz2v);
            _mm_store_pd(&(xyz[2]), xyzv);
            xyz[2] += vectorIn.xyz[2];
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
            __m128d xyzv = _mm_load_pd(xyz);
            __m128d xyz2v = _mm_load_pd(vectorIn.xyz);
            xyzv = _mm_sub_pd(xyzv, xyz2v);
            _mm_store_pd(xyz, xyzv);
            xyzv = _mm_load_pd(&(xyz[2]));
            xyz2v = _mm_load_pd(&(vectorIn.xyz[2]));
            xyzv = _mm_sub_pd(xyzv, xyz2v);
            _mm_store_pd(&(xyz[2]), xyzv);
        }
    else
        {
            xyz[0] -= vectorIn.xyz[0];
            xyz[1] -= vectorIn.xyz[1];
            xyz[2] -= vectorIn.xyz[2];
        }
    return *this;
}

Ped::Tvector& Ped::Tvector::operator*=(double factor) {
    scale(factor);
    return *this;
}

Ped::Tvector& Ped::Tvector::operator*=(const Tvector& vectorIn) {
    if(Ped::Tvector::VEC)
        {
            __m128d xyzv = _mm_load_pd(xyz);
            __m128d xyz2v = _mm_load_pd(vectorIn.xyz);
            xyzv = _mm_mul_pd(xyzv, xyz2v);
            _mm_store_pd(xyz, xyzv);
            xyzv = _mm_load_pd(&(xyz[2]));
            xyz2v = _mm_load_pd(&(vectorIn.xyz[2]));
            xyzv = _mm_mul_pd(xyzv, xyz2v);
            _mm_store_pd(&(xyz[2]), xyzv);
        }
    else
        {
            xyz[0] *= vectorIn.xyz[0];
            xyz[1] *= vectorIn.xyz[1];
            xyz[2] *= vectorIn.xyz[2];
        }
    return *this;
}

Ped::Tvector& Ped::Tvector::operator/=(double divisor) {
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
            __m128d xyzv = _mm_load_pd(vector1In.xyz);
            __m128d xyz2v = _mm_load_pd(vector2In.xyz);
            double ret[4] __attribute__((aligned(16)));
            xyzv = _mm_add_pd(xyzv, xyz2v);
            _mm_store_pd(ret, xyzv);
            xyzv = _mm_load_pd(&(vector1In.xyz[2]));
            xyz2v = _mm_load_pd(&(vector2In.xyz[2]));
            xyzv = _mm_add_pd(xyzv, xyz2v);
            _mm_store_pd(&(ret[2]), xyzv);
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
            __m128d xyzv = _mm_load_pd(vector1In.xyz);
            __m128d xyz2v = _mm_load_pd(vector2In.xyz);
            double ret[4] __attribute__((aligned(16)));
            xyzv = _mm_sub_pd(xyzv, xyz2v);
            _mm_store_pd(ret, xyzv);
            xyzv = _mm_load_pd(&(vector1In.xyz[2]));
            xyz2v = _mm_load_pd(&(vector2In.xyz[2]));
            xyzv = _mm_sub_pd(xyzv, xyz2v);
            _mm_store_pd(&(ret[2]), xyzv);
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

Ped::Tvector operator*(double factor, const Ped::Tvector& vector) {
    return vector.scaled(factor);
}

bool Ped::Tvector::VEC = false;
