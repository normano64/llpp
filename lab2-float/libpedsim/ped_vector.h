//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//

#ifndef _ped_vector_h_
#define _ped_vector_h_ 1

#ifdef WIN32
#define LIBEXPORT __declspec(dllexport)
#else
#define LIBEXPORT
#endif

#include <string>
#include <x86intrin.h>

namespace Ped {
    /// Vector helper class. This is basically a struct with some related functions attached.
    /// x, y, and z are public, so that they can be accessed easily.
    /// \author  chgloor
    /// \date    2010-02-12
    class LIBEXPORT Tvector {
    public:
        // Default constructor
        Tvector();

        // Initializing constructor

        Tvector(float px, float py) {
            data.xyz[0] = px;
            data.xyz[1] = py;
			data.xyz[2] = .0f;
			data.xyz[3] = .0f;
        };

        // Methods
        float length() const;
        float lengthSquared() const;

        void normalize();
        Tvector normalized() const;
        void scale(float factor);
        Tvector scaled(float factor) const;

        Tvector leftNormalVector() const;
        Tvector rightNormalVector() const;

        float polarRadius() const;
        float polarAngle() const;

        float angleTo(const Tvector &other) const;

        static float scalar(const Tvector &a, const Tvector &b);
        static float dotProduct(const Tvector &a, const Tvector &b);

        std::string to_string() const;

        // Operators
        Tvector operator+(const Tvector& other) const;
        Tvector operator-(const Tvector& other) const;
        Tvector operator*(float factor) const;
        Tvector operator/(float divisor) const;

        Tvector& operator+=(const Tvector& vectorIn);
        Tvector& operator-=(const Tvector& vectorIn);
        Tvector& operator*=(float factor);
        Tvector& operator*=(const Tvector& vectorIn);
        Tvector& operator/=(float divisor);

        inline float getX() const { return data.xyz[0]; }
        inline float getY() const { return data.xyz[1]; }

        void setX(float x) { data.xyz[0] = x;}
        void setY(float y) { data.xyz[1] = y;}
		Tvector rounded();
		operator __m128() { return data.vector; }
		operator const __m128() const { return data.vector; }
        static bool VEC;
	private:
		// Attributes
		union{
		  float xyz[4];
		  __m128 vector;
		} data;
    };
}

bool operator==(const Ped::Tvector& vector1In, const Ped::Tvector& vector2In);
bool operator!=(const Ped::Tvector& vector1In, const Ped::Tvector& vector2In);
Ped::Tvector operator-(const Ped::Tvector& vectorIn);
Ped::Tvector operator*(float factor, const Ped::Tvector& vector);

#endif
