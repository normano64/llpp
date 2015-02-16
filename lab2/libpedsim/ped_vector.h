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
        Tvector(float px, float py, float pz = .0f) 
		{
		  xyz[0] = px;
		  xyz[1] = py;
		  xyz[2] = pz;
		  xyz[3] = .0f;
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
        static Tvector crossProduct(const Tvector &a, const Tvector &b);

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


        // Attributes
		float xyz[4] __attribute__((aligned(16)));
/*        float x;
        float y;
        float z;*/

		inline float getX() const { return xyz[0]; }
		inline float getY() const { return xyz[1]; }
		inline float getZ() const { return xyz[2]; }
	    void setX(float x) { xyz[0] = x;}
		void setY(float y) { xyz[1] = y;}
	    void setZ(float z) { xyz[2] = z;}
		static bool VEC;
    };
}

bool operator==(const Ped::Tvector& vector1In, const Ped::Tvector& vector2In);
bool operator!=(const Ped::Tvector& vector1In, const Ped::Tvector& vector2In);
Ped::Tvector operator-(const Ped::Tvector& vectorIn);
Ped::Tvector operator*(float factor, const Ped::Tvector& vector);

#endif
