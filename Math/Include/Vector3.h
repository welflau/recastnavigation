#ifndef VECTOR3_H
#define VECTOR3_H

#include <algorithm>
#include "FloatConversion.h"
// #include "Runtime/Serialize/SerializeUtility.h"
// #include "Runtime/Serialize/SerializationMetaFlags.h"
// #include "Runtime/Logging/LogAssert.h"
// #include "Runtime/Modules/ExportModules.h"

class Vector3f
{
public:

    float x, y, z;

//     DEFINE_GET_TYPESTRING_IS_ANIMATION_CHANNEL(Vector3f)
    template<class TransferFunction> void Transfer(TransferFunction& transfer);

    Vector3f() {}  // Default ctor is intentionally empty for performance reasons
    Vector3f(const Vector3f& v) : x(v.x), y(v.y), z(v.z) {}   // Necessary for correct optimized GCC codegen
    Vector3f(float inX, float inY, float inZ)  { x = inX; y = inY; z = inZ; }
	Vector3f(float v[3]) { x = v[0]; y = v[1]; z = v[2]; }
    explicit Vector3f(const float* array)  { x = array[0]; y = array[1]; z = array[2]; }
    void Set(float inX, float inY, float inZ)  { x = inX; y = inY; z = inZ; }
    void Set(const float* array)   { x = array[0]; y = array[1]; z = array[2]; }
    void SetZero()                 { x = 0.0f; y = 0.0f; z = 0.0f; }

    float* GetPtr()                                { return &x; }
    const float* GetPtr() const                     { return &x; }
     float& operator[](int i)                       
	 {  
		 if (i >= 0 && i <= 2)
			 return (&x)[i];
		 else
			 return (&x)[0];
	 }
     const float& operator[](int i) const            
	 { 
		 if (i >= 0 && i <= 2)
			 return (&x)[i];
		 else
			 return (&x)[0];
	 }

    bool operator==(const Vector3f& v) const       { return x == v.x && y == v.y && z == v.z; }
    bool operator!=(const Vector3f& v) const       { return x != v.x || y != v.y || z != v.z; }

    Vector3f& operator+=(const Vector3f& inV)     { x += inV.x; y += inV.y; z += inV.z; return *this; }
    Vector3f& operator-=(const Vector3f& inV)     { x -= inV.x; y -= inV.y; z -= inV.z; return *this; }
    Vector3f& operator*=(float s)                 { x *= s; y *= s; z *= s; return *this; }
    Vector3f& operator/=(float s);

    Vector3f operator-() const                    { return Vector3f(-x, -y, -z); }

    Vector3f& Scale(const Vector3f& inV)           { x *= inV.x; y *= inV.y; z *= inV.z; return *this; }

	inline float Length() const { return (float)sqrt(x*x + y*y + z*z); }
	inline float LengthSqr() const { return (float)(x*x + y*y + z*z); }


	static const float        epsilon;
	static const float        infinity;
	static const Vector3f infinityVec;
	static const Vector3f zero;
	static const Vector3f one;
	static const Vector3f xAxis;
	static const Vector3f yAxis;
	static const Vector3f zAxis;
};

inline Vector3f Scale(const Vector3f& lhs, const Vector3f& rhs) { return Vector3f(lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z); }

inline Vector3f operator+(const Vector3f& lhs, const Vector3f& rhs)   { return Vector3f(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z); }
inline Vector3f operator-(const Vector3f& lhs, const Vector3f& rhs)   { return Vector3f(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z); }
inline Vector3f Cross(const Vector3f& lhs, const Vector3f& rhs);
inline float Dot(const Vector3f& lhs, const Vector3f& rhs)                 { return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z; }
inline float Dot2D(const Vector3f& lhs, const Vector3f& rhs) { return lhs.x * rhs.x + lhs.z * rhs.z; }
inline float Volume(const Vector3f& inV)                               { return inV.x * inV.y * inV.z; }

inline Vector3f operator*(const Vector3f& inV, const float s)         { return Vector3f(inV.x * s, inV.y * s, inV.z * s); }
inline Vector3f operator*(const float s, const Vector3f& inV)         { return Vector3f(inV.x * s, inV.y * s, inV.z * s); }
inline Vector3f operator/(const Vector3f& inV, const float s)         { Vector3f temp(inV); temp /= s; return temp; }
inline Vector3f Inverse(const Vector3f& inVec)                                 { return Vector3f(1.0F / inVec.x, 1.0F / inVec.y, 1.0F / inVec.z); }

inline float SqrMagnitude(const Vector3f& inV)                                 { return Dot(inV, inV); }
inline float Magnitude(const Vector3f& inV)                            {return SqrtImpl(Dot(inV, inV)); }

// Normalizes a vector, asserts if the vector can be normalized
inline Vector3f Normalize(const Vector3f& inV)                                 { return inV / Magnitude(inV); }
// Normalizes a vector, returns default vector if it can't be normalized
inline Vector3f NormalizeSafe(const Vector3f& inV, const Vector3f& defaultV = Vector3f::zero);

inline Vector3f ReflectVector(const Vector3f& inDirection, const Vector3f& inNormal)   { return -2.0F * Dot(inNormal, inDirection) * inNormal + inDirection; }

inline Vector3f Lerp(const Vector3f& from, const Vector3f& to, float t) { return to * t + from * (1.0F - t); }
Vector3f Slerp(const Vector3f& from, const Vector3f& to, float t);

// Returns a vector with the smaller of every component from v0 and v1
inline Vector3f min(const Vector3f& lhs, const Vector3f& rhs)          { return Vector3f(FloatMin(lhs.x, rhs.x), FloatMin(lhs.y, rhs.y), FloatMin(lhs.z, rhs.z)); }
// Returns a vector with the larger  of every component from v0 and v1
inline Vector3f max(const Vector3f& lhs, const Vector3f& rhs)              { return Vector3f(FloatMax(lhs.x, rhs.x), FloatMax(lhs.y, rhs.y), FloatMax(lhs.z, rhs.z)); }

/// Project one vector onto another.
inline Vector3f Project(const Vector3f& v1, const Vector3f& v2)    { return v2 * Dot(v1, v2) / Dot(v2, v2); }


/// Returns the abs of every component of the vector
inline Vector3f Abs(const Vector3f& v) { return Vector3f(Abs(v.x), Abs(v.y), Abs(v.z)); }

bool CompareApproximately(const Vector3f& inV0, const Vector3f& inV1, const float inMaxDist = Vector3f::epsilon);
// Orthonormalizes the three vectors, assuming that a orthonormal basis can be formed
void OrthoNormalizeFast(Vector3f* inU, Vector3f* inV, Vector3f* inW);
// Orthonormalizes the three vectors, returns false if no orthonormal basis could be formed.
void OrthoNormalize(Vector3f* inU, Vector3f* inV, Vector3f* inW);
// Orthonormalizes the two vectors. inV is taken as a hint and will try to be as close as possible to inV.
void OrthoNormalize(Vector3f* inU, Vector3f* inV);

// Calculates a vector that is orthonormal to n.
// Assumes that n is normalized
Vector3f OrthoNormalVectorFast(const Vector3f& n);

// Rotates lhs towards rhs by no more than max Angle
// Moves the magnitude of lhs towards rhs by no more than maxMagnitude
Vector3f RotateTowards(const Vector3f& lhs, const Vector3f& rhs, float maxAngle, float maxMagnitude);

// Spherically interpolates the direction of two vectors
// and interpolates the magnitude of the two vectors
Vector3f Slerp(const Vector3f& lhs, const Vector3f& rhs, float t);

/// Returns a Vector3 that moves lhs towards rhs by a maximum of clampedDistance
Vector3f MoveTowards(const Vector3f& lhs, const Vector3f& rhs, float clampedDistance);

inline bool IsNormalized(const Vector3f& vec, float epsilon = Vector3f::epsilon)
{
    return CompareApproximately(SqrMagnitude(vec), 1.0F, epsilon);
}

inline Vector3f Cross(const Vector3f& lhs, const Vector3f& rhs)
{
    return Vector3f(
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x);
}

inline Vector3f NormalizeSafe(const Vector3f& inV, const Vector3f& defaultV)
{
    float mag = Magnitude(inV);
    if (mag > Vector3f::epsilon)
        return inV / mag;
    else
        return defaultV;
}

/// - Handles zero vector correclty
inline Vector3f NormalizeFast(const Vector3f& inV)
{
    float m = SqrMagnitude(inV);
    // GCC version of __frsqrte:
    //  static inline double __frsqrte (double x) {
    //      double y;
    //      asm ( "frsqrte %0, %1" : /*OUT*/ "=f" (y) : /*IN*/ "f" (x) );
    //      return y;
    //  }
    return inV * FastInvSqrt(m);
}

/// - low precision normalize
/// - nan for zero vector
inline Vector3f NormalizeFastest(const Vector3f& inV)
{
    float m = SqrMagnitude(inV);
    // GCC version of __frsqrte:
    //  static inline double __frsqrte (double x) {
    //      double y;
    //      asm ( "frsqrte %0, %1" : /*OUT*/ "=f" (y) : /*IN*/ "f" (x) );
    //      return y;
    //  }
    return inV * FastestInvSqrt(m);
}

inline bool IsFinite(const Vector3f& f)
{
    return IsFinite(f.x) & IsFinite(f.y) & IsFinite(f.z);
}

inline Vector3f Round(const Vector3f& a, float factor)
{
    return Vector3f(Roundf(a.x / factor) * factor, Roundf(a.y / factor) * factor, Roundf(a.z / factor) * factor);
}

inline bool CompareApproximately(const Vector3f& inV0, const Vector3f& inV1, const float inMaxDist)
{
    return SqrMagnitude(inV1 - inV0) <= inMaxDist * inMaxDist;
}

inline Vector3f& Vector3f::operator/=(float s)
{
//     DebugAssert(!CompareApproximately(s, 0.0F));
	if (!CompareApproximately(s, 0.0F))
	{
		x /= s;
		y /= s;
		z /= s;
		return *this;
	}
	else
		return *this;;
}

template<class TransferFunction>
inline void Vector3f::Transfer(TransferFunction& t)
{
    t.AddMetaFlag(kTransferUsingFlowMappingStyle);
    t.Transfer(x, "x");
    t.Transfer(y, "y");
    t.Transfer(z, "z");
}

inline float SqrDistance(const Vector3f& a, const Vector3f& b)
{
	return SqrMagnitude(b - a);
}


// this may be called for vectors `a' with extremely small magnitude, for
// example the result of a cross product on two nearly perpendicular vectors.
// we must be robust to these small vectors. to prevent numerical error,
// first find the component a[i] with the largest magnitude and then scale
// all the components by 1/a[i]. then we can compute the length of `a' and
// scale the components by 1/l. this has been verified to work with vectors
// containing the smallest representable numbers.
Vector3f NormalizeRobust(const Vector3f& a);
// This also returns vector's inverse original length, to avoid duplicate
// invSqrt calculations when needed. If a is a zero vector, invOriginalLength will be 0.
Vector3f NormalizeRobust(const Vector3f& a, float &invOriginalLength);


float SqrDistancePointSegment2D(float* t, const Vector3f& pt, const Vector3f& s1, const Vector3f& s2);
bool ClosestHeightPointTriangle(float* h, const Vector3f& p, const Vector3f& a, const Vector3f& b, const Vector3f& c);

#endif
