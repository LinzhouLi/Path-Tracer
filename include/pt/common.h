/*
	This file is part of Nori, a simple educational ray tracer

	Copyright (c) 2015 by Wenzel Jakob
*/

#pragma once

#if defined(_MSC_VER)
/* Disable some warnings on MSVC++ */
#pragma warning(disable : 4127 4702 4100 4515 4800 4146 4512)
#define WIN32_LEAN_AND_MEAN     /* Don't ever include MFC on Windows */
#define NOMINMAX                /* Don't override min/max */
#endif

#include <iostream>
#include <algorithm>
#include <vector>
#include <Eigen/Dense>
#include <tinyformat.h>


#define Epsilon 1e-4f

#define M_PI         3.14159265358979323846f
#define INV_PI       0.31830988618379067154f
#define INV_TWOPI    0.15915494309189533577f
#define INV_FOURPI   0.07957747154594766788f
#define SQRT_TWO     1.41421356237309504880f
#define INV_SQRT_TWO 0.70710678118654752440f

#if defined(__APPLE__ )
#define PLATFORM_MACOS
#elif defined(__linux__)
#define PLATFORM_LINUX
#elif defined(WIN32)
#define PLATFORM_WINDOWS
#endif

#ifdef PLATFORM_WINDOWS
#define DIR_SEP '\\'
#else
#define DIR_SEP '/'
#endif


namespace pt {

/* Forward declarations */
struct Normal3f;
struct Color3f;
struct Color4f;
template <typename Scalar, int Dimension>  struct TVector;
template <typename Scalar, int Dimension>  struct TPoint;
template <typename Point, typename Vector> struct TRay;
template <typename Point>                  struct TBoundingBox;

/* Basic Nori data structures (vectors, points, rays, bounding boxes,
	kd-trees) are oblivious to the underlying data type and dimension.
	The following list of typedefs establishes some convenient aliases
	for specific types. */
typedef TVector<float, 1>       Vector1f;
typedef TVector<float, 2>       Vector2f;
typedef TVector<float, 3>       Vector3f;
typedef TVector<float, 4>       Vector4f;
//typedef TVector<double, 1>      Vector1d;
//typedef TVector<double, 2>      Vector2d;
//typedef TVector<double, 3>      Vector3d;
//typedef TVector<double, 4>      Vector4d;
typedef TVector<int, 1>         Vector1i;
typedef TVector<int, 2>         Vector2i;
typedef TVector<int, 3>         Vector3i;
typedef TVector<int, 4>         Vector4i;
typedef TPoint<float, 1>        Point1f;
typedef TPoint<float, 2>        Point2f;
typedef TPoint<float, 3>        Point3f;
typedef TPoint<float, 4>        Point4f;
typedef TPoint<double, 1>       Point1d;
typedef TPoint<double, 2>       Point2d;
typedef TPoint<double, 3>       Point3d;
typedef TPoint<double, 4>       Point4d;
typedef TPoint<int, 1>          Point1i;
typedef TPoint<int, 2>          Point2i;
typedef TPoint<int, 3>          Point3i;
typedef TPoint<int, 4>          Point4i;
typedef TBoundingBox<Point1f>   BoundingBox1f;
typedef TBoundingBox<Point2f>   BoundingBox2f;
typedef TBoundingBox<Point3f>   BoundingBox3f;
typedef TBoundingBox<Point4f>   BoundingBox4f;
typedef TBoundingBox<Point1d>   BoundingBox1d;
typedef TBoundingBox<Point2d>   BoundingBox2d;
typedef TBoundingBox<Point3d>   BoundingBox3d;
typedef TBoundingBox<Point4d>   BoundingBox4d;
typedef TBoundingBox<Point1i>   BoundingBox1i;
typedef TBoundingBox<Point2i>   BoundingBox2i;
typedef TBoundingBox<Point3i>   BoundingBox3i;
typedef TBoundingBox<Point4i>   BoundingBox4i;

/// Some more forward declarations
class Ray;
class Accel;
class BSDF;
class Bitmap;
class BlockGenerator;
class Camera;
class ImageBlock;
class Integrator;
class Intersaction;
class KDTree;
class Emitter;
struct EmitterQueryRecord;
class Mesh;
class Material;
class PhaseFunction;
class ReconstructionFilter;
class Sampler;
class Scene;
class Transform;


/// Import cout, cerr, endl for debugging purposes
using std::cout;
using std::cerr;
using std::endl;

/// Simple exception class, which stores a human-readable error description
class PathTracerException : public std::runtime_error {
public:
	/// Variadic template constructor to support printf-style arguments
	template <typename... Args> PathTracerException(const char* fmt, const Args &... args)
		: std::runtime_error(tfm::format(fmt, args...)) { }
};


/// Indent a string by the specified number of spaces
extern std::string indent(const std::string& string, int amount = 2);

/// Convert a string to lower case
extern std::string toLower(const std::string& value);

/// Convert a string into an boolean value
extern bool toBool(const std::string& str);

/// Convert a string into a signed integer value
extern int toInt(const std::string& str);

/// Convert a string into an unsigned integer value
extern unsigned int toUInt(const std::string& str);

/// Convert a string into a floating point value
extern float toFloat(const std::string& str);

/// Convert a string into a 3D vector
extern Eigen::Vector3f toVector3f(const std::string& str);

/// Tokenize a string into a list by splitting at 'delim'
extern std::vector<std::string> tokenize(const std::string& s, const std::string& delim = ", ", bool includeEmpty = false);

/// Check if a string ends with another string
extern bool endsWith(const std::string& value, const std::string& ending);


/// Simple floating point clamping function
inline float clamp(float value, float min, float max) {
    if (value < min)
        return min;
    else if (value > max)
        return max;
    else return value;
}

/// Simple integer clamping function
inline int clamp(int value, int min, int max) {
    if (value < min)
        return min;
    else if (value > max)
        return max;
    else return value;
}

}