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

static constexpr float Epsilon = 1e-4f;
static constexpr float M_PI = 3.14159265358979323846f;
static constexpr float INV_PI = 0.31830988618379067154f;
static constexpr float INV_TWOPI = 0.15915494309189533577f;
static constexpr float INV_FOURPI = 0.07957747154594766788f;
static constexpr float SQRT_TWO = 1.41421356237309504880f;
static constexpr float INV_SQRT_TWO = 0.70710678118654752440f;
static int threadCount = -1;

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

/// Some more forward declarations
class AABB;
class Ray;
class Accel;
class Bitmap;
class BlockGenerator;
class Camera;
class ImageBlock;
class Integrator;
class Intersection;
class BVHTree;
class TriangleMesh;
class Material;
class Sampler;
class Scene;
class Transform;
class Triangle;


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