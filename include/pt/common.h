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

#include <optional>
#include <iostream>
#include <algorithm>
#include <vector>

#include <pt/vector.h>
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
struct Color3f;
struct Color4f;
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
class AreaLight;
class Filter;
class TangentSpace;
class UniformLightSelector;


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

template <typename T>
inline T mix(const T& a, const T& b, float f) {
    return a * (1.0 - f) + b * f;
}

}