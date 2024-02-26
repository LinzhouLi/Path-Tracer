#pragma once

#include <pt/vector.h>

namespace pt {

class Ray {
public:
    Vector3f org;
    Vector3f dir;
    Vector3f dir_rcp;
    float min_dis;
    float max_dis;

    Ray() : min_dis(Epsilon), max_dis(std::numeric_limits<float>::infinity()) { }

    Ray(const Ray& ray) : org(ray.org), dir(ray.dir), 
        dir_rcp(ray.dir_rcp), min_dis(ray.min_dis), max_dis(ray.max_dis) { }

    Ray(
        const Vector3f& o, 
        const Vector3f& d, 
        float min_d = Epsilon, 
        float max_d = std::numeric_limits<float>::infinity()
    ) : org(o), dir(d), min_dis(min_d), max_dis(max_d) {
        update();
    }

    void update() {
        dir_rcp = dir.cwiseInverse();
    }

    Ray reverse() {
        Ray result;
        result.org = org; result.dir = -dir; result.dir_rcp = -dir_rcp;
        result.min_dis = min_dis; result.max_dis = max_dis;
        return result;
    }

    std::string toString() const {
        return tfm::format(
                "Ray[\n"
                "  o = %s,\n"
                "  d = %s,\n"
                "  mint = %f,\n"
                "  maxt = %f\n"
                "]", 
            org.toString(), dir.toString(), min_dis, max_dis
        );
    }
};

}
