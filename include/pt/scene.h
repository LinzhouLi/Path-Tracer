#pragma once

#include <pt/vector.h>

namespace pt {

struct LightInfo {
    LightInfo(const std::string& name, const Vector3f& c) : mtl_name(name), radiance(c) { }

    std::string mtl_name;
    Vector3f radiance;
};

class Scene {
public:
    Scene() { }

    ~Scene() {
        delete m_accel;
        delete m_camera;
        for (auto p : m_meshes) delete p;
        for (auto p : m_materials) delete p;
        for (auto p : m_shapes) delete p;
    }

    // Load mesh and material from OBJ file
    void loadOBJ(const std::string& filename);

    // Load camera and light description from XML file
    void loadXML(const std::string& filename);

    // Get camera
    Camera* getCamera() { return m_camera; }

    // Get mesh by name
    TriangleMesh* getMesh(const std::string& mesh_name);

    // Get material by name
    Material* getMaterial(const std::string& material_name);

    // Get mesh by index
    TriangleMesh* getMesh(const uint32_t mesh_id);

    // Get material by index
    Material* getMaterial(const uint32_t material_id);

    // Get material by intersection
    //Material* getMaterial(const Intersection& its);

    // Get all Lights
    const std::vector<AreaLight*>& getLights() const { return m_lights; }

    // Ray intersect with scene (use accelration struction)
    bool rayIntersect(const Ray& ray, Intersection& its) const;

    // if unocculded between p0 and p1
    bool unocculded(Vector3f p0, Vector3f p1, const Vector3f& n0 = Vector3f(0.0), const Vector3f& n1 = Vector3f(0.0)) const;

    // Create primitives, build accelration struction and integrator
    void preprocess();

    // Get primitives
    const std::vector<Triangle*>* getPrimitives() const { return &m_shapes; }

    // Get filter
    Filter* getFilter() const { return m_filter; }

private:
    void createPrimitives();
    void createAreaLights();

    std::vector<LightInfo> m_light_infos;

    std::vector<Triangle*> m_shapes;
    std::vector<TriangleMesh*> m_meshes;
    std::vector<Material*> m_materials;
    std::vector<AreaLight*> m_lights;

    Camera* m_camera = nullptr;
    Accel* m_accel = nullptr;
    Filter* m_filter = nullptr;
};

}