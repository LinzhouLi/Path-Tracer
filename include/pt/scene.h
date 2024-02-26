#pragma once

#include <pt/common.h>

namespace pt {

class Scene {
public:
    Scene() { }
    ~Scene() {
        delete m_accel;
        delete m_sampler;
        delete m_camera;
        delete m_integrator;
        for (auto p : m_meshes) delete p;
        for (auto p : m_materials) delete p;
    }

    // Load mesh and material from OBJ file
    void loadOBJ(const std::string& filename);

    // Load camera and light description from XML file
    void loadXML(const std::string& filename);

    // Get mesh by name
    Mesh* getMesh(const std::string& mesh_name);

    // Get material by name
    Material* getMaterial(const std::string& material_name);

    // Get mesh by index
    Mesh* getMesh(const uint32_t mesh_id);

    // Get material by index
    Material* getMaterial(const uint32_t material_id);

private:
    std::vector<Mesh*> m_meshes;
    std::vector<Material*> m_materials;

    Integrator* m_integrator = nullptr;
    Sampler* m_sampler = nullptr;
    Camera* m_camera = nullptr;
    Accel* m_accel = nullptr;
};

}