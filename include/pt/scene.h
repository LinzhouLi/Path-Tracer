#pragma once

#include <pt/common.h>

namespace pt {

class Scene {
public:
    Scene() { }

    void loadOBJ(const std::string& filename);

private:
    std::vector<Point3f> vertices;
    std::vector<Point3i> faces;
    std::vector<int> face2mesh;

    std::vector<Mesh*> m_meshes;
    Integrator* m_integrator = nullptr;
    Sampler* m_sampler = nullptr;
    Camera* m_camera = nullptr;
    Accel* m_accel = nullptr;
};

}