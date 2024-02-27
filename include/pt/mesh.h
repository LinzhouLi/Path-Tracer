#pragma once

#include <pt/vector.h>
#include <pt/common.h>

namespace pt {

class Mesh {
public:
    using TriVertex = std::array< Vector3f, 3 >;
    using TriNormal = std::array< Vector3f, 3 >;
    using TriUV = std::array< Vector2f, 3 >;

    Mesh(const std::string& name = "") : m_name(name) { }

    Mesh(
        const std::string& name, 
        const std::vector<TriVertex>& vertices,
        const std::vector<TriNormal>& normals,
        const std::vector<TriUV>& uvs,
        const std::vector<uint32_t>& mat_ids
    ) : m_name(name), m_tri_vertices(vertices), m_tri_normals(normals), m_tri_uvs(uvs), m_mat_ids(mat_ids){ }

    std::string getName() { return m_name; }

    size_t getTriangleCount() { return m_tri_vertices.size(); }

    const TriVertex& getTriangle(uint32_t i) { return m_tri_vertices[i]; }

    const TriNormal& getNormal(uint32_t i) { return m_tri_normals[i]; }

    const TriUV& getUV(uint32_t i) { return m_tri_uvs[i]; }

    const uint32_t& getMaterialId(uint32_t i) { return m_mat_ids[i]; }

private:
    std::string m_name;

    std::vector<TriVertex> m_tri_vertices;
    std::vector<TriNormal> m_tri_normals;
    std::vector<TriUV> m_tri_uvs;
    std::vector<uint32_t> m_mat_ids;
};

float triangleSurfaceArea(const Mesh::TriVertex& triangle);

bool rayTriangleIntersect(const Mesh::TriVertex& triangle, const Ray& ray, Vector3f& bary, float& t);

class Intersaction {
public:
    void setInfo(Mesh* mesh, uint32_t triangle_idx, const Vector3f& bary);

    void complete();

    Vector3f pos;
    Vector3f normal;
    Vector2f uv;
    uint32_t mat_id;

private:
    Mesh* m_mesh;
    uint32_t m_triangle_idx;
    Vector3f m_bary;
};

}