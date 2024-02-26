#pragma once

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

private:
    std::string m_name;

    std::vector<TriVertex> m_tri_vertices;
    std::vector<TriNormal> m_tri_normals;
    std::vector<TriUV> m_tri_uvs;
    std::vector<uint32_t> m_mat_ids;
};

}