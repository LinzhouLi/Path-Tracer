#pragma once

#include <pt/common.h>

namespace pt {

class TriangleMesh {
public:
    TriangleMesh(const std::string& name = "") : m_name(name) { }

    TriangleMesh(
        const std::string& name, 

        const std::vector<Vector3f>& vertices,
        const std::vector<Vector3f>& normals,
        const std::vector<Vector2f>& uvs,

        const std::vector<uint32_t>& vertex_ids,
        const std::vector<uint32_t>& normal_ids,
        const std::vector<uint32_t>& uv_ids,
        const std::vector<uint32_t>& mtl_ids
    ) : m_name(name), m_vertices(vertices), m_normals(normals), m_uvs(uvs), 
        m_vertex_ids(vertex_ids), m_normal_ids(normal_ids) , m_uv_ids(uv_ids), m_mtl_ids(mtl_ids) { }

    std::string getName() { return m_name; }

    size_t getVertexCount() { return m_vertices.size(); }

    size_t getTriangleCount() { return m_vertex_ids.size() / 3; }

    void getVertex(uint32_t face_id, Vector3f& v0, Vector3f& v1, Vector3f& v2) {
        v0 = m_vertices[m_vertex_ids[3 * face_id + 0] - 1];
        v1 = m_vertices[m_vertex_ids[3 * face_id + 1] - 1];
        v2 = m_vertices[m_vertex_ids[3 * face_id + 2] - 1];
    }

    bool getNormal(uint32_t face_id, Vector3f& n0, Vector3f& n1, Vector3f& n2) {
        if (m_normals.empty()) return false;
        if (m_normal_ids[3 * face_id] == 0) return false;
        n0 = m_normals[m_normal_ids[3 * face_id + 0] - 1];
        n1 = m_normals[m_normal_ids[3 * face_id + 1] - 1];
        n2 = m_normals[m_normal_ids[3 * face_id + 2] - 1];
        return true;
    }

    bool getUV(uint32_t face_id, Vector2f& uv0, Vector2f& uv1, Vector2f& uv2) {
        if (m_uvs.empty()) return false;
        if (m_uv_ids[3 * face_id] == 0) return false;
        uv0 = m_uvs[m_uv_ids[3 * face_id + 0] - 1];
        uv1 = m_uvs[m_uv_ids[3 * face_id + 1] - 1];
        uv2 = m_uvs[m_uv_ids[3 * face_id + 2] - 1];
        return true;
    }

    uint32_t getMaterialId(uint32_t face_id) { return m_mtl_ids[face_id]; } 

private:
    std::string m_name;

    std::vector<Vector3f> m_vertices;
    std::vector<Vector3f> m_normals;
    std::vector<Vector2f> m_uvs;

    std::vector<uint32_t> m_vertex_ids, m_normal_ids, m_uv_ids;
    std::vector<uint32_t> m_mtl_ids;
};

}