#include <iostream>
#include <map>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

using namespace std;
using namespace OpenMesh;

typedef TriMesh_ArrayKernelT<> Mesh;

struct TrianglePoint {
    Mesh::Point point;
    Mesh::VertexHandle vertex;
};

using TrianglePoints = array<TrianglePoint, 3>;
using IrregularEdges = map<pair<Mesh::VertexHandle, Mesh::VertexHandle>, Mesh::VertexHandle>;

short longest_edge(const TrianglePoints& t) {
    const auto e1 = (t[0].point - t[1].point).norm();
    const auto e2 = (t[1].point - t[2].point).norm();
    const auto e3 = (t[2].point - t[0].point).norm();

    if (e1 >= e2) {
        return e1 >= e3 ? 0 : 2;
    } else {
        return e2 >= e3 ? 1 : 2;
    }
}

TrianglePoints triangle_points(const Mesh& mesh, const Mesh::FaceHandle& triangle) {
    auto iter = mesh.cfv_ccwiter(triangle);
    TrianglePoints t;

    Mesh::VertexHandle h = *iter;
    t[0] = {mesh.point(h), h};

    h = *(++iter);
    t[1] = {mesh.point(h), h};

    h = *(++iter);
    t[2] = {mesh.point(h), h};

    return t;
}

void refine(Mesh& m, set<Mesh::FaceHandle> faces) {
    IrregularEdges irregular_edges;
    bool repair_mode = false;

    while (!faces.empty() || !irregular_edges.empty()) {
        if (faces.empty() && !irregular_edges.empty()) {
            repair_mode = true;

            for (const auto i : irregular_edges) {
                const auto& edge = i.first;
                const auto& f = m.face_handle(m.find_halfedge(edge.first, edge.second));

                faces.insert(f);
            }
        }

        const auto f = *faces.begin();
        const auto t = triangle_points(m, f);
        faces.erase(f);

        // determine edge
        short e;
        if (repair_mode) {
            /// TODO: Handle irregular faces in a better way.
            // Now they are not split along the longest edge, but otherwise their will be holes.
            if (irregular_edges.count({t[0].vertex, t[1].vertex}) == 1) {
                e = 0;
            } else if (irregular_edges.count({t[1].vertex, t[2].vertex}) == 1) {
                e = 1;
            } else if (irregular_edges.count({t[2].vertex, t[0].vertex}) == 1) {
                e = 2;
            } else {
                assert(false);
            }
        } else {
            e = longest_edge(t);
        }

        const auto& left = t[e];
        const auto& right = t[(e + 1) % t.size()];
        const auto edge = make_pair(left.vertex, right.vertex);

        // split
        const auto repair = irregular_edges.count(edge) == 1;
        const auto midpoint = repair ? irregular_edges[edge]
                                     : m.add_vertex((left.point + right.point) / 2.0);
        const auto& opposite = t[(e + 2) % t.size()];

        Mesh::Color face_color;
        if (m.has_face_colors()) {
           face_color = m.color(f);
        }

        m.delete_face(f, false);
        const auto first_half = m.add_face(opposite.vertex, left.vertex, midpoint);
        const auto second_half = m.add_face(opposite.vertex, midpoint, right.vertex);

        if (m.has_face_colors()) {
            m.set_color(first_half, face_color);
            m.set_color(second_half, face_color);
        }

        if (repair) {
            irregular_edges.erase(edge);
        } else {
            if (m.has_vertex_texcoords2D()) {
                m.set_texcoord2D(midpoint, (m.texcoord2D(edge.first) + m.texcoord2D(edge.second)) / 2.0);
            }

            const auto opposite_half_edge = m.find_halfedge(edge.second, edge.first);
            if (opposite_half_edge.is_valid() && !m.is_boundary(opposite_half_edge)) {
                irregular_edges.insert(make_pair(make_pair(edge.second, edge.first), midpoint));
            }
        }
    }

    m.garbage_collection();
}

void test_vertex_split() {
    Mesh m;

    vector<Mesh::VertexHandle> v{m.add_vertex({0, 0, 0}), m.add_vertex({1, -2, 0}),
                                 m.add_vertex({2,0,0}), m.add_vertex({1,1,0})};
    m.request_edge_status();
    m.request_face_status();
    m.request_vertex_texcoords2D();

    auto f = m.add_face(v[0], v[2], v[3]);
    m.add_face(v[0], v[1], v[2]);

    m.set_texcoord2D(v[0], {0, 0.5});
    m.set_texcoord2D(v[1], {0.5, 0});
    m.set_texcoord2D(v[2], {1, 0});
    m.set_texcoord2D(v[3], {0.5, 1});

    refine(m, {f});

    IO::write_mesh(m, "vertex_split.obj", IO::Options::VertexTexCoord);
}

int main() {
    test_vertex_split();
    return 0;
}

