#include "ply.h"

#include <cmath>
#include <filesystem>
#include <iostream>

static void addFaceGridWithHole(PlyMesh& mesh, float half, float z, float holeRadius, int grid) {
  const int n = grid;
  const float step = (2.0f * half) / n;

  // Build vertices for this face
  std::vector<int> idx((n + 1) * (n + 1));
  for (int j = 0; j <= n; ++j) {
    for (int i = 0; i <= n; ++i) {
      float x = -half + i * step;
      float y = -half + j * step;
      idx[j * (n + 1) + i] = static_cast<int>(mesh.vertices.size());
      mesh.vertices.push_back({x, y, z});
    }
  }

  auto addTri = [&](int a, int b, int c) {
    PlyMesh::Face f;
    f.indices = {a, b, c};
    mesh.faces.push_back(std::move(f));
  };

  auto inHole = [&](float x, float y) {
    return (x * x + y * y) <= (holeRadius * holeRadius);
  };

  for (int j = 0; j < n; ++j) {
    for (int i = 0; i < n; ++i) {
      int v00 = idx[j * (n + 1) + i];
      int v10 = idx[j * (n + 1) + (i + 1)];
      int v01 = idx[(j + 1) * (n + 1) + i];
      int v11 = idx[(j + 1) * (n + 1) + (i + 1)];

      const auto& p00 = mesh.vertices[v00];
      const auto& p10 = mesh.vertices[v10];
      const auto& p01 = mesh.vertices[v01];
      const auto& p11 = mesh.vertices[v11];

      // Triangle 1
      float cx1 = (p00.x + p10.x + p11.x) / 3.0f;
      float cy1 = (p00.y + p10.y + p11.y) / 3.0f;
      if (!inHole(cx1, cy1)) {
        if (z > 0) {
          addTri(v00, v10, v11);
        } else {
          addTri(v00, v11, v10);
        }
      }

      // Triangle 2
      float cx2 = (p00.x + p11.x + p01.x) / 3.0f;
      float cy2 = (p00.y + p11.y + p01.y) / 3.0f;
      if (!inHole(cx2, cy2)) {
        if (z > 0) {
          addTri(v00, v11, v01);
        } else {
          addTri(v00, v01, v11);
        }
      }
    }
  }
}

static void addCube(PlyMesh& mesh, float half, float holeRadius, int grid) {
  size_t base = mesh.vertices.size();
  mesh.vertices.push_back({-half, -half, -half}); // 0
  mesh.vertices.push_back({ half, -half, -half}); // 1
  mesh.vertices.push_back({ half,  half, -half}); // 2
  mesh.vertices.push_back({-half,  half, -half}); // 3
  mesh.vertices.push_back({-half, -half,  half}); // 4
  mesh.vertices.push_back({ half, -half,  half}); // 5
  mesh.vertices.push_back({ half,  half,  half}); // 6
  mesh.vertices.push_back({-half,  half,  half}); // 7

  auto addFace = [&](int a, int b, int c) {
    PlyMesh::Face f;
    f.indices = {static_cast<int>(base + a), static_cast<int>(base + b), static_cast<int>(base + c)};
    mesh.faces.push_back(std::move(f));
  };

  // -Z and +Z handled by grid with circular hole
  addFaceGridWithHole(mesh, half, -half, holeRadius, grid);
  addFaceGridWithHole(mesh, half,  half, holeRadius, grid);
  // -Y
  addFace(0, 5, 1);
  addFace(0, 4, 5);
  // +Y
  addFace(3, 2, 6);
  addFace(3, 6, 7);
  // -X
  addFace(0, 3, 7);
  addFace(0, 7, 4);
  // +X
  addFace(1, 5, 6);
  addFace(1, 6, 2);
}

static void addCylinderSurface(PlyMesh& mesh, float radius, float halfHeight, int segments) {
  size_t base = mesh.vertices.size();
  for (int i = 0; i < segments; ++i) {
    float t = static_cast<float>(i) / segments * 2.0f * static_cast<float>(M_PI);
    float x = radius * std::cos(t);
    float y = radius * std::sin(t);
    mesh.vertices.push_back({x, y, -halfHeight});
    mesh.vertices.push_back({x, y,  halfHeight});
  }

  auto addFace = [&](int a, int b, int c) {
    PlyMesh::Face f;
    f.indices = {static_cast<int>(base + a), static_cast<int>(base + b), static_cast<int>(base + c)};
    mesh.faces.push_back(std::move(f));
  };

  for (int i = 0; i < segments; ++i) {
    int i0 = i * 2;
    int i1 = ((i + 1) % segments) * 2;
    // side quads as two triangles
    addFace(i0, i1, i1 + 1);
    addFace(i0, i1 + 1, i0 + 1);
  }
}

int main(int argc, char** argv) {
  std::string outDir = "data";
  if (argc > 1) outDir = argv[1];

  std::filesystem::create_directories(outDir);

  const float cubeHalf = 0.5f;
  const float holeRadius = 0.2f;
  const float cylRadius = 0.2f;
  const float cylHalfHeight = 0.4f;
  const int segments = 48;
  const int grid = 60;

  PlyMesh cubeWithHole;
  addCube(cubeWithHole, cubeHalf, holeRadius, grid);
  addCylinderSurface(cubeWithHole, holeRadius, cubeHalf, segments);
  if (!writePlyAscii(outDir + "/cube_with_hole.ply", cubeWithHole)) {
    std::cerr << "Failed to write cube_with_hole.ply\n";
    return 1;
  }

  PlyMesh cylinder;
  addCylinderSurface(cylinder, cylRadius, cylHalfHeight, segments);
  if (!writePlyAscii(outDir + "/cylinder.ply", cylinder)) {
    std::cerr << "Failed to write cylinder.ply\n";
    return 1;
  }

  std::cout << "Wrote PLY files to " << outDir << "\n";
  return 0;
}
