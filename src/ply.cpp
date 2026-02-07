#include "ply.h"

#include <fstream>
#include <sstream>
#include <string>

bool readPlyAscii(const std::string& path, PlyMesh& out) {
  std::ifstream in(path);
  if (!in.is_open()) return false;

  std::string line;
  size_t vertexCount = 0;
  size_t faceCount = 0;
  bool header = true;

  while (std::getline(in, line)) {
    if (!header) break;
    if (line.rfind("element vertex ", 0) == 0) {
      std::istringstream ss(line);
      std::string tmp;
      ss >> tmp >> tmp >> vertexCount;
    } else if (line.rfind("element face ", 0) == 0) {
      std::istringstream ss(line);
      std::string tmp;
      ss >> tmp >> tmp >> faceCount;
    } else if (line == "end_header") {
      header = false;
      break;
    }
  }

  if (header) return false;

  out.vertices.clear();
  out.faces.clear();
  out.vertices.reserve(vertexCount);
  out.faces.reserve(faceCount);

  for (size_t i = 0; i < vertexCount; ++i) {
    if (!std::getline(in, line)) return false;
    std::istringstream ss(line);
    PlyMesh::Vertex v{};
    ss >> v.x >> v.y >> v.z;
    out.vertices.push_back(v);
  }

  for (size_t i = 0; i < faceCount; ++i) {
    if (!std::getline(in, line)) return false;
    std::istringstream ss(line);
    int n = 0;
    ss >> n;
    PlyMesh::Face f;
    f.indices.resize(n);
    for (int j = 0; j < n; ++j) ss >> f.indices[j];
    out.faces.push_back(std::move(f));
  }

  return true;
}

bool writePlyAscii(const std::string& path, const PlyMesh& mesh) {
  std::ofstream out(path);
  if (!out.is_open()) return false;

  out << "ply\n";
  out << "format ascii 1.0\n";
  out << "element vertex " << mesh.vertices.size() << "\n";
  out << "property float x\n";
  out << "property float y\n";
  out << "property float z\n";
  out << "element face " << mesh.faces.size() << "\n";
  out << "property list uchar int vertex_indices\n";
  out << "end_header\n";

  for (const auto& v : mesh.vertices) {
    out << v.x << " " << v.y << " " << v.z << "\n";
  }

  for (const auto& f : mesh.faces) {
    out << f.indices.size();
    for (int idx : f.indices) out << " " << idx;
    out << "\n";
  }

  return true;
}
