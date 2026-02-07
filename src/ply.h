#pragma once

#include <string>
#include <vector>

struct PlyMesh {
  struct Vertex {
    float x, y, z;
  };
  struct Face {
    std::vector<int> indices;
  };

  std::vector<Vertex> vertices;
  std::vector<Face> faces;
};

bool readPlyAscii(const std::string& path, PlyMesh& out);
bool writePlyAscii(const std::string& path, const PlyMesh& mesh);
