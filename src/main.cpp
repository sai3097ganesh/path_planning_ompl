#include "ply.h"

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

struct CylinderModel {
  double radius;
  double halfHeight;
};

struct CubeHoleModel {
  double half;
  double holeRadius;
};

static CylinderModel estimateCylinder(const PlyMesh& mesh) {
  double minZ = std::numeric_limits<double>::infinity();
  double maxZ = -std::numeric_limits<double>::infinity();
  double maxR = 0.0;
  for (const auto& v : mesh.vertices) {
    minZ = std::min(minZ, static_cast<double>(v.z));
    maxZ = std::max(maxZ, static_cast<double>(v.z));
    double r = std::sqrt(static_cast<double>(v.x * v.x + v.y * v.y));
    maxR = std::max(maxR, r);
  }
  CylinderModel c{};
  c.halfHeight = (maxZ - minZ) * 0.5;
  c.radius = maxR;
  return c;
}

static CubeHoleModel estimateCubeHole(const PlyMesh& mesh) {
  double maxAbs = 0.0;
  for (const auto& v : mesh.vertices) {
    maxAbs = std::max(maxAbs, std::abs(static_cast<double>(v.x)));
    maxAbs = std::max(maxAbs, std::abs(static_cast<double>(v.y)));
    maxAbs = std::max(maxAbs, std::abs(static_cast<double>(v.z)));
  }

  // Estimate hole radius from the dense face grids on z = ±half.
  double holeR = 0.0;
  std::vector<double> ringR;
  ringR.reserve(mesh.vertices.size());
  for (const auto& v : mesh.vertices) {
    if (std::abs(static_cast<double>(v.z)) >= maxAbs - 1e-6) {
      double r = std::sqrt(static_cast<double>(v.x * v.x + v.y * v.y));
      if (r < maxAbs - 1e-6) {
        ringR.push_back(r);
      }
    }
  }
  if (!ringR.empty()) {
    std::sort(ringR.begin(), ringR.end());
    // take a high percentile to approximate the boundary of the hole
    size_t idx = static_cast<size_t>(ringR.size() * 0.98);
    if (idx >= ringR.size()) idx = ringR.size() - 1;
    holeR = ringR[idx];
  }

  CubeHoleModel c{};
  c.half = maxAbs;
  c.holeRadius = holeR;
  return c;
}

struct SamplePoints {
  std::vector<std::array<double, 3>> points;
};

static SamplePoints buildCylinderSamples(const CylinderModel& cyl, int radialSegments, int heightSegments) {
  SamplePoints s;
  // Surface points
  for (int i = 0; i < radialSegments; ++i) {
    double t = static_cast<double>(i) / radialSegments * 2.0 * M_PI;
    double x = cyl.radius * std::cos(t);
    double y = cyl.radius * std::sin(t);
    for (int h = 0; h <= heightSegments; ++h) {
      double z = -cyl.halfHeight + (2.0 * cyl.halfHeight) * (static_cast<double>(h) / heightSegments);
      s.points.push_back({x, y, z});
    }
  }
  // End caps ring
  for (int i = 0; i < radialSegments; ++i) {
    double t = static_cast<double>(i) / radialSegments * 2.0 * M_PI;
    double x = cyl.radius * std::cos(t);
    double y = cyl.radius * std::sin(t);
    s.points.push_back({x, y, -cyl.halfHeight});
    s.points.push_back({x, y,  cyl.halfHeight});
  }
  // Axis points
  for (int h = 0; h <= heightSegments; ++h) {
    double z = -cyl.halfHeight + (2.0 * cyl.halfHeight) * (static_cast<double>(h) / heightSegments);
    s.points.push_back({0.0, 0.0, z});
  }
  return s;
}

static void quatToMatrix(double qw, double qx, double qy, double qz, double R[3][3]) {
  double xx = qx * qx;
  double yy = qy * qy;
  double zz = qz * qz;
  double xy = qx * qy;
  double xz = qx * qz;
  double yz = qy * qz;
  double wx = qw * qx;
  double wy = qw * qy;
  double wz = qw * qz;

  R[0][0] = 1.0 - 2.0 * (yy + zz);
  R[0][1] = 2.0 * (xy - wz);
  R[0][2] = 2.0 * (xz + wy);

  R[1][0] = 2.0 * (xy + wz);
  R[1][1] = 1.0 - 2.0 * (xx + zz);
  R[1][2] = 2.0 * (yz - wx);

  R[2][0] = 2.0 * (xz - wy);
  R[2][1] = 2.0 * (yz + wx);
  R[2][2] = 1.0 - 2.0 * (xx + yy);
}

static bool isCollisionFree(const ob::State* state,
                            const CubeHoleModel& cube,
                            const SamplePoints& samples,
                            double clearance) {
  const auto* se3 = state->as<ob::SE3StateSpace::StateType>();
  double px = se3->getX();
  double py = se3->getY();
  double pz = se3->getZ();
  const auto& rot = se3->rotation();

  double R[3][3];
  quatToMatrix(rot.w, rot.x, rot.y, rot.z, R);

  for (const auto& p : samples.points) {
    double x = R[0][0] * p[0] + R[0][1] * p[1] + R[0][2] * p[2] + px;
    double y = R[1][0] * p[0] + R[1][1] * p[1] + R[1][2] * p[2] + py;
    double z = R[2][0] * p[0] + R[2][1] * p[1] + R[2][2] * p[2] + pz;

    bool insideCube = (std::abs(x) <= cube.half + clearance) &&
                      (std::abs(y) <= cube.half + clearance) &&
                      (std::abs(z) <= cube.half + clearance);
    if (insideCube) {
      double r2 = x * x + y * y;
      bool insideHole = r2 <= (cube.holeRadius + clearance) * (cube.holeRadius + clearance);
      if (!insideHole) {
        return false;
      }
    }
  }

  return true;
}

static void writePathCSV(const std::string& path, const og::PathGeometric& pg) {
  std::ofstream out(path);
  out << "x,y,z,qw,qx,qy,qz\n";
  for (std::size_t i = 0; i < pg.getStateCount(); ++i) {
    const auto* se3 = pg.getState(i)->as<ob::SE3StateSpace::StateType>();
    const auto& q = se3->rotation();
    out << se3->getX() << "," << se3->getY() << "," << se3->getZ() << ","
        << q.w << "," << q.x << "," << q.y << "," << q.z << "\n";
  }
}

static double planWithPlanner(const std::string& name,
                              og::SimpleSetup& ss,
                              const std::function<ob::PlannerPtr(const ob::SpaceInformationPtr&)>& ctor,
                              double timeSeconds,
                              const std::string& outDir) {
  ss.clear();
  ss.setPlanner(ctor(ss.getSpaceInformation()));
  auto solved = ss.solve(timeSeconds);
  if (!solved) {
    std::cerr << name << ": no solution\n";
    return std::numeric_limits<double>::infinity();
  }

  ss.simplifySolution();
  const auto& path = ss.getSolutionPath();
  double length = path.length();

  std::filesystem::create_directories(outDir);
  writePathCSV(outDir + "/" + name + "_path.csv", path);

  std::ofstream summary(outDir + "/" + name + "_summary.txt");
  summary << "planner: " << name << "\n";
  summary << "states: " << path.getStateCount() << "\n";
  summary << "length: " << length << "\n";

  std::cout << name << ": states=" << path.getStateCount() << " length=" << length << "\n";
  return length;
}

int main(int argc, char** argv) {
  std::string dataDir = "data";
  std::string outDir = "output";
  if (argc > 1) dataDir = argv[1];
  if (argc > 2) outDir = argv[2];

  PlyMesh cylinderMesh;
  PlyMesh cubeMesh;
  if (!readPlyAscii(dataDir + "/cylinder.ply", cylinderMesh)) {
    std::cerr << "Failed to read " << dataDir << "/cylinder.ply\n";
    return 1;
  }
  if (!readPlyAscii(dataDir + "/cube_with_hole.ply", cubeMesh)) {
    std::cerr << "Failed to read " << dataDir << "/cube_with_hole.ply\n";
    return 1;
  }

  CylinderModel cylinder = estimateCylinder(cylinderMesh);
  CubeHoleModel cube = estimateCubeHole(cubeMesh);

  std::cout << "Cylinder radius=" << cylinder.radius
            << " halfHeight=" << cylinder.halfHeight << "\n";
  std::cout << "Cube half=" << cube.half << " hole radius=" << cube.holeRadius << "\n";

  SamplePoints samples = buildCylinderSamples(cylinder, 24, 8);
  double clearance = 1e-4;

  auto space = std::make_shared<ob::SE3StateSpace>();
  ob::RealVectorBounds bounds(3);
  double reach = cube.half * 2.5 + cylinder.halfHeight;
  bounds.setLow(-reach);
  bounds.setHigh(reach);
  space->setBounds(bounds);

  og::SimpleSetup ss(space);
  ss.setStateValidityChecker([&](const ob::State* state) {
    return isCollisionFree(state, cube, samples, clearance);
  });

  ob::ScopedState<ob::SE3StateSpace> start(space);
  ob::ScopedState<ob::SE3StateSpace> goal(space);

  // Start: outside and rotated (horizontal)
  start->setXYZ(-cube.half * 1.2, -cube.half * 0.8, -cube.half * 2.0);
  // Rotate 90 deg around X so cylinder is horizontal
  double angle = M_PI / 2.0;
  start->rotation().w = std::cos(angle / 2.0);
  start->rotation().x = std::sin(angle / 2.0);
  start->rotation().y = 0.0;
  start->rotation().z = 0.0;

  // Goal: centered in hole, aligned with Z
  goal->setXYZ(0.0, 0.0, 0.0);
  goal->rotation().w = 1.0;
  goal->rotation().x = 0.0;
  goal->rotation().y = 0.0;
  goal->rotation().z = 0.0;

  ss.setStartAndGoalStates(start, goal);

  const double timeSeconds = 3.0;
  planWithPlanner("RRTstar", ss,
                  [](const ob::SpaceInformationPtr& si) { return std::make_shared<og::RRTstar>(si); },
                  timeSeconds, outDir);
  planWithPlanner("PRMstar", ss,
                  [](const ob::SpaceInformationPtr& si) { return std::make_shared<og::PRMstar>(si); },
                  timeSeconds, outDir);
  planWithPlanner("BITstar", ss,
                  [](const ob::SpaceInformationPtr& si) { return std::make_shared<og::BITstar>(si); },
                  timeSeconds, outDir);

  std::cout << "Paths written to " << outDir << "\n";
  return 0;
}
