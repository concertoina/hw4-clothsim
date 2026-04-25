#include "iostream"
#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../clothSimulator.h"
#include "plane.h"

using namespace std;
using namespace CGL;

#define SURFACE_OFFSET 0.0001

void Plane::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with planes.
  //check which side of the plane we are on.
  double old_side = dot(pm.last_position - point, normal);
  double new_side = dot(pm.position - point, normal);

  // Crossed the plane if signs differ
  if (old_side * new_side <= 0) {
    Vector3D direction = pm.position - pm.last_position;

    // tangent_point where line segment intersects plane
    double t = dot(point - pm.last_position, normal) / dot(direction, normal);

    Vector3D tangent_point = pm.last_position + t * direction;

    Vector3D offset = normal * SURFACE_OFFSET;

    if (old_side < 0) {
      offset = -offset;
    }

    Vector3D target_position = tangent_point + offset;

    Vector3D correction = target_position - pm.last_position;

    pm.position = pm.last_position + correction * (1.0 - friction);
  }

}

void Plane::render(GLShader &shader) {
  nanogui::Color color(0.7f, 0.7f, 0.7f, 1.0f);

  Vector3f sPoint(point.x, point.y, point.z);
  Vector3f sNormal(normal.x, normal.y, normal.z);
  Vector3f sParallel(normal.y - normal.z, normal.z - normal.x,
                     normal.x - normal.y);
  sParallel.normalize();
  Vector3f sCross = sNormal.cross(sParallel);

  MatrixXf positions(3, 4);
  MatrixXf normals(3, 4);

  positions.col(0) << sPoint + 2 * (sCross + sParallel);
  positions.col(1) << sPoint + 2 * (sCross - sParallel);
  positions.col(2) << sPoint + 2 * (-sCross + sParallel);
  positions.col(3) << sPoint + 2 * (-sCross - sParallel);

  normals.col(0) << sNormal;
  normals.col(1) << sNormal;
  normals.col(2) << sNormal;
  normals.col(3) << sNormal;

  if (shader.uniform("u_color", false) != -1) {
    shader.setUniform("u_color", color);
  }
  shader.uploadAttrib("in_position", positions);
  if (shader.attrib("in_normal", false) != -1) {
    shader.uploadAttrib("in_normal", normals);
  }

  shader.drawArray(GL_TRIANGLE_STRIP, 0, 4);
}
