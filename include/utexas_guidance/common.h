#ifndef UTEXAS_GUIDANCE_COMMON_H
#define UTEXAS_GUIDANCE_COMMON_H

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>

#include "cross_product.hpp"

#include <GL/freeglut.h>

namespace utexas_guidance {

  typedef boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian> Point3f;

  inline void drawLine(const Point3f& loc1,
                       const Point3f& loc2,
                       float color_r = 1.0f,
                       float color_g = 1.0f,
                       float color_b = 1.0f,
                       float line_width = 0.1f,
                       bool arrow = false,
                       bool dashed = false,
                       float approx_dash_length = 0.5f,
                       float scale = 1.0f) {

    // This is the default direction for the cylinders to face in OpenGL
    Point3f z;
    z.set<0>(0.0f);
    z.set<1>(0.0f);
    z.set<2>(1.0f);

    // Get diff between two points you want cylinder along
    Point3f loc1_mutable(loc1);
    boost::geometry::subtract_point(loc1_mutable, loc2);

    // Get CROSS product (the axis of rotation)
    Point3f t = boost::geometry::cross_product(z, loc1_mutable);

    // Get angle. LENGTH is magnitude of the vector
    float angle = (180.f / M_PI) * acosf(boost::geometry::dot_product(z, loc1_mutable) /
                                         boost::geometry::distance(loc1, loc2));

    float total_length = boost::geometry::distance(loc1, loc2);
    float dash_length = total_length;
    int num_segments = 1;
    if (dashed) {
      num_segments = floor(total_length / approx_dash_length);
      if (num_segments % 2 == 0) {
        ++num_segments;
      }
      dash_length = total_length / float(num_segments);
    }

    float cone_length = 5.0f * line_width;
    cone_length = std::min(cone_length, total_length);
    float cone_width = cone_length / 2.0f;
    cone_width = std::max(cone_width, 2.0f * line_width);

    glPushMatrix();
    glTranslatef(loc2.get<0>() * scale, loc2.get<1>() * scale, loc2.get<2>() * scale);
    glRotatef(angle, t.get<0>(), t.get<1>(), t.get<2>());
    for (int i = 0; i < num_segments; i=i+2) {
      float length_reduction = 0.0f;
      if (i == num_segments - 1 && arrow) {
        length_reduction = (cone_length / cone_width) * line_width;
      }
      glutSolidCylinder(line_width * scale, (dash_length - length_reduction) * scale, 10, 1);
      glTranslatef(0.0f, 0.0f, 2.0f * dash_length * scale);
    }

    if (arrow) {
      glTranslatef(0.0f, 0.0f, -1.0f * (dash_length + cone_length) * scale);
      glutSolidCone(cone_width * scale, cone_length * scale, 10, 1);
    }

    glPopMatrix();

  }

} /* utexas_guidance */

#endif /* end of include guard: UTEXAS_GUIDANCE_COMMON_H */
