#ifndef UTEXAS_GUIDANCE_COMMON_H
#define UTEXAS_GUIDANCE_COMMON_H

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>

#include <GL/freeglut.h>

namespace utexas_guidance {

  typedef boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian> Point3f;

  inline void drawLine(Point3f loc1,
                Point3f loc2,
                bool dashed = false
                float color_r = 1.0f,
                float color_g = 1.0f,
                float color_b = 1.0f) {


  }

  
} /* utexas_guidance */

#endif /* end of include guard: UTEXAS_GUIDANCE_COMMON_H */
