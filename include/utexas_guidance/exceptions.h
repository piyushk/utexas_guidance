#ifndef UTEXAS_GUIDANCE_EXCEPTIONS_H
#define UTEXAS_GUIDANCE_EXCEPTIONS_H

#include <stdexcept>

namespace utexas_guidance {

  class IncorrectUsageException : public std::runtime_error {
    public:
      IncorrectUsageException(const std::string& message) : std::runtime_error(message) {}
  };

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_GUIDANCE_EXCEPTIONS_H */
