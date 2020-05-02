/**
 * Copyright (c) 2020 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */

#pragma once

#include <sstream>
#include <rcutils/logging.h>

#define LOG(x) \
  do { \
      std::stringstream ss; \
      ss << x; \
      rcutils_log(NULL, RCUTILS_LOG_SEVERITY_INFO, "lgsvl-bridge", "%s", ss.str().c_str()); \
  } while (0)

#define DEBUG(x) \
  do { \
      std::stringstream ss; \
      ss << x; \
      rcutils_log(NULL, RCUTILS_LOG_SEVERITY_DEBUG, "lgsvl-bridge", "%s", ss.str().c_str()); \
  } while (0)

#define ERROR(x) \
  do { \
      std::stringstream ss; \
      ss << x; \
      rcutils_log(NULL, RCUTILS_LOG_SEVERITY_ERROR, "lgsvl-bridge", "%s", ss.str().c_str()); \
  } while (0)

