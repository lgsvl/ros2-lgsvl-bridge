/**
 * Copyright (c) 2020 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */

#include "server.h"
#include "node.h"
#include "logging.h"

#include <cstdlib>
#include <memory>
#include <rcl/rcl.h>
#include <rcutils/logging.h>
#include <rcutils/cmdline_parser.h>

int main(int argc, char* argv[])
{
    char* loglevel = rcutils_cli_get_option(argv, argv + argc, "--log");
    if (loglevel)
    {
        int level = RCUTILS_LOG_SEVERITY_WARN;
        if (loglevel[0] == 'D') level = RCUTILS_LOG_SEVERITY_DEBUG;
        else if (loglevel[0] == 'I') level = RCUTILS_LOG_SEVERITY_INFO;
        else if (loglevel[0] == 'W') level = RCUTILS_LOG_SEVERITY_WARN;
        else if (loglevel[0] == 'E') level = RCUTILS_LOG_SEVERITY_ERROR;
        else if (loglevel[0] == 'F') level = RCUTILS_LOG_SEVERITY_FATAL;
        rcutils_logging_set_default_logger_level(level);
    }

    unsigned int port = 9090;
    char* portstr = rcutils_cli_get_option(argv, argv + argc, "--port");
    if (portstr)
    {
        port = std::strtoul(portstr, NULL, 10);
    }

    rcl_context_t context = rcl_get_zero_initialized_context();
    rcl_init_options_t init = rcl_get_zero_initialized_init_options();
    rcl_allocator_t alloc = rcl_get_default_allocator();
    rcl_ret_t rc;

    rc = rcl_init_options_init(&init, alloc);
    if (rc != RCL_RET_OK)
    {
        ERROR("rcl_init_options_init failed: " << rc);
        return EXIT_FAILURE;
    }

    rc = rcl_init(0, NULL, &init, &context);
    if (rc != RCL_RET_OK)
    {
        ERROR("rcl_init failed: " << rc);
        return EXIT_FAILURE;
    }

    {
        Node node(&alloc, &context);
        auto server = std::make_shared<Server>(node, port);
        server->run();
    }

    rc = rcl_init_options_fini(&init);
    if (rc != RCL_RET_OK)
    {
        ERROR("rcl_init_options_fini failed: " << rc);
    }
}
