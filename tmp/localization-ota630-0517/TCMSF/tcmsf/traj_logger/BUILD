load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "traj_logger_lib",
    srcs = ["src/traj_logger.cpp"],
    hdrs = ["include/traj_logger.h"],
    copts = [
        "-std=c++17",
        "-DMODULE_NAME=\\\"TCMSF_TRAJ\\\"",
    ],
    includes = [
        "include/",
    ],
    linkstatic = True,
    deps = [
        "//cyber_release",
        "@fmt",
    ],
    alwayslink = True,
)

cc_binary(
    name = "traj_parser",
    srcs = [
        "include/traj_logger.h",
        "src/traj_parser.cpp",
    ],
    copts = [
        "--std=c++17",
    ],
    deps = [
        ":traj_logger_lib",
        # "//cyber_release",
        "@fmt",
    ],
)
