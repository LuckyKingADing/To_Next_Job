load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "fail_detect_lib",
    srcs = glob(
        [
            "src/*.cpp",
            "include/*.h",
        ],
    ),
    hdrs = ["include/fail_detect_interface.h"],
    copts = [
        "-std=c++17",
        "-Wno-unused-but-set-variable",
        "-Wno-unused-variable",
        "-Wno-sign-compare",
        "-lpthread",
        "-DMODULE_NAME=\\\"TCMSF\\\"",
    ],
    includes = [
        "include/",
    ],
    linkstatic = True,
    deps = [
        "//cyber_release",
        "//modules/localization/src/TCMSF/tcmsf/sensor:base_lib",
        "@eigen",
        "@fmt",
        "//modules/localization/src/TCMSF/tcmsf/util:geotrans",
        # "@glog_gflags//:glog",
    ],
    alwayslink = True,
)
