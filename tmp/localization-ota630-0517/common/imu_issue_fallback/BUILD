load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "imu_issue_fallback_lib",
    srcs = glob(
        [
            "impl/src/*.cpp",
            "interface/src/*.cpp",
        ],
    ),
    hdrs = glob(
        [
            "impl/include/*.h",
            "interface/include/*.h",
        ],
    ),
    copts = [
        "-std=c++17",
        "-DMODULE_NAME=\\\"TCMSF_IIF\\\"",
    ],
    includes = [
        "impl/include/",
        "interface/include/",
    ],
    linkstatic = True,
    deps = [
        "//cyber_release",
        "//modules/msg/drivers_msgs:imu_cc_proto",
        "//modules/msg/drivers_msgs:veh_info_cc_proto",
        "@eigen",
        "@fmt",
    ],
    alwayslink = True,
)
