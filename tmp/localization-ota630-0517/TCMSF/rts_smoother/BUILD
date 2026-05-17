load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rts_smoother_lib",
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
        "-DMODULE_NAME=\\\"TCMSF_RTS\\\"",
    ],
    includes = [
        "impl/include/",
        "interface/include/",
    ],
    linkstatic = True,
    deps = [
        "//cyber_release",
        "//modules/localization/src/TCMSF/tcmsf/sensor:sins_lib",
        "//modules/localization/src/TCMSF/tcmsf/util:geotrans",
        "//modules/localization/src/TCMSF/tcmsf/util:rigid_lib",
        "//modules/localization/src/TCMSF/tcmsf/sensor:base_lib",
        "@eigen",
        "@fmt",
    ],
    alwayslink = True,
)
