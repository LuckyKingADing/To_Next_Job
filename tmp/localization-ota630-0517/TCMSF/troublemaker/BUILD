load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "troublemaker_lib",
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
        "-DMODULE_NAME=\\\"TCMSF\\\"",
    ],
    includes = [
        "impl/include/",
        "interface/include/",
    ],
    linkstatic = True,
    deps = [
        "//cyber_release",
        "//modules/localization/src/TCMSF/tcmsf/sensor:base_lib",
        "//modules/localization/src/TCMSF/tcmsf/util:rigid_lib",
        "@eigen",
    ],
    alwayslink = True,
)
