load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "motion_status_lib",
    srcs = glob(["src/motion_status.cpp"]) + ["include/motion_status.h"],
    hdrs = ["include/motion_status.h"],
    copts = [
        "-std=c++17",
        "-fPIC",
        "-DMODULE_NAME=\\\"TCMSF\\\"",
    ],
    includes = [
        "include/",
    ],
    linkstatic = True,
    deps = [
        "//cyber_release",
        "//modules/localization/src/TCMSF/tcmsf/sensor:base_lib",
        "//modules/localization/src/TCMSF/tcmsf/sensor:sins_lib",
        "//modules/localization/src/TCMSF/third_party/tomlplusplus-3.4.0:toml++",
        "@eigen",
    ],
)
