load("@rules_cc//cc:defs.bzl", "cc_library")

config_setting(
    name = "x86_mode",
    values = {"cpu": "k8"},
)

config_setting(
    name = "arm_mode",
    values = {"cpu": "aarch64"},
)

cc_library(
    name = "wgs84_to_mars",
    srcs = select({
        "x86_mode": ["lib/x86/libkcoords_plugin.so"],
        "arm_mode": ["lib/arm/libkcoords_plugin.so"],
    }),
    hdrs = ["include/Coord.h"],
    includes = [
        "./include",
    ],
    visibility = ["//visibility:public"],
)
