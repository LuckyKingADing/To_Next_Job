load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "iir",
    srcs = glob(["iir/*.cpp"]) + glob(["iir/*.h"]),
    hdrs = ["Iir.h"],
    copts = [
        "-Wall",
        "-pedantic",
        "-std=c++11",
    ],
    includes = ["./"],
    linkstatic = True,
    alwayslink = True,
)
