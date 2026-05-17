load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "filter_lib",
    srcs = glob(
        [
            "**/src/*.cpp",
        ],
    ),
    hdrs = glob(
        [
            "**/include/*.h",
        ],
    ),
    copts = [
        "-std=c++17",
    ],
    includes = [
        "include/",
    ],
    linkstatic = True,
    deps = [
        "@eigen",
        "@fmt",
    ],
    alwayslink = True,
)
