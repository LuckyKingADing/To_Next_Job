load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "sophus",
    hdrs = glob([
        "sophus/*.hpp",
    ]),
    includes = [
        "./",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@eigen",
    ],
)
