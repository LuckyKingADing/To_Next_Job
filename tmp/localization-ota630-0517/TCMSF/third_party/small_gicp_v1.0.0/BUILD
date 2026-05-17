load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "small_gicp",
    srcs = glob(["src/**/*.cpp"]),
    hdrs = glob(
        ["include/**/*.hpp"],
        exclude = ["include/pcl/*.hpp"],
    ),
    copts = [
        "-std=c++17",
        "-fopenmp",
    ],
    includes = ["include"],
    linkopts = ["-fopenmp"],
    linkstatic = True,
    deps = [
        "@eigen",
        "@fmt",
    ],
    alwayslink = True,
)
