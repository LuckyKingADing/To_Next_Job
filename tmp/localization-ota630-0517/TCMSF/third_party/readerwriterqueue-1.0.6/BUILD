load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "readerwriterqueue",
    hdrs = glob([
        "include/*.h",
    ]),
    copts = [
        "-std=c++11",
    ],
    includes = [
        "include/",
    ],
)
