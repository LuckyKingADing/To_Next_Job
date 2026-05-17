load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "toml++",
    hdrs = glob([
        "include/toml++/*.h*",
        "include/toml++/impl/*.*",
    ]),
    copts = [
        "-std=c++17",
    ],
    includes = [
        "include/",
    ],
)
