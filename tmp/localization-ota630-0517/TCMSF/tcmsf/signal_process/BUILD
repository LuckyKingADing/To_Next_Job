load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "signal_process_lib",
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
        "analysis/include/",
        "igg3/include/",
        "signal_filter/include/",
        "statistics/include/",
    ],
    linkstatic = True,
    deps = [
        # "//cyber_release",
        "@eigen",
        "//modules/localization/src/TCMSF/third_party/iir1-1.9.5:iir",
        # "@glog_gflags//:glog",
    ],
    alwayslink = True,
)
