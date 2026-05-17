load("@rules_cc//cc:defs.bzl", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "kinematic",
    srcs = glob([
        "kinematic/include/*.h",
        "kinematic/src/*.cpp",
    ]),
    copts = [
        "--std=c++17",
    ],
    includes = ["kinematic/include"],
    linkstatic = True,
    deps = [
        "//modules/msg/localization_msgs:localization_info_cc_proto",
        "@eigen",
        "@glog_gflags",
    ],
    alwayslink = True,
)

cc_library(
    name = "parser",
    srcs = glob([
        "parser/include/*.h",
        "parser/src/*.cpp",
    ]),
    copts = [
        "--std=c++17",
    ],
    includes = ["parser/include"],
    linkstatic = True,
    deps = [
        "//cyber_release",
        "//modules/msg/localization_msgs:localization_info_cc_proto",
        "@eigen",
        "@glog_gflags",
    ],
    alwayslink = True,
)

cc_library(
    name = "PCD",
    srcs = glob([
        "PCD/include/*.h",
        "PCD/src/*.cpp",
    ]),
    copts = [
        "--std=c++17",
    ],
    includes = ["PCD/include"],
    linkstatic = True,
    deps = [
        "@eigen",
        "@glog_gflags",
    ],
    alwayslink = True,
)

cc_library(
    name = "registration",
    srcs = glob([
        "registration/include/*.h",
        "registration/src/*.cpp",
    ]),
    copts = [
        "--std=c++17",
    ],
    includes = ["registration/include"],
    linkstatic = True,
    deps = [
        "PCD",
        "//modules/localization/src/TCMSF/third_party/small_gicp_v1.0.0:small_gicp",
        "@eigen",
    ],
    alwayslink = True,
)

cc_binary(
    name = "ToDrFrame",
    srcs = glob(["*.cc"]),
    copts = [
        "--std=c++17",
        "-fopenmp",
        "-DMODULE_NAME=\\\"Align\\\"",
    ],
    linkopts = ["-fopenmp"],
    deps = [
        "PCD",
        "kinematic",
        "parser",
        "registration",
        "//modules/msg/localization_msgs:localization_info_cc_proto",
        "@com_google_protobuf//:protobuf",
    ],
)
