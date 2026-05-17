load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "DR",
    srcs = [
        "dr_main.cc",
        "dr_main.h",
    ],
    copts = [
        "--std=c++17",
    ],
    deps = [
        "//cyber_release",
        "//modules/common/topic:topic_gflags",
        "//modules/localization/src/TCMSF/tcmsf/util:rigid_lib",
        "//modules/localization/src/dead_reckoning:dr_core",
        "//modules/msg/basic_msgs:module_status_cc_proto",
        "//modules/msg/drivers_msgs:gps_cc_proto",
        "//modules/msg/drivers_msgs:imu_cc_proto",
        "//modules/msg/drivers_msgs:ins_cc_proto",
        "//modules/msg/drivers_msgs:veh_info_cc_proto",
        "//modules/msg/localization_msgs:localization_info_cc_proto",
        "//modules/msg/localization_msgs:tcmsf_cc_proto",
        "//modules/msg/state_machine_msgs:top_state_cc_proto",
        "@com_google_protobuf//:protobuf",
        "@eigen",
        "@yaml_cpp//:yaml-cpp",
    ],
)

cc_library(
    name = "dr_core",
    srcs = glob(["core/src/*.cc"]) + glob(["interface/src/*.cc"]),
    hdrs = glob(["core/include/*.h"]) + glob(["interface/include/*.h"]),
    copts = [
        "-std=c++14",
    ],
    includes = [
        "core/include",
        "interface/include",
    ],
    linkstatic = True,
    deps = [
        ":iir",
        ":utils",
        "//cyber_release",
        "@eigen",
    ],
    alwayslink = True,
)

cc_library(
    name = "utils",
    srcs = glob(["utils/src/*.cc"]),
    hdrs = glob(["utils/include/*.h"]),
    copts = [
        "-std=c++14",
    ],
    includes = ["utils/include/"],
    linkstatic = True,
    deps = [
        "@eigen",
    ],
    alwayslink = True,
)

cc_library(
    name = "iir",
    srcs = glob(["iir/iir/*.cpp"]),
    hdrs = glob(["iir/iir/*.h"]) + ["iir/Iir.h"],
    copts = [
        "-Wall",
        "-pedantic",
        "-std=c++11",
    ],
    includes = ["iir/"],
    linkstatic = True,
    deps = [],
    alwayslink = True,
)

cc_binary(
    name = "libDR_component.so",
    srcs = [
        "dr_component.cc",
        "dr_component.h",
    ],
    copts = [
        "-std=c++14",
        "-DMODULE_NAME=\\\"DR\\\"",
    ],
    includes = ["./"],
    linkshared = True,
    linkstatic = True,
    deps = [
        ":dr_core",
        "//cyber_release",
        "//modules/localization/src/common/imu_issue_fallback:imu_issue_fallback_lib",
        "//modules/msg/basic_msgs:module_status_cc_proto",
        "//modules/msg/drivers_msgs:gps_cc_proto",
        "//modules/msg/drivers_msgs:imu_cc_proto",
        "//modules/msg/drivers_msgs:ins_cc_proto",
        "//modules/msg/drivers_msgs:veh_info_cc_proto",
        "//modules/msg/localization_msgs:localization_info_cc_proto",
        "//modules/msg/localization_msgs:tcmsf_cc_proto",
        "@eigen",
        "@yaml_cpp//:yaml-cpp",
    ],
)

filegroup(
    name = "release_conf",
    srcs = glob(["conf/*"]),
)

filegroup(
    name = "release_dag",
    srcs = ["dag/dr_onboard.dag"],
)

filegroup(
    name = "release_bin",
    srcs = [":libDR_component.so"],
)

filegroup(
    name = "release_libs",
    srcs = [
        "@yaml_cpp//:yaml-cpp",
    ],
)
