load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "processor_lib",
    srcs = glob(
        [
            "src/*.cpp",
            "include/*.h",
        ],
    ),
    hdrs = ["include/processor_interface.h"],
    copts = [
        "-std=c++17",
        "-Wno-unused-but-set-variable",
        "-Wno-unused-variable",
        "-Wno-sign-compare",
        "-lpthread",
        "-DMODULE_NAME=\\\"TCMSF\\\"",
    ],
    includes = [
        "include/",
    ],
    linkstatic = True,
    deps = [
        "//cyber_release",
        "//modules/localization/src/TCMSF/tcmsf/filter:filter_lib",
        # "//modules/localization/src/TCMSF/tcmsf/motion_status:motion_status_lib",
        "//modules/localization/src/TCMSF/tcmsf/sensor:base_lib",
        "//modules/localization/src/TCMSF/tcmsf/sensor:sins_lib",
        "//modules/localization/src/TCMSF/tcmsf/sensor:replay_mode_lib",
        "//modules/localization/src/TCMSF/tcmsf/signal_process:signal_process_lib",
        "//modules/localization/src/TCMSF/tcmsf/util:calc_lib",
        "//modules/localization/src/TCMSF/tcmsf/util:file_io_lib",
        "//modules/localization/src/TCMSF/tcmsf/util:state_analysis_lib",
        "//modules/localization/src/TCMSF/tcmsf/util:traj_analysis_lib",
        "//modules/localization/src/TCMSF/tcmsf/util:time_lib",
        "//modules/localization/src/TCMSF/tcmsf/traj_logger:traj_logger_lib",
        "//modules/localization/src/TCMSF/rts_smoother:rts_smoother_lib",
        "@eigen",
        "@fmt",
        "//modules/localization/src/TCMSF/third_party/wgs84_to_mars",
        # "@glog_gflags//:glog",
    ],
    alwayslink = True,
)
