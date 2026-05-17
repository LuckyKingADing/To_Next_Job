load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "geotrans",
    srcs = glob(["geo/src/*.cpp"]),
    hdrs = glob(["geo/include/*.h"]),
    copts = [
        "-std=c++17",
    ],
    includes = [
        "geo/include/",
    ],
    linkstatic = True,
    deps = [
        "@eigen",
    ],
)

cc_library(
    name = "calc_lib",
    srcs = glob(["calc/src/*.cpp"]),
    hdrs = glob(["calc/include/*.h"]),
    copts = [
        "-std=c++17",
    ],
    includes = [
        "calc/include/",
    ],
    linkstatic = True,
    deps = [
        "//modules/localization/src/TCMSF/tcmsf/util:rigid_lib",
        "@eigen",
        "@fmt",
        # "@glog_gflags//:glog",
    ],
)

cc_library(
    name = "csv_player_lib",
    srcs = glob(["csv_player/src/*.cpp"]),
    hdrs = glob(["csv_player/include/*.h"]),
    copts = [
        "-std=c++17",
    ],
    includes = [
        "csv_player/include/",
    ],
    linkstatic = True,
    deps = [
        "//modules/localization/src/TCMSF/tcmsf/sensor:base_lib",
        "//modules/localization/src/TCMSF/tcmsf/sensor:sins_lib",
        "@eigen",
        "@fmt",
        # "@glog_gflags//:glog",
    ],
)

cc_library(
    name = "time_lib",
    srcs = glob(["time/src/*.cpp"]),
    hdrs = glob(["time/include/*.h"]),
    copts = [
        "-std=c++17",
    ],
    includes = [
        "time/include/",
    ],
    linkstatic = True,
    deps = [
        "//cyber_release",
    ],
)

cc_library(
    name = "file_io_lib",
    srcs = glob(["file_io/src/*.cpp"]),
    hdrs = glob(["file_io/include/*.h"]),
    copts = [
        "-std=c++17",
    ],
    includes = [
        "file_io/include/",
    ],
    linkstatic = True,
    deps = [
        "@fmt",
        # "@glog_gflags//:glog",
    ],
)

cc_library(
    name = "rigid_lib",
    srcs = glob(["rigid/src/*.cpp"]),
    hdrs = glob(["rigid/include/*.h"]),
    copts = [
        "-std=c++17",
    ],
    includes = [
        "rigid/include/",
    ],
    linkstatic = True,
    deps = [
        "@eigen",
    ],
    alwayslink = True,
)

cc_library(
    name = "state_analysis_lib",
    srcs = glob(["state_analysis/src/*.cpp"]),
    hdrs = glob(["state_analysis/include/*.h"]),
    copts = [
        "-std=c++17",
    ],
    includes = [
        "state_analysis/include/",
    ],
    linkstatic = True,
    deps = [
        "@eigen",
    ],
    alwayslink = True,
)

cc_library(
    name = "diagnosis_lib",
    srcs = glob(["diagnosis/src/*.cpp"]),
    hdrs = glob(["diagnosis/include/*.h"]),
    copts = [
        "-std=c++17",
    ],
    includes = [
        "diagnosis/include/",
    ],
    linkstatic = True,
    alwayslink = True,
)

cc_library(
    name = "traj_analysis_lib",
    srcs = glob(["traj_analysis/src/*.cpp"]),
    hdrs = glob(["traj_analysis/include/*.h"]),
    copts = [
        "-std=c++17",
    ],
    includes = [
        "traj_analysis/include/",
    ],
    linkstatic = True,
    deps = [
        "//cyber_release",
        "@eigen",
    ],
    alwayslink = True,
)

cc_library(
    name = "util_lib",
    copts = [
        "-std=c++17",
    ],
    linkstatic = True,
    deps = [
        ":calc_lib",
        ":csv_player_lib",
        ":diagnosis_lib",
        ":file_io_lib",
        ":geotrans",
        ":rigid_lib",
        ":state_analysis_lib",
        ":time_lib",
        ":traj_analysis_lib",
    ],
    alwayslink = True,
)

# 初始化评估工具库
cc_library(
    name = "init_evaluator_lib",
    srcs = glob(["init_evaluator/src/*.cpp"]),
    hdrs = glob(["init_evaluator/include/*.h"]),
    copts = [
        "-std=c++17",
    ],
    includes = [
        "init_evaluator/include/",
    ],
    linkstatic = True,
    visibility = ["//visibility:public"],
)

# 初始化评估工具可执行文件
cc_binary(
    name = "tcmsf_init_evaluator",
    srcs = [
        "init_evaluator/tcmsf_init_evaluator_main.cc",
    ],
    copts = [
        "-std=c++17",
    ],
    includes = [
        "../interface/include",
        "../sensor/base/include",
        "init_evaluator/include",
    ],
    deps = [
        ":init_evaluator_lib",
        "//cyber_release",
        "//modules/localization/src/TCMSF/interface:tcmsf_interface",
        "//modules/localization/src/TCMSF/tcmsf/sensor:base_lib",
        "//modules/localization/src/TCMSF/third_party/wgs84_to_mars",
        "//modules/msg/drivers_msgs:gps_cc_proto",
        "//modules/msg/drivers_msgs:imu_cc_proto",
        "//modules/msg/drivers_msgs:veh_info_cc_proto",
        "//modules/msg/localization_msgs:localization_info_cc_proto",
        "//modules/msg/localization_msgs:vf_result_cc_proto",
        "//modules/msg/localization_msgs:sd_map_match_cc_proto",
        "@com_google_protobuf//:protobuf",
    ],
)
