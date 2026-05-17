load("@rules_cc//cc:defs.bzl", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "TCMSF",
    srcs = [
        "tcmsf_main.cc",
        "tcmsf_main.h",
    ],
    copts = [
        "-std=c++17",
    ],
    deps = [
        "//cyber_release",
        "//modules/common/topic:topic_gflags",
        "//modules/localization/src/TCMSF/interface:tcmsf_interface",
        "//modules/localization/src/TCMSF/tcmsf/sensor:replay_mode_lib",
        "//modules/localization/src/TCMSF/tcmsf/sensor:rtcm_lib",
        "//modules/localization/src/TCMSF/third_party/wgs84_to_mars",
        "//modules/localization/src/common/imu_issue_fallback:imu_issue_fallback_lib",
        "//modules/msg/basic_msgs:module_status_cc_proto",
        "//modules/msg/drivers_msgs:rtcm_cc_proto",
        "//modules/msg/localization_msgs:vf_result_cc_proto",
        "//modules/msg/state_machine_msgs:top_state_cc_proto",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_binary(
    name = "libTCMSF_component.so",
    srcs = [
        "tcmsf_component.cc",
        "tcmsf_component.h",
    ],
    copts = [
        "-std=c++17",
        "-DMODULE_NAME=\\\"TCMSF\\\"",
    ],
    includes = ["./"],
    linkshared = True,
    linkstatic = True,
    deps = [
        "//cyber_release",
        "//modules/localization/src/TCMSF/interface:tcmsf_interface",
        "//modules/localization/src/TCMSF/tcmsf/sensor:rtcm_lib",
        "//modules/localization/src/TCMSF/third_party/wgs84_to_mars",
        "//modules/localization/src/common/imu_issue_fallback:imu_issue_fallback_lib",
        "//modules/msg/basic_msgs:module_status_cc_proto",
        "//modules/msg/drivers_msgs:gps_cc_proto",
        "//modules/msg/drivers_msgs:imu_cc_proto",
        "//modules/msg/drivers_msgs:ins_cc_proto",
        "//modules/msg/drivers_msgs:rtcm_cc_proto",
        "//modules/msg/drivers_msgs:veh_info_cc_proto",
        "//modules/msg/environment_model_msgs:drive_boundary_cc_proto",
        "//modules/msg/localization_msgs:localization_info_cc_proto",
        "//modules/msg/localization_msgs:result_info_cc_proto",
        "//modules/msg/localization_msgs:tcmsf_cc_proto",
        "//modules/msg/localization_msgs:vf_result_cc_proto",
        "//modules/msg/state_machine_msgs:top_state_cc_proto",
    ],
)

filegroup(
    name = "release_conf",
    srcs = glob(["conf/*"]),
)

filegroup(
    name = "release_dag",
    srcs = ["dag/tcmsf_onboard.dag"],
)

filegroup(
    name = "release_bin",
    srcs = [":libTCMSF_component.so"],
)

filegroup(
    name = "release_libs",
    srcs = [
        "//modules/localization/src/TCMSF/third_party/wgs84_to_mars",
    ],
)
