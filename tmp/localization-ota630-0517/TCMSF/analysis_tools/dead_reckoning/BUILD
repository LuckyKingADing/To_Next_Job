load("@rules_cc//cc:defs.bzl", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "PARSER",
    srcs = [
        "include/parse_record.h",
        "src/parse_record.cc",
    ],
    copts = [
        "--std=c++17",
    ],
    includes = ["include"],
    deps = [
        "//cyber_release",
        "//modules/common/topic:topic_gflags",
        "//modules/localization/src/TCMSF/tcmsf/signal_process:signal_process_lib",
        "//modules/localization/src/TCMSF/third_party/wgs84_to_mars",
        "//modules/msg/basic_msgs:module_status_cc_proto",
        "//modules/msg/drivers_msgs:gps_cc_proto",
        "//modules/msg/drivers_msgs:imu_cc_proto",
        "//modules/msg/drivers_msgs:ins_cc_proto",
        "//modules/msg/drivers_msgs:veh_info_cc_proto",
        "//modules/msg/localization_msgs:localization_info_cc_proto",
        "//modules/msg/localization_msgs:result_info_cc_proto",
        "@com_google_protobuf//:protobuf",
    ],
)
