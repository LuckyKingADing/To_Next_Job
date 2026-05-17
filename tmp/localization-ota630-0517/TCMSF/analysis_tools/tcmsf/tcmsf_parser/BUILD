load("@rules_cc//cc:defs.bzl", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "PARSER",
    srcs = glob([
        "include/*.h",
        "src/*.cpp",
    ]),
    copts = [
        "--std=c++17",
        "-Wno-unused-variable",
        "-Wno-unused-but-set-variable",
    ],
    includes = ["include"],
    deps = [
        "//cyber_release",
        "//modules/localization/src/TCMSF/tcmsf/util:geotrans",
        "//modules/localization/src/TCMSF/tcmsf/util:rigid_lib",
        "//modules/localization/src/TCMSF/third_party/wgs84_to_mars",
        "//modules/localization/src/TCMSF/tcmsf/sensor:base_lib",
        "//modules/msg/drivers_msgs:gps_cc_proto",
        "//modules/msg/drivers_msgs:imu_cc_proto",
        "//modules/msg/drivers_msgs:ins_cc_proto",
        "//modules/msg/drivers_msgs:rtcm_cc_proto",
        "//modules/msg/drivers_msgs:veh_info_cc_proto",
        "//modules/msg/localization_msgs:local_map_cc_proto",
        "//modules/msg/localization_msgs:localization_info_cc_proto",
        "//modules/msg/localization_msgs:result_info_cc_proto",
        "//modules/msg/localization_msgs:tcmsf_cc_proto",
        "//modules/msg/localization_msgs:vf_result_cc_proto",
        "//modules/msg/orin_msgs:routing_map_cc_proto",
        "//modules/localization/src/TCMSF/tcmsf/sensor:rtcm_lib",
        "@com_github_nlohmann_json//:json",
        "@com_google_protobuf//:protobuf",
        "@eigen",
        "@fmt",
    ],
)