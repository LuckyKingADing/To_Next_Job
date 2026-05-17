load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

config_setting(
    name = "use_dimw",
    values = {
        "cpu": "aarch64",
        "define": "use_dimw_libs=true",
    },
)

cc_library(
    name = "tcmsf_interface",
    srcs = glob(["src/*.cpp"]) + ["include/tcmsf_interface_impl.h"] + ["include/kinematic_compensation.h"] + ["include/state_debouncer.h"],
    hdrs = ["include/tcmsf_interface.h"],
    copts = [
        "-std=c++17",
        "-DMODULE_NAME=\\\"TCMSF\\\"",
    ],
    includes = [
        "include/",
    ],
    linkstatic = True,
    deps = [
        "//cyber_release",
        "//modules/msg/localization_msgs:tcmsf_cc_proto",
        "//modules/msg/localization_msgs:sd_map_match_cc_proto",
        "//modules/msg/localization_msgs:vf_result_cc_proto",
        # "//modules/localization/src/TCMSF/tcmsf/motion_status:motion_status_lib",
        "//modules/localization/src/TCMSF/tcmsf/processor:processor_lib",
        "//modules/localization/src/TCMSF/tcmsf/sensor:persistence_lib",
        "//modules/localization/src/TCMSF/tcmsf/util:diagnosis_lib",
        "//modules/localization/src/TCMSF/third_party/concurrentqueue-1.0.4:concurrentqueue",
        "@eigen",
        "@fmt",
        # "//modules/localization/src/registration:registration_lib",
        "//modules/msg/basic_msgs:module_status_cc_proto",
        "//modules/msg/drivers_msgs:gps_cc_proto",
        "//modules/msg/drivers_msgs:imu_cc_proto",
        "//modules/msg/drivers_msgs:ins_cc_proto",
        "//modules/msg/drivers_msgs:rtcm_cc_proto",
        "//modules/msg/drivers_msgs:veh_info_cc_proto",
        "//modules/msg/environment_model_msgs:drive_boundary_cc_proto",
        "//modules/msg/localization_msgs:localization_info_cc_proto",
        "//modules/localization/src/TCMSF/troublemaker:troublemaker_lib",
        "//modules/localization/src/TCMSF/tcmsf/fail_detect:fail_detect_lib",
    ] + select({
        "use_dimw": ["@dimw//:dimw_cfgmgr"],
        "//conditions:default": [],
    }),
    alwayslink = True,
)
