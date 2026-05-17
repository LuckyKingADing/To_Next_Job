load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rtcm_lib",
    srcs = glob(["rtcm/src/rtcm*.cpp"]),
    hdrs = glob(["rtcm/include/rtcm*.h"]),
    copts = [
        "-std=c++17",
        "-DENACMP",
        "-DENAGAL",
        "-DENAGLO",
        "-DNFREQ=5",
        "-Wno-unused-variable",
        "-Wno-unused-but-set-variable",
        "-Wno-sign-compare",
        "-Wno-unused-function",
        "-Wno-unused-result",
        "-DMODULE_NAME=\\\"TCMSF\\\"",
    ],
    includes = [
        "rtcm/include/",
    ],
    linkstatic = True,
    deps = [
        "//cyber_release",
        # "//modules/localization/src/TCMSF/third_party/RTKLIB-b34k:rtklib",
        "//modules/localization/src/TCMSF/third_party/concurrentqueue-1.0.4:concurrentqueue",
        "@fmt",
        # "@glog_gflags//:glog",
    ],
    alwayslink = True,
)

cc_library(
    name = "base_lib",
    srcs = glob(["base/src/*.cpp"]),
    hdrs = glob(["base/include/*.h"]),
    copts = [
        "-std=c++17",
        "-DMODULE_NAME=\\\"TCMSF\\\"",
    ],
    includes = [
        "base/include/",
    ],
    linkstatic = True,
    deps = [
        "//cyber_release",
        "//modules/localization/src/TCMSF/tcmsf/util:geotrans",
        "//modules/localization/src/TCMSF/tcmsf/util:rigid_lib",
        "@eigen",
        "@fmt",
        "//modules/localization/src/TCMSF/third_party/tomlplusplus-3.4.0:toml++",
        "@com_github_nlohmann_json//:json",
        # "@glog_gflags//:glog",
    ],
    alwayslink = True,
)

cc_library(
    name = "sins_lib",
    srcs = glob(["sins/src/*.cpp"]),
    hdrs = glob(["sins/include/*.h"]),
    copts = [
        "-std=c++17",
        "-DMODULE_NAME=\\\"TCMSF\\\"",
    ],
    includes = [
        "sins/include/",
    ],
    linkstatic = True,
    deps = [
        "//cyber_release",
        "//modules/localization/src/TCMSF/tcmsf/util:rigid_lib",
        "//modules/localization/src/TCMSF/third_party/concurrentqueue-1.0.4:concurrentqueue",
        "@eigen",
        "@fmt",
        "//modules/localization/src/TCMSF/third_party/iir1-1.9.5:iir",
        "//modules/localization/src/TCMSF/third_party/tomlplusplus-3.4.0:toml++",
        # "@glog_gflags//:glog",
    ],
    alwayslink = True,
)

cc_library(
    name = "persistence_lib",
    srcs = glob(["persistence/src/*.cpp"]),
    hdrs = glob(["persistence/include/*.h"]),
    copts = [
        "-std=c++17",
        "-DMODULE_NAME=\\\"TCMSF\\\"",
    ],
    includes = [
        "persistence/include/",
    ],
    linkstatic = True,
    deps = [
        "//cyber_release",
        "@fmt",
        "//modules/localization/src/TCMSF/third_party/tomlplusplus-3.4.0:toml++",
        # "@glog_gflags//:glog",
    ],
    alwayslink = True,
)

cc_library(
    name = "replay_mode_lib",
    srcs = glob(["replay_mode/src/*.cpp"]),
    hdrs = glob(["replay_mode/include/*.h"]),
    copts = [
        "-std=c++17",
        "-DMODULE_NAME=\\\"TCMSF\\\"",
    ],
    includes = [
        "replay_mode/include/",
    ],
    linkstatic = True,
    deps = [
        "//cyber_release",
        "//modules/localization/src/TCMSF/tcmsf/filter:filter_lib",
        "//modules/localization/src/TCMSF/tcmsf/sensor:base_lib",
        "//modules/msg/localization_msgs:tcmsf_cc_proto",
        "@fmt",
    ],
    alwayslink = True,
)
