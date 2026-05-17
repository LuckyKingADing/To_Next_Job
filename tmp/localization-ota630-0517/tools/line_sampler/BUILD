load("@bazel_tools//tools/build_defs/pkg:pkg.bzl", "pkg_tar")
load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "line_sampler",
    srcs = ["line_sampler.cc"],
    copts = [
        "-std=c++17",
        "-O2",
    ],
    linkstatic = True,
    deps = [
        "@glog_gflags//:glog",
    ],
)

cpplint()
