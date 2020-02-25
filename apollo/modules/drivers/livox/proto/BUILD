load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "livox_proto",
    deps = [
        ":sensor_livox_proto",
    ],
)

cc_proto_library(
    name = "sensor_livox_proto",
    deps = [
        ":livox_proto_lib",
    ],
)

proto_library(
    name = "livox_proto_lib",
    srcs = [
        "config.proto",
    ],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)

cpplint()
