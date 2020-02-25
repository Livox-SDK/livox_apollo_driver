load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "liblivox_driver_component.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":livox_driver_component_lib"],
)

cc_library(
    name = "livox_driver_component_lib",
    srcs = ["livox_driver_component.cc"],
    hdrs = ["livox_driver_component.h"],
    copts = ['-DMODULE_NAME=\\"livox\\"'],
    deps = [
        "//cyber",
        "//modules/drivers/livox/driver",
    ],
)

cc_library(
    name = "driver",
    srcs = [
        "driver.cc",
        "hub_driver.cc",
    ],
    hdrs = [
        "driver.h",
        "driver_factory.h",
        "hub_driver.h",
    ],
    copts = ['-DMODULE_NAME=\\"livox\\"'],
    linkopts = [
        "-llivox_sdk_shared",
    ],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/drivers/livox/proto:livox_proto",
        "//modules/drivers/proto:sensor_proto",
    ],
)

cc_test(
    name = "driver_test",
    size = "small",
    srcs = ["driver_test.cc"],
    copts = ['-DMODULE_NAME=\\"livox\\"'],
    deps = [
        "//modules/drivers/livox/driver",
        "//modules/drivers/livox/proto:livox_proto",
        "@gtest//:main",
    ],
)

cpplint()
