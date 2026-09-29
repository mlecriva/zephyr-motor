# Copyright (c) 2026 Rtone
# Copyright (c) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_include_directories(include)

zephyr_include_directories(include)

add_subdirectory(drivers)
add_subdirectory(lib)
add_subdirectory(subsys)
