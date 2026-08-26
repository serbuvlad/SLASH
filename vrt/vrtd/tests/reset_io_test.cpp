/**
 * The MIT License (MIT)
 * Copyright (c) 2026 Advanced Micro Devices, Inc. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <gtest/gtest.h>

#include <cerrno>
#include <cstring>
#include <string>
#include <vector>

extern "C" {
#include "device.h"
#include "hotplug.h"
}

namespace {
struct slash_ctldev ctl_handle;
struct slash_qdma qdma_handle;
struct slash_hotplug hotplug_handle;
struct slash_ioctl_device_info ctl_info;
struct slash_qdma_info qdma_info;
int ctl_open_error, qdma_open_error;
bool ctl_info_error, qdma_info_error;
int ctl_open_count, ctl_close_count, qdma_open_count, qdma_close_count;
int info_alloc_count, info_free_count;
std::vector<std::string> removed;
unsigned int present_functions;
int fail_function, removal_error;

class ResetIoTest : public ::testing::Test {
protected:
    void SetUp() override {
        ctl_open_error = qdma_open_error = 0;
        ctl_info_error = qdma_info_error = false;
        ctl_open_count = ctl_close_count = qdma_open_count = qdma_close_count = 0;
        info_alloc_count = info_free_count = 0;
        ctl_info = {};
        qdma_info = {};
        std::strcpy(ctl_info.bdf, "0000:65:00.2");
        std::strcpy(qdma_info.bdf, "0000:65:00.1");
        removed.clear();
        present_functions = 0x0f; // PF0 through PF3
        fail_function = -1;
        removal_error = EIO;
        g_hotplug = &hotplug_handle;
    }
    void TearDown() override {
        EXPECT_EQ(ctl_open_count, ctl_close_count);
        EXPECT_EQ(qdma_open_count, qdma_close_count);
        EXPECT_EQ(info_alloc_count, info_free_count);
        g_hotplug = nullptr;
    }
};
}

extern "C" {
struct slash_ctldev *__wrap_slash_ctldev_open(const char *path) {
    EXPECT_STREQ(path, "/dev/slash_ctl12");
    if (ctl_open_error) {
        errno = ctl_open_error;
        return nullptr;
    }
    ++ctl_open_count;
    return &ctl_handle;
}
int __wrap_slash_ctldev_close(struct slash_ctldev *ctl) {
    EXPECT_EQ(ctl, &ctl_handle);
    ++ctl_close_count;
    return 0;
}
struct slash_ioctl_device_info *__wrap_slash_device_info_read(struct slash_ctldev *ctl) {
    EXPECT_EQ(ctl, &ctl_handle);
    if (ctl_info_error)
        return nullptr;
    ++info_alloc_count;
    return &ctl_info;
}
void __wrap_slash_device_info_free(struct slash_ioctl_device_info *info) {
    if (info != nullptr) {
        EXPECT_EQ(info, &ctl_info);
        ++info_free_count;
    }
}
struct slash_qdma *__wrap_slash_qdma_open(const char *path) {
    EXPECT_STREQ(path, "/dev/slash_qdma_ctl12");
    if (qdma_open_error) {
        errno = qdma_open_error;
        return nullptr;
    }
    ++qdma_open_count;
    return &qdma_handle;
}
int __wrap_slash_qdma_close(struct slash_qdma *qdma) {
    EXPECT_EQ(qdma, &qdma_handle);
    ++qdma_close_count;
    return 0;
}
int __wrap_slash_qdma_info_read(struct slash_qdma *qdma, struct slash_qdma_info *info) {
    EXPECT_EQ(qdma, &qdma_handle);
    if (qdma_info_error) {
        errno = ENODEV;
        return -1;
    }
    *info = qdma_info;
    return 0;
}
int __wrap_slash_hotplug_remove(struct slash_hotplug *hp, const char *bdf) {
    EXPECT_EQ(hp, &hotplug_handle);
    removed.emplace_back(bdf);
    int function = bdf[11] - '0';
    if (function == fail_function) {
        errno = removal_error;
        return -1;
    }
    if (!(present_functions & (1u << function))) {
        errno = ENODEV;
        return -1;
    }
    return 0;
}
}

TEST_F(ResetIoTest, RequiresBothNodesAndCorrectPciIdentity) {
    EXPECT_TRUE(device_nodes_ready("/dev/slash_ctl12", "0000:65:00"));
    EXPECT_EQ(ctl_open_count, 1);
    EXPECT_EQ(qdma_open_count, 1);
}

TEST_F(ResetIoTest, MissingControlOrPendingUdevPermissionsAreNotReady) {
    for (int error : {ENOENT, ENODEV, EACCES}) {
        ctl_open_error = error;
        EXPECT_FALSE(device_nodes_ready("/dev/slash_ctl12", "0000:65:00"));
    }
    EXPECT_EQ(qdma_open_count, 0);
}

TEST_F(ResetIoTest, MissingQdmaOrPendingUdevPermissionsAreNotReady) {
    for (int error : {ENOENT, ENODEV, EACCES}) {
        qdma_open_error = error;
        EXPECT_FALSE(device_nodes_ready("/dev/slash_ctl12", "0000:65:00"));
    }
    qdma_open_error = 0;
    EXPECT_TRUE(device_nodes_ready("/dev/slash_ctl12", "0000:65:00"));
}

TEST_F(ResetIoTest, RejectsStaleControlNodeForAnotherBoard) {
    std::strcpy(ctl_info.bdf, "0000:66:00.2");
    EXPECT_FALSE(device_nodes_ready("/dev/slash_ctl12", "0000:65:00"));
    EXPECT_EQ(qdma_open_count, 0);
}

TEST_F(ResetIoTest, RejectsQdmaNodeForAnotherBoard) {
    std::strcpy(qdma_info.bdf, "0000:66:00.1");
    EXPECT_FALSE(device_nodes_ready("/dev/slash_ctl12", "0000:65:00"));
}

TEST_F(ResetIoTest, RejectsFailedIdentityIoctls) {
    ctl_info_error = true;
    EXPECT_FALSE(device_nodes_ready("/dev/slash_ctl12", "0000:65:00"));
    ctl_info_error = false;
    qdma_info_error = true;
    EXPECT_FALSE(device_nodes_ready("/dev/slash_ctl12", "0000:65:00"));
}

TEST_F(ResetIoTest, RemovesPf3AndToleratesAbsentFunctions) {
    ASSERT_EQ(hotplug_remove_board("0000:65:00"), 0);
    ASSERT_EQ(removed.size(), 8u);
    for (int pf = 0; pf < 8; ++pf)
        EXPECT_EQ(removed[pf], "0000:65:00." + std::to_string(pf));
}

TEST_F(ResetIoTest, HandlesOlderThreeFunctionShells) {
    present_functions = 0x07;
    EXPECT_EQ(hotplug_remove_board("0000:65:00.2"), 0);
    EXPECT_EQ(removed.size(), 8u);
}

TEST_F(ResetIoTest, StopsOnRemovalFailureAndPreservesErrno) {
    fail_function = 3;
    removal_error = EBUSY;
    EXPECT_EQ(hotplug_remove_board("0000:65:00"), -1);
    EXPECT_EQ(errno, EBUSY);
    EXPECT_EQ(removed.size(), 4u);
}

TEST_F(ResetIoTest, MissingHotplugHandleDoesNotRemoveAnything) {
    g_hotplug = nullptr;
    EXPECT_EQ(hotplug_remove_board("0000:65:00"), -1);
    EXPECT_EQ(errno, ENODEV);
    EXPECT_TRUE(removed.empty());
}
