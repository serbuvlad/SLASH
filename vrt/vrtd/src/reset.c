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

/**
 * @file reset.c
 * @brief V80 boot-partition selection and PCIe Secondary Bus Reset.
 *
 * PF0 provides AMI management, PF1 QDMA, and PF2 control registers. Newer
 * shells also expose PF3 for memory access. All functions must be removed
 * before SBR; the hotplug ioctl then waits for PDI reload and link recovery.
 * After rescanning, wait for AMI plus usable PF1/PF2 nodes before rediscovery.
 */

#define _GNU_SOURCE

#include "reset.h"

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>

#include <sys/ioctl.h>

#include <ami.h>
#include <ami_device.h>
#include <ami_device_internal.h>
#include <ami_ioctl.h>
#include <ami_mem_access.h>
#include <vrtd/wire.h>

#include "device.h"
#include "hotplug.h"
#include "shell_build_id.h"
#include "utils.h"

#define GPIO_ALLOW_SBR 0x1040000
#define RESET_NODE_POLL_INTERVAL_US 100000
#define RESET_NODE_READY_TIMEOUT_US 10000000
#define RESCAN_MAX_RETRIES 5
#define RESCAN_RETRY_DELAY_US 3000000

/* Retain the AMI handle while waiting for the paired SLASH nodes. */
static bool reset_functions_ready(
    const char *pf0_bdf,
    const char *ctl_path,
    struct ami_device **ami_device
)
{
    if (*ami_device == NULL) {
        if (ami_dev_find(pf0_bdf, ami_device) != AMI_STATUS_OK) {
            ami_dev_delete(ami_device);
            return false;
        }
    }

    return device_nodes_ready(ctl_path, pf0_bdf);
}

static void reset_emit_progress(
    cfgmem_progress_callback progress_cb,
    void *progress_ctx,
    uint32_t phase
)
{
    if (progress_cb != NULL) {
        progress_cb(progress_ctx, phase, 0, 0);
    }
}

int shell_boot_partition(enum vrtd_shell_type shell, uint32_t *partition_out)
{
    PROPAGATE_ERROR_NULL_LOG(partition_out, LOG_ERR, "Internal error: null partition_out");

    switch (shell) {
    case VRTD_SHELL_SERVICE:
        *partition_out = 0;
        return 0;
    case VRTD_SHELL_COMPUTE:
        *partition_out = 1;
        return 0;
    default:
        return -1;
    }
}

/* Inverse of shell_boot_partition(): the shell a boot partition maps to. */
static enum vrtd_shell_type shell_from_boot_partition(uint32_t partition)
{
    switch (partition) {
    case 0:
        return VRTD_SHELL_SERVICE;
    case 1:
        return VRTD_SHELL_COMPUTE;
    default:
        return VRTD_SHELL_UNKNOWN;
    }
}

bool shell_reset_required(enum vrtd_shell_type current_shell, enum vrtd_shell_type required_shell)
{
    return current_shell == VRTD_SHELL_UNKNOWN || current_shell != required_shell;
}

bool shell_switch_blocked_by_jtag(
    enum vrtd_shell_type current_shell,
    enum vrtd_shell_type required_shell,
    bool jtag
)
{
    return jtag && shell_reset_required(current_shell, required_shell);
}

/**
 * Perform a full device reset using AMI firmware commands and PCIe hotplug.
 *
 * This function executes the complete reset sequence described in the file
 * header.  It takes ownership of @device (removes it from @devices) because
 * the device will be physically removed from the PCI bus during the reset.
 * After the reset and rescan, devices_discover_and_open() re-populates the
 * device list with the newly-enumerated device.
 *
 * @param device   The device to reset.  The caller must not use this pointer
 *                 after the call, as the device is removed from the tracked
 *                 list and freed.
 * @param devices  The global array of tracked device pointers.  The target
 *                 device is removed at the start; after a successful reset,
 *                 the newly-discovered device is added back.
 * @return VRTD_RET_OK on success, or a VRTD_RET_* error code on failure.
 */
uint16_t reset_with_ami_partition_progress(
    struct device *device,
    struct device_ptr_array *devices,
    uint32_t partition,
    cfgmem_progress_callback progress_cb,
    void *progress_ctx
)
{
    char pf0_bdf[VRTD_PCI_BDF_LEN] = {0};
    struct ami_device *ami_device = NULL;

    /* Save identity before removing and freeing the tracked device. Node
     * numbers are stable across remove/rescan with the current driver. */
    char target_bdf[VRTD_PCI_BDF_LEN] = {0};
    strncpy(target_bdf, device->pci_info.bdf, sizeof(target_bdf) - 1);
    _cleanup_(cleanup_free)
    char *ctl_path = strdup(device->path);
    if (ctl_path == NULL || g_hotplug == NULL)
        return VRTD_RET_INTERNAL_ERROR;

    int ret = pci_bdf_set_function(target_bdf, 0, pf0_bdf);
    if (ret != 0) {
        LOG(LOG_ERR, "reset_with_ami: failed to compute PF0 BDF from %s", target_bdf);
        return VRTD_RET_INTERNAL_ERROR;
    }

    /*
     * Step 2: Remove the device from vrtd's tracked device list.
     * The device is about to be reset and will disappear from the PCI bus,
     * so we must stop tracking it before proceeding.  After this point,
     * the @device pointer is invalid and must not be dereferenced.
     */
    reset_emit_progress(
        progress_cb,
        progress_ctx,
        VRTD_CFGMEM_PROGRAM_PHASE_RESET_PREPARING
    );

    // We are now removing this device.
    device_ptr_array_rm_by_reference(devices, device);
    device = NULL;

    /*
     * Step 3: Open the AMI management device on PF0 and request access.
     * AMI (Alveo Management Interface) runs on PF0 (the AVED function).
     * ami_dev_find() locates the AMI character device by PCI BDF, and
     * ami_dev_request_access() acquires exclusive access for management
     * operations.
     * TODO(vserbu): explain AMI access model -- is this a lock? exclusive open? capability grant?
     */
    // PF0 is AVED/AMI bdf
    ret = ami_dev_find(pf0_bdf, &ami_device);
    if (ret != AMI_STATUS_OK) {
        LOG(LOG_ERR, "reset_with_ami: ami_dev_find(%s) failed: %s", pf0_bdf, ami_get_last_error());
        return VRTD_RET_INTERNAL_ERROR;
    }

    ret = ami_dev_request_access(ami_device);
    if (ret != AMI_STATUS_OK) {
        LOG(LOG_ERR, "reset_with_ami: ami_dev_request_access(%s) failed: %s", pf0_bdf, ami_get_last_error());
        ami_dev_delete(&ami_device);
        return VRTD_RET_INTERNAL_ERROR;
    }

    /*
     * Step 4: Issue AMI_IOC_DEVICE_BOOT ioctl to select the boot partition.
     *
     * We issue AMI_IOC_DEVICE_BOOT directly rather than calling
     * ami_prog_device_boot(), even though the latter is the intended public
     * API for this operation.  The reason is that ami_prog_device_boot()
     * unconditionally calls ami_dev_hot_reset() after the ioctl succeeds.
     * ami_dev_hot_reset() performs its own full remove-device / toggle-SBR /
     * rescan cycle by opening the PCIe bridge config-space sysfs file
     * (/sys/bus/pci/devices/<port>/config) with O_RDWR.  That file is mode
     * 0600 and owned by root; vrtd runs as the unprivileged 'vrtd' user, so
     * the open always fails with EBADF regardless of any Linux capabilities
     * granted to the process.
     *
     * More fundamentally, even if the open succeeded, ami_dev_hot_reset would
     * conflict with vrtd's own hotplug reset sequence that follows immediately
     * below.  vrtd drives hotplug through the slash kernel module
     * (slash_hotplug_remove / slash_hotplug_toggle_sbr / slash_hotplug_rescan),
     * which is the authoritative hotplug path for SLASH devices.  Letting both
     * ami_dev_hot_reset and the slash hotplug sequence run would reset the
     * device twice and leave the AMI device handle in an inconsistent state.
     *
     * The correct behaviour is to issue only the AMI_IOC_DEVICE_BOOT ioctl to
     * inform the AMC firmware of the desired boot partition, then hand control
     * back to vrtd to drive the full hotplug sequence itself.  We set
     * cap_override from the device handle (populated earlier by
     * ami_dev_request_access) so that the kernel driver's per-ioctl permission
     * check passes for the unprivileged vrtd user without requiring
     * CAP_DAC_OVERRIDE or root.
     */
    reset_emit_progress(
        progress_cb,
        progress_ctx,
        VRTD_CFGMEM_PROGRAM_PHASE_SELECTING_PARTITION
    );

    {
        struct ami_ioc_data_payload boot_payload = { 0 };
        boot_payload.partition = partition;
        boot_payload.cap_override = ami_device->cap_override;

        if (ami_open_cdev(ami_device) != AMI_STATUS_OK) {
            LOG(LOG_ERR, "reset_with_ami: ami_open_cdev(%s) failed: %s", pf0_bdf, ami_get_last_error());
            ami_dev_delete(&ami_device);
            return VRTD_RET_INTERNAL_ERROR;
        }

        errno = 0;
        if (ioctl(ami_device->cdev, AMI_IOC_DEVICE_BOOT, &boot_payload) != 0) {
            LOG(LOG_ERR, "reset_with_ami: AMI_IOC_DEVICE_BOOT(%s) failed: errno %d (%s)",
                pf0_bdf, errno, strerror(errno));
            ami_dev_delete(&ami_device);
            return VRTD_RET_INTERNAL_ERROR;
        }
    }
    LOG(LOG_INFO, "reset_with_ami: AMI_IOC_DEVICE_BOOT(%s, partition=%u) OK",
        pf0_bdf, (unsigned int)partition);

    /*
     * Step 5: Write a trigger value to BAR0 register at offset 0x1040000
     * to initiate the firmware-level reconfiguration.
     *
     * This is a GPIO pin in the programmed logic that forms an AND gate
     * with the PCIe SBR signal, and needs to be turned on in order to
     * perform a scondary bus reset.
     */
    ret = ami_mem_bar_write(ami_device, 0, GPIO_ALLOW_SBR, 1);
    if (ret != AMI_STATUS_OK) {
        LOG(LOG_ERR, "reset_with_ami: ami_mem_bar_write(%s) failed: %s", pf0_bdf, ami_get_last_error());
        ami_dev_delete(&ami_device);
        return VRTD_RET_INTERNAL_ERROR;
    }

    LOG(LOG_INFO, "reset_with_ami: GPIO_ALLOW_SBR set on %s", pf0_bdf);

    /* Step 6: Close the AMI device handle -- we are done with firmware commands. */
    ami_dev_delete(&ami_device);

    /* Every function, including optional PF3, must be gone before SBR. */
    reset_emit_progress(
        progress_cb,
        progress_ctx,
        VRTD_CFGMEM_PROGRAM_PHASE_REMOVING_PCIE
    );
    ret = hotplug_remove_board(target_bdf);
    if (ret != 0)
        return hotplug_errno_to_vrtd_ret(errno);

    /*
     * Step 7a: Brief settle after PF removal, before toggling SBR.
     *
     * The AMI library's ami_dev_hot_reset() inserts a 1 ms delay here.
     * Its comment notes that "on some systems, the device that is being
     * reset disappears from the host, forcing a system reboot — adding a
     * delay before setting the SBR seems to mitigate this issue."
     */
    usleep(20000);

    /*
     * Step 8: Toggle Secondary Bus Reset (SBR) on the upstream PCIe bridge.
     *
     * SBR asserts the reset signal on the secondary side of the PCIe bridge,
     * forcing all downstream devices (our FPGA) to re-initialize.  This is
     * the mechanism that causes the FPGA to load the new configuration from
     * the boot partition selected in step 4.
     */
    LOG(LOG_INFO, "reset_with_ami: toggling SBR for %s", pf0_bdf);
    reset_emit_progress(
        progress_cb,
        progress_ctx,
        VRTD_CFGMEM_PROGRAM_PHASE_TOGGLING_SBR
    );
    ret = slash_hotplug_toggle_sbr(g_hotplug, pf0_bdf);
    if (ret != 0) {
        LOG(LOG_ERR, "reset_with_ami: hotplug toggle_sbr(%s) failed: %m", pf0_bdf);
        return hotplug_errno_to_vrtd_ret(errno);
    }
    LOG(LOG_INFO, "reset_with_ami: SBR toggle complete for %s", pf0_bdf);

    /*
     * Step 9-10: Rescan the PCI bus and verify the device reappears.
     * slash_hotplug_toggle_sbr() does not return until the V80 has had time
     * to reload its PDI and the upstream bridge reports a stable link.
     *
     * The rescan re-enumerates all functions. Poll the
     * AMI, QDMA, and control nodes until driver probing and udev
     * permissions are complete. If they are not all ready within 10 seconds,
     * retry the rescan after 3 seconds, up to 5 attempts total.
     */
    for (int attempt = 1; attempt <= RESCAN_MAX_RETRIES; attempt++) {
        bool ready = false;

        reset_emit_progress(
            progress_cb,
            progress_ctx,
            VRTD_CFGMEM_PROGRAM_PHASE_RESCANNING_PCIE
        );
        ret = slash_hotplug_rescan(g_hotplug);
        if (ret != 0) {
            LOG(LOG_ERR, "reset_with_ami: hotplug rescan failed: %m");
            return hotplug_errno_to_vrtd_ret(errno);
        }
        LOG(LOG_INFO, "reset_with_ami: rescan complete (attempt %d/%d)",
            attempt, RESCAN_MAX_RETRIES);

        for (int elapsed_us = 0;
             elapsed_us < RESET_NODE_READY_TIMEOUT_US;
             elapsed_us += RESET_NODE_POLL_INTERVAL_US) {
            if (reset_functions_ready(pf0_bdf, ctl_path, &ami_device)) {
                ready = true;
                break;
            }
            usleep(RESET_NODE_POLL_INTERVAL_US);
        }

        if (ready) {
            LOG(LOG_INFO, "reset_with_ami: PF0/PF1/PF2 ready after reset");
            break;
        }

        ami_dev_delete(&ami_device);
        if (attempt < RESCAN_MAX_RETRIES) {
            LOG(LOG_WARNING,
                "reset_with_ami: PF0/PF1/PF2 not ready (attempt %d/%d), retrying rescan in 3s",
                attempt, RESCAN_MAX_RETRIES);
            usleep(RESCAN_RETRY_DELAY_US);
        } else {
            LOG(LOG_ERR,
                "reset_with_ami: PF0/PF1/PF2 not ready after %d rescan attempts",
                RESCAN_MAX_RETRIES);
            return VRTD_RET_INTERNAL_ERROR;
        }
    }

    ami_dev_delete(&ami_device);

    /*
     * Step 11: Run device discovery to re-add the reset device to vrtd's
     * tracked device list.  This opens the QDMA function, sets up queues,
     * and makes the device available for user requests again.
     */
    // We now rescan for the reset device
    reset_emit_progress(
        progress_cb,
        progress_ctx,
        VRTD_CFGMEM_PROGRAM_PHASE_REDISCOVERING_DEVICE
    );
    ret = devices_discover_and_open(devices);
    if (ret != 0) {
        LOG(LOG_ERR, "reset_with_ami: devices_discover_and_open failed after reset");
        return VRTD_RET_INTERNAL_ERROR;
    }

    /*
     * Record the shell that is now booted so callers (e.g. v80-smi list) can
     * report it.  Rediscovery creates a fresh device struct with an UNKNOWN
     * shell, so without this the shell would be lost after every reset.
     */
    enum vrtd_shell_type booted_shell = shell_from_boot_partition(partition);
    for (size_t i = 0; i < devices->len; i++) {
        struct device *new_device = devices->d[i];
        if (new_device != NULL && strcmp(new_device->pci_info.bdf, target_bdf) == 0) {
            /*
             * The boot partition states which shell was intended. Confirm the
             * device agrees before recording it: a partition that did not take
             * effect would otherwise leave vrtd asserting a shell the hardware
             * is not running, and every later decision keyed on the shell —
             * whether a reset is required, which register windows exist — would
             * be made against the wrong design.
             */
            if (build_id_check_shell(
                    new_device->bar_files[BUILD_ID_BAR_NUMBER],
                    booted_shell,
                    "reset_with_ami"
                ) != 0) {
                return VRTD_RET_INTERNAL_ERROR;
            }
            new_device->current_shell = booted_shell;
            return VRTD_RET_OK;
        }
    }

    LOG(LOG_ERR, "reset_with_ami: target %s missing after rediscovery", target_bdf);
    return VRTD_RET_NOEXIST;
}

uint16_t reset_with_ami_partition(
    struct device *device,
    struct device_ptr_array *devices,
    uint32_t partition
)
{
    return reset_with_ami_partition_progress(device, devices, partition, NULL, NULL);
}

uint16_t reset_with_ami(
    struct device *device,
    struct device_ptr_array *devices,
    enum vrtd_shell_type target_shell
)
{
    uint32_t boot_partition = 0;
    if (shell_boot_partition(target_shell, &boot_partition) != 0) {
        LOG(LOG_ERR, "reset_with_ami: invalid target shell %u", (unsigned int)target_shell);
        return VRTD_RET_INVALID_ARGUMENT;
    }

    /* reset_with_ami_partition_progress() records the booted shell for us. */
    return reset_with_ami_partition(device, devices, boot_partition);
}
