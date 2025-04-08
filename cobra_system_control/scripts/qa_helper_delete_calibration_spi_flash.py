"""
file: qa_helper_delete_calibration_spi_flash.py

Copyright (C) 2023-2024 Lumotive, Inc. All rights reserved.

This file is a helper script during the QA process to
delete all the per-system calibration data on the sensor head.
Before running this script, the remote object should be stopped.
After running this script, the remote object should be restarted
to confirm that the system control software is able to boot
a new sensor head as it goes through the manufacturing
line or intake processes.

Note the hardcoded memory locations in the script may need to change
if the spi flash memory is reallocated.
"""
from cobra_system_control.cobra import Cobra
from cobra_system_control.mcs_updater import delete_memory


def main():
    with Cobra.open(
            system_type="m30", board_type="nxp") as c:
        delete_memory(c.sen.spi_flash, 0x025_0000, 0x029_c000)


if __name__ == "__main__":
    main()
