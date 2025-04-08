"""
file: qa_helper_careful_corrupt_flash_fpga_bitstream.py

Copyright (C) 2023-2024 Lumotive, Inc. All rights reserved.

This file is a helper script during the QA process to
corrupt a small portion of the spi flash were the primary
partition of the FPGA bitstream is located.
Before running this script, the remote object should be stopped.
After running this script, the remote object should be restarted
to confirm that the FPGA boots into its golden image and the FPGA
update process starts and succeeds.

Note the hardcoded memory locations in the script may need to change
if the spi flash memory is reallocated.
"""
from cobra_system_control.cobra import Cobra
from cobra_system_control.mcs_updater import delete_memory


if __name__ == "__main__":
    with Cobra.open(system_type="m30", board_type="nxp") as c:
        delete_memory(c.sen.spi_flash, 0x02_0000, 0x04_0000)
