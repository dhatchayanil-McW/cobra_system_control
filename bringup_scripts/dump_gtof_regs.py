import numpy as np
import pathlib

from cobra_system_control import COBRA_DIR
from cobra_system_control.cobra import Cobra
from cobra_system_control.itof import Itof, FrameSettings


DUMP_PATH = pathlib.Path(COBRA_DIR) / 'gtof_reg_dump.csv'


def main(itof: Itof):
    itof.setup()
    itof.apply_frame_settings(FrameSettings(0))
    regs = np.arange(0, 2000)
    vals = [itof._itof_spi_read(reg) for reg in regs]
    np.savetxt(DUMP_PATH, np.column_stack((regs, vals)), delimiter=',')


if __name__ == '__main__':
    with Cobra.open() as c_:
        main(c_.sen.itof)
