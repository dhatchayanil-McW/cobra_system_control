from cobra_system_control.mcs_reader import LcmMcsWriter
from cobra_system_control.metasurface import LcmAssembly


def main():

    lcmsa = LcmAssembly(255, 0, 0, 0)

    mcs_out = 'ninev_voltage_patterns.mcs'

    wr = LcmMcsWriter(0x10_0000,
                      lcmsa.pattern_table_list,
                      mirror_user_data=True,
                      write_eof_record=True,
                      out_file=mcs_out)

    wr.write_mcs()


if __name__ == "__main__":
    main()
