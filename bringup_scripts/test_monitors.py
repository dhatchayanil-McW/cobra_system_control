from cobra_system_control.cobra import Cobra


def get_adc_status(adc):
    return {result['name']: result['value']
            for result in adc.read_all_channels(scaled=True).values()}


def main(c: Cobra):
    print('\nGet system config')
    print(c.get_db_system_configuration())

    print('\nGet scan config')
    print(c.get_db_scan_table())

    print('\nGet MON CMB')
    print(c.get_db_mon_cmb_all())

    print('\nGet MON SH CMB')
    print(c.get_db_mon_sh_cmb_all())

    print('\nGet MON FPGA')
    print(c.get_db_mon_sh_fpga_all())


if __name__ == "__main__":
    with Cobra.remote() as c:
        main(c)
