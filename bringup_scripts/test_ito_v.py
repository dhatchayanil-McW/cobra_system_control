from cobra_system_control.cobra import Cobra


if __name__ == '__main__':
    with Cobra.remote() as c:
        ito = c.sen.ito_dac
        ito.enable()
        ito.set_voltage(9)
