from cobra_system_control.cobra import Cobra


if __name__ == '__main__':
    with Cobra.open() as c:
        lcm = c.sen.lcm
        lcm.enable()
        print('Enabled LCM')
        i = 0
        try:
            while True:
                lcm.move_table(i, 0)
                print('Moved table safely')
                lcm.apply(0)
                i = (i + 10) % 40
                print(f'Applied {i}')
        finally:
            lcm.disable()
