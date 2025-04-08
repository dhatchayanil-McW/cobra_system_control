import argparse
import sys
import time
from cobra_system_control.cobra import Cobra


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-r', '--reset',
                        action='store_true',
                        help='Soft reset')
    return parser.parse_args(argv)


if __name__ == '__main__':
    args = parse_args(sys.argv[1:])
    try:
        #c.sen[0].debug.write_fields(soft_reset=1)
        cnt = 0
        bads = 0
        t_error = 0
        while True:
            with Cobra.open() as c:
                time.sleep(0.1)
                c.sen[0].debug.write_fields(dbg_out_en=1)
                input('Enter to reset')
                c.sen[0].debug.write_fields(soft_reset=1)
                time.sleep(0.1)
                c.sen[0].debug.write_fields(dbg_out_en=1)
                input('Continue after reset')
                try:
                    c.sen[0].debug.write_fields(dbg_out_en=1)
                    c.sen[0].debug.write_fields(dbg_sel=0)
                except TypeError as e:
                    #input('TypeError. Press enter to disconnect')
                    #c.disconnect()
                    #raise e('TypeError on debug write fields')
                    #t_error += 1
                    #time.sleep(0.1)
                    continue
                time.sleep(0.1)
                c.sen[0].isp.periph.read_all_periph_fields(with_print=True)
                data = c.sen[0].isp.read_fields('quant_mode')
                if data == ((0xdbdbdbdb >> 4) & 0x3) :
                    raise ValueError(f'read 0xdb')
                    #bads += 1
                #else:
                #    print(f'sensor id == {data:#07x}')
            c.disconnect()
            cnt += 1
            #if cnt % 10 == 0:
            print(f'********* cnt is {cnt}, bads = {bads}, TypeErrors = {t_error} *********')
            time.sleep(.4)
            #input('enter to continue')
    except KeyboardInterrupt:
        c.disconnect()
        c.disable()
    finally:
        c.disconnect()
