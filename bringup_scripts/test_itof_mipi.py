import argparse
import sys
import time
from cobra_system_control.fpga_misc import FpgaDbg
from cobra_system_control.itof import Itof, FrameSettings, NumFramesOv, PlecoMode
from cobra_system_control.mipi import MipiTx, MipiRx
from cobra_system_control.cobra import Cobra


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-a', '--apply',
                        action='store_true',
                        help='Apply settings to the Pleco')
    parser.add_argument('-r', '--reset',
                        action='store_true',
                        help='Soft reset')
    parser.add_argument('-t', '--trigger',
                        action='store_true',
                        help='Software Trigger the Pleco')
    parser.add_argument('-w', '--hw',
                        action='store_true',
                        help='Hardware Trigger the Pleco')
    parser.add_argument('-d', '--dump',
                        action='store_true',
                        help='Dump the MIPI registers')
    parser.add_argument('-c', '--cont',
                        action='store_true',
                        help='Enable continuous mode')
    parser.add_argument('-p', '--pack',
                        action='store_true',
                        help='enable packet fifo')
    return parser.parse_args(argv)


def get_status(c: Cobra, extra):
    empty = c.sen[0].isp.read_fields("mipi_rx_pkt_fifo_empty")
    full = c.sen[0].isp.read_fields("mipi_rx_pkt_fifo_full")
    count = c.sen[0].isp.read_fields("mipi_rx_pkt_fifo_count")
    wm = c.sen[0].isp.read_fields("mipi_rx_pkt_fifo_watermark")
    dt = c.sen[0].isp.read_fields("mipi_rx_pkt_fifo_dt")
    wc_lo = c.sen[0].isp.read_fields("mipi_rx_pkt_fifo_wc_lo")
    wc_hi = c.sen[0].isp.read_fields("mipi_rx_pkt_fifo_wc_hi")
    print(f'{extra}'
          f' empty = {empty}, '
          f' full = {full}, '
          f' count = {count}, '
          f' wm = {wm}, '
          f' dt = {dt:#05x}, '
          f' wc = {wc_hi << 8 | wc_lo}, '
    )



if __name__ == '__main__':
    args = parse_args(sys.argv[1:])
    try:
        with Cobra.open() as c:
            print(f'{c.sen[0].debug.read_fields("git_sha"):#010x}')
            if args.reset:
                input('Press enter to reset. CAREFUL This resets all registers and internal state.')
                c.sen[0].debug.write_fields(soft_reset=1)
                time.sleep(1)
                c.setup()
            print(f'{c.sen[0].debug.read_fields("git_sha"):#010x}')

            c.sen[0].debug.write_fields(dbg_out_en=1)
            c.sen[0].debug.write_fields(dbg_sel=6)
            c.sen[0].debug.write_fields(itof_reset_b=1)
            if args.pack:
                c.sen[0].isp.write_fields(pkt_fifo_en=1)
            time.sleep(0.5)
            if args.dump:
                print('\nrx read')
                c.sen[0].mipi_rx.periph.read_all_periph_fields(with_print=True)
                print('\ntx read')
                c.sen[0].mipi_tx.periph.read_all_periph_fields(with_print=True)
                print('\nisp read')
                c.sen[0].isp.periph.read_all_periph_fields(with_print=True)
                print(f'{c.sen[0].debug.read_fields("git_sha"):#010x}')

            if args.apply:
                print('Applying frame settings')
                if args.pack:
                    nfrm = 2
                elif args.cont:
                    nfrm = 0
                else:
                    nfrm = 2
                c.sen[0].itof.apply_frame_settings(FrameSettings(0, 20, n_frames_capt=nfrm))
                print('Done applying settings')

            if args.trigger:
                print('First itof Trigger')
                if args.hw:
                    c.sen[0].itof.write_fields(so_freq_en=0)
                    c.sen[0].scan.write_fields(itof_trigger_override='on')
                    c.sen[0].scan.write_fields(itof_trigger_override='off')
                else:
                    c.sen[0].itof.write_fields(so_freq_en=1)
                    c.sen[0].itof.soft_trigger(check_limits=False)
                    c.sen[0].itof.write_fields(so_freq_en=0)
                time.sleep(1)
                if args.pack:
                    empty = c.sen[0].isp.read_fields("mipi_rx_pkt_fifo_empty")
                    count = 0
                    while empty != 1:
                        c.sen[0].isp.write_fields(pkt_fifo_pop=1)
                        empty = c.sen[0].isp.read_fields("mipi_rx_pkt_fifo_empty")
                        count += 1
                    print(f'Popped {count} after first trigger')

                print(f'{c.sen[0].debug.read_fields("git_sha"):#010x}')
                if not args.cont:
                    input('Second itof trigger. Press Enter!')
                    if args.hw:
                        c.sen[0].itof.write_fields(so_freq_en=0)
                        c.sen[0].scan.write_fields(itof_trigger_override='on')
                        c.sen[0].scan.write_fields(itof_trigger_override='off')
                    else:
                        c.sen[0].itof.write_fields(so_freq_en=1)
                        c.sen[0].itof.soft_trigger(check_limits=False)
                        c.sen[0].itof.write_fields(so_freq_en=0)
                    time.sleep(1)
                if args.pack:
                    empty = c.sen[0].isp.read_fields("mipi_rx_pkt_fifo_empty")
                    idx = 0
                    while empty != 1:
                        c.sen[0].isp.write_fields(pkt_fifo_pop=1)
                        get_status(c, f'{idx}')
                        empty = c.sen[0].isp.read_fields("mipi_rx_pkt_fifo_empty")
                        idx += 1
                    print(f'Popped {idx} after second trigger')

            if args.dump:
                print(f'{c.sen[0].debug.read_fields("git_sha"):#010x}')
                print('\nrx read')
                c.sen[0].mipi_rx.periph.read_all_periph_fields(with_print=True)
                print('\ntx read')
                c.sen[0].mipi_tx.periph.read_all_periph_fields(with_print=True)
                print('\nisp read')
                c.sen[0].isp.periph.read_all_periph_fields(with_print=True)
                print(f'{c.sen[0].debug.read_fields("git_sha"):#010x}')

    except KeyboardInterrupt:
        c.disconnect()
    finally:
        c.disconnect()
