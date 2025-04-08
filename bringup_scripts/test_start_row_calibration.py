try:
    import matplotlib.pyplot as plt
    from mpl_toolkits.axes_grid1 import make_axes_locatable
except ImportError as exc:
    raise exc

from cobra_system_control.metasurface import LcmAssembly
from cobra_system_control.pixel_mapping import DEFAULT_PIXEL_MAPPING
from cobra_system_control.start_row_calibration import (
    get_preliminary_arrays, get_row_start_change_thresholds,
    RowCalTable,
)


def main():

    pmap = DEFAULT_PIXEL_MAPPING
    pmap_a2a_tuple = pmap.generate_a2a_arrays()
    for lam in [910,]:
        for lcm in [2,]:
            print('LCM', lcm, 'lambda', lam)
            lcmsa = LcmAssembly(lcm, 1, 0, 1, lam)
            order_fields, adc_array, pixel_shift_array = (
                get_preliminary_arrays(
                    # Here, there are ADC vals that come up NaN and screw things up
                    lcmsa, pmap_a2a_tuple, 0.000378, 0.0019, 1.22, 'm30')
                )
            thresholds = get_row_start_change_thresholds(
                order_fields, adc_array, pixel_shift_array)

            rt = RowCalTable.build(thresholds)
            print(rt)

            fig, ax = plt.subplots()
            im = ax.pcolormesh(order_fields, adc_array, pixel_shift_array, shading='nearest')
            divider = make_axes_locatable(ax)
            cax = divider.append_axes('right', size='10%', pad=0.1)
            _ = fig.colorbar(im, cax=cax, orientation='vertical')
            cax.set_ylabel('Pixel Shift')
            ax.set_title(f'Vectorized LCM {lcm}, {lam}')
            ax.set_xlabel('Order')
            ax.set_ylabel('ADC Ints >> 4')

    plt.show()


if __name__ == "__main__":
    main()
