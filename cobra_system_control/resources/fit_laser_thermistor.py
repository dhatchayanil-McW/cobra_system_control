# https://github.com/fbaeuerlein/jupyter-steinhart-hart

import csv
import numpy as np
try:
    from scipy.optimize import curve_fit
except ModuleNotFoundError as e:
    print('Need to install dev dependencies')
    raise e


# CSV file
CSV_FILE = 'ntcg063jf103ftb.csv'
CSV_SKIP = 6
TEMP_COLUMN = 0
R_MIN_COLUMN = 1
R_TYP_COLUMN = 2
R_MAX_COLUMN = 3


# Steinhart-Hart-Equation
def sh_eq(x, a0, a1, a2, a3):
    return 1. / (a0 + a1 * np.log(x) + a2 * np.power(np.log(x),2)
                 + a3 * np.power(np.log(x),3))


def temp_c(res_therm):
    tk = sh_eq(res_therm, params[0], params[1], params[2], params[3])
    tc = tk - 273.15
    return tc


if __name__ == "__main__":

    temp = []
    ohm = []
    min_ohm = []
    max_ohm = []

    line_num = 0
    # read data from CSV file
    with open(CSV_FILE, newline='') as csvfile:
        datareader = csv.reader(csvfile, delimiter=',')
        for row in datareader:
            # skip the header
            if line_num >= CSV_SKIP:
                # print(row)
                temp.append(float(row[TEMP_COLUMN]) + 273.15) # convert temperature to absolute temperature
                ohm.append(float(row[R_TYP_COLUMN])*1000) # resistance values in kOhm
                min_ohm.append(float(row[R_MIN_COLUMN])*1000)
                max_ohm.append(float(row[R_MAX_COLUMN])*1000)
            line_num = line_num + 1

    # Do the fit with initial values (needed, otherwise no meaningful result)
    params, cov = curve_fit(sh_eq, ohm, temp, p0=[1e-4, 1e-4, 1e-4, 1e-4])
    print("Coeffs: {}".format(params))

    # generate curve with fit result
    y2 = []
    for v in ohm:
        y2.append(sh_eq(v, *params))

    # plot original data and result
    # plt.figure()
    # plt.plot(ohm, temp, '-o', label='data')
    # plt.plot(ohm, y2, label='fit')
    #
    # plt.plot(ohm, np.abs(np.subtract(temp, y2))) # error values
    #
    # plt.figure()
    # plt.semilogy(np.array(temp)-273.15, ohm)


    # Check the fit
    thermistor_data = []
    model_tolerance = 0.1

    tuple = (-40, 188.5*1000)
    thermistor_data.append(tuple)
    tuple = (-20,  67.79*1000)
    thermistor_data.append(tuple)
    tuple = (  0,  27.28*1000)
    thermistor_data.append(tuple)
    tuple = ( 20,  12.09*1000)
    thermistor_data.append(tuple)
    tuple = ( 60,   3.019*1000)
    thermistor_data.append(tuple)
    tuple = ( 80,   1.668*1000)
    thermistor_data.append(tuple)
    tuple = (100,   0.975*1000)
    thermistor_data.append(tuple)
    tuple = (125,   0.534*1000)
    thermistor_data.append(tuple)

    for temp, res in thermistor_data:
        temp_calc = sh_eq(res, params[0], params[1], params[2], params[3]) - 273.15
        error = abs(temp_calc - temp)
        if (error > 0.1):
            msg1 = f'Calculated temp does not match MFG data! '
            msg2 = f'Calculated {temp_calc} deg C for R={res}. '
            msg3 = f'Expected {temp} deg C'
            raise RuntimeError(msg1 + msg2 + msg3)

    # plt.show()
