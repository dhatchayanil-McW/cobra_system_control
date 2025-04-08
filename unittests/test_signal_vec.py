import unittest
from parameterized.parameterized import parameterized_class
from parameterized import parameterized
import numpy as np
from cobra_system_control.numerical_utilities import SignalVec


@parameterized_class(
    ('sign', 'n_bits', 'n_frac',
     'float_vec',
     'expected_fxp_vec',
     'expected_dig_vec',
     'expected_bin_vec',
     'expected_error'),
    (
            (True, 0, 0,  # FXPformat
             [],  # input float vector
             [],  # expected fxp vector
             [],  # expected dig vector
             [],  # expected bin vector,
             ValueError  # expected error (if any)
             ),

            (True, 1, 0,
             np.array([-1.0, 0.0]),
             np.array([-1.0, 0.0]),
             np.array([1, 0]),
             np.array(['1.', '0.']),
             np.array(['1']),
             None),

            (False, 8, 0,
             0,
             np.array((0,)),
             np.array([0]),
             np.array(['00000000.']),
             None),

            (False, 8, 0,
             [0, 1, 3],
             np.array((0, 1, 3)),
             np.array([0, 1, 3]),
             np.array(['00000000.', '00000001.', '00000011.', ]),
             None),

            (True, 16, 9,
             np.array([-2, -1.5117, -0.584, 0.4004, 0.6738]),
             np.array([-2, -1.51171875, -0.58398438, 0.40039062, 0.67382812]),
             np.array([64512, 64762, 65237, 205, 345]),
             np.array(['1111110.000000000', '1111110.011111011', '1111111.011010101',
                       '0000000.011001101', '0000000.101011000']
                      ),
             None),

            (True, 16, 14,
             np.array([-2, -1.51234, -.5834, 0.4, 0.6736]),
             np.array([-2., -1.5123291, -0.58337402, 0.40002441, 0.67358398]),
             np.array([32768, 40758, 55978, 6554, 11036]),
             np.array(('10.00000000000000', '10.01111100110110', '11.01101010101010',
                       '00.01100110011001', '00.10101100011100')
                      ),
             None)
    )
)
class TestSignalVec(unittest.TestCase):

    def setUp(self) -> None:
        if self.n_bits < 1:
            self.assertRaises(self.expected_error, SignalVec,
                              *(self.sign, self.n_bits, self.n_frac))
        else:
            self.sv = SignalVec(self.sign, self.n_bits, self.n_frac)
            self.sv.set_float_vec(self.sv._clamp(
                self.float_vec,
                self.sv.fxpformat.fxp_min,
                self.sv.fxpformat.fxp_max))

    @parameterized.expand(
        [
            (0.0, -1, 1, 0),
            (0.0, 1, 2, 1),
            (4, 1, 3, 3),
            (4, 3, 1, ValueError)

        ]
    )
    def test_clamp(self, inpu, mini, maxi, expected):
        if self.n_bits < 1:
            self.skipTest('fail case')
        if type(expected) is not int:
            with self.assertRaises(expected):
                self.sv._clamp(inpu, mini, maxi)
        else:
            actual = self.sv._clamp(inpu, mini, maxi)
            self.assertEqual(expected, actual)

    def test_float_to_fxp(self):
        if self.n_bits < 1:
            self.skipTest('fail case')
        np.testing.assert_array_almost_equal(self.sv.get_fxp_vec(),
                                             self.expected_fxp_vec)

    def test_float_to_dig(self):
        if self.n_bits < 1:
            self.skipTest('fail case')
        np.testing.assert_array_equal(self.sv.get_dig_vec(),
                                      self.expected_dig_vec)

    def test_twos_com_vec(self):
        if self.n_bits < 1:
            self.skipTest('fail case')
        np.testing.assert_array_equal(self.sv.get_twoscomp_vec(),
                                      self.expected_bin_vec)

    def test_get_float_vec(self):
        if self.n_bits < 1:
            self.skipTest('fail case')
        np.testing.assert_array_almost_equal(self.sv.get_float_vec(),
                                             self.float_vec)

    def test_dig_to_float(self):
        if self.n_bits < 1:
            self.skipTest('fail case')
        self.sv.set_dig_vec(self.expected_dig_vec)
        np.testing.assert_array_almost_equal(self.float_vec,
                                             self.sv.get_float_vec(), decimal=4)

    def test_set_dig_vec(self):
        if self.n_bits < 1:
            self.skipTest('Fail init')
        self.sv.set_dig_vec(self.expected_dig_vec)
        if isinstance(self.float_vec, int):
            exp = np.array(self.float_vec)
        else:
            exp = self.float_vec
        np.testing.assert_array_almost_equal(self.sv.get_float_vec(),
                                             exp, decimal=4)

    def test_set_fxp_vec(self):
        if self.n_bits < 1:
            self.skipTest('Fail init')
        self.sv.set_fxp_vec(self.expected_fxp_vec)
        if isinstance(self.float_vec, int):
            exp = np.array(self.float_vec)
        else:
            exp = self.float_vec
        np.testing.assert_array_almost_equal(self.sv.get_float_vec(),
                                             exp, decimal=4)

    def test_float_to_bin(self):
        if self.n_bits < 1:
            self.skipTest('fail init')
        if isinstance(self.float_vec, int):
            self.assertRaises(ValueError, self.sv.float2bin, self.float_vec)
        elif isinstance(self.float_vec, list):
            self.assertRaises(ValueError, self.sv.float2bin, self.float_vec)
        else:
            np.testing.assert_array_equal(self.sv.float2bin(self.float_vec), self.expected_bin_vec)

    def test_fixed_to_dig_to_fixed(self):
        if self.n_bits < 1:
            self.skipTest('Fail init')
        dig = self.sv.fixed_to_dig()
        np.testing.assert_array_equal(dig, self.expected_dig_vec)
        np.testing.assert_array_almost_equal(
            self.expected_fxp_vec, self.sv.dig_to_fixed(dig), decimal=7)

    def check_quantize_no_op(self):
        if self.n_bits < 1:
            self.skipTest('Fail init')
        np.testing.assert_array_equal(self.expected_fxp_vec, self.quantize())


if __name__ == '__main__':
    unittest.main()
