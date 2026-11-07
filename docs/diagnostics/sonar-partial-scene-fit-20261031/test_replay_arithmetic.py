import unittest
import numpy as np
from verify import derived_power_equal


class DerivedArithmeticTests(unittest.TestCase):
    def test_one_ulp_in_derived_power_is_not_a_raw_input_change(self):
        original = np.array([0.,1e-100,.001,4.767,1e100])
        rounded = np.nextafter(original, np.inf)
        rounded[0] = 0.
        derived_power_equal(rounded, original)

    def test_non_arithmetic_or_missing_weak_return_change_is_rejected(self):
        for actual, expected in [(1.+64*np.finfo(float).eps,1.),
                                  (0.,1e-100),(1e-100,0.),(np.nan,np.nan),(np.inf,np.inf)]:
            with self.assertRaises(AssertionError):
                derived_power_equal(np.array([actual]),np.array([expected]))


if __name__ == '__main__':
    unittest.main()
