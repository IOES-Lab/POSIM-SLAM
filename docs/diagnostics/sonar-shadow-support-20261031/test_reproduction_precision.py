# SPDX-License-Identifier: Apache-2.0
"""Preserve exact observations while allowing arithmetic roundoff on recompute."""
import unittest
import numpy as np
from verify import assert_derived


class ReproductionPrecision(unittest.TestCase):
    def test_actual_cross_platform_origin_roundoff(self):
        saved=[-.05678341256813268,.20587599122438915,.729505]
        computed=[-.05678341256813278,.20587599122438918,.729505]
        with self.assertRaises(AssertionError):np.testing.assert_array_equal(saved,computed)
        original=list(saved)
        assert_derived(saved,computed)
        self.assertEqual(saved,original)

    def test_changed_metric_or_assignment_fails(self):
        expected=[dict(component=7,beam=24,edge_range_m=3.2,boundary_ranges_m=[3.1,3.2,3.5,3.6])]
        changed=[dict(expected[0],edge_range_m=3.200001)]
        with self.assertRaises(AssertionError):assert_derived(changed,expected)
        with self.assertRaises(AssertionError):assert_derived([dict(expected[0],beam=25)],expected)
        with self.assertRaises(AssertionError):assert_derived([],expected)

    def test_exact_export_check_still_rejects_one_ulp(self):
        coordinates=np.array([[3.2,.5,-3.4]])
        changed=coordinates.copy();changed[0,0]=np.nextafter(changed[0,0],np.inf)
        with self.assertRaises(AssertionError):np.testing.assert_array_equal(changed,coordinates)


if __name__=='__main__':unittest.main()
