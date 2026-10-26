import unittest
import numpy as np
from posim_slam.auxiliary_inputs import bracketed_gyro, hydrostatic_height, validate_auxiliary_ack, fused_recent, dvl_status, measured_dvl_packet


class AuxiliaryInputTests(unittest.TestCase):
    def test_interpolation_preserves_clock_and_propagates_independent_variance(self):
        samples = [(10**9, [0, 2, 4], (np.eye(3)*.04).ravel()),
                   (1_020_000_000, [4, 6, 8], (np.eye(3)*.16).ravel())]
        value = bracketed_gyro(samples, 1_005_000_000, model_sigma=.1)
        np.testing.assert_allclose(value['gyro'], [1, 3, 5])
        np.testing.assert_allclose(value['covariance'], np.eye(3)*(.75**2*.04+.25**2*.16+.01))
        self.assertTrue(value['interpolated'])
        self.assertEqual(value['unknown_covariance_inputs'], 0)

    def test_unknown_covariance_requires_explicit_model_and_never_extrapolates(self):
        samples = [(10**9, [0, 0, 0], [-1]+[0]*8),
                   (1_010_000_000, [0, 0, 0], [0]*9)]
        value = bracketed_gyro(samples, 1_005_000_000, model_sigma=.01)
        self.assertEqual(value['unknown_covariance_inputs'], 2)
        np.testing.assert_allclose(value['covariance'], np.eye(3)*.0001)
        for stamp, sigma in ((999_999_999, .01), (1_010_000_001, .01), (10**9, 0)):
            with self.subTest(stamp=stamp, sigma=sigma), self.assertRaises(ValueError):
                bracketed_gyro(samples, stamp, model_sigma=sigma)
        with self.assertRaisesRegex(ValueError, 'clock'):
            bracketed_gyro(samples[::-1], 10**9, model_sigma=.01)

    def test_dvl_identity_lock_and_original_clock(self):
        value = dict(session_nonce='trial', frame_id='dvl', source='/vehicle/sensors/dvl',
                     version='0.1.0', valid=True, reason='bottom_locked', locked_beams=4,
                     stamp=dict(sec=1, nanosec=5_000_000), velocity_m_s=[1, 0, 0],
                     covariance=(np.eye(3)*.04).ravel().tolist())
        t, payload = dvl_status(value, nonce='trial', frame='dvl')
        self.assertIsNone(dvl_status(value, nonce='other', frame='dvl'))
        samples = [(10**9, [0, 0, 0], [0]*9), (1_010_000_000, [0, 2, 0], [0]*9)]
        packet = measured_dvl_packet(t, payload, samples, gyro_sigma=.01)
        self.assertEqual(packet['stamp_ns'], 1_005_000_000)
        self.assertEqual(packet['gyro_stamp_ns'], t)
        np.testing.assert_allclose(packet['gyro'], [0, 1, 0])
        np.testing.assert_allclose(packet['covariance'], np.eye(3)*.04)
        for change in [dict(frame_id='wrong'), dict(locked_beams=2),
                       dict(covariance=[0]*9), dict(stamp=dict(sec=1, nanosec=10**9))]:
            with self.subTest(change=change), self.assertRaises(ValueError):
                dvl_status(dict(value, **change), nonce='trial', frame='dvl')
        lost = dict(value, valid=False, reason='bottom_lock_lost', velocity_m_s=None, covariance=None)
        lost_t, missing = dvl_status(lost, nonce='trial', frame='dvl')
        self.assertEqual(lost_t, t)
        self.assertEqual(missing, {'valid': False})
        with self.assertRaises(ValueError):
            measured_dvl_packet(t, missing, samples, gyro_sigma=.01)
        with self.assertRaises(ValueError):
            dvl_status(dict(value, valid=False), nonce='trial', frame='dvl')

    def test_pressure_does_not_invent_surface_height_or_accept_unknown_variance(self):
        calibration = dict(reference_pa=101325, density_kg_m3=1025, gravity_m_s2=9.80665)
        scale = 1025*9.80665
        value = hydrostatic_height(123, 101325+2*scale, 900, **calibration)
        self.assertEqual(value['packet']['stamp_ns'], 123)
        self.assertAlmostEqual(value['packet']['height_m'], -2)
        self.assertAlmostEqual(value['packet']['variance_m2'], 900/scale**2)
        surface = hydrostatic_height(124, 101325, 900, **calibration)
        self.assertFalse(surface['valid'])
        self.assertNotIn('packet', surface)
        with self.assertRaises(ValueError):
            hydrostatic_height(125, 101325+scale, 0, **calibration)

    def test_acknowledged_fusion_is_distinct_from_reception(self):
        self.assertEqual(validate_auxiliary_ack({'pressure':{'owner_received':True}},dict(pressure=True)),())
        self.assertEqual(validate_auxiliary_ack({'pressure':{'owner_received':True,'optimization_included':True}},dict(pressure=True)),('pressure',))
        for report,expected in [({'owner_received':True},False),({'optimization_included':True},False),({'owner_received':True,'optimization_included':1},True),([],True)]:
            with self.assertRaises(RuntimeError):validate_auxiliary_ack({'pressure':report},dict(pressure=expected))

    def test_freshness_requires_both_clocks_and_a_real_measurement(self):
        self.assertTrue(fused_recent(2_000_000_000,1_000_000_000,0.,1.))
        for state,fused,wall,now in [(3_000_000_000,1_000_000_000,0.,1.),(2_000_000_000,1_000_000_000,0.,2.),(2,0,0.,1.),(1,2,0.,1.),(2,1,None,1.),(2,1,2.,1.)]:
            self.assertFalse(fused_recent(state,fused,wall,now))


if __name__ == '__main__': unittest.main()
