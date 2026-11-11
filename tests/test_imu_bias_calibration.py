"""Full PSD calibration, exact modes and explicit model certificate handling."""
import unittest
import numpy as np
from posim_slam.imu_bias_calibration import InitialImuBiasCalibration,initial_bias_calibration_from_model
from posim_slam.live_client import LiveClient


class InitialBiasCalibrationTests(unittest.TestCase):
    def test_full_cross_axis_covariance_and_nonzero_mean_keep_their_information(self):
        rng=np.random.default_rng(984);q=np.linalg.qr(rng.normal(size=(6,6)))[0]
        for eigenvalues in ([.01,.02,.03,.1,.2,.3],[0,0,.01,.02,.2,.3],[0]*6):
            covariance=(q*np.array(eigenvalues))@q.T;mean=np.arange(6)/100
            calibration=InitialImuBiasCalibration(mean,covariance,'bench-calibration')
            same=InitialImuBiasCalibration.read(calibration.packet());rows=same.rows()
            white=rows['jacobian'];hard=rows['constraint_jacobian']
            np.testing.assert_allclose(white.T@white,np.linalg.pinv(covariance,rcond=1e-12),atol=2e-11)
            np.testing.assert_allclose(hard@covariance,0,atol=2e-15)
            np.testing.assert_allclose(rows['offset'],-white@mean)
            np.testing.assert_allclose(rows['constraint_offset'],-hard@mean)
            self.assertEqual(len(white)+len(hard),6)

    def test_zero_ros_covariance_or_unavailable_calibration_does_not_certify_bias(self):
        self.assertIsNone(initial_bias_calibration_from_model(dict(covariance=[0]*9)))
        self.assertIsNone(initial_bias_calibration_from_model(dict(simulated_bias_calibration=dict(available=False))))
        with self.assertRaisesRegex(ValueError,'external_bias_covariance'):
            initial_bias_calibration_from_model(dict(simulated_bias_calibration=dict(available=True,source='external')))

    def test_explicit_noiseless_model_is_an_initial_constraint_and_does_not_change_evolution(self):
        value=dict(available=True,source='generated_sdf_noiseless_imu',gyro_bias=[0]*3,accel_bias=[0]*3,
            gyro_bias_covariance=np.zeros((3,3)),accel_bias_covariance=np.zeros((3,3)))
        result=initial_bias_calibration_from_model(dict(simulated_bias_calibration=value))
        self.assertEqual(result.packet()['scope'],'first_state');self.assertEqual(len(result.rows()['constraint_jacobian']),6)
        self.assertNotIn('gyro_bias_random_walk',result.packet())
        value['gyro_bias']=[.1,0,0]
        with self.assertRaisesRegex(ValueError,'noiseless_bias_certificate'):
            initial_bias_calibration_from_model(dict(simulated_bias_calibration=value))

    def test_indefinite_missing_frame_and_nonfinite_inputs_reject(self):
        good=InitialImuBiasCalibration(np.zeros(6),np.eye(6),'external').packet()
        for change in [dict(frame='world'),dict(scope='forever'),dict(covariance=-np.eye(6)),
                       dict(mean=[float('nan')]*6),dict(covariance=np.eye(3)),dict(source='')]:
            with self.subTest(change=change),self.assertRaises(ValueError):InitialImuBiasCalibration.read(dict(good,**change))

    def test_worker_must_acknowledge_calibration_instead_of_silently_ignoring_it(self):
        client=object.__new__(LiveClient)
        client.processing_request=dict(initial_imu_bias_calibration=InitialImuBiasCalibration(np.zeros(6),np.zeros((6,6)),'explicit').packet())
        report=dict(source='explicit',scope='first_state',finite_modes=0,exact_modes=6,
            stochastic_evolution_unchanged=True,generic_initial_bias_prior_replaced=True)
        client._check_processing_ready(dict(initial_imu_bias_calibration=report))
        for value in ({},dict(initial_imu_bias_calibration=dict(report,exact_modes=5)),
                      dict(initial_imu_bias_calibration=dict(report,stochastic_evolution_unchanged=False))):
            with self.assertRaisesRegex(ValueError,'calibration_contract'):client._check_processing_ready(value)


if __name__=='__main__':unittest.main()
