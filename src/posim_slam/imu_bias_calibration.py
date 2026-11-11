"""Explicit initial-IMU bias calibration; stochastic evolution stays separate."""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class InitialImuBiasCalibration:
    mean: np.ndarray
    covariance: np.ndarray
    source: str

    def __post_init__(self):
        mean=np.asarray(self.mean,dtype=float);c=np.asarray(self.covariance,dtype=float)
        if (mean.shape!=(6,) or c.shape!=(6,6) or not np.isfinite(mean).all() or not np.isfinite(c).all()
                or np.max(np.abs(mean))>1000 or np.max(np.abs(c))>1000
                or not np.allclose(c,c.T,rtol=0,atol=1e-12)
                or not isinstance(self.source,str) or not 1<=len(self.source)<=128 or not self.source.strip()):
            raise ValueError('explicit_full_initial_bias_calibration_required')
        values=np.linalg.eigvalsh(c);tol=64*np.finfo(float).eps*float(np.max(np.abs(values)))
        if values[0]<-tol:raise ValueError('positive_semidefinite_bias_covariance_required')
        mean=mean.copy();c=c.copy();mean.flags.writeable=False;c.flags.writeable=False
        object.__setattr__(self,'mean',mean);object.__setattr__(self,'covariance',c)

    @classmethod
    def read(cls,packet):
        if (packet.get('schema')!='posim-slam-initial-imu-bias-calibration-1'
                or packet.get('frame')!='input_imu' or packet.get('scope')!='first_state'):
            raise ValueError('initial_bias_calibration_frame_scope_required')
        return cls(packet['mean'],packet['covariance'],packet['source'])

    def packet(self):
        return dict(schema='posim-slam-initial-imu-bias-calibration-1',frame='input_imu',scope='first_state',
            mean=self.mean.tolist(),covariance=self.covariance.tolist(),source=self.source)

    def rows(self):
        values,vectors=np.linalg.eigh(self.covariance)
        tolerance=64*np.finfo(float).eps*float(np.max(np.abs(values)));positive=values>tolerance
        white=vectors[:,positive].T/np.sqrt(values[positive])[:,None]
        hard=vectors[:,~positive].T
        return dict(jacobian=white,offset=-white@self.mean,constraint_jacobian=hard,constraint_offset=-hard@self.mean)


def initial_bias_calibration_from_model(imu_metadata):
    """Use the explicit noiseless-model certificate, never ROS covariance/RMS."""
    value=imu_metadata.get('simulated_bias_calibration',{})
    if value.get('available') is not True:return None
    if value.get('source')!='generated_sdf_noiseless_imu':
        raise ValueError('complete_external_bias_covariance_required')
    means=np.r_[value['gyro_bias'],value['accel_bias']]
    c=np.zeros((6,6));c[:3,:3]=value['gyro_bias_covariance'];c[3:,3:]=value['accel_bias_covariance']
    if np.any(means!=0) or np.any(c!=0):raise ValueError('invalid_noiseless_bias_certificate')
    return InitialImuBiasCalibration(means,c,value['source'])
