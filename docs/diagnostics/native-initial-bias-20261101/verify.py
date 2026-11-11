"""Recompute the paired native joint uncertainty from public recorded inputs."""
from pathlib import Path
import gzip
import hashlib
import json
import xml.etree.ElementTree as ET
import numpy as np
from posim_slam.imu_bias_calibration import initial_bias_calibration_from_model
from posim_slam.native_surface import NativeSurfaceSnapshot
from posim_slam.surface_prior import Variable

ROOT=Path(__file__).resolve().parent


def main():
    for name,digest in json.loads((ROOT/'sha256.json').read_text()).items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
    source=json.loads((ROOT/'input-certificate.json').read_text())
    sensor=ET.fromstring(source['imu_sensor_xml'])
    assert sensor.get('type')=='imu' and all(n.get('type')=='none' for n in sensor.iter('noise'))
    bias=initial_bias_calibration_from_model(source['imu'])
    assert bias.packet()==source['initial_calibration']
    assert np.array_equal(bias.mean,np.zeros(6)) and np.array_equal(bias.covariance,np.zeros((6,6)))
    config=json.loads((ROOT/'configuration-certified.json').read_text())
    assert config.pop('initial_imu_bias_calibration')==bias.packet()
    assert config==json.loads((ROOT/'configuration-unknown.json').read_text())
    assert source['native_report']==dict(exact_modes=6,finite_modes=0,generic_initial_bias_prior_replaced=True,
        scope='first_state',source='generated_sdf_noiseless_imu',stochastic_evolution_unchanged=True)
    assert source['camera_frames']==source['ping_fused_states']==source['sonar_snapshots_supplied']==0
    entry=source['snapshot']
    with gzip.open(ROOT/'native-snapshot.json.gz','rb') as stream:raw=stream.read(entry['source_bytes']+1)
    assert len(raw)==entry['source_bytes']<=128*2**20
    assert hashlib.sha256(raw).hexdigest()==entry['source_sha256']
    snapshot=NativeSurfaceSnapshot.read(json.loads(raw))
    assert snapshot._archive.max_recovery_bytes==32*2**20
    assessment=json.loads((ROOT/'assessment.json').read_text())
    stamps=[row['stamp_ns'] for row in assessment['rows']]
    query=tuple(Variable('state',t,6,snapshot.reference('state',t).reference[:12]) for t in stamps)
    got=snapshot.joint_marginal(query)
    with np.load(ROOT/'expected-marginal.npz',allow_pickle=False) as expected:
        for name,key in [('covariance','observable_covariance'),('deltas','deltas'),('linearized_mode','linearized_mode')]:
            np.testing.assert_allclose(got[key],expected[name],rtol=1e-7,atol=1e-8)
        assert expected['unbounded_basis'].shape==got['unbounded_basis'].shape==(24,0)
    assert got['workspace_bound_bytes']<=32*2**20 and not got['map_qualified']
    assert not got['live_coordinates_treated_as_known'] and not got['independent_pose_observations']
    previous=ROOT.parent/'native-joint-query-20261101'
    unknown=json.loads((previous/'assessment.json').read_text())
    for i,(row,old) in enumerate(zip(assessment['rows'],unknown['rows'])):
        assert row['stamp_ns']==old['stamp_ns']
        block=got['observable_covariance'][6*i:6*i+6,6*i:6*i+6]
        position=3*np.sqrt(np.linalg.eigvalsh(block[:3,:3]).max())
        rotation=np.rad2deg(3*np.sqrt(np.linalg.eigvalsh(block[3:,3:]).max()))
        np.testing.assert_allclose(position,row['observable_position_three_sigma_m'],rtol=1e-7,atol=1e-8)
        np.testing.assert_allclose(rotation,row['observable_rotation_three_sigma_deg'],rtol=1e-7,atol=1e-8)
        assert position<old['observable_position_three_sigma_m']
        assert rotation<old['observable_rotation_three_sigma_deg']
    before=np.genfromtxt(previous/'states-snapshot.csv',delimiter=',',skip_header=1)
    after=np.genfromtxt(ROOT/'states-certified.csv',delimiter=',',skip_header=1)
    assert before.shape==after.shape==(271,8)
    np.testing.assert_array_equal(before[:,0],after[:,0])
    aligned=np.genfromtxt(ROOT/'all-271-aligned-states.csv',delimiter=',',skip_header=1)
    assert aligned.shape==(271,9)
    np.testing.assert_allclose(np.sqrt(np.mean(aligned[:,-2]**2)),assessment['trajectory']['position_rmse_m'],rtol=1e-10)
    np.testing.assert_allclose(np.sqrt(np.mean(aligned[:,-1]**2)),assessment['trajectory']['rotation_rmse_rad'],rtol=1e-10)
    assert not assessment['map_accepted'] and not source['scene_and_truth_used_in_inference']
    print('PASS: explicit SDF initial-bias certificate, unchanged noise/clock, 24D correlated covariance and 271 assessment rows; no map acceptance')


if __name__=='__main__':main()
