"""Real owner-thread control, expiry and file transport in the native worker.

Synthetic delayed rows test estimator ownership and transport. They do not
establish sonar correspondence, physical map accuracy or product acceptance.
"""
import json
import os
from pathlib import Path
import tempfile
import unittest

import numpy as np

from posim_slam.native_surface import prepare_native_ranges
from posim_slam.surface_prior import Variable
from posim_slam.imu_bias_calibration import InitialImuBiasCalibration
import test_sonar_clock_native as fixture


@unittest.skipUnless(os.environ.get('POSIM_SLAM_STREAM_BINARY'), 'Native worker binary required')
class NativeSurfaceStreamTests(unittest.TestCase):
    receive=fixture.SonarClockTests.receive
    initial=1_000_000_000
    metrics={}

    @classmethod
    def tearDownClass(cls):
        print(json.dumps(dict(scope='Synthetic native stream ownership; no physical sonar map', **cls.metrics), sort_keys=True))

    def worker(self, root, enabled=True, **extra):
        (root/'pressure-calibration.json').write_text(json.dumps(dict(
            schema='posim-slam-pressure-calibration-1', imu_to_pressure=[0, 0, 0])))
        settings=dict(enable_joint_pressure=True)
        settings.update(extra)
        if enabled:settings.update(enable_surface_owner=True, surface_owner_epoch='stream-owner-control')
        client=fixture.SonarClockTests.worker(self, root, **settings)
        self.assertEqual(client.surface_transport, enabled)
        client.gravity_seed(self.initial, [0, 0, 1])
        return client

    def advance(self, client, count=28, start=0):
        result=[]
        for index in range(start, start+count):
            stamp=self.initial+index*200_000_000
            first=stamp if index==0 else stamp-180_000_000
            for t in range(first, stamp+10_000_001, 10_000_000):
                client.imu(t, [0, 0, 0], [0, 0, 9.80665])
            client.pressure(stamp, -2., .01)
            client.sonar_tick(stamp)
            value=self.receive(client, 'state')
            self.assertEqual(value['stamp_ns'], stamp)
            self.assertFalse(value['map_qualified'])
            result.append(value)
        return result

    def capture(self, client, request):
        client.surface_snapshot(request)
        reply=self.receive(client, 'surface_control')
        self.assertTrue(reply['accepted'], reply)
        self.assertEqual(reply['request_id'], request)
        return client.read_surface_snapshot(reply), reply

    def raw_update(self, client, request, packet):
        """Bypass the Python preparation checks to exercise the native boundary."""
        client._surface_request(request)
        path=client.surface_directory/('worker-surface-update-'+str(request)+'.json')
        path.write_text(json.dumps(packet))
        client.send(15, json.dumps(dict(op='update', request_id=request,
            path=str(path), bytes=path.stat().st_size)).encode())
        result=self.receive(client, 'surface_control');path.unlink()
        return result

    def increment(self, snapshot, *, beam=1, stamp=None):
        stamp=self.initial if stamp is None else stamp
        reference=snapshot.reference('state', stamp)
        pose=Variable('state', stamp, 6, reference.reference[:12])
        plane=next((v for v in snapshot.order if v.kind=='plane'), Variable('plane', 0, 3, (0, 0, 1, 3)))
        current=np.r_[snapshot.recover([stamp])['deltas'], np.zeros(3)]
        for v in snapshot.order:
            if v.kind=='plane':current[-3:]=[snapshot._archive.root[('plane', v.ident, i)] for i in range(3)]
        j=np.zeros((1, 9));j[0, 2]=10.;j[0, 8]=-10.
        return prepare_native_ranges(snapshot, order=(pose, plane), delta=current,
            jacobian=j, residual=[.03], range_keys=[(stamp, 4, beam)])

    def test_owner_adoption_preserves_the_no_range_trajectory_and_control_has_no_clock(self):
        traces=[]
        for enabled in (False, True):
            with self.subTest(enabled=enabled), tempfile.TemporaryDirectory() as folder:
                client=self.worker(Path(folder), enabled)
                values=self.advance(client)
                traces.append([[*v['position'], *v['orientation_xyzw'], *v['velocity']] for v in values])
                if enabled:
                    snap, reply=self.capture(client, 1)
                    self.assertEqual(snap.stamp_ns, values[-1]['stamp_ns'])
                    self.assertGreater(reply['conditional_nodes'], 0)
                    self.assertGreater(reply['file']['bytes'], 256*1024)
                    self.metrics['snapshot_bytes']=reply['file']['bytes']
                    self.metrics['conditional_nodes_after_28_states']=reply['conditional_nodes']
                    directory=client.surface_directory
                    client.surface_release(2)
                    released=self.receive(client, 'surface_control')
                    self.assertTrue(released['accepted'], released)
                    self.assertEqual(list(directory.iterdir()), [])
                    following=self.advance(client, 1, 28)[0]
                    self.assertEqual(following['stamp_ns'], snap.stamp_ns+200_000_000)
                    self.assertEqual(client.poll(), [])
                    client.close();self.assertFalse(directory.exists())
                else:client.close()
        np.testing.assert_allclose(traces[0], traces[1], atol=1e-9, rtol=1e-9)
        self.metrics['no_range_max_trajectory_difference']=float(np.max(np.abs(np.asarray(traces[0])-traces[1])))

    def test_snapshot_information_includes_live_uncertainty_without_changing_the_owner(self):
        with tempfile.TemporaryDirectory() as folder:
            client=self.worker(Path(folder));self.addCleanup(client.close)
            self.advance(client)
            first,_=self.capture(client,1);second,_=self.capture(client,2)
            self.assertEqual((first.owner_revision,first.prior_revision,first.stamp_ns),
                             (second.owner_revision,second.prior_revision,second.stamp_ns))
            np.testing.assert_array_equal(first._information.jacobian,second._information.jacobian)
            np.testing.assert_array_equal(first._information.residual,second._information.residual)
            self.assertEqual(first._archive.root,second._archive.root)
            reference=first.reference('state',self.initial)
            query=(Variable('state',self.initial,6,reference.reference[:12]),first.order[-1])
            got=first.joint_marginal(query)
            conditioned=first.recover([self.initial])['observable_conditional_covariance']
            added=got['observable_covariance'][:6,:6]-conditioned
            self.assertGreater(np.linalg.norm(added),1e-7)
            self.assertGreaterEqual(np.linalg.eigvalsh(added).min(),-1e-7)
            self.assertGreater(got['active_native_counts']['imu_intervals'],0)
            self.assertGreater(got['active_native_counts']['pressure_observations'],0)
            self.assertFalse(got['live_coordinates_treated_as_known']);self.assertFalse(got['map_qualified'])
            self.assertEqual(client.poll(),[])

    def test_initial_calibration_replaces_generic_bias_prior_and_survives_retirement(self):
        for diagonal in ([0.,0.,0.,0.,0.,0.],[0.,.001,.002,0.,.01,.02]):
            with self.subTest(diagonal=diagonal),tempfile.TemporaryDirectory() as folder:
                covariance=np.diag(diagonal);calibration=InitialImuBiasCalibration(np.zeros(6),covariance,'stream-calibration-control')
                client=self.worker(Path(folder),initial_imu_bias_calibration=calibration.packet())
                self.advance(client,1)
                first,_=self.capture(client,1);state=next(v for v in first.order if v.kind=='state')
                result=first.joint_marginal((state,));self.assertEqual(state.size,15)
                np.testing.assert_allclose(result['observable_covariance'][9:15,9:15],covariance,atol=2e-10)
                self.assertEqual(result['root_constraint_rank'],sum(v==0 for v in diagonal))
                self.advance(client,28,1)
                after,_=self.capture(client,2);old=after.reference('state',self.initial)
                result=after.joint_marginal((old,));hard=calibration.rows()['constraint_jacobian']
                np.testing.assert_allclose(hard@result['observable_covariance'][9:15,9:15],0,atol=2e-9)
                np.testing.assert_allclose(hard@result['deltas'][9:15],0,atol=2e-9)
                self.assertTrue(any(n.deterministic.shape[1] for n in after._archive.nodes.values()))
                client.close()

    def test_nonzero_exact_calibration_is_in_the_raw_imu_frame(self):
        with tempfile.TemporaryDirectory() as folder:
            bg=np.array([.01,-.02,.03]);ba=np.array([.1,-.05,.2])
            calibration=InitialImuBiasCalibration(np.r_[bg,ba],np.zeros((6,6)),'nonzero-calibration-control')
            client=self.worker(Path(folder),initial_imu_bias_calibration=calibration.packet());next_imu=self.initial
            for index in range(29):
                stamp=self.initial+index*200_000_000
                while next_imu<=stamp+10_000_000:
                    client.imu(next_imu,bg,np.array([0.,0.,9.80665])+ba);next_imu+=10_000_000
                client.pressure(stamp,-2.,.01);client.sonar_tick(stamp);state=self.receive(client,'state')
                np.testing.assert_allclose(state['position'],[0,0,0],atol=1e-7)
                np.testing.assert_allclose(state['initialization_window']['gyro_bias'],bg,atol=1e-7)
                np.testing.assert_allclose(state['initialization_window']['accel_bias'],ba,atol=1e-7)
            after,_=self.capture(client,1);reference=after.reference('state',self.initial)
            np.testing.assert_allclose(reference.reference[15:18],bg,atol=1e-12)
            np.testing.assert_allclose(reference.reference[18:21],ba,atol=1e-12)
            client.close()

    def test_successive_delayed_updates_use_fresh_native_snapshots_and_retire(self):
        with tempfile.TemporaryDirectory() as folder:
            client=self.worker(Path(folder));self.advance(client)
            snap, first=self.capture(client, 1)
            self.assertNotIn(self.initial, [v.ident for v in snap.order if v.kind=='state'])
            update=self.increment(snap)
            self.assertTrue(update.as_packet()['conditionals'])
            client.surface_update(2, update)
            accepted=self.receive(client, 'surface_control')
            self.assertTrue(accepted['accepted'], accepted);self.assertTrue(accepted['optimized'], accepted)
            self.assertEqual(accepted['ledger_entries'], 1)
            self.assertEqual(list(client.surface_directory.iterdir()), [])
            self.advance(client, 3, 28)
            fresh, _=self.capture(client, 3)
            self.assertGreater(fresh.prior_revision, snap.prior_revision)
            self.assertEqual(len(fresh.ledger.recent), 1)
            self.assertEqual(sum(v.kind=='plane' for v in fresh.order), 1)
            client.surface_update(4, self.increment(fresh, beam=2))
            again=self.receive(client, 'surface_control')
            self.assertTrue(again['accepted'], again);self.assertTrue(again['optimized'], again)
            self.assertEqual(again['ledger_entries'], 2)
            self.assertGreater(again['conditional_nodes'], first['conditional_nodes'])
            self.metrics['successive_installed_readings']=again['ledger_entries']

    def test_stale_update_is_rejected_without_losing_sensor_owner(self):
        with tempfile.TemporaryDirectory() as folder:
            client=self.worker(Path(folder));self.advance(client)
            snap, _=self.capture(client, 1);update=self.increment(snap)
            self.advance(client, 1, 28)
            client.surface_update(2, update)
            reply=self.receive(client, 'surface_control')
            self.assertFalse(reply['accepted']);self.assertRegex(reply['reason'], 'snapshot')
            self.assertEqual(client._surface_files, {})
            fresh, _=self.capture(client, 3)
            self.assertEqual(len(fresh.ledger.recent), 0)
            client.surface_update(4, self.increment(fresh))
            reply=self.receive(client, 'surface_control')
            self.assertTrue(reply['accepted'], reply)
            self.advance(client, 1, 29)

    def test_native_independently_rejects_partial_closure_and_reused_readings(self):
        with tempfile.TemporaryDirectory() as folder:
            client=self.worker(Path(folder));self.advance(client)
            snap, _=self.capture(client, 1);update=self.increment(snap)
            partial=update.as_packet();partial['conditionals']=[]
            reply=self.raw_update(client, 2, partial)
            self.assertFalse(reply['accepted']);self.assertIn('closure', reply['reason'])
            client.surface_update(3, update)
            reply=self.receive(client, 'surface_control');self.assertTrue(reply['accepted'], reply)
            fresh, _=self.capture(client, 4)
            duplicate=self.increment(fresh, beam=2).as_packet()
            duplicate['range_keys']=[[self.initial, 4, 1]]
            reply=self.raw_update(client, 5, duplicate)
            self.assertFalse(reply['accepted']);self.assertIn('reused', reply['reason'])
            after, _=self.capture(client, 6)
            self.assertEqual(after.prior_revision, fresh.prior_revision)
            self.assertEqual(after.ledger, fresh.ledger)
            self.advance(client, 1, 28)

    def test_expiry_keeps_requested_dependency_closure_and_closes_range_reuse(self):
        with tempfile.TemporaryDirectory() as folder:
            client=self.worker(Path(folder));self.advance(client)
            old, _=self.capture(client, 1)
            client.surface_expire(2, old.stamp_ns, protect=[self.initial])
            reply=self.receive(client, 'surface_control');self.assertTrue(reply['accepted'], reply)
            protected, _=self.capture(client, 3)
            self.assertTrue(np.isfinite(protected.recover([self.initial])['deltas']).all())
            self.assertEqual(protected.ledger.closed_through_ns, old.stamp_ns)
            with self.assertRaisesRegex(ValueError, 'closed'):
                self.increment(protected)
            client.surface_expire(4, old.stamp_ns)
            reply=self.receive(client, 'surface_control');self.assertTrue(reply['accepted'], reply)
            self.assertEqual(reply['conditional_nodes'], 0)
            expired, _=self.capture(client, 5)
            with self.assertRaises(ValueError):expired.recover([self.initial])
            client.surface_expire(6, old.stamp_ns-1)
            reply=self.receive(client, 'surface_control')
            self.assertFalse(reply['accepted']);self.assertIn('monotonic', reply['reason'])
            self.advance(client, 2, 28)


if __name__=='__main__':unittest.main()
