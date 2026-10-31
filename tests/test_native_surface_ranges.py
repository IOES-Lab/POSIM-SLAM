import copy
import unittest

import numpy as np

from posim_slam.native_surface import NativeSurfaceSnapshot
from posim_slam.native_surface_ranges import prepare_native_surface_ranges,linearize_native_surface_ranges
from posim_slam.range_surface_joint import linearize_surface_ranges,normal_basis
from test_range_surface_joint import supported_fixture


def fixture():
    args=supported_fixture();variables=[]
    for stamp,state in sorted(args['states'].items()):
        variables.append(dict(kind='state',ident=stamp,size=6,
            reference=[*state['position'],*np.eye(3).ravel()]))
    variables.append(dict(kind='pressure',ident=0,size=1,reference=[0.]))
    packet=dict(schema='posim-slam-native-surface-snapshot-1',snapshot_id=1,owner_revision=2,prior_revision=3,
        epoch='joint-native-ranges',stamp_ns=5,variables=variables,delta=np.zeros(31).tolist(),conditionals=[],
        archive_closed_through_ns=0,protected_states=[],range_ledger=dict(recent=[],closed_through_ns=0))
    plane=args['planes'][0]
    hypotheses={0:dict(initial_normal=plane['normal'],initial_distance=plane['distance'],
        **{k:plane[k] for k in ('support_keys','max_plane_deviation','max_support_distance')})}
    request=dict(planes=hypotheses,observations=args['observations'],measurement_covariance=args['measurement_covariance'],
        uncertainty_source=args['uncertainty_source'])
    return packet,request,args


class NativeSurfaceRangeTests(unittest.TestCase):
    def test_raw_joint_rows_match_direct_information_with_free_unknown_plane(self):
        packet,request,args=fixture();snapshot=NativeSurfaceSnapshot.read(packet)
        result=prepare_native_surface_ranges(snapshot,**request)
        direct=linearize_surface_ranges(**args);expected=np.zeros((len(direct['residual']),34))
        expected[:,:30]=direct['jacobian'][:,:30];expected[:,31:]=direct['jacobian'][:,30:]
        actual=result.increment.new_live_rows.jacobian;offset=result.increment.new_live_rows.offset
        np.testing.assert_allclose(actual.T@actual,expected.T@expected,atol=1e-9)
        np.testing.assert_allclose(actual.T@offset,expected.T@direct['residual'],atol=1e-9)
        self.assertEqual(result.increment.original_keys,tuple(direct['factor_keys']))
        self.assertEqual(len(result.increment.original_keys),2)
        self.assertEqual(len(result.diagnostics['source_keys']),6)
        self.assertFalse(result.diagnostics['physical_identity_verified'])
        self.assertFalse(result.diagnostics['poses_are_independent_observations'])
        self.assertNotIn('prior',result.increment.as_packet())

    def test_global_translation_and_yaw_do_not_become_absolute_measurements(self):
        packet,request,_=fixture();result=prepare_native_surface_ranges(NativeSurfaceSnapshot.read(packet),**request)
        j=result.increment.new_live_rows.jacobian;n=request['planes'][0]['initial_normal']
        for axis in np.eye(3):
            direction=np.zeros(34)
            for i in range(5):direction[6*i:6*i+3]=axis
            direction[-1]=n@axis
            np.testing.assert_allclose(j@direction,0,atol=1e-11)
        yaw=np.zeros(34);axis=np.array([0,0,1.])
        for i,v in enumerate(packet['variables'][:5]):
            yaw[6*i:6*i+3]=np.cross(axis,v['reference'][:3]);yaw[6*i+3:6*i+6]=axis
        yaw[-3:-1]=normal_basis(n).T@axis
        np.testing.assert_allclose(j@yaw,0,atol=1e-11)

    def test_unknown_noise_duplicate_keys_and_expired_poses_do_not_produce_updates(self):
        packet,request,_=fixture();snapshot=NativeSurfaceSnapshot.read(packet)
        with self.assertRaises(ValueError):prepare_native_surface_ranges(snapshot,**dict(request,measurement_covariance=None))
        duplicate=copy.deepcopy(request);duplicate['observations'].append(duplicate['observations'][0])
        with self.assertRaises(ValueError):prepare_native_surface_ranges(snapshot,**duplicate)
        packet['range_ledger']['recent']=[[1,0,0]]
        with self.assertRaisesRegex(ValueError,'reused'):
            prepare_native_surface_ranges(NativeSurfaceSnapshot.read(packet),**request)
        packet['archive_closed_through_ns']=2
        with self.assertRaisesRegex(ValueError,'admissible'):
            prepare_native_surface_ranges(NativeSurfaceSnapshot.read(packet),**request)

    def test_existing_plane_coordinates_come_from_the_owner(self):
        packet,request,args=fixture();normal=request['planes'][0].pop('initial_normal')
        distance=request['planes'][0].pop('initial_distance')
        packet['variables'].append(dict(kind='plane',ident=0,size=3,reference=[*normal,distance]))
        packet['delta'] += [.02,-.03,.07]
        request['require_support']=False;args['require_support']=False
        args['planes'][0].pop('normal');args['planes'][0].update(normal_reference=normal,normal_delta=[.02,-.03],distance=distance+.07)
        result=prepare_native_surface_ranges(NativeSurfaceSnapshot.read(packet),**request)
        np.testing.assert_allclose(result.diagnostics['predicted_ranges'],linearize_surface_ranges(**args)['predicted_ranges'],atol=1e-12)
        self.assertEqual(result.increment.as_packet()['new_planes'],[])
        request['planes'][0]['initial_distance']=999.
        with self.assertRaisesRegex(ValueError,'coordinates_are_owned'):
            prepare_native_surface_ranges(NativeSurfaceSnapshot.read(packet),**request)

    def test_nominal_support_is_rebuilt_from_owned_poses_and_original_cones(self):
        packet,request,_=fixture();before=prepare_native_surface_ranges(NativeSurfaceSnapshot.read(packet),**request)
        packet['variables'][1]['reference'][0]=4.
        with self.assertRaisesRegex(ValueError,'support|planar'):
            prepare_native_surface_ranges(NativeSurfaceSnapshot.read(packet),**request)
        self.assertEqual(len(before.diagnostics['support_only_indices']),4)
        self.assertNotIn('support_points',request['planes'][0])
        request['planes'][0]['support_points']=np.zeros((4,3))
        with self.assertRaisesRegex(ValueError,'unknown_native_surface_plane_fields'):
            prepare_native_surface_ranges(NativeSurfaceSnapshot.read(fixture()[0]),**request)

    def test_readonly_relinearization_never_reuses_a_consumed_likelihood(self):
        packet,request,_=fixture();packet['range_ledger']['recent']=[[1,0,0],[1,1,0]]
        snapshot=NativeSurfaceSnapshot.read(packet)
        result=linearize_native_surface_ranges(snapshot,**request)
        self.assertEqual(result.original_keys,((1,0,0),(1,1,0)))
        self.assertFalse(hasattr(result,'increment'))
        with self.assertRaisesRegex(ValueError,'reused'):prepare_native_surface_ranges(snapshot,**request)

    def test_unbounded_plane_modes_are_distinct_from_a_predictable_fan_range(self):
        packet,request,_=fixture();snapshot=NativeSurfaceSnapshot.read(packet)
        result=linearize_native_surface_ranges(snapshot,**request)
        info=result.diagnostics['conditional_plane_information'][0]
        self.assertEqual(info['cone_information']['rank'],1)
        self.assertEqual(info['cone_information']['unbounded_basis'].shape,(3,2))
        self.assertFalse(info['fan_predictions'][0]['unbounded_from_cone_rows'])
        self.assertGreater(info['fan_predictions'][0]['conditional_cone_only_stddev_m'],0)
        changed=copy.deepcopy(request);changed['require_support']=False
        # A laterally displaced fan needs a normal direction absent from the
        # one actual cone factor, despite four support-only nominal seeds.
        changed['observations'][1]['imu_to_sensor']=[.4,0,0]
        other=linearize_native_surface_ranges(snapshot,**changed).diagnostics['conditional_plane_information'][0]
        self.assertTrue(other['fan_predictions'][0]['unbounded_from_cone_rows'])
        self.assertIsNone(other['fan_predictions'][0]['conditional_cone_only_stddev_m'])
        self.assertEqual(other['cone_information']['rank'],1)

    def test_new_plane_uses_nonlinear_rows_to_initialize_without_an_extra_prior(self):
        packet,request,_=fixture();snapshot=NativeSurfaceSnapshot.read(packet)
        request['planes'][0]['initial_distance']=3.03
        result=prepare_native_surface_ranges(snapshot,**request)
        initialization=result.diagnostics['initialization']
        self.assertLess(initialization['final_cost'],initialization['initial_cost']*1e-8)
        self.assertFalse(initialization['added_prior'])
        self.assertGreater(initialization['iterations'],0)
        self.assertAlmostEqual(result.increment.as_packet()['new_planes'][0]['reference'][3],3.,places=6)
        self.assertEqual(snapshot.ledger.recent,frozenset())
        self.assertEqual(request['planes'][0]['initial_distance'],3.03)

    def test_inconsistent_same_plane_candidate_is_rejected_whole_before_owner_install(self):
        packet,request,_=fixture();snapshot=NativeSurfaceSnapshot.read(packet)
        request['observations'][1]['measured_range']=2.5
        original=copy.deepcopy(request)
        with self.assertRaisesRegex(ValueError,'support|planar|nonlinear_seed'):
            prepare_native_surface_ranges(snapshot,**request)
        self.assertEqual(len(request['observations']),6)
        self.assertEqual(snapshot.ledger.recent,frozenset())
        self.assertEqual(request['observations'][1]['measured_range'],original['observations'][1]['measured_range'])


if __name__=='__main__':unittest.main()
