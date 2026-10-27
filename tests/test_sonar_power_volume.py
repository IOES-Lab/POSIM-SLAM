"""Mathematical/ownership checks for the isolated reconstruction diagnostic."""
from pathlib import Path
import sys,tempfile,unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from sonar_power_volume import Projector,analytic_probe,load_packet,reconstruction,solve,write_cells


class PowerVolumeTests(unittest.TestCase):
    def packet(self):
        raw=np.arange(48)*.1;coarse=raw.reshape(-1,4).mean(1)
        poses=[dict(position=np.array([x,0,z]),rotation=np.eye(3)) for x,z in [(0,0),(.25,-.1),(.5,.1),(.75,0)]]
        az=np.linspace(-.3,.3,13);point=np.array([[2.,0.,-.15]])
        model=Projector(point,poses,az,coarse,.5,native_ranges=raw,maximum_range=4.7)
        return dict(positions=np.array([p['position'] for p in poses]),rotations=np.array([p['rotation'] for p in poses]),stamps=np.arange(1,5,dtype=np.int64),observed=model.forward(np.array([2.])),train=np.array([0,2]),test=np.array([1,3]),azimuth=az,ranges=coarse,native_ranges=raw,vertical_fov=np.array(.5),maximum_range=np.array(4.7),beam_pixel_width=np.array(az[1]-az[0]),range_power=np.array(3.),frame_id=np.array('test/estimated_segment_0'))

    def test_analytic_operator_contracts(self):
        self.assertTrue(all(analytic_probe().values()))

    def test_physical_projection_is_invariant_to_observation_batch_membership(self):
        packet=self.packet();poses=[dict(position=p,rotation=q) for p,q in zip(packet['positions'],packet['rotations'])]
        points=np.array([[2.,0.,-.15],[2.2,.1,-.2],[2.5,-.15,-.4]])
        args=(points,poses,packet['azimuth'],packet['ranges'],.5)
        whole=Projector(*args,native_ranges=packet['native_ranges'])
        training=Projector(points,poses[:2],*args[2:],native_ranges=packet['native_ranges'])
        testing=Projector(points,poses[2:],*args[2:],native_ranges=packet['native_ranges'])
        scattering=np.array([.4,.2,.7]);expected=whole.forward_scattering(scattering)
        np.testing.assert_allclose(training.forward_scattering(scattering),expected[:2],rtol=1e-12)
        np.testing.assert_allclose(testing.forward_scattering(scattering),expected[2:],rtol=1e-12)
        optimizer_coordinates=scattering/training.scale
        np.testing.assert_allclose(training.forward(optimizer_coordinates),expected[:2],rtol=1e-12)
        # Passing training-normalized coordinates directly to another projector
        # changes material strength. That previous usage violates this contract.
        self.assertGreater(np.linalg.norm(testing.forward(optimizer_coordinates)-expected[2:]),.1*np.linalg.norm(expected[2:]))
        rng=np.random.default_rng(22);y=rng.normal(size=testing.shape)
        np.testing.assert_allclose(np.sum(expected[2:]*y),scattering@testing.adjoint_scattering(y),rtol=1e-12)

    def test_exported_scattering_reproduces_the_reported_heldout_prediction(self):
        packet=self.packet();result=reconstruction(packet,cell=.3,refine=0,iterations=40)
        poses=[dict(position=packet['positions'][i],rotation=packet['rotations'][i]) for i in packet['test']]
        test=Projector(result['points'],poses,packet['azimuth'],packet['ranges'],float(packet['vertical_fov']),
                       native_ranges=packet['native_ranges'],maximum_range=float(packet['maximum_range']),
                       beam_pixel_width=float(packet['beam_pixel_width']))
        np.testing.assert_allclose(test.forward_scattering(result['albedo']),result['predicted'],rtol=1e-12)

    def test_fista_matches_independent_dense_nonnegative_solve(self):
        from scipy.optimize import nnls
        packet=self.packet();poses=[dict(position=p,rotation=q) for p,q in zip(packet['positions'],packet['rotations'])]
        points=np.array([[2.,0.,-.15],[2.2,.1,-.2],[2.5,-.15,-.4]])
        model=Projector(points,poses,packet['azimuth'],packet['ranges'],.5,native_ranges=packet['native_ranges'])
        dense=np.column_stack([model.forward(v).ravel() for v in np.eye(len(points))]);observed=model.forward(np.array([0.,2.,.3]))
        expected,_=nnls(dense,observed.ravel());actual,_=solve(model,observed,fraction=0.,iterations=200)
        np.testing.assert_allclose(actual,expected,atol=1e-5)

    def test_test_intensities_do_not_change_fit_or_refinement(self):
        packet=self.packet();first=reconstruction(packet,cell=.3,refine=1,iterations=40)
        second_packet={k:v.copy() for k,v in packet.items()};second_packet['observed'][packet['test']]*=100
        second=reconstruction(second_packet,cell=.3,refine=1,iterations=40)
        for key in ('points','coefficient','albedo','cell_m'):np.testing.assert_array_equal(first[key],second[key])
        self.assertNotEqual(first['stages'][-1]['heldout_relative_power_l2'],second['stages'][-1]['heldout_relative_power_l2'])

    def test_packet_rejects_unsigned_time_reversal_and_shared_split(self):
        packet=self.packet()
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'inputs.npz';np.savez(path,**packet);load_packet(path)
            packet['stamps']=np.array([4,3,2,1],dtype=np.uint64);np.savez(path,**packet)
            with self.assertRaises(ValueError):load_packet(path)
            packet=self.packet();packet['test']=np.array([0,1]);np.savez(path,**packet)
            with self.assertRaises(ValueError):load_packet(path)

    def test_zero_signal(self):
        packet=self.packet();packet['observed']*=0
        result=reconstruction(packet,cell=.3,refine=0,iterations=20)
        self.assertFalse(np.any(result['coefficient']))

    def test_unsupported_signal_has_empty_solution(self):
        poses=[dict(position=np.zeros(3),rotation=np.eye(3))]*2
        model=Projector(np.array([[-2.,0.,0.]]),poses,np.linspace(-.3,.3,13),np.arange(1,10)*.2,.5)
        result,report=solve(model,np.ones(model.shape))
        self.assertFalse(np.any(result));self.assertTrue(report['unsupported_signal'])

    def test_direct_api_rejects_invalid_calibration_before_grid(self):
        packet=self.packet();packet['maximum_range']=np.array(np.inf)
        with self.assertRaises(ValueError):reconstruction(packet)
        packet=self.packet();packet.pop('native_ranges')
        with self.assertRaisesRegex(ValueError,'native_range_bins_required'):reconstruction(packet)

    def test_fractional_range_response_matches_independent_dft(self):
        az=np.linspace(-.4,.4,13);native=np.arange(80)*.1;ranges=native.reshape(-1,4).mean(1)
        poses=[dict(position=np.zeros(3),rotation=np.eye(3))]*2
        f=np.arange(80);w=.54-.46*np.cos(2*np.pi*(f+1)/80);w/=np.linalg.norm(w)
        curvature=(2*np.pi/80)**2*2*(w.sum()*(w@(f*f))-(w@f)**2)/80
        for samples in (4,8,16):
            points=np.array([[3.15,0,0],[3.157,0,0]])
            model=Projector(points,poses,az,ranges,.5,native_ranges=native,fractional_samples=samples)
            dense=np.column_stack([model.forward(x).ravel() for x in np.eye(2)])
            np.testing.assert_allclose(np.sum(dense**2,axis=0),1.,atol=1e-12)
            rng=np.random.default_rng(3);x=rng.normal(size=2);y=rng.normal(size=model.shape)
            np.testing.assert_allclose(np.sum(model.forward(x)*y),x@model.adjoint(y),rtol=1e-12,atol=1e-12)
            for index,distance in enumerate((3.15,3.157)):
                expected=abs(np.fft.fft(w*np.exp(2j*np.pi*f*distance/.1/80)))**2/80
                expected=expected.reshape(-1,4).mean(1);predicted=model.forward(np.eye(2)[index])[0].sum(1)
                predicted/=np.linalg.norm(predicted);reference=expected/np.linalg.norm(expected)
                if index==0:np.testing.assert_allclose(predicted,reference,atol=1e-12)
                bound=curvature/(8*samples*samples)
                self.assertLessEqual(np.linalg.norm(predicted-reference),2*np.sqrt(len(ranges))*bound/np.linalg.norm(expected))

    def test_fractional_response_requires_a_calibrated_uniform_source_grid(self):
        packet=self.packet();poses=[dict(position=p,rotation=q) for p,q in zip(packet['positions'],packet['rotations'])]
        kwargs=dict(points=np.array([[2.,0,0]]),poses=poses,azimuth=packet['azimuth'],ranges=packet['ranges'],vertical_fov=.5,fractional_samples=8)
        with self.assertRaisesRegex(ValueError,'native_ranges_required'):Projector(**kwargs)
        irregular=packet['native_ranges'].copy();irregular[3]+=.001
        with self.assertRaisesRegex(ValueError,'source_uniform_range_grid_required'):Projector(**kwargs,native_ranges=irregular)

    def test_export_preserves_all_positive_cells_with_explicit_scope(self):
        result=dict(points=np.arange(12).reshape(4,3),coefficient=np.array([1.,0.,.01,2.]),albedo=np.array([2.,0.,.02,4.]),cell_m=np.full(4,.1))
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'cells.ply';write_cells(path,result,'test/segment_1',123)
            header,body=path.read_bytes().split(b'end_header\n',1)
            self.assertIn(b'comment scope experimental_sonar_scattering_cells',header)
            self.assertIn(b'comment map_qualified false',header);self.assertIn(b'element vertex 3',header)
            values=np.frombuffer(body,dtype='<f4').reshape(-1,6);np.testing.assert_array_equal(values[:,:3],result['points'][[0,2,3]])


if __name__=='__main__':unittest.main()
