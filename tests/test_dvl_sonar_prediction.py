"""Independent positive/negative and measurement-leakage controls for the CLI."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

TOOL=Path(__file__).resolve().parents[1]/'tools/evaluate_dvl_sonar_prediction.py'


class DvlSonarPredictionTests(unittest.TestCase):
    def setUp(self):
        self.temporary=tempfile.TemporaryDirectory();self.addCleanup(self.temporary.cleanup)
        self.root=Path(self.temporary.name);self.inputs=self.root/'inputs';self.inputs.mkdir()
        self.ranges=np.linspace(.2,6.,233);angles=np.linspace(-np.pi/3,np.pi/3,32)
        self.directions=np.column_stack((np.cos(angles),np.sin(angles),np.zeros(32)))
        self.dump(self.inputs/'timeline.json',dict(sonar=[dict(stamp_ns=2_000_000_000,file='frame.npz')]))
        (self.inputs/'sensor-model.sdf').write_text('''<sdf xmlns:gz="http://gazebosim.org"><model><link><sensor>
          <gz:multibeam_sonar><ray><scan><vertical><rays>256</rays>
          <min_angle>-0.17453292519943295</min_angle><max_angle>0.17453292519943295</max_angle>
          </vertical></scan></ray></gz:multibeam_sonar></sensor></link></model></sdf>''')
        rotation=[[np.cos(.35),0,np.sin(.35)],[0,1,0],[-np.sin(.35),0,np.cos(.35)]]
        self.dump(self.root/'calibration.json',dict(sonar=dict(imu_to_sonar=[0,0,0],sonar_to_imu=rotation,
            vertical_fov=np.pi/9,horizontal_fov=2*np.pi/3,range_min=.2,range_max=6.)))
        (self.root/'states.csv').write_text('stamp_ns,x,y,z,qx,qy,qz,qw\n2000000000,0,0,0,0,0,0,1\n')
        self.dump(self.root/'receipt.json',dict(sonar_factors_omitted=True,camera_frames=0,sonar_track_updates_supplied=0))
        self.surface(-1.)

    def dump(self,path,value):path.write_text(json.dumps(value))

    def surface(self,height):
        support=[[x,y,height] for x in np.arange(.6,3.7,.2) for y in np.arange(-2.8,2.9,.2)]
        self.dump(self.root/'spatial.json',dict(truth_is_estimator_input=False,online_state_updated=False,
            config=dict(surface_sigma_m=.05,neighborhood_m=.75),rows=[dict(stamp_ns=2_000_000_000,
            beams=[dict(source_id=1,association_rejection=None,plane_patch=dict(normal=[0,0,1],distance=height,
                support_points=support,support_stamps_ns=[1_000_000_000]*len(support)))])]))

    def run_prediction(self,onset=2.8,success=True):
        np.savez_compressed(self.inputs/'frame.npz',ranges=self.ranges,beam_directions=self.directions,
            intensity_db=np.repeat(np.where(self.ranges<onset,0.,30.)[:,None],32,axis=1),stamp=np.array([2,0]))
        output=self.root/'result.json'
        command=[sys.executable,str(TOOL),'--record',str(self.inputs),'--calibration',str(self.root/'calibration.json'),
            '--states',str(self.root/'states.csv'),'--replay-receipt',str(self.root/'receipt.json'),
            '--spatial-report',str(self.root/'spatial.json'),'--output',str(output)]
        result=subprocess.run(command,capture_output=True,text=True,timeout=30)
        if not success:
            self.assertNotEqual(result.returncode,0)
            return result
        self.assertEqual(result.returncode,0,result.stderr)
        return json.loads(output.read_text())

    def test_changing_onsets_does_not_change_predictions_or_support(self):
        first=self.run_prediction(2.8);later=self.run_prediction(3.6)
        self.assertGreater(first['selected_predictions'],20)
        key=lambda report:[(r['beam_id'],r['dvl_stamp_ns'],r['predicted_range_m']) for r in report['rows']]
        self.assertEqual(key(first),key(later))
        self.assertTrue(all(b['observed_range_m']-a['observed_range_m']>.7
            for a,b in zip(first['rows'],later['rows'])))
        self.assertGreater(later['absolute_residual_m']['maximum'],1.)
        self.assertFalse(later['onset_range_used_for_prediction_or_support'])
        self.assertFalse(later['map_qualified'])

    def test_out_of_range_plane_does_not_manufacture_a_prediction(self):
        self.surface(-8.);result=self.run_prediction()
        self.assertEqual(result['selected_predictions'],0)
        self.assertIsNone(result['absolute_residual_m'])

    def test_circular_sonar_conditioned_poses_are_rejected(self):
        self.dump(self.root/'receipt.json',dict(sonar_factors_omitted=False,camera_frames=0,sonar_track_updates_supplied=0))
        result=self.run_prediction(success=False)
        self.assertIn('independent_camera_free_no_acoustic_poses_required',result.stderr)


if __name__=='__main__':unittest.main()
