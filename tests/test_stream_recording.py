import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

TOOLS=Path(__file__).resolve().parents[1]/'tools'
def module(name):
    spec=importlib.util.spec_from_file_location(name,TOOLS/(name+'.py'))
    value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value);return value
stream=module('stream_recording');replay=module('replay_vio')

class RecordingTests(unittest.TestCase):
    def test_seed_uses_history_and_retains_all_subsequent_images(self):
        imu=[dict(stamp_ns=str(1_000_000_000+i*10_000_000),gx='0',gy='0',gz='0',ax='0',ay='0',az='9.80665') for i in range(201)]
        frames=[dict(stamp_ns=1_000_000_000+i*100_000_000) for i in range(21)]
        selected,samples,seed=stream.seeded_inputs(frames,imu)
        self.assertEqual(seed,(2_000_000_000,[0,0,1]))
        self.assertEqual(selected,frames[10:])
        self.assertEqual(int(samples[0]['stamp_ns']),1_800_000_000)
        with self.assertRaisesRegex(ValueError,'seed_history_missing'):stream.seeded_inputs(frames[:5],imu)
        chosen,_,later=stream.seeded_inputs(frames,imu,2_300_000_000)
        self.assertEqual(chosen,frames[13:]);self.assertEqual(later[0],2_300_000_000)
        for stamp in (1_500_000_000,2_300_000_001):
            with self.assertRaisesRegex(ValueError,'requested_seed_image_not_covered'):
                stream.seeded_inputs(frames,imu,stamp)
        broken=imu[:30]+imu[40:]
        with self.assertRaisesRegex(ValueError,'imu_time_gap'):stream.seeded_inputs(frames,broken)

    def test_ping_never_associates_nearby_timestamp(self):
        rows=[dict(stamp_ns='100',valid='1',range_m='2'),dict(stamp_ns='200',valid='0',range_m='inf')]
        pings=stream.exact_pings(rows)
        self.assertEqual(pings,{100:(100,2.)})
        self.assertIsNone(pings.get(101))
        with self.assertRaisesRegex(ValueError,'invalid_recorded_ping'):stream.exact_pings(rows+rows[:1])

    def test_optional_ping_inventory_is_complete_and_hash_checked(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder).resolve();(root/'image').write_bytes(b'image')
            image_hash=hashlib.sha256(b'image').hexdigest()
            (root/'frames.json').write_text(json.dumps([dict(file='image',sha256=image_hash)]*5))
            for name in ['calibration.json','imu.csv','ping.csv','ping-calibration.json']:(root/name).write_text('{}')
            hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in root.iterdir() if p.name!='image'}
            receipt=dict(passed=True,input_files=hashes)
            self.assertEqual(replay.verify(root,receipt),5)
            (root/'ping.csv').write_text('changed')
            with self.assertRaisesRegex(ValueError,'recording_input_changed:ping.csv'):replay.verify(root,receipt)
            del hashes['ping.csv']
            with self.assertRaisesRegex(ValueError,'inventory_missing'):replay.verify(root,receipt)

if __name__=='__main__':unittest.main()
