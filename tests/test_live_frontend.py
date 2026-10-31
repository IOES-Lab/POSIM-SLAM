import json
from pathlib import Path
import tempfile
import unittest
from posim_slam.live_client import LiveClient


class FrontendContractTests(unittest.TestCase):
    def test_invalid_frontends_fail_before_process_start(self):
        for value in (None, True, 1, [], {}, '', 'unregistered'):
            with self.subTest(value=value), tempfile.TemporaryDirectory() as folder:
                path = Path(folder) / 'calibration.json'
                path.write_text(json.dumps({'optical_flow_type': value}))
                with self.assertRaisesRegex(ValueError, 'invalid_optical_flow_type'):
                    LiveClient('/missing-binary-must-not-start', path, None)

    def test_explicit_frontend_requires_native_acknowledgement(self):
        client = LiveClient.__new__(LiveClient)
        for requested in ('frame_to_frame', 'patch'):
            client.processing_request = {'optical_flow_type': requested}
            client._check_processing_ready({'optical_flow_type': requested})
            for actual in (None, 'other'):
                with self.assertRaisesRegex(ValueError, 'worker_optical_flow_type'):
                    client._check_processing_ready({'optical_flow_type': actual})

    def test_unrequested_legacy_frontend_keeps_its_transport_contract(self):
        client = LiveClient.__new__(LiveClient)
        client.processing_request = {}
        client._check_processing_ready({})


if __name__ == '__main__':
    unittest.main()
