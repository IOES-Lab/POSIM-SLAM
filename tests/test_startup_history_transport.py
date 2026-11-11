import json
import struct
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch
from posim_slam.live_client import LiveClient


class StartupHistoryTransportTests(unittest.TestCase):
    def client(self):
        client=object.__new__(LiveClient)
        client.processing_request={'retain_startup_measurements':True}
        client.retained_startup_transport=True;client.ping_transport=True
        client.packets=[];client.send=lambda kind,payload:client.packets.append((kind,payload))
        return client

    def test_original_result_request_contains_no_image_or_regenerated_tracks(self):
        client=self.client();client.retained_image(12)
        self.assertEqual(client.packets,[(16,struct.pack('<q',12))])
        client.retained_image(13,ping=(13,2.5))
        self.assertEqual(client.packets[-1],(16,struct.pack('<qqd',13,13,2.5)))

    def test_capability_is_explicit_and_default_is_unchanged(self):
        client=self.client()
        for capability in ({},{'retained_startup_transport':None},
                           {'retained_startup_transport':'other'}):
            with self.assertRaisesRegex(ValueError,'worker_startup_retention_unsupported'):
                client._check_processing_ready(capability)
        client._check_processing_ready({'retained_startup_transport':'original_frontend_once_v1'})
        for request in ({},{'retain_startup_measurements':False}):
            client.processing_request=request;client._check_processing_ready({})
            with self.assertRaisesRegex(ValueError,'not_requested'):client.retained_image(12)
        client.processing_request={'retain_startup_measurements':True};client.retained_startup_transport=False
        with self.assertRaisesRegex(ValueError,'unsupported'):client.retained_image(12)

    def test_timestamp_and_exact_ping_checks_are_shared_with_fresh_images(self):
        client=self.client()
        for stamp in (True,0,-1,2**63,1.5):
            with self.assertRaisesRegex(ValueError,'invalid_retained_image_stamp'):client.retained_image(stamp)
        for ping in ((13,2.5),(12,0),(12,float('nan')),(12,True)):
            with self.assertRaisesRegex(ValueError,'invalid_exact_image_ping'):client.retained_image(12,ping=ping)
        client.ping_transport=False
        with self.assertRaisesRegex(ValueError,'worker_image_ping_unsupported'):
            client.retained_image(12,ping=(12,2.5))
        self.assertEqual(client.packets,[])

    def test_non_boolean_is_refused_before_starting_a_process(self):
        with TemporaryDirectory() as directory,patch('subprocess.Popen') as process:
            calibration=Path(directory)/'calibration.json'
            for value in (1,'true',None):
                calibration.write_text(json.dumps({'retain_startup_measurements':value}))
                with self.assertRaisesRegex(ValueError,'invalid_startup_retention_request'):
                    LiveClient('unused',calibration,None)
            process.assert_not_called()


if __name__=='__main__':unittest.main()
