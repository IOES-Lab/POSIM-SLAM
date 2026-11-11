import unittest
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch
from posim_slam.live_client import LiveClient


class NativeStateInformationTransportTests(unittest.TestCase):
    def test_requested_report_requires_explicit_worker_acknowledgement(self):
        client=object.__new__(LiveClient)
        client.processing_request={'record_current_state_information':True}
        for report in ({},{'current_state_information_transport':None},
                       {'current_state_information_transport':'other'}):
            with self.assertRaisesRegex(ValueError,'worker_current_state_information_unsupported'):
                client._check_processing_ready(report)
        client._check_processing_ready({'current_state_information_transport':'undamped_joint_selected_state_v1'})

    def test_default_does_not_require_the_optional_report(self):
        client=object.__new__(LiveClient)
        for request in ({},{'record_current_state_information':False}):
            client.processing_request=request
            client._check_processing_ready({})

    def test_non_boolean_request_is_refused_before_process_creation(self):
        with TemporaryDirectory() as directory,patch('subprocess.Popen') as process:
            calibration=Path(directory)/'calibration.json'
            for value in (1,'true',None):
                calibration.write_text(json.dumps({'record_current_state_information':value}))
                with self.assertRaisesRegex(ValueError,'invalid_current_state_information_request'):
                    LiveClient('unused',calibration,None)
            process.assert_not_called()


if __name__=='__main__':unittest.main()
