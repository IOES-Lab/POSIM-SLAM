import io
import json
import queue
import threading
import unittest
from types import SimpleNamespace
from posim_slam.live_client import LiveClient


class OutputNotificationTests(unittest.TestCase):
    def client(self):
        client=object.__new__(LiveClient)
        client.output=queue.Queue(maxsize=8);client.output_notification=None
        client.closed=threading.Event();client.ready=threading.Event();client.ready.set()
        client.fault=None
        return client

    def test_reader_wakeup_keeps_the_original_packet_for_its_owner(self):
        client=self.client();packet=dict(type='state',stamp_ns=12,position=[1,2,3])
        client.process=SimpleNamespace(stdout=io.BytesIO((json.dumps(packet)+'\n').encode()))
        called=threading.Event();threads=[]
        def wake():
            threads.append(threading.get_ident());called.set();client.closed.set()
        client.set_output_notification(wake)
        reader=threading.Thread(target=client._read);reader.start();reader.join(1)
        self.assertFalse(reader.is_alive());self.assertTrue(called.is_set())
        self.assertNotEqual(threads[0],threading.get_ident())
        self.assertEqual(client.output.get_nowait(),packet);self.assertIsNone(client.fault)

    def test_installation_wakes_pending_output_and_removal_keeps_it_queued(self):
        client=self.client();packet=dict(type='map',stamp_ns=12)
        client.output.put(packet);calls=[]
        client.set_output_notification(lambda:calls.append(True))
        self.assertEqual(calls,[True]);self.assertIs(client.output.queue[0],packet)
        client.set_output_notification(None);client._notify_output()
        self.assertEqual(calls,[True]);self.assertIs(client.output.get_nowait(),packet)
        with self.assertRaisesRegex(ValueError,'invalid_worker_output_notification'):
            client.set_output_notification(True)

    def test_callback_failure_is_visible_without_silently_losing_its_packet(self):
        client=self.client();packet=dict(type='state',stamp_ns=12)
        client.process=SimpleNamespace(stdout=io.BytesIO((json.dumps(packet)+'\n').encode()))
        def broken():raise RuntimeError('event loop stopped')
        client.set_output_notification(broken);client._read()
        self.assertEqual(client.fault,'worker_output_notification_failed')
        self.assertEqual(client.output.get_nowait(),packet)

    def test_closed_reader_never_notifies_a_destroyed_event_loop(self):
        client=self.client();calls=[];client.closed.set()
        client.output.put(dict(type='state',stamp_ns=12))
        client.set_output_notification(lambda:calls.append(True));client._notify_output()
        self.assertEqual(calls,[])


if __name__=='__main__':unittest.main()
