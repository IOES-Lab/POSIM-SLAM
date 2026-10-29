"""Increment-only reference API, native snapshot identity and bounded files."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from posim_slam.delayed_surface_update import prepare_delayed_ranges
from posim_slam.native_surface import NativeSurfaceSnapshot,prepare_native_ranges
from posim_slam.retired_pose_archive import coordinates
from test_delayed_surface_update import physical_case,descriptor


def node(n):
    value=dict(id=n.ident,variables=[descriptor(v) for v in n.variables],kept_indices=n.kept,removed_indices=n.removed,
        regression=n.regression.tolist(),offset=n.offset.tolist(),noise_factor=n.noise.tolist(),unbounded_basis=n.unbounded.tolist())
    if n.deterministic.shape[1]:value['deterministic_basis']=n.deterministic.tolist()
    return value


def snapshot(archive,*,ledger=(),identity=1,owner=8,prior=9,stamp=None,delta=None):
    return dict(schema='posim-slam-native-surface-snapshot-1',snapshot_id=identity,owner_revision=owner,prior_revision=prior,
        epoch=archive.epoch,stamp_ns=stamp or archive.stamp_ns,variables=[descriptor(v) for v in archive.root_variables],
        delta=[archive.root[k] for k in coordinates(archive.root_variables)] if delta is None else list(delta),
        conditionals=[node(n) for n in archive.nodes.values()],archive_closed_through_ns=0,protected_states=[],
        range_ledger=dict(recent=list(ledger),closed_through_ns=0))


def arguments(kwargs):return {key:kwargs[key] for key in ('order','delta','jacobian','residual','range_keys')}


class NativeSurfaceAPI(unittest.TestCase):
    def test_increment_matches_complete_reference_without_returning_the_old_prior(self):
        prior,archive,kwargs,*_=physical_case();native=NativeSurfaceSnapshot.read(snapshot(archive,ledger=[(3,99,0)]))
        before=native.recover([1,2])['deltas'].copy();result=prepare_native_ranges(native,**arguments(kwargs))
        full=prepare_delayed_ranges(prior,archive,**kwargs)
        np.testing.assert_array_equal(result.new_live_rows.jacobian,full.new_live_rows.jacobian)
        np.testing.assert_array_equal(result.new_live_rows.offset,full.new_live_rows.offset)
        self.assertFalse(hasattr(result,'prior'));self.assertNotIn('prior',result.as_packet())
        self.assertEqual(result.as_packet()['snapshot_id'],1);self.assertEqual(result.as_packet()['prior_revision'],9)
        self.assertEqual(len(result.as_packet()['conditionals']),2)
        self.assertEqual({tuple(k) for k in result.as_packet()['range_keys']},set(kwargs['range_keys']))
        np.testing.assert_array_equal(native.recover([1,2])['deltas'],before)
        self.assertEqual(native.ledger.recent,frozenset({(3,99,0)}))

    def test_a_fresh_native_snapshot_after_optimization_does_not_need_a_fake_verification_prior(self):
        _,archive,kwargs,*_=physical_case();first=prepare_native_ranges(NativeSurfaceSnapshot.read(snapshot(archive)),**arguments(kwargs))
        updated=first._likelihood.archive;delta=np.array([updated.root[k] for k in coordinates(updated.root_variables)])+.01
        new=NativeSurfaceSnapshot.read(snapshot(updated,ledger=first._likelihood.ledger.recent,identity=2,owner=10,prior=15,stamp=5,delta=delta))
        pose=new.reference('state',1);plane=new.reference('plane',0);order=(pose,plane)
        current=np.r_[new.recover([1])['deltas'],delta[-6:-3]]
        j=np.zeros((1,9));j[0,2]=1;j[0,8]=-.5
        result=prepare_native_ranges(new,order=order,delta=current,jacobian=j,residual=[.02],range_keys=[(1,44,3)])
        packet=result.as_packet();self.assertEqual((packet['snapshot_id'],packet['owner_revision'],packet['prior_revision']),(2,10,15))
        self.assertGreaterEqual(len(packet['conditionals']),1)
        self.assertTrue(first._likelihood.ledger.recent<result._likelihood.ledger.recent)

    def test_original_measurements_cannot_be_reused_or_reopened(self):
        _,archive,kwargs,*_=physical_case();key=next(iter(kwargs['range_keys']))
        value=snapshot(archive,ledger=[key]);native=NativeSurfaceSnapshot.read(value)
        with self.assertRaisesRegex(ValueError,'reused'):prepare_native_ranges(native,**arguments(kwargs))
        value=snapshot(archive);value['range_ledger']['closed_through_ns']=2
        with self.assertRaisesRegex(ValueError,'closed'):prepare_native_ranges(NativeSurfaceSnapshot.read(value),**arguments(kwargs))

    def test_malformed_native_identity_ledger_and_references_are_rejected(self):
        _,archive,_,*_=physical_case();base=snapshot(archive)
        for change in [dict(snapshot_id=True),dict(owner_revision=-1),dict(prior_revision=True),dict(stamp_ns=0),
                       dict(epoch=''),dict(protected_states=[999]),dict(range_ledger=dict(recent=[(999,0,0)],closed_through_ns=0))]:
            with self.subTest(change=change),self.assertRaises(ValueError):NativeSurfaceSnapshot.read(dict(base,**change))
        bad=copy.deepcopy(base);bad['variables'][0]['reference'][0]+=.1
        with self.assertRaisesRegex(ValueError,'reference'):NativeSurfaceSnapshot.read(bad)
        with self.assertRaises(ValueError):prepare_native_ranges(base,order=(),delta=[],jacobian=[],residual=[],range_keys=[])

    def test_native_file_is_bound_to_the_owned_directory_identity_and_size(self):
        _,archive,_,*_=physical_case();value=snapshot(archive)
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary).resolve();path=root/'worker-surface-snapshot-1.json';path.write_text(json.dumps(value))
            metadata=dict(schema='posim-slam-native-surface-file-1',snapshot_id=1,bytes=path.stat().st_size,path=str(path),stamp_ns=4,epoch=archive.epoch)
            parsed=NativeSurfaceSnapshot.read_file(metadata,root);self.assertEqual(parsed.order,archive.root_variables)
            for change in [dict(bytes=path.stat().st_size+1),dict(bytes=128*1024*1024+1),dict(snapshot_id=2),dict(stamp_ns=5),dict(path=str(root/'other.json'))]:
                with self.subTest(change=change),self.assertRaises(ValueError):NativeSurfaceSnapshot.read_file(dict(metadata,**change),root)
            outside=root/'outside';outside.mkdir()
            with self.assertRaises(ValueError):NativeSurfaceSnapshot.read_file(metadata,outside)

    def test_native_file_symlinks_are_not_followed(self):
        _,archive,_,*_=physical_case()
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary).resolve();outside=root/'private.json';outside.write_text(json.dumps(snapshot(archive)))
            alias=root/'worker-surface-snapshot-1.json'
            try:alias.symlink_to(outside)
            except OSError:self.skipTest('Symlink creation requires this platform privilege')
            meta=dict(schema='posim-slam-native-surface-file-1',snapshot_id=1,bytes=outside.stat().st_size,path=str(alias),stamp_ns=4,epoch=archive.epoch)
            with self.assertRaises(ValueError):NativeSurfaceSnapshot.read_file(meta,root)
            self.assertTrue(outside.is_file())


if __name__=='__main__':unittest.main()
