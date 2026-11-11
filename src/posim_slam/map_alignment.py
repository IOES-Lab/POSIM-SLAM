"""Causal map/local mean coordinates. No filter feedback or covariance claim."""
import numpy as np

CONTRACT='source_pinned_map_local_mean_v1'


def _positive(value):
    return type(value) is int and 0<value<2**63


def _rotation(q):
    q=np.asarray(q,dtype=float)
    if q.shape!=(4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>1e-6:
        raise ValueError('mapping_alignment_quaternion')
    x,y,z,w=q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def _quaternion(r):
    # Select the largest component to remain well-conditioned at 180 degrees.
    candidates=np.array([1+r[0,0]-r[1,1]-r[2,2],1-r[0,0]+r[1,1]-r[2,2],
                         1-r[0,0]-r[1,1]+r[2,2],1+np.trace(r)])
    index=int(np.argmax(candidates));q=np.zeros(4);q[index]=np.sqrt(candidates[index])/2
    d=4*q[index]
    if index==3:q[:3]=[r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1]];q[:3]/=d
    else:
        j,k=(index+1)%3,(index+2)%3
        q[j]=(r[j,index]+r[index,j])/d;q[k]=(r[k,index]+r[index,k])/d
        q[3]=(r[k,j]-r[j,k])/d
    return (q/np.linalg.norm(q)).tolist()


class MapAlignment:
    """One worker epoch, called only by the consumer of matched pose/map rows.

    A revision becomes available at its first delivered pair. Older poses are
    never revised. Missing/error snapshots withhold the aligned output. The
    original local state and its uncertainty remain owned by the native VIO.
    """
    def __init__(self,epoch):
        if not isinstance(epoch,str) or not epoch:raise ValueError('mapping_alignment_epoch')
        self.epoch=epoch;self.revision=0;self.anchor=0;self.last_stamp=0
        self.first_seen=0;self.transform=None

    def paired_pose(self,state,packet):
        stamp=state.get('stamp_ns')
        if (not _positive(stamp) or not _positive(packet.get('stamp_ns')) or stamp<=self.last_stamp or packet.get('stamp_ns')!=stamp
                or state.get('type')!='state' or packet.get('type')!='map'):
            raise ValueError('mapping_alignment_pair_order')
        self.last_stamp=stamp
        alignment=packet.get('mapping_alignment');geometry=packet.get('point_geometry',{})
        if not isinstance(geometry,dict):raise ValueError('mapping_alignment_contract')
        if alignment is None or geometry.get('error') is not None:
            self.transform=None
            return None
        if (not isinstance(alignment,dict) or alignment.get('schema')!=CONTRACT
                or alignment.get('epoch')!=self.epoch or alignment.get('source')!='pinned_mapping_input_pose'
                or alignment.get('uncertainty_available') is not False or alignment.get('navigation_ready') is not False):
            raise ValueError('mapping_alignment_contract')
        revision=alignment.get('optimization_revision');anchor=alignment.get('anchor_stamp_ns')
        if (not _positive(revision) or not _positive(anchor) or anchor>stamp
                or not _positive(geometry.get('optimization_revision')) or not _positive(geometry.get('optimized_through_ns'))
                or geometry.get('optimization_revision')!=revision or geometry.get('optimized_through_ns')!=anchor
                or revision<self.revision or anchor<self.anchor):
            raise ValueError('mapping_alignment_revision')
        transform=np.asarray(alignment.get('map_from_local'),dtype=float)
        if (transform.shape!=(4,4) or not np.isfinite(transform).all()
                or not np.allclose(transform[3],[0,0,0,1],rtol=0,atol=1e-10)
                or not np.allclose(transform[:3,:3].T@transform[:3,:3],np.eye(3),rtol=0,atol=1e-8)
                or abs(np.linalg.det(transform[:3,:3])-1)>1e-8):
            raise ValueError('mapping_alignment_rigid_transform')
        if revision==self.revision:
            if self.transform is None or anchor!=self.anchor or not np.array_equal(transform,self.transform):
                raise ValueError('mapping_alignment_revision_changed')
        else:
            self.revision=revision;self.anchor=anchor;self.first_seen=stamp
            self.transform=transform.copy()
        position=np.asarray(state.get('position'),dtype=float)
        if position.shape!=(3,) or not np.isfinite(position).all():raise ValueError('mapping_alignment_position')
        rotation=_rotation(state.get('orientation_xyzw'))
        return dict(stamp_ns=stamp,epoch=self.epoch,pose_role='map_aligned_imu_mean',
            coordinates='native_mapping_world',optimization_revision=revision,anchor_stamp_ns=anchor,
            alignment_observed_stamp_ns=self.first_seen,
            position=(transform[:3,:3]@position+transform[:3,3]).tolist(),
            orientation_xyzw=_quaternion(transform[:3,:3]@rotation),
            map_from_local=dict(translation_m=transform[:3,3].tolist(),orientation_xyzw=_quaternion(transform[:3,:3])),
            uncertainty_available=False,navigation_ready=False)
