"""Reproduce association from saved detections, without OpenCV or raw images."""
from pathlib import Path
import json
import numpy as np
from projective_assoc import ProjectiveTracks


def descriptor_pairs(previous, current):
    if min(len(previous),len(current)) < 2:
        return {}
    # Hamming distance independently reproduces the stored OpenCV BF result.
    distance = np.unpackbits(previous[:,None,:]^current[None,:,:],axis=2).sum(axis=2)
    forward = np.argsort(distance,axis=1,kind='stable')[:,:2]
    reverse = np.argsort(distance,axis=0,kind='stable')[:2,:]
    good = {}
    for i,(j,k) in enumerate(forward):
        a,b = reverse[:,j]
        if distance[i,j] < .8*distance[i,k] and a==i and distance[a,j] < .8*distance[b,j]:
            good[int(j)] = int(i)
    return good


def replay(root, mode='projective', samples=257):
    root = Path(root)
    with np.load(root/'detected-features.npz',allow_pickle=False) as file:
        data = dict(file)
    calibration = json.loads((root.parent/'sonar-inspection-scene-20261031/calibration.json').read_text())['sonar']
    geometry = {k:calibration[k] for k in ('imu_to_sonar','sonar_to_imu','range_min','range_max','horizontal_fov','vertical_fov')}
    geometry['imu_to_sonar'] = np.array(geometry['imu_to_sonar'])
    geometry['sonar_to_imu'] = np.array(geometry['sonar_to_imu'])
    poses = {int(t):dict(position=p,attitude=r) for t,p,r in zip(data['nav_stamps_ns'],data['nav_positions'],data['nav_attitudes'])}
    tracker = ProjectiveTracks(geometry,samples=samples)
    tracks,events,previous,previous_ids,next_id = {},[],None,[],0
    for index, raw_stamp in enumerate(data['stamps_ns']):
        stamp = int(raw_stamp)
        begin,end = data['offsets'][index:index+2]
        pixels, descriptors = data['pixels'][begin:end],data['descriptors'][begin:end]
        polar = np.column_stack((np.interp(pixels[:,1],np.arange(len(data['ranges'])),data['ranges']),
                                 np.interp(pixels[:,0],np.arange(len(data['angles'])),data['angles'])))
        observations = [dict(range_m=float(r),azimuth_rad=float(a)) for r,a in polar]
        if mode == 'projective':
            tracker.push(stamp,poses[stamp],pixels,observations,data['ranges'],data['angles'])
        elif mode == 'descriptor':
            matches = {} if previous is None else descriptor_pairs(previous,descriptors)
            current_ids, current = [],[]
            for pixel_id,obs in enumerate(observations):
                if pixel_id in matches:
                    ident = previous_ids[matches[pixel_id]]
                else:
                    ident,next_id = next_id,next_id+1
                    tracks[ident] = dict(observations=[])
                tracks[ident]['observations'].append(dict(stamp_ns=stamp,**obs))
                current_ids.append(ident)
                current.append(dict(track_id=ident,pixel_index=pixel_id,length=len(tracks[ident]['observations'])))
            events.append(dict(stamp_ns=stamp,detected=len(current),matched=len(matches),new=len(current)-len(matches),observations=current))
            previous,previous_ids = descriptors,current_ids
        else:
            raise ValueError('unknown_control')
    return (tracker.rows,tracker.tracks) if mode=='projective' else (events,tracks)


if __name__=='__main__':
    root = Path(__file__).resolve().parent
    for mode in ('descriptor','projective'):
        events, tracks = replay(root,mode)
        print(mode,dict(frames=len(events),unique_tracks=len(tracks),matched=sum(e['matched'] for e in events),
            repeated_tracks=sum(len(t['observations'])>=3 for t in tracks.values())))
