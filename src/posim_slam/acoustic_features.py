"""Bounded range/bearing detections from acoustic power, never geometry truth.

Elevation is unresolved in ProjectedSonarImage; these are polar observations,
not reconstructed XYZ points. Thresholds are experimental front-end settings.
"""
import numpy as np


def detect(intensity_db, ranges, directions, *, contrast_db=12., separation_m=.25, peaks_per_beam=3):
    power=np.asarray(intensity_db,dtype=float)
    ranges=np.asarray(ranges,dtype=float)
    directions=np.asarray(directions,dtype=float)
    if (power.ndim!=2 or power.size>2_000_000 or min(power.shape)<3 or
        ranges.shape!=(power.shape[0],) or directions.shape!=(power.shape[1],3)):
        raise ValueError('invalid_acoustic_shape')
    if (not np.isfinite(ranges).all() or ranges[0]<0 or np.any(np.diff(ranges)<=0) or
        not np.isfinite(directions).all() or
        not np.allclose(np.linalg.norm(directions,axis=1),1,atol=1e-4)):
        raise ValueError('invalid_acoustic_geometry')
    if (not np.isfinite(contrast_db) or contrast_db<=0 or not np.isfinite(separation_m) or
        separation_m<=0 or type(peaks_per_beam) is not int or not 1<=peaks_per_beam<=8):
        raise ValueError('invalid_acoustic_threshold')
    # The pinned producer publishes horizontal unit vectors. Do not invent
    # a bearing for a vertical vector or silently flatten other instruments.
    if np.any(np.abs(directions[:,2])>1e-5):raise ValueError('unsupported_acoustic_elevation')
    detections=[]
    for beam in range(power.shape[1]):
        column=power[:,beam];finite=np.isfinite(column)
        if finite.sum()<3:continue
        floor=float(np.median(column[finite]))
        clean=np.where(finite,column,-np.inf)
        candidates=np.flatnonzero(finite[1:-1] & (clean[1:-1]>clean[:-2]) &
                                 (clean[1:-1]>=clean[2:]) & (clean[1:-1]>=floor+contrast_db))+1
        selected=[]
        for index in sorted(candidates,key=lambda i:(-clean[i],i)):
            if ranges[index]<=0 or any(abs(ranges[index]-ranges[j])<separation_m for j in selected):continue
            selected.append(index)
            detections.append({'beam':beam,'bin':int(index),'range_m':float(ranges[index]),
                               'bearing_rad':float(np.arctan2(directions[beam,1],directions[beam,0])),
                               'power_db':float(clean[index]),'contrast_db':float(clean[index]-floor)})
            if len(selected)>=peaks_per_beam:break
    return detections
