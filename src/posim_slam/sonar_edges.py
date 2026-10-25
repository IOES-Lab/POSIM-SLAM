"""Background-relative echo-boundary candidates, not identified 3D landmarks.

Only measured dB, calibrated range bins and explicit ground-onset assumptions
enter this frontend. A shadow boundary is not a stationary reflector. No shadow
height or zero-elevation point is manufactured. Thresholds are diagnostic, not a
calibrated constant-false-alarm detector.
"""
from dataclasses import dataclass
import numpy as np
from .sonar_input import validate_frame
from .sonar_surface import _median


@dataclass(frozen=True)
class EchoEdgeConfig:
    minimum_range_m: float = .4
    noise_range_max_m: float = 1.
    return_margin_db: float = 12.
    background_margin_db: float = 6.
    rise_db: float = 6.
    rise_span_m: float = .10
    minimum_beams: int = 3


def echo_tracking_image(frame):
    """Fixed background-relative dB mapping; invariant to uniform gain shifts."""
    validate_frame(frame)
    smooth=_median(_median(np.asarray(frame['intensity_db'],dtype=float),3,1),3,0)
    relative=smooth-np.median(smooth,axis=1)[:,None]
    return np.uint8(np.clip((relative+10.)*(255./40.),0,255)),smooth


def echo_boundaries(frame, *, ground_entry_band_m, config=EchoEdgeConfig()):
    """Return a normalized tracking image and eligible rising-edge mask.

    Median filtering rejects isolated spikes. Same-range median subtraction
    reduces broad ring/gain structure without treating it as a seabed label.
    Coherence, a measured positive range gradient and the noise floor constrain
    seeds. The supplied uncertain ground-entry band is excluded; its limits
    never become newly detected edges. It is a local-plane hypothesis, not truth.
    """
    validate_frame(frame)
    floats=[getattr(config,k) for k in config.__dataclass_fields__ if k!='minimum_beams']
    if (not np.isfinite(floats).all() or min(floats)<=0 or config.noise_range_max_m<=config.minimum_range_m
            or type(config.minimum_beams) is not int or not 3<=config.minimum_beams<=15 or config.minimum_beams%2!=1):
        raise ValueError('invalid_echo_edge_config')
    a=np.asarray(frame['intensity_db'],dtype=float);ranges=np.asarray(frame['ranges'],dtype=float)
    rows,beams=a.shape;band=np.asarray(ground_entry_band_m,dtype=float)
    if band.shape!=(beams,2) or np.isnan(band).any() or np.any(band<=0) or np.any(band[:,0]>band[:,1]):
        raise ValueError('invalid_ground_entry_band')
    noise_rows=(ranges>.2)&(ranges<config.noise_range_max_m)
    if noise_rows.sum()<4:raise ValueError('insufficient_noise_window')
    image,smooth=echo_tracking_image(frame)
    noise=float(np.median(smooth[noise_rows]))
    # Use metric offsets so differing range resolution preserves edge scale.
    before=np.searchsorted(ranges,ranges-config.rise_span_m/2,side='right')-1
    after=np.searchsorted(ranges,ranges+config.rise_span_m/2,side='left')
    interior=(before>=0)&(after<rows)
    gradient=smooth[np.minimum(after,rows-1)]-smooth[np.maximum(before,0)]
    returned=smooth[np.minimum(after,rows-1)]
    # A dark shadow returning to ordinary seabed brightness is not a bright
    # object echo. Require excess over the same-range background as well.
    background=np.median(smooth,axis=1)[np.minimum(after,rows-1),None]
    rise=((gradient>=config.rise_db)&(returned>=noise+config.return_margin_db)
          &(returned>=background+config.background_margin_db))
    # Require adjacent angular support, then retain the centre beam only.
    windows=np.lib.stride_tricks.sliding_window_view(np.pad(rise,((0,0),(config.minimum_beams//2,)*2)),config.minimum_beams,axis=1)
    coherent=np.all(windows,axis=-1)
    # Local range maxima locate the sharpest part of the onset, not every bin
    # in a broad rising ramp. An edge crossing the excluded band is not seeded.
    peak=(gradient>=np.roll(gradient,1,axis=0))&(gradient>np.roll(gradient,-1,axis=0))
    ground=(ranges[:,None]>=band[None,:,0])&(ranges[:,None]<=band[None,:,1])
    eligible=coherent&peak&~ground&interior[:,None]&(ranges[:,None]>config.minimum_range_m)
    eligible[:2]=False;eligible[-2:]=False;eligible[:,:2]=False;eligible[:,-2:]=False
    # A corner response integrates a patch and need not peak on the one-bin
    # range-gradient maximum. Use the same two-pixel support for seeding and
    # tracking; never extend it into the excluded ground band or image border.
    support=np.any(np.lib.stride_tricks.sliding_window_view(np.pad(eligible,2),(5,5)),axis=(-2,-1))
    support&=~ground&interior[:,None]&(ranges[:,None]>config.minimum_range_m)
    support[:2]=False;support[-2:]=False;support[:,:2]=False;support[:,-2:]=False
    return dict(image=image,edge_mask=eligible,tracking_mask=support,noise_db=noise,gradient_db=gradient,
                ground_excluded_pixels=int((coherent&peak&ground).sum()),
                scope='candidate echo boundaries; not physical identities or elevations')


def near_edge(pixels,mask,*,radius=2):
    """Whether a tracked image coordinate stays near a measured eligible edge."""
    pixels=np.asarray(pixels,dtype=float).reshape(-1,2);mask=np.asarray(mask,dtype=bool)
    result=np.zeros(len(pixels),dtype=bool)
    for i,(x,y) in enumerate(pixels):
        if not np.isfinite([x,y]).all():continue
        x,y=int(round(x)),int(round(y))
        if 0<=x<mask.shape[1] and 0<=y<mask.shape[0]:
            result[i]=bool(mask[max(0,y-radius):y+radius+1,max(0,x-radius):x+radius+1].any())
    return result
