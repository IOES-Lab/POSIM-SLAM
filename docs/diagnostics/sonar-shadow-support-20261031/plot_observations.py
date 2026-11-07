"""Plot actual raw acoustic data and its supplied-plane residual in metres."""
from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from shadow_extract import extract,plane_background
from posim_slam.sonar_surface_power import SurfacePowerModel

ROOT=Path(__file__).resolve().parent


def edges(values):
    return np.r_[values[0]-(values[1]-values[0])/2,(values[1:]+values[:-1])/2,
                 values[-1]+(values[-1]-values[-2])/2]


def main():
    inputs=json.loads((ROOT/'inputs.json').read_text())
    refs=json.loads((ROOT/'references.json').read_text())
    cal=inputs['calibration']['sonar'];model=None
    fig,axes=plt.subplots(2,3,figsize=(16,9),layout='constrained')
    for column,reference in enumerate(refs['representative_raw']):
        with np.load(ROOT/reference['path'],allow_pickle=False) as z:frame=dict(z)
        inp=inputs['frames'][reference['index']]
        az=np.arctan2(frame['beam_directions'][:,1],frame['beam_directions'][:,0])
        if model is None:
            model=SurfacePowerModel(az,np.linspace(-cal['vertical_fov']/2,cal['vertical_fov']/2,256),
                        frame['ranges'],maximum_range_m=15.)
        predicted=plane_background(model,inp['normal_sonar'],inp['plane_distance_m'])
        result=extract(frame,normal=inp['normal_sonar'],distance=inp['plane_distance_m'],
                     vertical_fov=cal['vertical_fov'],range_max=15.,expected_power=predicted)
        ae=edges(az);re=np.maximum(edges(frame['ranges']),0)
        xx=-re[:,None]*np.sin(ae)[None,:];yy=re[:,None]*np.cos(ae)[None,:]
        raw=axes[0,column].pcolormesh(xx,yy,frame['intensity_db'],shading='flat',cmap='gray',vmin=10,vmax=70)
        # Explicit warped cell edges prevent polar centre/edge ambiguity.
        r=result['sample_ranges'];radial_edges=np.vstack((r[0]-(r[1]-r[0])/2,
                       (r[1:]+r[:-1])/2,r[-1]+(r[-1]-r[-2])/2))
        polar_edges=np.column_stack((radial_edges[:,0]-(radial_edges[:,1]-radial_edges[:,0])/2,
                       (radial_edges[:,1:]+radial_edges[:,:-1])/2,
                       radial_edges[:,-1]+(radial_edges[:,-1]-radial_edges[:,-2])/2))
        # Invalid plane columns still need finite plotting vertices. They are
        # masked in the data; interpolate vertices only for drawing geometry.
        for row in polar_edges:
            good=np.isfinite(row)
            row[~good]=np.interp(np.flatnonzero(~good),np.flatnonzero(good),row[good])
        x=-polar_edges*np.sin(ae)[None,:];y=polar_edges*np.cos(ae)[None,:]
        residual=axes[1,column].pcolormesh(x,y,result['contrast'],shading='flat',cmap='RdBu_r',vmin=-15,vmax=15)
        xc=-r*np.sin(az)[None,:];yc=r*np.cos(az)[None,:]
        axes[1,column].contour(xc,yc,result['mask'].astype(float),levels=[.5],colors='#64ff50',linewidths=.65)
        stamp=inp['stamp_ns']/1e9
        axes[0,column].set_title(f'Original native sonar | {stamp:.2f} s')
        axes[1,column].set_title(f'Physical-plane residual | common gain {result["gain_db"]:.2f} dB')
        for ax in axes[:,column]:
            ax.set(xlim=(-4,4),ylim=(0,6),aspect='equal',xlabel='Starboard, m',ylabel='Forward, m')
    fig.colorbar(raw,ax=axes[0,:],label='Raw intensity, dB',shrink=.8,pad=.015)
    fig.colorbar(residual,ax=axes[1,:],label='Measured minus conditional background, dB',shrink=.8,pad=.015)
    fig.suptitle('Actual 15 m sonar / diagnostic 0–6 m crop / near at bottom\n'
                 'Green outlines: dark candidates; uniform DVL-plane assumption; no object labels or map admission',fontsize=15)
    fig.savefig(ROOT/'shadow-observations.png',dpi=140)
    plt.close(fig)


if __name__=='__main__':main()
