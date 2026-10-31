from pathlib import Path
import gzip,json,sys
base=Path(__file__).resolve().parent
sys.path[:0]=[str(base/'library/src')]
from posim_slam.native_surface import NativeSurfaceSnapshot
from posim_slam.native_surface_ranges import prepare_native_surface_ranges
for mode in ('dvl-only','dvl-fls'):
 folder=base/('original-'+mode);request=json.loads((folder/'original-request.json').read_text())
 request['planes']={int(k):v for k,v in request['planes'].items()}
 snapshot=NativeSurfaceSnapshot.read(json.loads(gzip.decompress((folder/'snapshot-before.json.gz').read_bytes())))
 try:
  result=prepare_native_surface_ranges(snapshot,**request)
  print(mode,'prepared',result.diagnostics['initialization'])
 except ValueError as error:print(mode,'whole candidate rejected:',error)
