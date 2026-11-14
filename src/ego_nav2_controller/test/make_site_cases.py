"""Export real static-map snapshots for the isolated controller test; no ROS publishing."""
import argparse, json, math, struct, sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'cleaning_coverage'))
from offline.engine.coverage_map_input import load_map
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('output',type=Path)
parser.add_argument('--map',type=Path,default=ROOT/'maps/nav2/map.yaml')
parser.add_argument('--sequence',type=Path,default=ROOT/'cleaning_coverage/output/ring_ego_20261025/action_sequence.json')
args=parser.parse_args()
m=load_map(args.map)
stages=json.loads(args.sequence.read_text())['stages']
cases=[]
for stage in stages:
    if stage['kind']!='follow_path':continue
    points=[]
    for item in stage['goal']['path']['poses']:
        p=item['pose'];q=p['orientation'];points.append([p['position']['x'],p['position']['y'],2*math.atan2(q['z'],q['w'])])
    points=np.array(points,dtype='<f8')
    for index in sorted({0,len(points)//2,max(0,len(points)-7)}):
        p=points[index];r=.05;n=200
        xx,yy=np.meshgrid(p[0]-5+(np.arange(n)+.5)*r,p[1]-5+(np.arange(n)+.5)*r)
        local=m.to_local(np.column_stack([xx.ravel(),yy.ravel()]))
        col=np.floor(local[:,0]/m.resolution).astype(int);row=m.grid.shape[0]-1-np.floor(local[:,1]/m.resolution).astype(int)
        inside=(col>=0)&(row>=0)&(col<m.grid.shape[1])&(row<m.grid.shape[0])
        cells=np.full(n*n,255,dtype='uint8');g=m.grid[row[inside],col[inside]]
        cells[inside]=np.where(g==254,0,np.where(g==0,254,255))
        cases.append((stage['id'],index,p,points[index:],cells))
with args.output.open('wb') as out:
    out.write(struct.pack('<I',len(cases)))
    for stage,index,p,points,cells in cases:
        out.write(struct.pack('<III',stage,index,len(points)));out.write(p.tobytes());out.write(points.tobytes());out.write(cells.tobytes())
print(f'Exported {len(cases)} static-map cases from {args.sequence} to {args.output}')
