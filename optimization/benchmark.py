"""Benchmark CPU GLB loading and bounding-box calculation, not rendering FPS."""
import json
from pathlib import Path
import time
import trimesh
from step_16_geometry import read_glb,stats
ROOT=Path(__file__).resolve().parents[1]
NAMES=['step_15_assembled.glb','dragonfly_static_balanced.glb','dragonfly_static_performance.glb','dragonfly_animated.glb','dragonfly_animated_balanced.glb','dragonfly_animated_performance.glb']

def benchmark():
    report={}
    for name in NAMES:
        p=ROOT/'models'/name;d,b=read_glb(p);times=[]
        for _ in range(2):
            t=time.perf_counter();s=trimesh.load(p,force='scene',process=False);bounds=s.bounds;times.append(time.perf_counter()-t)
        report[name]=dict(stats(d),bytes=p.stat().st_size,load_and_bounds_seconds=times,bounds_mm=(bounds*1000).tolist())
    return report

if __name__=='__main__':
    result=benchmark();path=ROOT/'docs'/'BENCHMARK_STEP_16.json';path.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
