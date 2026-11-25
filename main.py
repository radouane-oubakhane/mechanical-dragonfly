"""Build or inspect any completed Dragonfly stage from one CLI."""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'components'))
from step_01_chassis import create_chassis
from step_02_power_core import create_power_core, attach_module
from step_03_electronics import build_stage_03
from step_04_transmission import build_stage_04
from step_05_actuators import build_stage_05
from step_06_wing_joints import build_stage_06
from step_07_head import build_stage_07
from step_08_eyes import build_stage_08
from step_09_facial_armor import build_stage_09
from step_10_abdominal_spine import build_stage_10
from step_11_abdominal_armor import build_stage_11
from step_12_legs import build_stage_12
from step_13_wing_skeleton import build_stage_13
from step_14_wing_membranes import build_stage_14
from step_15_thorax_armor import build_stage_15


def build(stage=15, scale=1.0):
    if stage==1: return create_chassis(scale).scene
    if stage==2: return attach_module(create_chassis(scale),'power',create_power_core(scale))
    builders={3:build_stage_03,4:build_stage_04,5:build_stage_05,6:build_stage_06,7:build_stage_07,8:build_stage_08,9:build_stage_09,10:build_stage_10,11:build_stage_11,12:build_stage_12,13:build_stage_13,14:build_stage_14,15:build_stage_15}
    return builders[stage](scale)[-1]

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage',type=int,choices=range(1,16),default=15)
    parser.add_argument('--scale',type=float,default=1.0)
    parser.add_argument('--show',action='store_true',help='Open optional pyglet viewer')
    parser.add_argument('--output',type=Path,help='Override export path')
    args=parser.parse_args()
    scene=build(args.stage,args.scale)
    dest=args.output or ROOT/'models'/f'generated_stage_{args.stage:02d}.glb'
    dest.parent.mkdir(parents=True,exist_ok=True)
    scene.export(str(dest))
    print(f'{len(scene.geometry)} meshes | {scene.extents*1000} mm | {dest}')
    if args.show: scene.show()
