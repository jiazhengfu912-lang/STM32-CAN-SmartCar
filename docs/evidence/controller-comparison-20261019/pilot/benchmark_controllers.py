#!/usr/bin/env python3
"""同一Nav2系统内顺序比较DWB/RPP/MPPI，冻结参数、交错顺序、保留每轮完整证据。"""
import argparse
import copy
import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import yaml


def profiles(ws):
    """仅替换FollowPath插件和其参数，共享规划器、恢复、代价图、平滑及保护。"""
    base=yaml.safe_load((ws/'src/smartcar_nav2/config/nav2.yaml').read_text())
    rpp=copy.deepcopy(base['controller_server']['ros__parameters']['FollowPath'])
    dwb={'plugin':'dwb_core::DWBLocalPlanner','debug_trajectory_details':False,
         'min_vel_x':0.,'min_vel_y':0.,'max_vel_x':.3,'max_vel_y':0.,'max_vel_theta':.6,
         'min_speed_xy':0.,'max_speed_xy':.3,'min_speed_theta':0.,
         'acc_lim_x':.5,'acc_lim_y':0.,'acc_lim_theta':1.5,
         'decel_lim_x':-.5,'decel_lim_y':0.,'decel_lim_theta':-1.5,
         'vx_samples':15,'vy_samples':1,'vtheta_samples':25,'sim_time':2.4,
         'linear_granularity':.05,'angular_granularity':.025,'transform_tolerance':.5,
         'xy_goal_tolerance':.15,'trans_stopped_velocity':.02,
         'short_circuit_trajectory_evaluation':True,'stateful':True,
         'critics':['RotateToGoal','Oscillation','ObstacleFootprint','GoalAlign','PathAlign','PathDist','GoalDist'],
         'ObstacleFootprint.scale':.02,'PathAlign.scale':32.,'PathAlign.forward_point_distance':.1,
         'GoalAlign.scale':24.,'GoalAlign.forward_point_distance':.1,'PathDist.scale':32.,'GoalDist.scale':24.,
         'RotateToGoal.scale':32.,'RotateToGoal.slowing_factor':5.,'RotateToGoal.lookahead_time':-1.}
    mppi={'plugin':'nav2_mppi_controller::MPPIController','time_steps':24,'model_dt':.1,'batch_size':1000,
          'vx_std':.15,'vy_std':0.,'wz_std':.4,'vx_max':.3,'vx_min':0.,'vy_max':0.,'wz_max':.6,
          'iteration_count':1,'prune_distance':1.5,'transform_tolerance':.5,
          'temperature':.3,'gamma':.015,'motion_model':'DiffDrive','visualize':False,
          'regenerate_noises':False,
          'critics':['ConstraintCritic','CostCritic','GoalCritic','GoalAngleCritic','PathAlignCritic',
                     'PathFollowCritic','PathAngleCritic','PreferForwardCritic']}
    for name,weight,extra in [
        ('ConstraintCritic',4.,{}),('CostCritic',3.81,{'critical_cost':300.,'consider_footprint':True,
          'collision_cost':1000000.,'near_goal_distance':.5}),
        ('GoalCritic',5.,{'threshold_to_consider':.8}),('GoalAngleCritic',3.,{'threshold_to_consider':.4}),
        ('PathAlignCritic',14.,{'max_path_occupancy_ratio':.05,'trajectory_point_step':3,
          'threshold_to_consider':.4,'offset_from_furthest':5,'use_path_orientations':False}),
        ('PathFollowCritic',5.,{'offset_from_furthest':6,'threshold_to_consider':.8}),
        ('PathAngleCritic',2.,{'offset_from_furthest':4,'threshold_to_consider':.4,
          'max_angle_to_furthest':1.,'forward_preference':True}),
        ('PreferForwardCritic',5.,{'threshold_to_consider':.4})]:
        mppi[name]={'enabled':True,'cost_power':1,'cost_weight':weight,**extra}
    configs={}
    for name,plugin in [('DWB',dwb),('RPP',rpp),('MPPI',mppi)]:
        config=copy.deepcopy(base);config['controller_server']['ros__parameters']['FollowPath']=plugin
        configs[name]=config
    return configs


def main():
    """每轮重建隔离仿真，失败保留，不根据结果删除轮次或自动修改控制器参数。"""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--repeats',type=int,default=3)
    parser.add_argument('--pilot',action='store_true')
    parser.add_argument('--goal-timeout',type=float,default=240.)
    args=parser.parse_args()
    if not 1<=args.repeats<=10:parser.error('重复次数须为1..10')
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    ws=Path(__file__).resolve().parents[1];configdir=out/'configs';configdir.mkdir()
    config_paths={}
    for name,config in profiles(ws).items():
        p=configdir/(name.lower()+'.yaml');p.write_text(yaml.safe_dump(config,sort_keys=False));config_paths[name]=p
    cases=['straight'] if args.pilot else ['straight','turn','detour','two_doors']
    order=[];names=['DWB','RPP','MPPI']
    for repeat in range(args.repeats):
        for case_index,case in enumerate(cases):
            shift=(repeat+case_index)%3
            for name in names[shift:]+names[:shift]:order.append({'repeat':repeat+1,'case':case,'controller':name})
    protocol={'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'phase':'PILOT' if args.pilot else 'FORMAL',
              'repeats':args.repeats,'order':order,'goal_timeout_wall_s':args.goal_timeout,
              'head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ws.parent,text=True).strip(),
              'scope':'SIMULATION_ONLY; no system/controller default changes; no physical hardware',
              'common':{'controller_hz':10,'forward_limit_mps':.3,'controller_angular_limit_rps':.6,
                'goal_xy_m':.15,'goal_yaw_rad':.2,'global_planner':'NavFn A*','replan_hz':1,
                'costmaps':'unchanged','gates':'unchanged','recovery':'unchanged, max 4, no automatic clears',
                'threads_env':1,'execution':'serial, rotated controller order'},
              'selection_policy':{'primary':'lexicographic: correct task outcome + verified stop, then model geometry overlap safety check',
                'tie_break':'among controllers passing all planned trials, equal-case mean normalized cost: sim_time 40%, travel_length 25%, angular_total_variation_per_sim_s 20%, controller_process_cpu 15%; lower is better',
                'blocked_case':'expected ABORTED+stop; excluded from efficiency score, included in reliability',
                'failed_trials':'never removed; partial-success efficiency cannot defeat reliable completion'},
              'limitations':['Three repeats do not establish statistical significance or universal optimality',
                'AMCL/Gazebo and MPPI random noise are not seed-controlled in this interface',
                'CPU process includes local costmap and middleware; not pure optimizer time',
                'existing user GUI workload remains; record host load and real-time factor',
                'clearance is a 2D model geometry proxy, not a contact sensor or hardware measurement'],
              'config_sha256':{name:hashlib.sha256(p.read_bytes()).hexdigest() for name,p in config_paths.items()}}
    (out/'protocol.json').write_text(json.dumps(protocol,indent=2))
    for source in ['benchmark_controllers.py','verify_replanning.py']:
        (out/source).write_bytes((ws/'tools'/source).read_bytes())
    env=os.environ.copy();env.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
    results=[]
    for index,item in enumerate(order):
        name=f'{index+1:02d}_{item["controller"].lower()}_{item["case"]}_r{item["repeat"]}'
        run=out/'runs'/name;run.parent.mkdir(exist_ok=True)
        command=[sys.executable,str(ws/'tools/verify_replanning.py'),'--output',str(run),
                 '--case',item['case'],'--params-file',str(config_paths[item['controller']]),
                 '--domain-id',str(100+index),'--goal-timeout',str(args.goal_timeout)]
        print(json.dumps({'event':'run_start','index':index+1,'total':len(order),'name':name}),flush=True)
        started=time.monotonic()
        with (out/(name+'.stdout.log')).open('w') as stream:
            child=subprocess.run(command,env=env,cwd=ws.parent,stdout=stream,stderr=subprocess.STDOUT)
        report=json.loads((run/'result.json').read_text()) if (run/'result.json').exists() else {'result':'NO_REPORT'}
        result={**item,'run':str(run.relative_to(out)),'exit_code':child.returncode,
                'result':report['result'],'duration_wall_s':time.monotonic()-started,'error':report.get('error')}
        results.append(result);(out/'progress.json').write_text(json.dumps(results,indent=2))
        print(json.dumps({'event':'run_complete',**result}),flush=True)
    return 0


if __name__=='__main__':raise SystemExit(main())
