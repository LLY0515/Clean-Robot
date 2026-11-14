import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import yaml
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'cleaning_coverage'))
from execute_forward_coverage import RosBackend

class Preflight(unittest.TestCase):
    def setUp(self):
        base = yaml.safe_load((ROOT / 'src/mid360_nav2/config/nav2_params.yaml').read_text())
        override = yaml.safe_load((ROOT / 'src/ego_nav2_controller/config/controllers.yaml').read_text())
        params = base['controller_server']['ros__parameters']
        params.update(override['controller_server']['ros__parameters'])
        self.actual = {}
        for key, value in params.items():
            if isinstance(value, dict): self.actual.update({key+'.'+k: v for k,v in value.items()})
            else: self.actual[key] = value
        self.behaviors = {'robot_base_frame': 'drive_base_link', 'global_frame': 'odom',
                          'max_rotational_vel': .2, 'min_rotational_vel': .1}
        for key, val in override['behavior_server']['ros__parameters'].items():
            if isinstance(val, dict): self.behaviors.update({key+'.'+k:v for k,v in val.items()})
            else: self.behaviors[key] = val
        self.backend = RosBackend.__new__(RosBackend)
        self.warnings = []
        self.backend.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(warning=self.warnings.append))
        costmap = override['local_costmap']['local_costmap']['ros__parameters']
        self.memory = {'plugins': costmap['plugins'],
                       'width': costmap['width'], 'height': costmap['height'],
                       'stvl_lidar.lidar_mark.obstacle_range': base['local_costmap_common']['ros__parameters']['stvl_lidar']['lidar_mark']['obstacle_range'],
                       'stvl_lidar.decay_model': base['local_costmap_common']['ros__parameters']['stvl_lidar']['decay_model'],
                       'stvl_camera.decay_model': base['local_costmap_common']['ros__parameters']['stvl_camera']['decay_model']}
        self.backend.service = lambda *_: SimpleNamespace(current_state=SimpleNamespace(id=3))
        self.backend.parameters = lambda node, names: {k:(self.actual if node=='controller_server' else self.memory if node=='local_costmap/local_costmap' else self.behaviors).get(k) for k in names}
        self.backend.geometry=None;self.backend.clients={};self.backend.active={};self.backend.poll=lambda:None
    def check(self):
        lifecycle=SimpleNamespace(GetState=SimpleNamespace(Request=lambda:None))
        with patch.dict(sys.modules, {'lifecycle_msgs.srv':lifecycle}), patch('execute_forward_coverage.time.monotonic',side_effect=[0.,2.]):
            self.backend.preflight()
    def assert_warning(self, pattern):
        self.check()
        self.assertRegex('\n'.join(self.warnings), pattern)
    def test_accepts_complete_ego(self):
        self.check()
        self.assertEqual(self.warnings, [])
    def test_inactive_lifecycle_still_rejected(self):
        self.backend.service = lambda *_: SimpleNamespace(current_state=SimpleNamespace(id=2))
        with self.assertRaisesRegex(RuntimeError, 'not active'):
            self.check()
    def test_warns_nonpersistent_stvl(self):
        self.memory['stvl_lidar.decay_model'] = 0
        self.assert_warning('STVL持久标记')
    def test_warns_old_forward_speed(self):
        self.actual['CoverageForward.max_linear_vel']=.40
        self.assert_warning('CoverageForward.max_linear_vel')
    def test_warns_old_speed_preview(self):
        self.actual['CoverageForward.curve_preview_dist']=1.5
        self.assert_warning('curve_preview_dist')
    def test_warns_missing_maneuver_cap(self):
        self.actual.pop('CoverageForward.maneuver_max_linear_vel')
        self.assert_warning('maneuver_max_linear_vel')
    def test_warns_stale_approach_speed(self):
        self.actual['CoverageApproach.max_linear_vel']=.15
        self.assert_warning('CoverageApproach.max_linear_vel')
    def test_warns_stale_turn_speed(self):
        self.actual['CoverageForward.max_angular_vel']=.2
        self.assert_warning('max_angular_vel')
    def test_warns_stale_spin_speed(self):
        self.behaviors['max_rotational_vel']=.2
        self.assert_warning('Spin坐标系或速度')
    def test_warns_old_clearance(self):
        self.actual['CoverageForward.obstacle_clearance']=.5
        self.assert_warning('obstacle_clearance')
    def test_warns_old_soft_margin(self):
        self.actual['CoverageForward.optimization_clearance_margin']=.6
        self.assert_warning('optimization_clearance_margin')
    def test_warns_missing_return_policy(self):
        self.actual.pop('CoverageForward.return_confirmations')
        self.assert_warning('return_confirmations')
    def test_warns_disabled_smooth_detour(self):
        self.actual['CoverageForward.smooth_detour_enabled']=False
        self.assert_warning('smooth_detour_enabled')
    def test_warns_old_lookahead(self):
        self.actual['CoverageForward.lookahead_dist']=.5
        self.assert_warning('lookahead_dist')
    def test_warns_short_perception_range(self):
        self.memory['stvl_lidar.lidar_mark.obstacle_range']=2.5
        self.assert_warning('预瞄感知')
    def test_standalone_and_navigation_marking_ranges_match(self):
        common = yaml.safe_load((ROOT/'src/mid360_nav2/config/nav2_params.yaml').read_text())
        self.assertEqual(
            common['local_costmap_common']['ros__parameters']['stvl_lidar']['lidar_mark']['obstacle_range'],
            self.memory['stvl_lidar.lidar_mark.obstacle_range'])
    def test_warns_mixed_controllers(self):
        self.actual['CoverageApproach.plugin']='dwb_core::DWBLocalPlanner'
        self.assert_warning('控制器混用')
    def test_warns_unguarded_spin(self):
        self.behaviors['spin.plugin']='nav2_behaviors/Spin'
        self.assert_warning('恢复行为')
    def test_warns_stale_spin_braking(self):
        self.behaviors['rotational_acc_lim']=1.
        self.assert_warning('平滑Spin')
    def test_warns_missing_map_correction_invalidation(self):
        self.actual.pop('CoverageForward.replan_frame_rotation')
        self.assert_warning('replan_frame_rotation')
    def test_warns_spin_without_heading_anchor(self):
        self.behaviors['spin.heading_frame']='odom'
        self.assert_warning('平滑Spin')
    def continuous_setup(self):
        import json
        self.backend.plan={'planner':{'motion_model':'continuous_forward'}}
        fp=[[.75,.325],[.75,-.325],[-.25,-.325],[-.25,.325]]
        self.backend.geometry={'footprint':fp,'margin':.05,'axle_x':-.25}
        self.actual.update({'CoverageContinuous.continuous_tracking_revision':7,'CoverageApproach.start_handover_revision':6})
        self.memory.update({'footprint':json.dumps(fp),'footprint_padding':.05,'robot_base_frame':'drive_base_link','global_frame':'odom'})
        self.smoother={'max_velocity':[.8,0.,.3],'max_accel':[.2,0.,.4],'max_decel':[-.4,0.,-.8]}
        self.planner={'GridBased.plugin':'ego_nav2_controller::ExactGoalLattice'}
        self.backend.parameters=lambda node,names:{k:(self.actual if node=='controller_server' else self.memory if 'costmap' in node else self.smoother if node=='velocity_smoother' else self.planner if node=='planner_server' else self.behaviors).get(k) for k in names}
        self.backend.transform=lambda *_:SimpleNamespace(translation=SimpleNamespace(x=-.25,y=0.,z=0.))
    def test_continuous_accepts_exact_goal_and_revision(self):
        self.continuous_setup();self.check()
    def test_continuous_rejects_old_smoother_cap(self):
        self.continuous_setup();self.smoother['max_velocity']=[.6,0.,.3]
        with self.assertRaisesRegex(RuntimeError,'平滑器'):self.check()
    def test_continuous_rejects_old_cruise_or_revision(self):
        for key,value in [('CoverageContinuous.max_linear_vel',.08),
                          ('CoverageContinuous.continuous_tracking_revision',6),
                          ('CoverageApproach.max_linear_vel',.3)]:
            self.continuous_setup();self.actual[key]=value
            with self.assertRaisesRegex(RuntimeError,'continuous controller'):self.check()
            self.setUp()
    def test_continuous_rejects_old_tight_start_checker(self):
        self.continuous_setup();self.actual['exact_start_checker.xy_goal_tolerance']=.03
        with self.assertRaisesRegex(RuntimeError,'continuous controller'):self.check()
    def test_continuous_rejects_old_planner_or_controller(self):
        self.continuous_setup();self.planner['GridBased.plugin']='nav2_smac_planner/SmacPlannerLattice'
        with self.assertRaisesRegex(RuntimeError,'精确终点'):self.check()
        self.continuous_setup();self.actual['CoverageApproach.start_handover_revision']=4
        with self.assertRaisesRegex(RuntimeError,'continuous controller'):self.check()

if __name__ == '__main__': unittest.main()
