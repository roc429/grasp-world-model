"""MuJoCo environment for push-grasp task."""
import numpy as np
import mujoco
import yaml, os

class PushGraspEnv:
    def __init__(self, config):
        self.config = config
        self.model = mujoco.MjModel.from_xml_string(self._xml())
        self.data = mujoco.MjData(self.model)
        self._goal_pos = np.array([0.30, -0.08], dtype=np.float32)
        self._has_obstacle = True

    def _xml(self):
        return """<mujoco>
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    <geom type="box" size="0.20 0.15 0.005" pos="0.18 0 -0.005" rgba="0.6 0.4 0.2 1"/>
    <body name="target" pos="0.18 0.02 0.015">
      <freejoint/><geom type="box" size="0.015 0.015 0.015" rgba="1 0.2 0.2 1"/>
    </body>
    <body name="obstacle" pos="0.18 0.08 0.015">
      <freejoint/><geom type="box" size="0.015 0.015 0.015" rgba="0.2 0.2 1 1"/>
    </body>
    <body name="pusher" pos="0.18 0.15 0.04">
      <geom type="cylinder" size="0.005 0.03" rgba="0.3 0.3 0.3 1"/>
      <joint name="px" type="slide" axis="1 0 0" range="-0.3 0.3"/>
      <joint name="py" type="slide" axis="0 1 0" range="-0.3 0.3"/>
      <joint name="pz" type="slide" axis="0 0 1" range="0.0 0.3"/>
    </body>
    <site name="goal" pos="0.30 -0.08 0.005" size="0.03" rgba="0 1 0 0.4" type="cylinder"/>
  </worldbody>
  <actuator>
    <position name="ax" joint="px" kp="50"/>
    <position name="ay" joint="py" kp="50"/>
    <position name="az" joint="pz" kp="50"/>
  </actuator>
</mujoco>"""

    def reset(self, layout_id=1):
        mujoco.mj_resetData(self.model, self.data)
        path = f'config/scenes/layout_{layout_id}.yaml'
        if os.path.exists(path):
            with open(path) as f:
                cfg = yaml.safe_load(f)
        else:
            cfg = {'target':{'pos':[0.18,0.02,0.015]},'obstacle':{'enabled':True,'pos':[0.18,0.08,0.015]},'goal_zone':{'pos':[0.30,-0.08,0.005]}}
        tp = cfg['target']['pos']
        self.data.qpos[7:10] = tp
        self.data.qpos[10:14] = [1,0,0,0]
        obs = cfg.get('obstacle',{})
        if obs.get('enabled',False):
            self.data.qpos[14:17] = obs['pos']
            self.data.qpos[17:21] = [1,0,0,0]
            self._has_obstacle = True
        else:
            self.data.qpos[14:17] = [0,0,-5]
            self._has_obstacle = False
        self.data.qpos[21:24] = [tp[0], tp[1]+0.05, 0.04]
        gz = cfg.get('goal_zone',{'pos':[0.30,-0.08]})
        self._goal_pos = np.array(gz['pos'][:2], dtype=np.float32)
        mujoco.mj_forward(self.model, self.data)
        return self._state()

    def step(self, action):
        sx,sy,a,d = float(action[0]),float(action[1]),float(action[2]),float(action[3])
        ex,ey = sx+d*np.cos(a), sy+d*np.sin(a)
        self._move(sx,sy,0.06,150); self._move(sx,sy,0.02,150)
        self._move(ex,ey,0.02,300); self._move(ex,ey,0.06,150)
        ns = self._state()
        return ns, -np.sqrt((ns[0]-self._goal_pos[0])**2+(ns[1]-self._goal_pos[1])**2), self._done(ns), {'graspable':self._can_grasp(ns)}

    def _move(self, x, y, z, steps):
        for _ in range(steps):
            self.data.ctrl[:3] = [x,y,z]; mujoco.mj_step(self.model, self.data)

    def _state(self):
        s = np.zeros(10, dtype=np.float32)
        s[0:3] = self.data.qpos[7:10]; s[3:6] = self.data.qpos[14:17]
        s[6:8] = self._goal_pos; return s

    def _can_grasp(self, s):
        if not (0.08<s[0]<0.28 and -0.08<s[1]<0.08): return False
        if self._has_obstacle and np.sqrt((s[0]-s[3])**2+(s[1]-s[4])**2)<0.04: return False
        return True

    def _done(self, s):
        if not (-0.05<s[0]<0.40 and -0.20<s[1]<0.20): return True
        if np.sqrt((s[0]-self._goal_pos[0])**2+(s[1]-self._goal_pos[1])**2)<0.03: return True
        return False

    def get_objects_state(self):
        s=self._state(); return {'target':s[0:3],'obstacle':s[3:6],'goal':self._goal_pos}

    def close(self): pass
