"""Scene generation and scoring are separate from the pixel-only controller."""
import math
from pathlib import Path
import mujoco
import numpy as np
from .controller import EyeController
from .timing import frame_count

WIDTH,HEIGHT,DT=320,240,.04
SCENARIOS=('normal','occlusion','dim_light','fast_motion','lookalike')

class EyeSimulation:
    def __init__(self):
        self.model=mujoco.MjModel.from_xml_path(str(Path(__file__).with_name('scene.xml')))
        self.data=mujoco.MjData(self.model);self.renderer=mujoco.Renderer(self.model,height=HEIGHT,width=WIDTH)
        self.cam=self.model.camera('eye_camera').id;self.target_id=self.model.geom('target_ball').id
        self.distractor_id=self.model.geom('distractor_ball').id
        self.target_mocap=int(self.model.body('target').mocapid[0]);self.distractor_mocap=int(self.model.body('distractor').mocapid[0]);self.occluder_mocap=int(self.model.body('occluder').mocapid[0])
    def close(self):self.renderer.close()
    def reset(self,seed):
        mujoco.mj_resetData(self.model,self.data);self.rng=np.random.default_rng(seed)
        self.phase=self.rng.uniform(-.3,.3);self.amplitude=self.rng.uniform(.9,1.1);self.controller=EyeController()
        self.model.geom_rgba[self.distractor_id]=[.04,.3,.9,1]
        mujoco.mj_forward(self.model,self.data)
    def set_scene(self,t,scenario):
        speed=3.0 if scenario=='fast_motion' else .65
        target=np.array([1.6*self.amplitude*math.sin(speed*t+self.phase),3.1+.3*math.cos(.4*t),1.3+.55*math.sin(.75*t+self.phase)])
        self.data.mocap_pos[self.target_mocap]=target
        distractor=np.array([-1.5*math.sin(.7*t+.7),2.8,1.4+.3*math.cos(.9*t)])
        if scenario=='lookalike':
            self.model.geom_rgba[self.distractor_id]=[.95,.035,.055,1]
            if t<3:distractor=[3.7,3.1,1.5]
        self.data.mocap_pos[self.distractor_mocap]=distractor
        # A physical screen, controlled by the scene generator, obscures the target.
        if scenario=='occlusion' and 4<=t<5.2:
            origin=np.array([0,0,1.23]);self.data.mocap_pos[self.occluder_mocap]=origin+.55*(target-origin)
        else:self.data.mocap_pos[self.occluder_mocap]=[0,1.8,-4]
        mujoco.mj_forward(self.model,self.data)
    def render_rgb(self,scenario):
        self.renderer.disable_segmentation_rendering();self.renderer.update_scene(self.data,camera='eye_camera');rgb=self.renderer.render().copy()
        if scenario=='dim_light':rgb=np.clip(rgb.astype(float)*.22+self.rng.normal(0,2,rgb.shape),0,255).astype(np.uint8)
        return rgb
    def truth(self):
        self.renderer.enable_segmentation_rendering();self.renderer.update_scene(self.data,camera='eye_camera');seg=self.renderer.render().copy();self.renderer.disable_segmentation_rendering()
        mask=(seg[:,:,0]==self.target_id)&(seg[:,:,1]==int(mujoco.mjtObj.mjOBJ_GEOM))
        pos=self.data.geom_xpos[self.target_id]-self.data.cam_xpos[self.cam]
        local=self.data.cam_xmat[self.cam].reshape(3,3).T@pos
        f=HEIGHT/(2*math.tan(math.radians(54)/2))
        uv=None if local[2]>=0 else [(WIDTH-1)/2+f*local[0]/(-local[2]),(HEIGHT-1)/2-f*local[1]/(-local[2])]
        return mask,uv
    def overview(self):
        cam=mujoco.MjvCamera();cam.lookat[:]=[0,1.5,1.1];cam.distance=5.4;cam.azimuth=210;cam.elevation=-20
        self.renderer.disable_segmentation_rendering();self.renderer.update_scene(self.data,camera=cam);return self.renderer.render().copy()
    def closeup(self):
        cam=mujoco.MjvCamera();cam.lookat[:]=[0,0,1.06];cam.distance=1.45;cam.azimuth=225;cam.elevation=-10
        self.renderer.disable_segmentation_rendering();self.renderer.update_scene(self.data,camera=cam);return self.renderer.render().copy()
    def run(self,scenario='normal',seed=7,seconds=10,mode='tracking',callback=None):
        if scenario not in SCENARIOS or mode not in ('tracking','fixed'):raise ValueError('Invalid simulation configuration.')
        frames = frame_count(seconds, DT)
        self.reset(seed);rows=[]
        for frame in range(frames):
            t=frame*DT;self.set_scene(t,scenario);rgb=self.render_rgb(scenario)
            result=self.controller.update(rgb,self.data.qpos.copy(),DT)
            # Ground truth is computed only after the controller has returned.
            mask,uv=self.truth();visible=int(mask.sum())>=8;det=result['detection']
            matched=False
            if det is not None:
                u,v=int(round(det.u)),int(round(det.v))
                matched=bool(0<=u<WIDTH and 0<=v<HEIGHT and mask[v,u])
            error=None if uv is None else math.hypot(uv[0]-(WIDTH-1)/2,uv[1]-(HEIGHT-1)/2)
            row={'t':round(t,3),'pan':float(self.data.qpos[0]),'tilt':float(self.data.qpos[1]),'state':result['state'],
                 'visible':visible,'matched':matched,'detected':det is not None,'error_px':error,'centered':bool(visible and error is not None and error<=20),
                 'command':result['command'].tolist(),'target_pixels':int(mask.sum()),'candidate_count':result['candidate_count']}
            rows.append(row)
            if callback:callback(rgb,self.overview(),self.closeup(),row,det)
            self.data.ctrl[:]=result['command'] if mode=='tracking' else 0
            mujoco.mj_step(self.model,self.data,nstep=20)
        visible_rows=[r for r in rows if r['visible']];detections=sum(r['detected'] for r in rows)
        summary={'scenario':scenario,'seed':seed,'mode':mode,'frames':len(rows),'visible_fraction':len(visible_rows)/len(rows),
           'centered_fraction':sum(r['centered'] for r in rows)/len(rows),
           'mean_visible_error_px':float(np.mean([r['error_px'] for r in visible_rows])) if visible_rows else None,
           'detection_recall':sum(r['matched'] for r in visible_rows)/len(visible_rows) if visible_rows else None,
           'association_precision':sum(r['matched'] for r in rows)/detections if detections else None,
           'false_associations':sum(r['detected'] and not r['matched'] for r in rows),
           'search_fraction':sum(r['state']=='SEARCH' for r in rows)/len(rows),
           'reacquisition_s':next((round(r['t']-5.2,3) for r in rows if r['t']>=5.2 and r['matched'] and r['state']=='TRACK'),None) if scenario=='occlusion' else None}
        return {'summary':summary,'frames':rows}
