"""Bounded gimbal control with alpha-beta angular tracking and loss handling."""
import math
import numpy as np
from .vision import detect_red,bearing

LIMIT_LOW=np.array([-1.35,-.65]);LIMIT_HIGH=np.array([1.35,.75]);MAX_RATE=1.2

class EyeController:
    def __init__(self):
        self.estimate=None;self.velocity=np.zeros(2);self.missing=0.;self.hits=0;self.command=np.zeros(2);self.time=0.;self.search_anchor=np.zeros(2)
    def update(self,rgb,joints,dt=.04):
        if not 0<dt<=.2:raise ValueError('dt must be in (0, .2].')
        joints=np.asarray(joints,dtype=float)
        if joints.shape!=(2,) or not np.isfinite(joints).all():raise ValueError('Expected two finite joint angles.')
        self.time+=dt;detections=detect_red(rgb);chosen=None;measured=None
        predicted=None if self.estimate is None else self.estimate+self.velocity*dt
        if detections:
            observations=[bearing(d,joints,rgb.shape[1],rgb.shape[0]) for d in detections]
            if predicted is None:chosen=detections[0];measured=observations[0]
            else:
                distances=[np.linalg.norm(o-predicted) for o in observations];idx=int(np.argmin(distances))
                if distances[idx]<.28+.2*self.missing:chosen=detections[idx];measured=observations[idx]
        if chosen is not None:
            if predicted is None:self.estimate=measured;self.velocity[:]=0
            else:
                residual=measured-predicted;self.estimate=predicted+.65*residual
                self.velocity=np.clip(self.velocity+.08*residual/dt,-1.8,1.8)
            self.missing=0.;self.hits+=1;state='TRACK' if self.hits>=3 else 'ACQUIRE'
            target=self.estimate+.10*self.velocity;self.search_anchor=self.estimate.copy()
        else:
            self.hits=0;self.missing+=dt
            if predicted is not None and self.missing<=.48:
                self.estimate=predicted;target=self.estimate;state='COAST'
            else:
                self.estimate=None;self.velocity[:]=0;state='SEARCH'
                target=self.search_anchor+np.array([.65*math.sin(1.4*self.missing),.15*math.sin(.8*self.missing)])
        target=np.clip(target,LIMIT_LOW,LIMIT_HIGH)
        delta=np.clip(target-self.command,-MAX_RATE*dt,MAX_RATE*dt)
        self.command=np.clip(self.command+delta,LIMIT_LOW,LIMIT_HIGH)
        return {'command':self.command.copy(),'state':state,'detection':chosen,'candidate_count':len(detections),
                'estimate':None if self.estimate is None else self.estimate.copy(),'missing_seconds':self.missing}
