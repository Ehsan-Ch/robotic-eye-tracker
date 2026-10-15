"""Evaluate frozen pixel-tracking code on reserved scene seeds; preserve failures."""
import argparse,hashlib,json,platform,sys,time
from datetime import datetime,timezone
from pathlib import Path
import cv2,mujoco,numpy as np
from .simulation import EyeSimulation,SCENARIOS,DT

class Video:
    def __init__(self,path,label):
        import imageio.v2 as imageio
        self.writer=imageio.get_writer(path,fps=25,codec='libx264',quality=7,macro_block_size=1,ffmpeg_params=['-movflags','+faststart'])
        self.path=Path(path);self.label=label
    def __call__(self,rgb,world,eye,row,detection):
        from PIL import Image,ImageDraw,ImageFont
        frame=rgb.copy();cv2.circle(frame,(160,120),20,(90,220,210),1);cv2.drawMarker(frame,(160,120),(230,235,240),cv2.MARKER_CROSS,12,1)
        if detection:
            x,y,w,h=detection.bbox;cv2.rectangle(frame,(x,y),(x+w,y+h),(255,195,65),1)
        canvas=Image.new('RGB',(960,560),(12,22,33));canvas.paste(Image.fromarray(frame).resize((640,480)),(0,40))
        canvas.paste(Image.fromarray(world),(640,40));canvas.paste(Image.fromarray(eye).resize((320,240)),(640,280))
        draw=ImageDraw.Draw(canvas);draw.rectangle((640,280,960,306),fill=(12,22,33))
        draw.text((14,13),self.label+' | ROBOT CAMERA / SELECTED DETECTION',fill=(227,237,247));draw.text((654,13),'WORLD VIEW',fill=(227,237,247))
        draw.text((654,288),'PAN-TILT EYE / ACTUAL MOTOR MOTION',fill=(227,237,247))
        error='hidden' if not row['visible'] else f"{row['error_px']:.1f}px"
        draw.text((14,532),f"t={row['t']:.2f}s | {row['state']} | Target error: {error} | Pan {np.degrees(row['pan']):+.1f} deg | Tilt {np.degrees(row['tilt']):+.1f} deg",fill=(115,229,212))
        self.writer.append_data(np.asarray(canvas))
        if self.path.stem=='tracking_normal' and abs(row['t']-2)<.001:canvas.save(self.path.parent/'preview.png')
    def close(self):self.writer.close()

def group_summary(trials):
    rows=[r for t in trials for r in t['frames']];visible=[r for r in rows if r['visible']];selected=sum(r['detected'] for r in rows)
    return {'trials':len(trials),'frames':len(rows),'visible_fraction':len(visible)/len(rows),
      'centered_fraction':sum(r['centered'] for r in rows)/len(rows),
      'mean_visible_error_px':float(np.mean([r['error_px'] for r in visible])) if visible else None,
      'recall_on_visible':sum(r['matched'] for r in visible)/len(visible) if visible else None,
      'association_precision':sum(r['matched'] for r in rows)/selected if selected else None,
      'false_associations':sum(r['detected'] and not r['matched'] for r in rows),
      'mean_reacquisition_s':float(np.mean([t['summary']['reacquisition_s'] for t in trials if t['summary']['reacquisition_s'] is not None]))
         if any(t['summary']['reacquisition_s'] is not None for t in trials) else None,
      'reacquired_trials':sum(t['summary']['reacquisition_s'] is not None for t in trials)}

def evaluate(out='reports',seeds=(100,101,102,103,104),seconds=10,videos=True):
    if not seeds or len(set(seeds))!=len(seeds) or 7 in seeds:raise ValueError('Use distinct held-out seeds; seed 7 is reserved for development.')
    out=Path(out);out.mkdir(parents=True,exist_ok=True);sim=EyeSimulation();trials=[];start=time.perf_counter()
    groups=[('fixed','normal')]+[('tracking',s) for s in SCENARIOS]
    try:
        for mode,scenario in groups:
            for i,seed in enumerate(seeds):
                video=Video(out/(mode+'_'+scenario+'.mp4'),mode.upper()+' / '+scenario.replace('_',' ')) if videos and i==0 else None
                try:result=sim.run(scenario,seed,seconds,mode,video)
                finally:
                    if video:video.close()
                trials.append(result)
                (out/'progress.json').write_text(json.dumps([t['summary'] for t in trials],indent=2),encoding='utf-8')
            print(mode,scenario,json.dumps(group_summary(trials[-len(seeds):])),flush=True)
    finally:sim.close()
    summary={mode+'_'+scenario:group_summary([t for t in trials if t['summary']['mode']==mode and t['summary']['scenario']==scenario]) for mode,scenario in groups}
    root=Path(__file__).parent
    payload={'created_utc':datetime.now(timezone.utc).isoformat(),'duration_s':seconds,'seeds':list(seeds),'development_seed':7,
      'runtime':{'python':sys.version.split()[0],'mujoco':mujoco.__version__,'opencv':cv2.__version__,'numpy':np.__version__,'platform':platform.platform()},
      'wall_seconds':time.perf_counter()-start,'source_sha256':{p.name:hashlib.sha256(p.read_text(encoding='utf-8').encode()).hexdigest() for p in [root/'scene.xml',root/'vision.py',root/'controller.py',root/'simulation.py']},
      'summary':summary,'trials':trials,
      'protocol':{'camera':'320x240 RGB, vertical FOV 54 degrees, 25 Hz','centered_radius_px':20,
      'visibility':'At least eight target pixels in the simulator segmentation render; not supplied to the controller.',
      'match':'Selected detection centroid falls inside the target segmentation mask.',
      'centering':'Visible target projected center lies within 20 pixels of image center; denominator is ALL frames, including occlusion.',
      'error':'Mean projected target-center error on geometrically visible frames only; inspect visible fraction alongside it.',
      'reacquisition':'Time after the screen removal at 5.2 s until a correct detection is in TRACK state; null means never reacquired.',
      'freeze':'Detector threshold was adjusted on development seed 7; no tuning after reserved-seed evaluation.'}}
    (out/'evaluation.json').write_text(json.dumps(payload,indent=2),encoding='utf-8',newline='\n');(out/'progress.json').unlink(missing_ok=True)
    return payload

def main():
    p=argparse.ArgumentParser();p.add_argument('--out',default='reports');p.add_argument('--no-video',action='store_true');p.add_argument('--count',type=int,default=5)
    a=p.parse_args()
    if not 1<=a.count<=20:p.error('count must be 1–20')
    evaluate(a.out,tuple(range(100,100+a.count)),videos=not a.no_video)
if __name__=='__main__':main()
