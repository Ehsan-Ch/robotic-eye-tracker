import inspect,math,unittest
from pathlib import Path
import cv2,numpy as np
from eye_lab.vision import detect_red,Detection,bearing
from eye_lab.controller import EyeController,LIMIT_LOW,LIMIT_HIGH,MAX_RATE

def frame(u=160,v=120,color=(255,0,0)):
    image=np.zeros((240,320,3),np.uint8);cv2.circle(image,(u,v),12,color,-1);return image

class VisionTests(unittest.TestCase):
    def test_red_centroid(self):
        d=detect_red(frame(75,90));self.assertEqual(len(d),1);self.assertAlmostEqual(d[0].u,75);self.assertAlmostEqual(d[0].v,90)
    def test_rgb_not_bgr(self):self.assertEqual(detect_red(frame(color=(0,0,255))),[])
    def test_red_hue_wrap(self):self.assertEqual(len(detect_red(frame(color=(255,0,35)))),1)
    def test_dark_red_still_detected(self):self.assertEqual(len(detect_red(frame(color=(50,0,0)))),1)
    def test_dark_sensor_noise_rejected(self):
        noise=np.random.default_rng(10).integers(0,15,(240,320,3),dtype=np.uint8);self.assertEqual(detect_red(noise),[])
    def test_small_noise_rejected(self):
        image=np.zeros((240,320,3),np.uint8);image[20:22,20:22]=[255,0,0];self.assertEqual(detect_red(image),[])
    def test_candidates_sorted_by_area(self):
        image=frame(80,100);cv2.circle(image,(220,110),22,(255,0,0),-1)
        detections=detect_red(image);self.assertEqual(len(detections),2);self.assertGreater(detections[0].area,detections[1].area)
    def test_invalid_image_rejected(self):
        for image in (np.zeros((20,20)),np.zeros((20,20,4),np.uint8)):
            with self.assertRaises(ValueError):detect_red(image)
    def test_center_ray_matches_encoder_bearing(self):
        np.testing.assert_allclose(bearing(Detection(159.5,119.5,100,(0,0,1,1)),[.3,.2]),[.3,.2],atol=1e-12)
    def test_right_pixel_turns_pan_negative(self):self.assertLess(bearing(Detection(220,120,100,(0,0,1,1)),[0,0])[0],0)
    def test_above_pixel_turns_tilt_positive(self):self.assertGreater(bearing(Detection(160,60,100,(0,0,1,1)),[0,0])[1],0)

class ControllerTests(unittest.TestCase):
    def test_three_hits_to_track(self):
        c=EyeController();states=[c.update(frame(),[0,0])['state'] for _ in range(3)];self.assertEqual(states,['ACQUIRE','ACQUIRE','TRACK'])
    def test_loss_coasts_then_searches(self):
        c=EyeController()
        for _ in range(3):c.update(frame(),[0,0])
        blank=np.zeros((240,320,3),np.uint8);self.assertEqual(c.update(blank,[0,0])['state'],'COAST')
        for _ in range(15):r=c.update(blank,[0,0])
        self.assertEqual(r['state'],'SEARCH');self.assertIsNone(r['estimate'])
    def test_reacquisition_requires_confirmation(self):
        c=EyeController()
        for _ in range(20):c.update(np.zeros((240,320,3),np.uint8),[0,0])
        self.assertEqual(c.update(frame(),[0,0])['state'],'ACQUIRE')
    def test_rate_and_angle_bounds(self):
        c=EyeController();previous=np.zeros(2)
        for _ in range(100):
            command=c.update(frame(10,10),[1.3,.7])['command']
            self.assertTrue(np.all(command<=LIMIT_HIGH));self.assertTrue(np.all(command>=LIMIT_LOW))
            self.assertTrue(np.all(np.abs(command-previous)<=MAX_RATE*.04+1e-12));previous=command
    def test_update_does_not_mutate_pixels_or_encoders(self):
        image=frame();before=image.copy();joints=np.array([.1,.1]);EyeController().update(image,joints)
        np.testing.assert_array_equal(image,before);np.testing.assert_array_equal(joints,[.1,.1])
    def test_invalid_time_or_encoders(self):
        for dt in (0,-1,1):
            with self.assertRaises(ValueError):EyeController().update(frame(),[0,0],dt)
        with self.assertRaises(ValueError):EyeController().update(frame(),[math.nan,0])
    def test_far_distractor_rejected_while_tracking(self):
        c=EyeController()
        for _ in range(4):c.update(frame(80,120),[0,0])
        r=c.update(frame(300,120),[0,0]);self.assertIsNone(r['detection']);self.assertEqual(r['state'],'COAST')
    def test_controller_has_no_simulator_input(self):
        self.assertEqual(list(inspect.signature(EyeController.update).parameters),['self','rgb','joints','dt'])
        source=inspect.getsource(__import__('eye_lab.controller',fromlist=['x']))
        self.assertNotIn('mujoco',source);self.assertNotIn('mocap',source)

class ModelTests(unittest.TestCase):
    def test_two_physical_joints_with_bounded_actuators(self):
        import mujoco
        model=mujoco.MjModel.from_xml_path(str(Path(__file__).resolve().parents[1]/'eye_lab/scene.xml'))
        self.assertEqual(model.nq,2);self.assertEqual(model.nu,2);self.assertTrue(np.all(model.actuator_ctrllimited))
        np.testing.assert_array_equal(model.actuator_forcerange,[[-4,4],[-4,4]])
    def test_motor_command_changes_joint_angle(self):
        import mujoco
        model=mujoco.MjModel.from_xml_path(str(Path(__file__).resolve().parents[1]/'eye_lab/scene.xml'));data=mujoco.MjData(model)
        data.ctrl[:]=[.3,.2];mujoco.mj_step(model,data,nstep=200)
        self.assertGreater(data.qpos[0],.1);self.assertGreater(data.qpos[1],.05)

if __name__=='__main__':unittest.main()
