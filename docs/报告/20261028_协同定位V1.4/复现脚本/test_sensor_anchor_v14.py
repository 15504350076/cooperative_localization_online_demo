"""检查新离线仿真的滤波计算与无误差极限，不触及运行中的算法。"""
import unittest
import numpy as np
import evaluate_sensor_anchor_v14 as sim


class AnchorSimulationTest(unittest.TestCase):
    def test_body_heading_is_tangent_and_stationary_start_is_continuous(self):
        velocity=np.array([[[0.,0.]],[[0.,1.]],[[1.,1.]],[[1.,0.]]])
        heading=sim.tangent_heading(velocity)
        np.testing.assert_allclose(heading[:,0],[np.pi/2,np.pi/2,np.pi/4,0])
        lateral=-velocity[:,:,0]*np.sin(heading)+velocity[:,:,1]*np.cos(heading)
        np.testing.assert_allclose(lateral,0,atol=1e-12)

    def test_scalar_measurement_update_matches_closed_form(self):
        x=np.array([[1.,2.]]);p=np.eye(2)[None];h=np.array([[1.,0.]])
        sim.update(x,p,h,np.array([-1.]),1.)
        np.testing.assert_allclose(x,[[.5,2.]])
        np.testing.assert_allclose(p,[np.diag([.5,1.])])

    def test_three_anchors_recover_positions_without_odometry(self):
        truth=np.array([[[15.,65.],[30.,65.],[20.,78.]]])
        surveyed=sim.ANCHORS[None]
        ranges=np.linalg.norm(truth[:,:,None,:]-surveyed[:,None,:,:],axis=-1)[:,None]
        got=sim.anchor_only(3,surveyed,ranges,truth+np.array([1.,-1.]))
        np.testing.assert_allclose(got[:,0],truth,atol=1e-8)

    def test_inertial_prediction_integrates_acceleration(self):
        times=np.arange(11)*.1;initial=np.zeros((1,3,2))
        acceleration=np.ones((1,11,6))
        data={'imu':{'acceleration':acceleration}}
        position,_=sim.fusion('imu',0,False,.05,data,initial,None,None,None,times)
        np.testing.assert_allclose(position[0,-1],.5,atol=1e-12)

    def test_position_metrics_use_relative_difference(self):
        truth=np.array([[[0.,0.],[1.,0.],[0.,1.]]])
        est=truth[None]+[3.,4.]
        metric=sim.metrics(est,truth)[0]
        self.assertEqual(metric['position_rmse_m'],0.)
        self.assertEqual(metric['field_position_rmse_m'],5.)


if __name__=='__main__':unittest.main()
