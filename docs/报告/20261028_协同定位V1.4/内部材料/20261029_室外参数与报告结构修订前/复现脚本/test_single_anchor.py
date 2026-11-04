"""核对单点初始化、测量更新和相对误差口径。"""
import inspect
import unittest
import numpy as np
import evaluate_single_anchor as sim


class SingleAnchorTest(unittest.TestCase):
    def test_measurement_only_initialization(self):
        t=np.arange(601)*.1;p0=np.array([[10.,0.],[14.,4.],[7.,8.]])
        d=np.zeros((len(t),3,2))
        for n in range(3):d[:,n,0]=(n+1)*.04*t;d[:,n,1]=(n+1)*.0005*t*t
        angles=np.array([.5,-.3,.8]);local=sim.rotate(d,-angles);p=p0+d
        ranges=np.column_stack([np.linalg.norm(p[:,i]-p[:,j],axis=-1) for i,j in sim.EDGES]+[np.linalg.norm(p[:,i],axis=-1) for i in range(3)])
        got,theta,detail=sim.initialize(local,ranges,t,.02)
        np.testing.assert_allclose(got,p[-1],atol=1e-6);np.testing.assert_allclose(theta,angles,atol=1e-6)
        self.assertLess(detail['range_fit_rmse_m'],1e-8)

    def test_single_anchor_jacobian_updates_radial_direction(self):
        x=np.array([[3.,4.]]);P=np.eye(2)[None];h=np.array([[.6,.8]])
        sim.scalar_update(x,P,h,np.array([1.]),1.)
        np.testing.assert_allclose(x,[[3.3,4.4]])
        np.testing.assert_allclose(P@np.array([[-.8],[.6]]),np.array([[[-.8],[.6]]]))

    def test_common_translation_cancels_in_relative_error(self):
        truth=np.array([[[0.,0.],[2.,0.],[0.,2.]]]);position=truth[None]+[3.,4.]
        result=sim.score(position,truth,0)[0]
        self.assertEqual(result['position_rmse_m'],0.);self.assertEqual(result['field_position_rmse_m'],5.)

    def test_estimator_does_not_accept_ground_truth(self):
        for f in (sim.initialize,sim.estimate):
            self.assertNotIn('truth',inspect.signature(f).parameters)
        self.assertEqual(sim.MODES,('start','peer','single'))


if __name__=='__main__':unittest.main()
