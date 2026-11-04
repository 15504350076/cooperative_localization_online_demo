import unittest
import numpy as np
from evaluate_anchor_layout import initialize_peer_only, rotate
from evaluate_rte import EDGES


class PeerInitializationTest(unittest.TestCase):
    def test_motion_recovers_relative_frame_without_anchor(self):
        t=np.arange(601)*.1;p0=np.array([[0.,0.],[4.,0.],[-3.,8.]])
        d=np.zeros((len(t),3,2))
        for n in range(3):d[:,n,0]=(n+1)*.04*t;d[:,n,1]=(n+1)*.0005*t*t
        angle=np.array([.5,-.3,.8]);p=p0+d
        ranges=np.column_stack([np.linalg.norm(p[:,i]-p[:,j],axis=-1) for i,j in EDGES])
        got,theta,detail=initialize_peer_only(rotate(d,-angle),ranges,t,.02)
        np.testing.assert_allclose(got,p[-1],atol=1e-6);np.testing.assert_allclose(theta,angle,atol=1e-6)
        self.assertEqual(detail['jacobian_rank'],6);self.assertEqual(detail['anchor_measurements_used'],0)

    def test_static_distances_do_not_recover_local_headings(self):
        t=np.arange(601)*.1;p=np.array([[0.,0.],[4.,0.],[-3.,8.]])
        r=np.tile([np.linalg.norm(p[i]-p[j]) for i,j in EDGES],(len(t),1))
        _,_,detail=initialize_peer_only(np.zeros((len(t),3,2)),r,t,.1)
        self.assertEqual(detail['jacobian_rank'],3)

    def test_reject_anchor_measurements(self):
        with self.assertRaises(AssertionError):initialize_peer_only(np.zeros((601,3,2)),np.ones((601,6)),np.arange(601)*.1,.1)


if __name__=='__main__':unittest.main()
