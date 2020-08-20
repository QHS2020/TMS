# -*- coding: utf-8 -*-

from RequiredModules import *


class FD():
    """
    Fundamental diagram of the lane

    """
    qm = 1800.0  # veh/h
    km = 45.0  # veh/km
    kj = 135.0  # veh/km
    vf = 40.0
    w = 20.0  # km/h

    def __init__(self, qm=1800.0, kj=135.0, km=45.0, DegradationCoefficient=1.0):
        """
        qm is 1800 	veh/h
        km is 45	veh/km
        kj is 135	veh/km
        """
        # channelized section capacity degradation coefficient.
        # DegradationCoefficient will influnce:
        #	- self.qm
        #	- self.w
        #
        self.DegradationCoefficient = DegradationCoefficient

        self.km = km  # veh/km
        self.kj = kj  # veh/km
        # to m/s  ------> self.FD.w*1000.0/3600
        self.vf = 1.0*self.qm/self.km  # unit in

        self.qm = qm*DegradationCoefficient*1.0  # veh/h
        self.w = -1.0*qm*DegradationCoefficient / \
            (self.kj - qm*DegradationCoefficient/self.vf)  # note the minus

    def vd(self):
        # the velocity of correspond to maximal flow after capacity degradation
        #	from the geometric relationship
        #	equal to (DegradationCoefficient*qm)/(kj-(DegradationCoefficient*qm)/w)
        return (self.DegradationCoefficient*self.qm)/(self.kj-(self.DegradationCoefficient*self.qm)/abs(self.w))
        # return	self.DegradationCoefficient*self.vf

    def stoppingwave(self, q):
        """
        Given upstream flow rate, Get the stopping wave speed. 
        unit is m/s, RATHER THAN km/h.
        """

        return 1.0*q/(q/self.vf - self.kj) * 1000/3600

    def Degradation(self):
        """
        When the channelzed section spillover happens, the 'capacity' is
        reduced by self.qm*self.DegradationCoefficient. 




        """

        pass

    def set_vf(self, ):
    	"""

    	"""


    	pass

    def Get_q_From_k(self, k=0):
        """

        """
        if k < 0 or k > self.kj:
            raise ValueError('k shoube be in [0,k_jam]')

        if k <= self.km:
            return self.vf*k
        else:
            return self.w*k-self.w*self.kj
