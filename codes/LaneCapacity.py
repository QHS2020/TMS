# -*- coding: utf-8 -*-

"""
This module contains the implementation of lane capacity
Especially the capacity of signal controlled lane. 
The capacity is dependent on the signal settings and 
down stream capcity degradation. 

If there is no spillover on downstream 



"""
#############################################################
from RequiredModules import *
#from Demand import RouteDemand, LaneDemand#demand class
from RGP import RGP#red-green class
from GeneralClasses import GeneralContinuousIntervals
from LaneFD import FD

class LaneCapacity(GeneralContinuousIntervals):
	
	#Subclassed from GeneralContinuousIntervals
	#thus have following properties:
		#TimeIntervals	=	pyinter.IntervalSet()
		#Demand			=	sllist()
		#MinimalMoment			=	None#scalar
		#MaximalMoment			=	None
		#LowerValues				=	set()
		#UpperValues				=	set()
		#IntervalLength			=	sllist()
		
	#the length of capacity (as a linked list) should be identical to 
	#the number of Intervals
	Capacity=sllist()
	
	def __init__(self):
		
		
		pass
		
	def RGP2Capacity(self,rgp,saturationflowrate=FD.qm):
		"""
		Based on the RGP to assign capacity
		default saturetion flow rate is the optimal flowrate of FD.
		when signal is red, the outflowcapacity is zero.
		when it is green, it is saturationflowrate
		
		Steps:
			1) clear the self.Capacity and self.TimeIntervals. 
			2) WRT each RGP interval, assign the red and green to self.TimeIntervals
				and self.Capacity as zero or saturationflow rate.
		
		
		INPUT
			1) rgp:instance of RGP class
				the major RGP class property is rgp.reds
			2) saturationflowrate: degraded capacity
		OutPUT
			no output. But the self.Capacity property is changed.
		
		"""
		
		#Clear the Capacity and TimeIntervals at the same time in order to
		#keep in accordance with RGP input
		#	Clear all nodes
		self.Capacity.clear()
		#	Update the self.Capacity and self.TimeIntervals together
		self.TimeIntervals	=	pyinter.IntervalSet()
		
		#add each red and green, except the last red time interval
		#Note that all intervals are all OPEN at both sides. 
		for idx in range(len(rgp.StartMomentsOfReds)-1):
			#tmp1 and tmp2 are red interval and greed interval respectively
			tmp1 = pyinter.interval.open(rgp.StartMomentsOfReds[idx],rgp.EndMomentsOfReds[idx])
			tmp2 = pyinter.interval.open(rgp.EndMomentsOfReds[idx],rgp.StartMomentsOfReds[idx+1])
			
			#capacity at red time is zero.
			self.TimeIntervals.add(tmp1);self.Capacity.appendright(0)
			#capacity at green time is saturationflowrate
			self.TimeIntervals.add(tmp2);self.Capacity.appendright(saturationflowrate)
		
		#Add the last red and related capacity(it is 0)
		tmp=pyinter.interval.open(rgp.StartMomentsOfReds[-1],rgp.EndMomentsOfReds[-1])
		self.TimeIntervals.add(tmp);self.Capacity.appendright(0)
		
	def SpilloverOperation(self,s,e,saturationflowrate=FD.qm):
		"""
		s and e are start moment and ending moment of the down stream spillover.
		e shoule be greater than s
		
		If the spillover is induced by the downstream channelized section spillover,
		then the saturation flowrate is degraded to given saturationflowrate1
		where saturationflowrate1<=FD.qm, dependent on the detailed settings.
		
		"""
		
		
		
	def ExtendCapacityBackward(self,t=3600,saturationflowrate=FD.qm):
		"""
		extend the capacity backward by t.
		default is 3600 seconds, i.e. 2 hours.
		After the sxtension, the time intervals and Demand is changed. 
		
		Input: t
			type is float, means the length of the extension.
		Input:saturationflowrate
			the capacity, dependent of other type factors
		output:
			no output, but the time intervals and Capacity is changed. 
		"""
		#Extend time intervals
		self.TimeIntervals.add(pyinter.interval.open(self.MinimalMoment-t,self.MinimalMoment))
		#Extend the demand as zero
		self.Capacity.appendleft(saturationflowrate)
		
		#sub classed from GeneralContinuousIntervals
		self.refreshAfterTimeIntervalsChanged()	
		
		
	def ExtendCapacityForward(self,t=3600, saturationflowrate=FD.qm):
		"""
		extend the Capacity forward by t.
		default is 3600 seconds, i.e. 2 hours.
		After the sxtension, the time intervals and capacity is changed. 
		
		Input: t
			type is float, means the length of the extension.
		Input:saturationflowrate
			the capacity, dependent of other type factors
		Output: 
			no output, but self.TimeIntervals and self.Capacity are changed
		"""
		#Extend time intervals
		self.TimeIntervals.add(pyinter.interval.open(self.MaximalMoment,self.MaximalMoment+t))
		#Extend the Capacity as saturationflowrate
		self.Capacity.appendright(saturationflowrate)
		
		#sub classed from GeneralContinuousIntervals
		self.refreshAfterTimeIntervalsChanged()
