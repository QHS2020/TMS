# -*- coding: utf-8 -*-
#from GeneralClasses import GeneralDiscreteIntervals, GeneralContinuousIntervals
"""


"""
from RequiredModules import *
from LaneFD import FD

class GeneralDiscreteIntervals():
	"""
	Implementation of Discrete intervals sets, typically red time
	each red is a interval with both sided closed, such as [30,60]
	which means the red time is from 30s to 60s. 
	
	"""
	

	
	def idx_rgp(self,t):
		"""
		Given moment t (scalar), determine the idx of the rgp which the t is in. 
		For a red signal (s,s+r), this rgp is (s,s+r,s+r+g). Then if
		t in [s,s+r+g) then return this idx. 
		
			If t is smaller than time domain, t < self.StartMomentsOfReds[0].
				raiseError
			If t is greter than the maximalmoment, it does not matter, since rightside of the time domain, the signal is assumed as green. 
		
		If you want to get the interval instance, you can use::
			sorted(self.reds)[returnned_value]
		-----------------------------------------------------------------
		Input: scalar 
			moment
		Output:
			idx, start from 0.
		
		"""
		
		if t < self.StartMomentsOfReds[0]:
			raise ValueError('t shall not be smaller than maximal moemnt')
		else:
			return np.searchsorted(self.StartMomentsOfReds, t, side='right')-1
	
	def ExtendForward(self):
		"""
		
		
		"""
		
		
		pass
	
	
	
	
	
	def __init__(self):
		"""

		"""
		#General (Major) property of the GeneralDiscreteIntervals class.
		#Note that it is closed at both sides. 
		#So if spillover, it is easier to calculate. 
		self.reds		= pyinter.IntervalSet()
		#the meaning is different from the same property in class GeneralContinuousIntervals
		#	
		self.TimeIntervals	=	pyinter.IntervalSet()
		
		#AffiliatedProperties.
		#	the start and end moments of all reds
		#	length is the same as self.reds
		self.StartMomentsOfReds 	= np.array([])#unit is second
		self.EndMomentsOfReds		= np.array([])#unit is second
		
	def refreshAfterTimeIntervalsChanged(self):
		"""
		OK
		After major properties are changed(self.reds), other affiliated variables should be changed either. 
		Here major property is self.reds
		
		Affiliated properties include:
			StartMomentsOfReds
			EndMomentsOfReds
		--------------------------------------
		Steps:
			based on the self.reds, change the self.StartMomentsOfReds and self.EndMomentsOfReds
		
		"""
		#Changed the self.StartMomentsOfReds and self.EndMomentsOfRed
		self.StartMomentsOfReds		=	np.array([])
		self.EndMomentsOfReds		=	np.array([])
		for red in sorted(self.reds):
			self.StartMomentsOfReds=np.append(self.StartMomentsOfReds,np.array([red.lower_value]))
			self.EndMomentsOfReds=np.append(self.EndMomentsOfReds,np.array([red.upper_value]))
		
		self.ToContinuousIntervals()
		
	def ToContinuousIntervals(self):
		"""
		Convert the discrete self.reds to continuous TimeIntervals
		Before the converting, self.reds are pyinter.IntervalSet, each is a red time domain, and is closed in both sides. 
		
		Output: no output
			the self.TimeIntervals is changed.
			Note that len(self.TimeIntervals) is  2*reds_number - 1
			Because the last green is not included
			self.TimeIntervals is r-g-r-g-r-g-r.....-g-r (the last one is r, not g.)
		"""
		self.TimeIntervals	=	pyinter.IntervalSet()
		#add each red and green, except the last red time interval
		#Note that all intervals are all OPEN at both sides. 
		for idx in range(len(self.StartMomentsOfReds)-1):
			tmp1 = pyinter.interval.open(self.StartMomentsOfReds[idx],self.EndMomentsOfReds[idx])
			tmp2 = pyinter.interval.open(self.EndMomentsOfReds[idx],self.StartMomentsOfReds[idx+1])
			self.TimeIntervals.add(tmp1)
			self.TimeIntervals.add(tmp2)
		#Add the last red
		tmp=pyinter.interval.open(self.StartMomentsOfReds[-1],self.EndMomentsOfReds[-1])
		self.TimeIntervals.add(tmp)
		
		#return TimeIntervals#pyinter.IntervalSet type data.
		
				
class GeneralContinuousIntervals():
	

	
	def __init__(self):
		"""

		"""
		#Major properties. If they are changed, affiliated properties need to be refreshed
		self.TimeIntervals	=	pyinter.IntervalSet()#normally unit in seconds
		self.Demand			=	sllist()#normally unit in veh/h
		
		#Belows are ALL AFFILIATED variables, need to e refreshed. 
		#overall vehicle numbers currently at self.Demand, scalar.
		self.OverallVehicleNumbers = 0
		#all moments of the above TimeIntervals, including lowervalues and uppervalues
		#	this property is used to accelerate index of t in all intervals
		#	thus can determine the locale in self.Demand
		self.MinimalMoment			=	None#scalar
		self.MaximalMoment			=	None
		self.OverallMoments			=	[]
		#lower and upper values for all intervals
		#can be sorted as sorted(self.LowerValues)
		self.LowerValues				=	set()
		self.UpperValues				=	set()
		
		#unit is in seconds, np.array, 1-dimentional. 
		self.IntervalLength			=	None
	
	def Reset_TimeIntervals_by_OverallMoments(self):
		"""
		This method reconstruct the self.TimeIntervals from self.OverallMoments
		which is equivalent to sorted(self.LowerValues.union([self.MaximalMoment]). 
		
		The length of self.OverallMoments should be equal to self.Demand.size+1. 
		
		
		"""
		self.TimeIntervals = pyinter.IntervalSet()
		
		for idx in range(len(self.OverallMoments)-1):
			s=self.OverallMoments[idx]
			e=self.OverallMoments[idx+1]
			self.TimeIntervals.add(pyinter.interval.open(s,e))
			
		if len(self.TimeIntervals) != self.Demand.size:
			print('TimeIntervals------'+str(len(self.TimeIntervals))+'Demand------'+str(self.Demand.size))
			raise ValueError('self.TimeInterval length is not equal to self.Demand.')
	
	def ResetData(self,moments,cumu_N, qm=FD.qm):
		"""
		reset the data given the moments and cumu_N
		--------------------------------------------------
		Input:
			moments and cumu_N, both np.array, 1-dimentional, shape must be the same , i.e. (N,)
			moments unit is seconds
			cumu_N unit is veh, for example 200 means 200 vehicles
		Output:
			no output, the self.Demand and self.TimeIntervals are changed
		
		"""
		if len(moments)<=1:
			raise ValueError('Moments must be at least 2 values')
		if len(moments) != len(cumu_N):
			raise ValueError('length of moments and cumu vehicle numbers should be the same')
		
		tmpTimeIntervals	=	pyinter.IntervalSet()#normally unit in seconds
		tmpDemand			=	sllist()#normally unit in veh/h	
		
		for idx in range(len(moments)-1):
			s=moments[idx]
			e=moments[idx+1]
			flowrate=1.0*(cumu_N[idx+1]-cumu_N[idx])/((e-s)/3600.0)
			
			tmpTimeIntervals.add(pyinter.interval.open(s,e))
			tmpDemand.appendright(min([qm,flowrate]))
			
		self.Demand = tmpDemand
		self.TimeIntervals = tmpTimeIntervals
		self.refreshAfterTimeIntervalsChanged()
	
	def idx_Left(self,t):
		"""
		find the idx of the moment t in self.TimeIntervals
		t is a scalar, or a 1-dimentional array. 
		
		t should be within the time domain
		
		For a interval (a,b), it is transfered as [a,b) and then if t in [a,b), return idx
		
		"""
		if t < self.MinimalMoment or t >= self.MaximalMoment:
			raise ValueError('t shall be within the time domain')
		
		if t==self.MaximalMoment:
			raise ValueError('t shall be smaller than maximal moemnt')
		else:
			return np.searchsorted(np.array(sorted(self.UpperValues)), t, side='right')
	
	def idx_Right(self,t):
		"""
		find the idx of the moment t in self.TimeIntervals
		t should be within the time domain, otherwise an error raises. 
		
		For a interval (a,b), it is transfered as (a,b] and then if t in (a,b], return idx
		
		"""
		if t <= self.MinimalMoment or t > self.MaximalMoment:
			raise ValueError('t shall be within the time domain')
		
		if t==self.MaximalMoment:
			#the idx is the maximal one. 
			return len(self.UpperValues)
		else:
			return np.searchsorted(np.array(sorted(self.UpperValues)), t, side='left')
	
	def idx_openopen(self,t):
		"""
		find the idx of moment t in self.TimeIntervals
		Because each time interval is OPENED at both sides, i.e. (m,n)
		t should be not in boundary of self.TimeIntervals. 
			....
			
		
		"""
		#self.DemandSplitTime(set([t]))
		if t<=self.MinimalMoment or t>=self.MaximalMoment:
			raise ValueError('t shall be within the time domain and not boundary.')
		if t in set(self.LowerValues):
			raise ValueError('t shall not be interval boundary.')
		
		return np.searchsorted(self.UpperValues, t)
	
	def idx_Moment_openclosed(self,t):
		"""
		each OPENED time interval (s,e) is transformed to (s,e]
		"""
		
		for idx,timeinterval in enumerate(sorted(self.TimeIntervals)):
			if t in pyinter.interval.openclosed(timeinterval.lower_value,timeinterval.upper_value):
				return idx
	
	def idx_Moment_closedopen(self,t):
		"""
		each OPENED time interval (s,e) is transformed to [s,e)
		"""
		#self.DemandSplitTime(set([t]))
		
		for idx,timeinterval in enumerate(sorted(self.TimeIntervals)):
			if t in pyinter.interval.closedopen(timeinterval.lower_value,timeinterval.upper_value):
				return idx
	
	def CumuN2Moment(self, N):
		"""
		Get the moment when cunumative vehicle number is N.
		If N out of scope, raise Error. 
		
		If there are many moments correspond to a N, then return the first
		-----------------------------------
		Input: N
			scalar
		Output:t
			scalar
		
		
		"""
		#cumuN
		cumuN = self.VehiclesNumber()
		
		#remain 6 digits due to interploation error.
		precision = 6
		N = 1.0 * int(N* 10**precision)/(10**precision)
		
		if N<cumuN[1,0] or N>cumuN[1,-1]:
			raise ValueError('Cumulative N out of scope. N is --->', N,'[',cumuN[1,0],cumuN[1,-1],']')
		
		#q-dimentional np.array.
		deltaN = cumuN[1,:] - N
		
		#If in deltaN there is some zero value, then find the idx and return
		#	If no zero, then find the interface between positive and negative 
		#	interploate it. 
		idx = np.nonzero(deltaN==0)[0]#nonzero() return tuple, each element correspond to 1 dimention.
		#means that deltaN has zero value. len(idx) is number of zero values.
		if len(idx)>0:
			return cumuN[0,idx[0]]
		
		#no zero value, means that there are positive and negtive values
		#	find the 1-st negative value index---->idx
		#	then interploate.
		idx = np.nonzero(deltaN>0)[0][0]
		return np.interp(0, [deltaN[idx-1], deltaN[idx]], [cumuN[0,idx-1], cumuN[0,idx]])
	
	

	def ExtendDemandBackward(self,t=3600):
		"""
		extend the demand backward by t.
		default is 3600 seconds, i.e. 2 hours.
		After the sxtension, the time intervals and Demand is changed. 
		
		Input: t
			type is float, means the length of the extension.
		output:
			no output, but the time intervals and Demand is changed. 
		"""
		if t <= 0:
			raise ValueError('t should be positive')
		
		#Extend time intervals
		self.TimeIntervals.add(pyinter.interval.open(self.MinimalMoment-t,self.MinimalMoment))
		#Extend the demand as zero
		self.Demand.appendleft(0)
		
		#
		self.refreshAfterTimeIntervalsChanged()
		


	def ExtendDemandForward(self,t=3600):
		"""
		extend the demand forward by t.
		default is 3600 seconds, i.e. 2 hours.
		After the sxtension, the time intervals and Demand is changed. 
		
		Input: t
			type is float, means the length of the extension.
		Output: 
			no output, but self.TimeIntervals and self.Demand are changed
		"""
		#Extend time intervals
		self.TimeIntervals.add(pyinter.interval.open(self.MaximalMoment,self.MaximalMoment+t))
		#Extend the demand as zero
		self.Demand.appendright(0)
		
		self.refreshAfterTimeIntervalsChanged()
	
	def PickUp_moments(self,s,e):
		"""
		Get the moments with in [s,e].
		returning value include s and e. 
		The method does not change self. 
		
		If (s,e) are outside the time domain, then raise Error. 
		"""
		if s >= e:
			raise NameError('s should be smaller than e')
		if s >=self.MaximalMoment or e<=self.MinimalMoment:
			raise ValueError('s and e outside the Demand time domain')
		
		Moments	=	set()
		#tmp1 is all the moments satisfy e=<moment<=e in LowerValues.
		tmp = [t for t in self.OverallMoments if t<=e and t>=s]
		
		Moments.update( tmp )
		Moments.update( set([s,e]) )
		
		return Moments
	
	def ShiftTimeIntervals(self,t=0.0):
		"""
		shift the time domain by t. 
		i.e. each interval (s,e) becomes (s+t,e+t). t can be negative. 
		
		Other properties does not change. 
		
		"""
		
		tmp	=	pyinter.IntervalSet()
		for interval in sorted(self.TimeIntervals):
			s	=	interval.lower_value + t
			e	=	interval.upper_value + t
			tmp.add(pyinter.interval.open(s,e))
		self.TimeIntervals = tmp
		self.refreshAfterTimeIntervalsChanged()
		
	def Backup_DemandSplitTime(self, t=set()):
		"""
		Split the demand based on the self.TimeInterval and the input t.
		if the t is equal to lowervalue or uppervalue, then the function 
		will not change the self.Demand. 
		Otherwise the demand will be splited. 
		for example, t is in (t1,t2); and the correspond demand is d,
		then after the split, it becames:
		(t1,t):d and (t,t2):d. 
		If some moment in t is smaller than minimal moment in self.TimeIntervals
		then the method ExtendDemandBackward is called. 
		Vice versa, the method of ExtendDemandFOrward is called. 
		This method is used to overlap various route demand together.
		The processes include:
			- Extend backward or forward if needed
			- For each monent, test split is needed
			- Split
		
		Input: t
			set of moments (each moment is float), each moment is a float. 
		
		Output:
			no output, but just the Demand and self.TimeIntervals may be changed
		
		"""
		
		#t is smaller than the minimal time horizon, extend the demand
		if min(t)<self.MinimalMoment:
			self.ExtendDemandBackward(t=self.MinimalMoment-min(t))
		#t is bigger than the maximal time horizon, extend the demand
		if max(t)>self.MaximalMoment:
			self.ExtendDemandForward(t=max(t)-self.MaximalMoment)
		
		for moment in t:
			#if moment is NOT IN self.TimeIntervals, there is NO NEED to split
			#since each interval is OPEN both sides, it is necessary
			#if True, means that moment is either in LowerValues or in UpperValues
			if moment in self.LowerValues:continue
			if moment==self.MaximalMoment:continue
			
			#step 1: fine the index, moment should be inserted before the index to maintain order
			idx	=	np.searchsorted(np.array(sorted(self.UpperValues)), moment, side='left')
			#Step 2: change the properties
			self.TimeIntervals=self.TimeIntervals.difference(pyinter.interval.closed(moment,moment))
			#split the demand, index in the sllist is idx
			#just need to append the same value before node self.Demand.nodeat(idx)
			#insertafter(x,node) insert x after node
			self.Demand.insertafter(self.Demand[idx], self.Demand.nodeat(idx))
			
			#because this "moment" insert just use the self.UpperValues, thus here it is updated
			#other affilizated values are updated after a while (last line)
			self.UpperValues.update([moment])
			
		#change all other affiliated variables
		self.refreshAfterTimeIntervalsChanged()
		
	
	
	def DemandSplitTime(self, t=set()):
		"""
		Split the demand based on the self.TimeInterval and the input t.
		if the t is equal to lowervalue or uppervalue, then the function 
		will not change the self.Demand. 
		Otherwise the demand will be splited. 
		for example, t is in (t1,t2); and the correspond demand is d,
		then after the split, it becames:
		(t1,t):d and (t,t2):d. 
		If some moment in t is smaller than minimal moment in self.TimeIntervals
		then the method ExtendDemandBackward is called. 
		Vice versa, the method of ExtendDemandFOrward is called. 
		This method is used to overlap various route demand together.
		The processes include:
			- Extend backward or forward if needed
			- For each monent, test split is needed
			- Split
		
		Input: t
			set of moments (each moment is float), each moment is a float. 
		
		Output:
			no output, but just the Demand and self.TimeIntervals may be changed
		
		"""
		
		#t is smaller than the minimal time horizon, extend the demand
		if min(t)<self.MinimalMoment:
			self.ExtendDemandBackward(t=self.MinimalMoment-min(t))
		#t is bigger than the maximal time horizon, extend the demand
		if max(t)>self.MaximalMoment:
			self.ExtendDemandForward(t=max(t)-self.MaximalMoment)
		
		#this iteration just change the self.OverallMoments and self.Demand, 
		#	after the interation, the self.TimeIntervals need to be re-constructured 
		#	from self.OverallMoments. 
		for moment in t:
			#if moment is NOT IN self.TimeIntervals, there is NO NEED to split
			#since each interval is OPEN both sides, it is necessary
			#if True, means that moment is either in LowerValues or in UpperValues
			if moment in self.OverallMoments:continue
			
			#step 1: fine the index, moment should be inserted before the index to maintain order
			idx	=	bisect_left(self.OverallMoments, moment)
			#Step 2: change the properties, insert moment before idx
			self.OverallMoments.insert(idx, moment)
			#	split the demand, index in the sllist is idx
			#	just need to append the same value before node self.Demand.nodeat(idx)
			#	insertafter(x,node) insert x after node
			#	note the idx -1 .
			_=self.Demand.insertafter(self.Demand.nodeat(idx-1).value, self.Demand.nodeat(idx-1))
		
		#Reset the self.TimeIntervals from self.OverallMoments
		self.Reset_TimeIntervals_by_OverallMoments()
		
		#change all other affiliated variables
		self.refreshAfterTimeIntervalsChanged()
	
	def VehiclesNumberFrom_t(self, a):
		"""
		Compute the cumulative vehicle number from moment a
		if a is out of the time domain, then the extend method is called
		Thus the resulting self.TimeIntervals always cover a. 
		
		Input:a
			moment, is a Scalar. 
		
		OutPut: (deltaT_cumun, momemt_cumu)
			deltaT_cumun
				np.array, shape is (2,N), 1st row is deltaT, 2nd row is cumulative number
				1st element at 1st row should be zero, means no time increment.
				For example, if 2nd element at 1st row is 5 sec, and 2nd element at 2nd row is 10, it means that 5 sec from the moment of a, the cumulative number is 10 vehicles. 
				
			momemt_cumu
				np.array, shape is (2,N), 1st row is absolute moment, 2nd row is cumulative number. 
		
		"""
		
		#this will make the self.TimeIntervals cover the moment a.
		#	also the self.refreshAfterTimeIntervalsChanged() is called. 
		#	thus self.LowerValues and self.UpperValues are updated. 
		self.DemandSplitTime(set([a]))
		
		#cumulative vehicle number as (2,N) np.array, N=len(self.UpperValues)+1
		#	1-st row is the absolute moment
		#	it is assumed that there is no repeation of moments in 1st row.
		cumunumber	=	self.VehiclesNumber()
		
		#find the index of a in the 1st row
		#	np.where return tuple, (t1,t2) each is a np.array
		idx			=	np.where(cumunumber[0,:]==a)[0][0]
		#Vehicle number at moment a
		vehiN		=	cumunumber[1,idx]
		#return tuple
		#	1st element is deltaT from a versus cumulative number
		#	2nd element is moment from a versus cumulative number
		#	the 2nd row of each tuple element should be the same
		return np.array([cumunumber[0,idx:]-a, cumunumber[1,idx:]-vehiN]),np.array([cumunumber[0,idx:], cumunumber[1,idx:]-vehiN])
		
	def VehiclesNumber1(self):
		"""
		TESTED_with small error for vehicle number
		
		Get vehicles number for each time interval
		The sequence is very important.
		
		OutPut:
			vehiclesN, list, length is (self.Demand.size,)
			each time interval has a vehicle number value in returned value. 
		"""
		return list(1.0*self.IntervalLength  * np.array(list(self.Demand))/3600.0)
		
		
		#the following is without bug, but the efficiency is not assured. 
		vehiclesN = []
		for deltat,demand in zip(sorted(self.TimeIntervals),self.Demand):
			#unit is second
			timelength=deltat.upper_value-deltat.lower_value
			#unit is vehicles 
			vehiclesN.append(timelength*1.0/3600*demand)
		return vehiclesN
	
	def VehicleNumberAt_t(self,t):
		"""
		Cumulative number at moments t
		if t is greater than self.MaximalMoment, then maximal vehicle number is returned.
		-----------------------------
		input:
			t: scalar or 1-dimentional array
				if it is a scalar, then it will be transformed to  1-dimentional array
			
		----------------------------
		Output: np.array
		"""
		
		xp=self.VehiclesNumber()[0,:]
		yp=self.VehiclesNumber()[1,:]
		t = np.asarray(t).reshape(1, -1)[0,:]		
		res = np.interp(t,xp,yp,left=yp[0],right=yp[-1])
		
		#numpy.clip(a, a_min, a_max, out=None)
		#	Clip (limit) the values in an array.
		return np.clip(res, yp[0], yp[-1])
	
	def VehiclesNumber(self):
		"""
		Get the vehicles number of the whole time domain
		
		Output: cumu, (2,N) shape np.arrarys.
			N=len(self.UpperValues)+1
			1st row is absolute moment
			2nd row is cumu vehicle number at that moment. 
		"""
		
		#use the following style to ignore some return values:
		#	 _,_=self.VehiclesNumber()
		
		#vehicles number, 0 means before the time domain, it is zero
		tmp1 = [0] + self.VehiclesNumber1()
		#moments
		tmp2 = [self.MinimalMoment] + list(sorted(self.UpperValues)) 
		
		#1st row is moments; 2nd row is cumulative vehicle numbers
		return np.array([tmp2,np.cumsum(tmp1)])
	
	def RescaleRouteDemand_by_spillover(self,s,e, qm=FD.qm):
		"""
		chane the self.Demand and self.TimeIntervals due to spillover which are indicated by the 
		duration (s,e). if (s,e) is outside the time domain, then nothing changed. 
		
		Steps
			1) operaing the (s,e) interval, recording the remaining vehicle number
			2) operating the interval after (s,e)
		"""
		
		if e<=s:
			raise ValueError('e shoube be exactly greater than s.')
		if e>=self.MaximalMoment or s<=self.MinimalMoment:
			#no need to operate this method since the spillover outside the time domain.
			#this may happen when multiple input for downstream spillover lane. 
			return
		
		#confine the operation within the time domain.
		s=max([self.MinimalMoment,s])
		e=min([self.MaximalMoment,e])
		
		self.DemandSplitTime(set([s,e]))
		
		#idx of time inverval which moment s is in in self.TimeIntervals. 
		idx_s=self.idx_Left(s)
		idx_e=self.idx_Right(e)#e must not be the last in self.TimeIntervals
		
		#demand within the spillover interval.
		remainingvehiclesnumber = 0.0
		for idx in range(idx_s,idx_e+1):
			interval =	sorted(self.TimeIntervals)[idx]
			timelength = 1.0*(interval.upper_value-interval.lower_value)
			#compute the blocked vehicle number in this interval
			remainingvehiclesnumber += self.Demand.nodeat(idx).value*timelength/3600.0
			#blocked, thus set to zero.
			self.Demand.nodeat(idx).value=0.0
		
		#idx_e is not in the last time interval (i.e. e != self.MaximlMoment)
		#	then run until the lase time interval.
		if idx_e < self.Demand.size-1:
			for idx in range(idx_e+1,self.Demand.size):
				#means all residule vehicles have been considered, no need to continue
				if remainingvehiclesnumber==0.0:
					#no need to refresh either.
					return
				#interval =	sorted(self.TimeIntervals)[idx]
				#timelength = 1.0*(interval.upper_value-interval.lower_value)
				timelength=self.IntervalLength[idx]
				residula_q = 3600.0*remainingvehiclesnumber/timelength
				#still some vehicles remaining. 
				if self.Demand.nodeat(idx).value+residula_q > qm:
					remainingvehiclesnumber = (self.Demand.nodeat(idx).value+residula_q - qm)*timelength/3600
					self.Demand.nodeat(idx).value = qm
				#all vehicles before this time interval have been contained, with q smaller than qm
				else:
					self.Demand.nodeat(idx).value = self.Demand.nodeat(idx).value+residula_q
					remainingvehiclesnumber = 0.0
		
		#there is still some vehicles left outside the original time domain. 
		if remainingvehiclesnumber>0.0:
			extend_time = remainingvehiclesnumber/qm*3600.0#in seconds
			self.TimeIntervals.add(pyinter.interval.open(self.MaximalMoment,self.MaximalMoment+extend_time))
			#Extend the demand as zero
			self.Demand.appendright(qm)
		
		self.refreshAfterTimeIntervalsChanged()
	
	
		
	def RescaleDemand_by_qm(self,qm=FD.qm):
		"""
		some time intervals, the flow rate may exceed the qm, thus need to be rescaled. 
		The remaining vehicles will be retained to next time interval. 
		
		"""
		remainingvehiclesnumber = 0.0
		
		
		for interval,idx in zip(sorted(self.TimeIntervals),range(self.Demand.size)):
			timelength = 1.0*(interval.upper_value-interval.lower_value)
			residula_q = 3600.0*remainingvehiclesnumber/timelength
			if self.Demand.nodeat(idx).value+residula_q > qm:
				remainingvehiclesnumber = (self.Demand.nodeat(idx).value+residula_q - qm)*timelength/3600
				self.Demand.nodeat(idx).value = qm
			else:
				self.Demand.nodeat(idx).value = self.Demand.nodeat(idx).value+residula_q
				remainingvehiclesnumber = 0.0
		
		#there is still some vehicles left outside the time domain. 
		if remainingvehiclesnumber>0.0:
			
			extend_time = remainingvehiclesnumber/qm*3600.0#in seconds
			self.TimeIntervals.add(pyinter.interval.open(self.MaximalMoment,self.MaximalMoment+extend_time))
			#Extend the demand as zero
			self.Demand.appendright(qm)
			self.refreshAfterTimeIntervalsChanged()
	
	
		
	def Check_Consistency(self):
		"""
		check 
			* the time moments of self.Demand and self.TimeIntervals are aligned or not. 

		If not, raise ValueError
		
		"""
		
		if len(self.TimeIntervals) != self.Demand.size:
			raise ValueError('number of time intervals is not equal to the demand sllist size')
	
	def refreshAfterTimeIntervalsChanged(self):
		"""
		since the self.TimeIntervals changed, other affiliated variables should be changed
		including:
			lower_values,
			upper_values,
			MinimalMoment,
			MaximalMoment
		
		"""
		#lower_values and upper_values
		self.LowerValues = set([interval.lower_value for interval in self.TimeIntervals])
		self.UpperValues = set([interval.upper_value for interval in self.TimeIntervals])
		
		#minimal and maximal moment of the instance time domain
		self.MinimalMoment=sorted(self.TimeIntervals)[0].lower_value
		self.MaximalMoment=sorted(self.TimeIntervals)[-1].upper_value
		
		self.IntervalLength = np.array(sorted(self.UpperValues))-np.array(sorted(self.LowerValues))
		
		#a list
		self.OverallMoments = sorted(self.LowerValues.union([self.MaximalMoment]))
		
		self.OverallVehicleNumbers	=	self.VehiclesNumber()[1,-1]
		
		if self.Demand.size != len(self.LowerValues):
			raise ValueError('zero interval length')
		
		
