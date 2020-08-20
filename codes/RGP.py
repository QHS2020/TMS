# -*- coding: utf-8 -*-

"""
This module contains the definition of so called RGP (red-green pair).



"""

from RequiredModules import *
#GeneralContinuousIntervals is a top level class for RGP
from GeneralClasses import GeneralDiscreteIntervals

class RGP(GeneralDiscreteIntervals):
    """
    reds is reazlized as a set of intervals in the pyinter package
        each red (<class 'pyinter.interval.Interval'>) 
        can be accessed using 'for' loop
        spillover behavior is rea
    """
    #initialization of all reds, note that it is CLOSED at both sides
    #it is already in the parent class GeneralDiscreteIntervals
    #reds       =pyinter.IntervalSet([pyinter.interval.closed(0,0)])
    

    
    def NextCycleStartMoment(self,indx_rgp):
        """
        determine the startmoment of next cycle
        if there is no successive cycle, then return np.inf
        
        """
        
        if indx_rgp+1 >= len(self.StartMomentsOfReds):  
            return np.inf
        else:
            return self.StartMomentsOfReds[indx_rgp+1]
    
    def Add_one_red(self,r_start,r_end):
        """
        
        """
        
        if r_start > r_end:
            raise ValueError('The start moment of the added red is greater than the end.')
        self.reds.add(pyinter.closed(r_start, r_end))
        self.refreshAfterTimeIntervalsChanged()
    
    def ClearAllredsAfter(self, t, retain_last = True):
            """   
            Clear all the reds after moment t. Except the last .
            
            It means that, all the red signals after momemt t will siwitch to green. 
            """
            lastred = sorted(self.reds)[-1]
            reds = self.reds.difference(pyinter.open(t,lastred.upper_value))
            if retain_last:reds.add(lastred)
            self.reds = reds
            self.refreshAfterTimeIntervalsChanged()
        
    def CurrentRed(self,idx):
        """
        return the current red signal length
        If idx is greater than len(self.StartMomentsOFReds)
        , then Error
        ---------------------------------------
        input:
            idx, integer, the index of the current RGP cycle, start
            from 0.
        
        
        ---------------------------------------
        
        
        
        
        """
        if idx > len(self.StartMomentsOfReds) or idx <0:
            print('The input index is ',idx,' and the number of cycles\n is ', len(self.StartMomentsOfReds))
            raise ValueError('idx is not ')
        
        return self.EndMomentsOfReds[idx]-self.StartMomentsOfReds[idx]
    
    def AddSignal(self,r_start,r_end,side='right'):
        """
        Add a signal, the red is described by the onset and ending moment respectively.
        ------------------------------------------
        Input:r_start, r_end
            both are scalar. 
        Output:
            no output, but the following are changed:
                self.StartMomentsOfReds
                self.EndMomentsOfReds
        -------------------------------------------
        
        """
        if r_start <= self.EndMomentsOfReds[-1]:
            raise ValueError('The start moment of the added red is less than the last signal ending moment.')
        
        self.StartMomentsOfReds = np.array(list(self.StartMomentsOfReds) + [r_start])
        
        self.EndMomentsOfReds = np.array(list(self.EndMomentsOfReds) + [r_end])
        
        
        
    
    def reset_signals(self, reds):
        """
        Input: reds
            list, red[i] = (start, end), the absolute moment of red signal
        
        """
            
        self.reds = pyinter.IntervalSet()
        for red in reds:
            tmp_red=pyinter.interval.closed(red[0],red[1])
            self.reds.add(tmp_red)
        
        self.StartMomentsOfReds     =   np.array([])
        self.EndMomentsOfReds       =   np.array([])
        for red in sorted(self.reds):
            self.StartMomentsOfReds=np.append(self.StartMomentsOfReds,np.array([red.lower_value]))
            self.EndMomentsOfReds=np.append(self.EndMomentsOfReds,np.array([red.upper_value]))
        
        
        #self.refreshAfterTimeIntervalsChanged()
    
    def __init__(self, r=60,g=60,o=0, T=360):
        """
        
        input: g
            scalar, green time in seconds
        input: r
            scalar, red time in seconds
        input: o
            scalr, offset to the begning, i.e. the start moment of the first
            red signal
        inuut: T
            scalr, the time horizon of RGP in seconds.
            Default the sum of all cycle equal to or bigger than 7200 seconds
            (2 hours)
        ---------------------------
        Initialized attributes:
            self.r, scalar
            self.o, scalar
            self.g, scalar
            self.StartMomentsOfReds, list
            self.EndMomentsOfReds, list
        """
        
        GeneralDiscreteIntervals.__init__(self)
        
        #####red time, green time and offset
        #   offset is the difference between the begining of red and the t=0 moment
        self.r      =   r
        self.g      =   g
        self.o      =   o
        
        #####set self.reds
        tmp_begning_red=o
        tmp_ending_red=tmp_begning_red+r
        #first red duration, note that the interval is closed both sides
        #Even the red signal length is zero
        tmp_red=pyinter.interval.closed(tmp_begning_red,tmp_ending_red)
        while tmp_ending_red<=T:
            self.reds.add(tmp_red)
            #begining and ending moments for the next red signal
            tmp_begning_red=tmp_ending_red+g
            tmp_ending_red=tmp_begning_red+r
            tmp_red=pyinter.interval.closed(tmp_begning_red,tmp_ending_red)
        
        #Change the self.StartMomentsOfReds AND self.EndMomentsOfRed
        self.refreshAfterTimeIntervalsChanged()
        
    def SpilloverIntervalAdd(self, s,e):
        """
        Under the case of spillover, reconstructure the RGP
        This function will change the red-series and green-series
        ------------------------------------------
        input: s
            scalar, the starting moment of the spillover
        input: e
            scalar, the ending moment of the spillover
            must satisfy e>s
        ------------------------------------------
        No output, the self.reds property will be changed
        """
        if e>=s:
            raise ValueError('end moment must greater than start moment')
        
        #Add new spillover intervals. The self.reds property will be changed. 
        NewRed      =   pyinter.interval.closed(s,e)
        self.reds.add(NewRed)
        
        self.refreshAfterTimeIntervalsChanged()
        
    def SignalState(self, t=0):
        """
        return the signal state at time t.
        if RED return False
        if GREEN return Ture
        
        """
        
        if t in self.reds:
            return False
        else:
            return True
        
    def SpilloverIntervalsInto(self, intervalset):
        """
        
        Input:intervalset
            pyinter.IntervalSet instance. 
            
        OUtput: 
            no output. self.reds will be changed. 
        """
        
        #note that need to be assigned, not like the add method, which just take a interval
        #   as input. 
        self.reds = self.reds.union(intervalset)
        self.refreshAfterTimeIntervalsChanged()
        
    
    def Overlap(self,AnotherRGP):
        """
        Overlap between two RGPs
            this is realized by union the two reds of the RGPs
        
        """
        self.reds = copy.deepcopy(self.reds.union(AnotherRGP.reds))
        
        #after self.reds are changed, refreshed all other necessary
        self.refreshAfterTimeIntervalsChanged()
        
    def ExtendForeward(self, T=3600):
        """
        When the time horizion exceeds the whole time domain of RGP, 
        then the whole RGP will be extended by 3600 seconds, 
        i.e. 2 hours of Green signal
        """
        
        self.refreshAfterTimeIntervalsChanged()
        pass
    
    
    def ExtendBackward(self,t=3600):
        """
        Extend the RGP backward by 3600 seconds, i.e. 1 hours
        ------------------------------
        Steps:
            - add a closed pyinter.interval into self.reds
            - refresh self.StartMomentsOfReds and self.EndMomentsOfReds
        """
        if t <= 0:
            raise ValueError('t should be positive')
        #min(self.reds).lower_value is the minimum value 
        #of all reds value(upbound and downbound)
        LowestValue =   self.StartMomentsOfReds[0]
        tmp =   pyinter.interval.closed(LowestValue-t,LowestValue-t)
        self.reds.add(tmp)
        
        self.refreshAfterTimeIntervalsChanged()
    
    def DeterminationRGP_cycle_index(self,t):
        """
        Determin which RGP pair is for the moment t.
        
        For the i-th RGP cycle, i.e. 
        
        """
        
        
        pass
    
    def NumberofCycles(self):
        """
        OK
        Number of cycles, each cycle is a RGP, note that the cycle where red is zero also is considered as a cycle
        """
        
        return len(self.reds)

    def Plot(self):
        """
        Plot all RGPs, together with Begining cycle indication
        return ax
        """
        
        #get time moments
        
        #get red moments
        
        #plot
        
        
        
        pass

    def GetMomentsRelativeToRed(self, t = 0):
        """
        Get the moments relative to red termination. 
        If need to be greater than red termination, t is positive, otherwise it should be negative. 
        """
        res = []
        for r in self.EndMomentsOfReds:
            res.append(r+t)
        return res

