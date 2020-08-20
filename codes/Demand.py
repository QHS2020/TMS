# -*- coding: utf-8 -*-

"""
This module contains the RouteDemand and LaneDemand that for a lane. 
Demand is realized by assigning a value (veh/hour) to each time duration. 
Within the duration the demand is stable. 

Time is absolute time, which means systime may not needed. 

"""

#from .RequiredModules import *

#from .LaneFD import FD
#from .LaneCapacity import LaneCapacity
#from .GeneralClasses import GeneralDiscreteIntervals,GeneralContinuousIntervals
from RequiredModules import *

from LaneFD import FD
#from .LaneCapacity import LaneCapacity
from GeneralClasses import GeneralDiscreteIntervals,GeneralContinuousIntervals


class RouteDemand(GeneralContinuousIntervals):
    """
    major property is TimeIntervals and Demand
    They are subclassed from GeneralContinuousIntervals
    
    Time intervals is realized in pyinter.IntervalSet()
    Demand is a LIST in order with respect to the intervals.
    So the length of Demand property and the number of the intervals are the same.
    During each Interval, the demand is stable.
    The time interval is absolute time, thus systime is not required. 
    
    """
    
    #note the open setting. The time is absolute time. 
    #   all the intervals should be open two sides. 
    #   The following properties already in parent class GeneralContinuousIntervals
    #   TimeIntervals   =   pyinter.IntervalSet()
    #   Demand          =   sllist(), in veh/h
    
    
    
    
    
    def CheckConsistency(self, tolerance = 0.00001):
        """
        different from the CheckConsistency2() in that, the numerical tolerance is permitted and revised.
        While in CheckConsistency2(), the 
        ------------------------------------------------------
        
        
        """
        print('=========================================================')
        #the self.ts shoule be increasing strictly!
        #make sure that the self.ts is strictly increasing
        if np.all(np.diff(self.ts) < 0):
            raise ValueError('self.ts is strictly decreasing!!')
        
        #eliminate the error. 
        self.PrecisionControl(precision=tolerance)
        
        if len(np.where(np.diff(self.ts)==0)[0])>0:
            builtins.ts = copy.deepcopy(self.ts)
            builtins.N = copy.deepcopy(self.N)
            raise ValueError('There are some identical self.ts!!!!')
        
        
        #revise the self.N
        if not np.all(np.diff(self.N) >= 0):
            if min(np.diff(self.N))>tolerance:
                raise ValueError('self.N is not increasing')
            #find the index of not increasing
            tmp_N = np.array(copy.deepcopy(self.N))
            index_es = np.where(np.diff(self.N)<0)[0]
            for i in index_es:
                tmp_N[np.where(tmp_N[:i+1]>tmp_N[i+1])] = tmp_N[i+1]
            self.raw_N1 = self.N
            self.N = list(tmp_N)
        """
        print('self.ts have ---->', len(self.ts),'  values')
        print('self.N have ---->', len(self.N),'  values')
        print('self.CCC have ---->', len(self.CCC),'  values')
        """
        
        if len(self.ts) != len(self.N) or len(self.ts) != len(self.CCC) or len(self.CCC) != len(self.N):
            raise ValueError('Not consistent!!!!!!!!')
        
        pass
    
    
    
    
    def CheckConsistency2(self):
        print('=========================================================')
        #the self.ts shoule be increasing strictly!
        #make sure that the self.ts is strictly increasing
        if not np.all(np.diff(self.ts) > 0):
            raise ValueError('self.ts is not strictly increasing')
        if not np.all(np.diff(self.N) >= 0):
            raise ValueError('self.N is not increasing')
        
        
        print('The self.ts is strictly increasing. The self.N is increasing!')
        print('self.ts have ---->', len(self.ts),'  values')
        print('self.N have ---->', len(self.N),'  values')
        print('self.CCC have ---->', len(self.CCC),'  values')
        if len(self.ts) != len(self.N) or len(self.ts) != len(self.CCC) or len(self.CCC) != len(self.N):
            raise ValueError('Not consistent!!!!!!!!')
        
    def ExtendCCCBack(self, T = 1000 , DeltaT = 4):
        """
        extend the converted cumulative curve by T =1000sec
        The updated three attributes are:
            self.ts
            self.N
            self.CCC
        -------------------------------
        Input:
            deltaT, the extended time interval
        -------------------------------
        Steps:
            - change ts and N
            - then change the  CCC
        """
        #If cumulative curve does not exist, comput it. 
        if not hasattr(self, 'N'):
            self.CumulativeCurve()
        
        if T<=0:
            raise ValueError('Extended T should be positive!!!!')
        #make sure enough moments
        if T < self.ts[-1] + 2 * DeltaT:
            DeltaT = T/3.0
        
        #n is the number of moments that are inserted.
        #   if the number is lesser than 4, then maintain 4.
        #       else 
        if T*1.0/DeltaT <= 4:
            n = 4
        else:
            n = int(T*1.0/4)
            
        t = list(np.linspace(self.ts[0]-1.0*T,self.ts[0]-DeltaT,n))
        N = [self.N[0] for i in t]
        
        #   extend the self.ts and self.N
        t.extend(list(self.ts))
        N.extend(list(self.N))
        
        self.ts = t
        self.N=N
        #change the CCC
        self.ConvertedCumulativeCurve()
    
    def ExtendCCC(self, T = 1000 , DeltaT=2):
        """
        extend the converted cumulative curve by T =1000sec
        The updated three attributes are:
            self.ts
            self.N
            self.CCC
        -----------------------
        Input:
            deltaT, the extended time interval
        ----------------------
        Steps:
            - Compute and change the cumulative curve, self.N
            - re-compute the converted cumulative curve. self.CCC
        """
        #If cumulative curve does not exist, comput it. 
        if not hasattr(self, 'N'):
            self.CumulativeCurve()
        
        #make sure enough moments
        if T < self.ts[-1] + 2 * DeltaT:
            DeltaT = T/3.0
        
        #n is the number of moments that are inserted.
        #   if the number is lesser than 4, then maintain 4.
        #       else 
        if T*1.0/DeltaT <= 4:
            n = 4
        else:
            n = int(T*1.0/4)
        
        t = list(np.linspace(self.ts[-1]+DeltaT,self.ts[-1]+1.0*T,n))
        N = [self.N[-1] for i in t]
        
        self.ts = list(self.ts)
        self.ts.extend(t)
        self.N.extend(N)
        self.ConvertedCumulativeCurve()
    
    
    def ShiftCCC(self, tf):
        """
        shift the converted cumulative by tf. Downstream shift is positive, and upstream is negative. 
        -------------------------
        Input:
            the free flow time, in sec
        -----------------------------
        Steps:
            1- shift the N
            2- compute the CCC
        -----------------------------
        The following properties are changed:
            self.ts
            self.N
            self.CCC
        
        """
        new_ts = [ts+tf for ts in self.ts]
        self.ts = new_ts
        self.ConvertedCumulativeCurve()
    
    def Insert_yku_gv(self, ts, N):
        """
        ts and N are list. 
        this function is run when spillover happens. And self is directional demand. therefore , when directional N is calculated by other methods, they are input to the self.
        
        self.ts self.N and self.CCC are changed.
        
        ------------------------------------------------------
        Input:
            ts and N
            are lists, ts is between yku and gv, including yku and gv
        -----------------------------------------------
        Steps:
                        - insert moment ts[-1] and ts[0]
            - clear the demand between yku and gv, 
            - insert the ts and N
            
        
        """
        self.InsertMoment(ts[-1])
        self.InsertMoment(ts[0])
        
        #decompose the moments into three components.
        #   1. [~, yku)
        #   2. [yku,gv]
        #   3. (gv,~]
        idx_es_1 = np.where(np.array(self.ts)<ts[0])
        ts_1 = list(np.array(self.ts)[idx_es_1])
        N_1  = list(np.array(self.N)[idx_es_1])
        
        idx_es_3 = np.where(np.array(self.ts)>ts[-1])
        ts_3 = list(np.array(self.ts)[idx_es_3])
        N_3  = list(np.array(self.N)[idx_es_3])
        
        self.ts = copy.deepcopy(ts_1 + ts + ts_3)
        self.N = copy.deepcopy(N_1 + N + N_3)
        
        self.ConvertedCumulativeCurve()
        self.CheckConsistency()


    def LimitDemand(self, deltaT = 3.0, qm = FD.qm):
        """
        Limit the demand based on qm.
        It is implimented by N.
        
        """
        
        #
        new_N = copy.deepcopy(self.N)
        new_ts = copy.deepcopy(self.ts)
        
        for i in range(1,len(self.ts)):
            
            deltat = self.ts[i] - self.ts[i-1]
            
            if (self.N[i]-new_N[i-1])/(deltat/3600.0) > qm:
                new_N[i] = new_N[i-1] + (qm/3600.0)*deltat
            else:
                new_N[i] = self.N[i]
        
        #after the domain
        while self.N[-1]>new_N[-1]:
            
            if 3600.0*(self.N[-1]-new_N[-1])/deltaT>qm:
                tmp = new_N[-1]
                
                new_N.append(tmp + (qm/3600.0)*deltaT)
                new_ts.append(new_ts[-1]+deltaT)
                
                self.N.append(self.N[-1])
                self.ts.append(self.ts[-1]+deltaT)
            else:
                new_N.append(self.N[-1])
                new_ts.append(new_ts[-1]+deltaT)
                
                self.N.append(self.N[-1])
                self.ts.append(self.ts[-1]+deltaT)
                
                break
        
        self.ts = new_ts
        self.N = new_N

        return new_ts,new_N

    
    def InsertMoment(self,t):
        """
        the self.ts may not contain some moments, then insert the moment
        such that self.ts contain t.
        
        The following attributes are changed:
            self.ts
            self.N
            self.CCC
        If the t is not in the self.ts, extend it.
        --------------------------------
        Input: t
            a scalar.
        
        """
        
        if t in self.ts:
            return
        
        #it t is outside the time domain, just extend it.
        if t < self.ts[0]:
            self.ExtendCCCBack(self.ts[0]-t)
            
            #debug
            print(t)
            self.CheckConsistency()
            
            return
        if t > self.ts[-1]:
            self.ExtendCCC(t-self.ts[-1])
            
            #debug
            print(t)
            self.CheckConsistency()
            
            return
        
        #@@@@@debug
        #print('insert the demand at input moment ',t)
        
        #insert the t
        #   find the idx of t in self.ts such that
        #   self.ts[idx] < t < self.ts[idx+1]
        idx = np.searchsorted(self.ts,t)-1
        #   compute the N and the CCC
        delta_t = 1.0*self.ts[idx+1] - self.ts[idx]
        delta_n = 1.0*self.N[idx+1] - self.N[idx]
        new_n = self.N[idx]+delta_n/delta_t*(t-self.ts[idx])
        new_CCC = t - new_n*3600.0/FD.qm
        #   insert the t in self.ts self.N and self.CCC
        self.ts.insert(idx+1, t)
        self.N.insert(idx+1, new_n)
        self.CCC.insert(idx+1, new_CCC)
    
    
    def PointWiseCCC2(self,ak,InverseCCC):
        """
        The difference between PointWiseCCC2() and PointWiseCCC2()
            is the input parameters
        
        The converted cumulative curve is g(t). However, there may be multivalues that lead to g(.) = t, theirefore, this function is to get the pair of (t, g(t)). The 
        return interval of t is [ak,InverseCCC]
            i.e. returned_value[0,0]=ak and returned_value[0,-1]=InverseCCC
        --------------------------
        Input: ak
            Current characteristic arrival moment
        Input: InverseCCC
            the moment where g(InverseCCC) = g(ak)+rk
        --------------------------
        Output:
            2d np.array,
            1st row is t
            2nd row is g(t)
        ------------------------
        Steps:
            - find the idx of ak and InverseCCC
            - return the value
        """
        
        #insert the ak and InverseCCC into the self.ts
        self.InsertMoment(ak)
        self.InsertMoment(InverseCCC)
        
        #find the idx that self.ts[idx_1]==ak and 
        #   self.ts[idx_2]==InverseCCC 
        idx_1 = self.ts.index(ak)
        idx_2 = self.ts.index(InverseCCC)
        
        return np.array([self.ts[idx_1:idx_2+1],self.CCC[idx_1:idx_2+1]])
    
    def PointWiseCCC(self, ak, rk):
        """
        The converted cumulative curve is g(t). However, there may be multivalues that lead to g(.) = t, theirefore, this function is to get the pair of (t, g(t)). The 
        return interval of t is [ak,a_{k+1}]
        -----------------------------------------
        Input: ak
            ak, the start x-value of the pair of points
        input: rk
            g(g_k) + rk 
            g^{-1}(g(g_k) + rk)
        --------------------------------------
        Output:
            2d np.array,
            1st row is t
            2nd row is g(t)
        --------------------------------
        Steps:
            - find the t that g(t) =g(g_k) + rk, any g(t+ small value) > g(g_k) + rk
            - from ak to t, point wise interp g(t)
        """
        
        #find the t that g(t) =g(g_k) + rk
        #   i.e. tmp is next ak candidate.
        #   if there are multiple value of tmp, then the 
        #       value last appear is self.CCC[idx], where 
        #       idx=np.searchsorted(self.CCC, tmp, side='right')-1
        tmp = self.InverseCCC(self.CCC_evaluation(ak) + rk)
        
        res_x = []
        res_y = []
        for t in np.arange(ak, tmp, min([ (tmp-ak)/3.0, 1])):
            res_x.append(t)
            res_y.append(self.CCC_evaluation(t))
        
        #the last one
        res_x.append(tmp)
        res_y.append(self.CCC_evaluation(ak) + rk)
        
        return np.array([res_x,res_y])
    
    def InverseN(self, n, side = 'left'):
        """
        find the absolute moment when the cumulative curve is n
        if multiple values exist, then the first appear return if 
        side = 'left'
        ----------------------------------------------
        Input: n 
            scalar
        input: side
            the value returned when multiple result exist
        -----------------------------------------
        output:
            scalar,
        """
        #make sure that the self.ts and self.N are increasing
        
        
        #make sure that the input is within the self.N
        if self.N[0]>n:
            raise ValueError('input n is smaller than smallest cumulative')
        if self.N[-1]<n:
            raise ValueError('input n is greater than greatest cumulative')
        
        #more than 1 (include 1) values exist that self.N == n
        if sum(np.array(self.N) == n)>=1:
            #if more than one value that N(t) =n
            idx_es = np.where(np.array(self.N) == n)[0]
            if side =='left':
                return self.ts[idx_es[0]]
            elif side=='right':
                return self.ts[idx_es[-1]]
        else:
            #find the index that self.N[idx]< n < self.Nidx+1
            idx = np.searchsorted(self.N,n)-1
            #interp the moment
            delta_t = 1.0*self.ts [idx+1] - self.ts[idx]
            delta_n = 1.0*self.N [idx+1] - self.N[idx]
            
            return self.ts[idx] + delta_t/delta_n*(n-self.N[idx])
    
    def InverseCCC(self,t):
        """
        compute the inverse converted cumulative curve, which is 
        defined as g(t) = t - \frac{N(t)}{q_m}
        
        ------------------------
        Input:
            scalar, function value, t(unit of function g(t) is also the same as t)
        Output:
            scalar.
        -------------------------
        If t is greater than maximal self.CCC(i.e. self.CCC[-1]), then extend the self.CCC
        If there is multivalues that g() =t, then the first apper is returned
        
        """
        
        #as long as the temporal domain is not enough, extend it
        while t>self.CCC[-1]:
            self.ExtendCCC()
        
        #if there are multiple value of t, then the 
        #   value first appear is self.CCC[idx], where 
        #   idx= np.searchsorted(self.CCC, t, side='left')
        if sum(np.array(self.CCC)==t)>=1:
            idx=np.searchsorted(self.CCC, t, side='left')
            return self.ts[idx]
        #find the idx of the t in self.CCC
        #   self.CCC[idx]<= t <= self.CCC[idx+1]
        idx = np.searchsorted(self.CCC, t, side='right')-1
        deltay = self.ts[idx+1]-self.ts[idx]
        deltax = self.CCC[idx+1]-self.CCC[idx]
        return 1.0*self.ts[idx]+deltay/deltax*1.0*(t-self.CCC[idx])
    
    
    def Useless_N_evaluation(self,t):
        """
        evaluate the fun of converted cumulative curve, which is defined as g(t) = t - \frac{N(t)}{q_m}
        -----------------
        Input:
            moment, t
        Output:
            g(t) = t - \frac{N(t)}{q_m}
        ------------------
        If t is the minimal of maximal, return the value directly
        otherwise, interp it. 
        
        """
        
        #make sure that the moments, i.e. self.t is strict increasing
        if not np.all(np.diff(self.ts) > 0):
            raise ValueError('time is not increasing!!')
        
        while t<self.ts[0]:
            self.ExtendCCCBack()
        while t>self.ts[-1]:
            self.ExtendCCC()
        
        #input moment t is the first one or last one        
        if t == self.ts[0]:
            return 1.0*self.N[0]
        elif t == self.ts[-1]:
            return 1.0*self.N[-1]
        #find the index of t in the temporal domain
        #   resulting t \in [self.t[idx],self.t[idx+1]]
        idx = np.searchsorted(self.ts, t, side='right')-1
        deltax = self.ts[idx+1]-self.ts[idx]
        deltay = self.N[idx+1]-self.N[idx]
        return 1.0*self.N[idx] + 1.0*(t-self.ts[idx])*(deltay)/(deltax)
    
        def Backup_demand(self, t, typee = 'after'):
                """ 
                Backup the demand, i.e. the self.ts and self.N
                
                ----------------------------------
                Step: 
                        - insert the moment t
                        - find the idx of t
                        - store the ts and N.
                """
                self.InsertMoment(t)
                
        idx_1 = np.where(np.array(self.ts)==t)[0][0]
        self.mcts_backup_ts  = self.ts[idx_1:]
        self.mcts_backup_N  = self.N[idx_1:]
        
        
        def ChangeDemand_lognormal(self):
                """
                Change the demand to log-normal distribution. 
                
                Using FlowRateSample_shifted_lognormal_headway(flowrate, tau=2,  sigma = .32, size = 1000)
                --------------------------------------------------------------------
                Input: route_demand
                    A RouteDemand class. This function will use route_demand.ts and route_demand.N attributes. 
                    
                
                --------------------------------------------------------------------
                Steps:
                    - For each ts 
                
                """
                #new cumulative number initialize
                new_N = []
                new_N.append(self.N[0])
            
                for idx in range(len(self.ts)-1):
                    
                        #unit is in veh/h
                        delta_t = self.ts[idx+1] - self.ts[idx]
                        delta_N = self.N[idx+1] - self.N[idx]
                        q = min(1.0*(delta_N)/(delta_t/3600.0), FD.qm)
                    
                        #sample from log-normal distribution. 
                        #       if size=1, functin will return array([ 75.39434789]) type data.
                        new_q = FlowRateSample_shifted_lognormal_headway(q, tau=2,  sigma = .32, size = 1)[0]
                        if np.isnan(new_q):
                                print(q,new_q)
                                raise ValueError('sdfsdf')
                        new_delta_N =  new_N[idx] + new_q/3600.0*delta_t
                        new_N.append(new_delta_N)
                route_demand.N = new_N
        
    def CCC_evaluation(self,t):
        """
        evaluate the fun of converted cumulative curve, which is defined as g(t) = t - \frac{N(t)}{q_m}
        -----------------
        Input:
            moment, t
        Output:
            g(t) = t - \frac{N(t)}{q_m}
        ------------------
        If t is outside the time horizon, then extend it forward or backward.  
        
        """
        
        #make sure that the moments, i.e. self.t is strict increasing
        if not np.all(np.diff(self.ts) > 0):
            raise ValueError('time is not increasing!!')
        
        #If cumulative curve does not exist, comput it. 
        if not hasattr(self, 'CCC'):
            self.ConvertedCumulativeCurve()
        
        #if t is not within the time horizon, extend it. 
        while t<self.ts[0]:
            self.ExtendCCCBack()
        while t>self.ts[-1]:
            self.ExtendCCC()
        
        #input moment t is the first one or last one        
        if t == self.ts[0]:
            return 1.0*self.CCC[0]
        elif t == self.ts[-1]:
            return 1.0*self.CCC[-1]
        #find the index of t in the temporal domain
        #   resulting t \in [self.t[idx],self.t[idx+1]]
        idx = np.searchsorted(self.ts, t, side='right')-1
        deltax = self.ts[idx+1]-self.ts[idx]
        deltay = self.CCC[idx+1]-self.CCC[idx]
        return 1.0*self.CCC[idx] + 1.0*(t-self.ts[idx])*(deltay)/(deltax)
    
    
    def VehicleNumberAt_t(self,t):
        """
        note that self.VehicleNumberAt_t2(). The input of VehicleNumberAt_t() is a scalar. While in VehicleNumberAt_t2(), the input t is transformed to array. 
        
        If t is smaller than self.ts[0], return self.N[0]
        if t is greatr than self.ts[-1], return self.N[-1.]
        -------------------------------
        Input:
            t, scalar
        ----------------------------
        
        
        """
        if t <= self.ts[0]:
            return self.N[0]
        if t >= self.ts[-1]:
            return self.N[-1]
        if t in self.ts:
            #the index of each_t in self.ts is self.ts.index(each_t)
            return self.N[self.ts.index(t)]
        
        idx = np.searchsorted(self.ts,t)-1
        delta_t = 1.0*self.ts[idx+1] - self.ts[idx]
        delta_n = self.N[idx+1] - self.N[idx]
        return self.N[idx]+delta_n/delta_t*(t-self.ts[idx])
        
    
    
    def VehicleNumberAt_t2(self,t):
        """
        Cumulative number at moments t
        if t is greater than self.MaximalMoment, then maximal vehicle number is returned.
        if t is smaller than self.ts[0], then the first value is returned. 
        if t is a scalar, then return scalar
        if t is a list or np.array, return an array.
        -----------------------------
        input:
            t: scalar or 1-dimentional array
                if it is a scalar, then it will be transformed to  1-dimentional array
            
        ----------------------------
        Output: np.array
        """
        #transform the input to array. t may be scalar
        #   :if hasattr(t, "__len__"):
        t1 = list(np.asarray(t).reshape(1, -1)[0,:])
        
        #make sure that the self.ts is strictly increasing
        if not np.all(np.diff(self.ts) > 0):
            raise ValueError('self.ts is not strictly increasing')
        
        #make sure that the input t is within the time domain
        #   if not, extend it. 
        if min(t1)<self.ts[0]:
            self.ExtendCCCBack(T = self.ts[0]-t + 10)
        if max(t1) > self.ts[-1]:
            self.ExtendCCC(T = t-self.ts[-1]+10)
        
        #find the returned value.
        res= []#returned value
        for each_t in t1:
            #if each_t is one element in self.ts
            if each_t in self.ts:
                #the index of each_t in self.ts is self.ts.index(each_t)
                res.append(self.N[self.ts.index(each_t)])
            else:
                #each_t is not in self.ts, thus need to interp it
                #   find the index that self.ts[idx]<each_t<self.ts[idx+1]
                idx = np.searchsorted(self.ts,each_t)-1
                delta_t = 1.0*self.ts[idx+1] - self.ts[idx]
                delta_n = self.N[idx+1] - self.N[idx]
                res.append( self.N[idx]+delta_n/delta_t*(each_t-self.ts[idx]) )
        
        #return the value
        #   check the input is scalar or list
        if hasattr(t, "__len__"):
            return res
        else:
            return res[0]
        
    def CumulativeCurve(self):
        """
        Generate the cumulative vehicles number, i.e. self.N.
        because self.ts and self.y (self.y is the flow rate, in veh/h) are with the same length. therefore self.N should be consistent and the first value is set to zero. 
        --------------------------------------
        
        """
        #First moment N is zero
        N = [0.0]
        
        for i,_ in enumerate(self.ts):
            if i==0:continue
            deltaT = self.ts[i]-self.ts[i-1]
            deltaN = deltaT/3600.0*self.y[i]
            N.append(deltaN+N[-1])
        
        self.N=N
        
    def ConvertedCumulativeCurve(self, qm = FD.qm):
        """
        generate Converted Cumulative C urve
        ---------------------------------------
        Steps:
            find the moment t with its correponding N, and compute
                g(t)=t-\frac{N}{q_m}
        """
        
        #如果尚未计算累计车辆数,那么计算他
        if not hasattr(self, 'N'):
            self.CumulativeCurve()
        #计算ConvertedCumulativeCurve
        CCC=[]
        #3600 is to make sure the unit is uniform
        for i,t in enumerate(self.ts):
            CCC.append(t - self.N[i]*3600.0/qm)
        
        self.CCC=CCC
    
    def PlotN_raw(self):
        #plot(t,y)
        fig, ax = plt.subplots()
        T = self.ts
        ax.plot(T,self.N,'.-', label = 'new');
        ax.set_xlabel("Time (in sec)")
        ax.set_ylabel("N (in veh)")
        ax.plot(self.raw_ts,self.raw_N,'r.-',label = 'raw');
        ax.legend();
        
        plt.show()
        
        #return T,Y
    
    def PlotN(self):
        """
        plot the cumulative curve
        
        """
        #plot(t,y)
        fig, ax = plt.subplots()
        T = self.ts
        ax.plot(T,self.N);
        ax.set_xlabel("Time (in sec)")
        ax.set_ylabel("Cumulative curve (in veh)")
        
        plt.show()
        
    
    def PlotCCC(self):
        
        #plot(t,y)
        fig, ax = plt.subplots()
        T = self.ts
        ax.plot(T,self.CCC);
        ax.set_xlabel("Time (in sec)")
        ax.set_ylabel("CCC (in sec)")
        
        plt.show()
        
        #return T,Y
    
    
    def Spillover(self, yku, ykv, qm = FD.qm):
        """
        change the g(t) and N due to the spillover
            g(t) = t-\frac{N}{q_m}, it is self.CCC
            N is self.N
        The following attributes are changed:
            self.ts.
            self.N.
            self.CCC.
        -----------------------------------------
        Input:yku, ykv
            the start and ending moment of spillover
        Input: qm
            that is used to find the gv, absolute moment.
            that the saturation release duration.
                Therefore, in [yku, ykv], output flow rate is 0
                and in  [ykv, gv], the flow rate is maximal.
        Output: gv
            absolutely moment when saturation flow release end.
        --------------------------------------------
        Steps:(first reconstruct the N, and compute the CCC)
            - find the idx of yku, ykv, gv.
            - 
        """
        
        if yku>=ykv:
            raise ValueError('ykv should be greater than yku, yku=',yku,',  ykv=',ykv)
        
        if yku >= self.ts[-1]:
            print("the spillover onset moment is greater than the time horizon")
            return
        #Make sure that, the linear line with qm slope intersect
        #with self.N. 
        #   y is the function of saturation flow rate line.
        #       i.e. y(t) evaluate the cumulative N, 
        #       when the flow rate is qm
        spillover_onset_N = self.VehicleNumberAt_t(yku)
        y = lambda t:qm/3600.0*(t-ykv) + spillover_onset_N
        #   if the two line don't intersect. 
        while self.N[-1] >= y(self.ts[-1]):
            self.ExtendCCC()
        
        #find the gv (absolute moment)
        #   Get the difference between two N.--->difference_N
        moments = np.linspace(ykv, self.ts[-1],max([2, (self.ts[-1]-ykv)/3]))
        difference_N = [self.VehicleNumberAt_t(t)-y(t)  for t in moments]
        #   find the gv.
        #       if there is some moment 
        #       that difference_N is zero ,then return the moment,
        #       otherwise interp it.
        if difference_N[0]<0 or difference_N[-1]>0:
            print('difference_N[0]= ',difference_N[0])
            print('difference_N[-1]= ',difference_N[-1])
            builtins.difference_N = difference_N
            builtins.moments = moments
            raise ValueError('The two curves des not intersec.', yku, ykv, difference_N[0], difference_N[-1])
        #       exist some value is zero
        if len(np.where(np.array(difference_N)==0)[0])>=1:
            #note that the gv may be the same as ykv,
            #   which means that flow rate during yku and ykv are zero.
            gv = moments[np.where(np.array(difference_N)==0)[0][0]]
        else:
            #find the two points of negative and positive
            idx1 = np.where(np.array(difference_N)>0)[0][-1]
            idx2 = np.where(np.array(difference_N)<0)[0][0]
            #two points: () and ()
            t1 = moments[idx1];t2 = moments[idx2];
            n1 = difference_N[idx1];n2 = difference_N[idx2];
            gv = t1 - 1.0*(n1*(t2-t1))/(n2-n1)
        
        #reset the ts, N, CCC
        #   four sections: [self.ts[0], yku]--[yku,ykv]--[ykv,gv]--[gv,]
        #   note that the gv may identical to ykv.
        idx1 = np.where(np.array(self.ts) < yku)[0][-1]
        t1 = self.ts[:idx1+1];
        n1 = self.N[:idx1+1];
        #   find the cumulative number at yku and set 2nd N
        #       spillover_onset_N = self.VehicleNumberAt_t(yku)
        t2 = list(np.linspace(yku,ykv,max([ int((ykv-yku)/2.0) ,2])))
        t2.pop()#delete the ykv from t2, because it is in t3.
        n2 = [spillover_onset_N for i in t2]
        #   set N at 3rd domain
        #       because the gv may ==ykv
        if gv == ykv:
            t3 = []
            n3 = []
        elif gv>ykv:
            t3 = list(np.linspace(ykv,gv,max([ int((gv-ykv)/2.0) ,2])))
            n3 = [y(t) for t in t3]
        else:
            raise ValueError('gv (absolute moment) is smaller than ykv.')
        #   the last time horizon
        idx4 = np.where(np.array(self.ts) > gv)[0][0]
        t4 = self.ts[idx4:];
        n4 = self.N[idx4:];
        #reset the self.ts, self.N self.CCC 
        self.ts = list(itertools.chain(t1, t2, t3, t4))
        self.N = list(itertools.chain(n1, n2, n3, n4))
        self.ConvertedCumulativeCurve()
        
        return gv
    
    def reset_demand(self,demandtype = "linear", **kwargs):
        """
        sadfasdfsadf
        different from reset_demand1() in that, this method can set demand based on curve types.
                
                
        ------------------------------
        Input:
            demandtype, str, denote the type of the demand, such as linear, parabolic
        Input:
            linear:
                flowrate = 100, T = 3600, deltat = 2
            piecelinear:
                                list_tuple_of_three_values, baseflow =10, T = 3600, deltat = 2, qm = FD.qm.
                                
                                for each element X in list_tuple_of_three_values, 
                                X = (l,r, flow) denote the left-moment, right-moment and flow rate .
                                
                        
                        
                        parabolic:
                list_tuple_of_three_values, baseflow =10, T = 3600, deltat = 2, qm = FD.qm
                                
                                for each element X in list_tuple_of_three_values, 
                                X = (l,r, maxflow) denote the left-moment, right-moment and maximal flow rate .
                                
                                Usage:
                                        rightdemand = RouteDemand()
                                        list_tuple_of_three_values = [(100,500, 600), (300, 1000, 555)]
                                        baseflow =10
                                        T = 3600
                                        deltat = 2
                                        qm = FD.qm
                                        rightdemand.reset_demand(demandtype = 'parabolic', list_tuple_of_three_values=list_tuple_of_three_values, baseflow =10, T = 3600, deltat = 2, qm = FD.qm)
                                        rightdemand.PlotFlowRate()
        """
        if demandtype == "linear":
            flowrate = min(kwargs['flowrate'], FD.qm)
            T = kwargs['T']
            deltat      = kwargs['deltat']
            
            #define the self.ts and self.q
            ts = list(np.arange(0,T,deltat))
            q  = [flowrate for t in ts]
            
            self.y = q
            self.ts = ts
            
            #compute the self.N and self.CCC
            self.CumulativeCurve()
            
            #conpute the converted cumulative curve.
            self.ConvertedCumulativeCurve()
            
            #store the raw
            self.raw_ts = copy.deepcopy(self.ts)
            self.raw_N = copy.deepcopy(self.N)
            self.raw_CCC = copy.deepcopy(self.CCC)
            
        elif demandtype == "piecelinear":
                        
                        
                        
                        
            pass
                
        elif demandtype == "parabolic":
            
            list_tuple_of_three_values = kwargs['list_tuple_of_three_values']
            if len(list_tuple_of_three_values)==0:
                flowrate = kwargs['baseflow']
                deltat      = kwargs['deltat']
                T           = kwargs['T']
                self.reset_demand(demandtype = "linear", flowrate = flowrate, T= T, deltat = deltat)
                
                return
                                
            baseflow    = kwargs['baseflow']
            deltat      = kwargs['deltat']
            qm          = kwargs['qm']
            T           = kwargs['T']
            
            #define the self.ts and self.q
            ts = list(np.arange(0,T,deltat))
            q  = list(np.arange(0,T,deltat))
            for i,t in enumerate(ts):
                res = -np.inf
                for j,parabolic in enumerate(list_tuple_of_three_values):
                    l = parabolic[0]
                    r = parabolic[1]
                    maximum = parabolic[2]
                    a = -4.0*maximum/((l-r)*(l-r))
                    res = max([1.0*a*(t-l)*(t-r), res])
                q[i] = min([FD.qm,max([res,baseflow])])
            
            self.y = q
            self.ts = ts
            
            #compute the self.N and self.CCC
            self.CumulativeCurve()
            
            #conpute the converted cumulative curve.
            self.ConvertedCumulativeCurve()
            
            #store the raw
            self.raw_ts = copy.deepcopy(self.ts)
            self.raw_N = copy.deepcopy(self.N)
            self.raw_CCC = copy.deepcopy(self.CCC)
            
            
        elif demandtype == "ts_and_N":
            #raise ValueError("sdfsdf")
            ts = kwargs['ts']
            N = kwargs['N']
            idx_es  = np.diff(self.ts)<1.0e-5
            if sum(idx_es) != 0:
                new_ts = list(np.array(ts)[~idx_es])
                new_N = list(np.array(N)[~idx_es])
            else:
                new_ts = ts
                new_N = N
            #eliminate the time difference where diff is too small
            #   to create the error. 
            
            self.ts = copy.deepcopy(new_ts)
            self.N = copy.deepcopy(new_N)
            
            #conpute the converted cumulative curve.
            self.ConvertedCumulativeCurve()
            
            self.raw_ts = copy.deepcopy(self.ts)
            self.raw_N = copy.deepcopy(self.N)
            self.raw_CCC = copy.deepcopy(self.CCC)
            
        
        
        pass
    
    def RandomDisturb(self, magnitude0 = 10, qm = FD.qm):
        """
        randomely disturbe the flow rate 
        The following are changed:
            self.N
            self.CCC
        ---------------------------------------------------------
                Input: magnitude
                        unit in veh/h.
                
                
                ----------------------------------------------------------
        Steps:
            - for each self.ts[k] and self.ts[k+1]:
                - fcompute the flow rate, q = ( self.N[k+1]-  self.N[k])/(self.ts[k+1] -  self.ts[k])
                - ramdom disturbe the flow rate, q = q + random(magnitude)
                - reset the self.N[k+1]
        """
        magnitude = magnitude0/3600.0
        #get the original qs, in veh/s
        qs = [( self.N[k+1]-  self.N[k])/(self.ts[k+1] -  self.ts[k]) for k in range(len(self.ts)-1)]
        #get the disturbed q
        for i in range(len(qs)):
            qs[i] = max(0, min(qm/3600.0, qs[i] + magnitude*np.random.uniform(-1,1)))
        #compute the N
        for k in range(len(self.ts)-1):
            self.N[k+1] = self.N[k] + (self.ts[k+1] -  self.ts[k])*qs[k]
        
        self.ConvertedCumulativeCurve()
    
    def ReconstructQueueProfileFromN_backup(self, l, spillovers, fd = FD):
        """
        self is a RouteDemand class.
        re-construct the queue profile from self.raw_N and self.N
        len(self.raw_ts)==len(self.raw_N)
        len(self.ts)==len(self.N)
        -------------------------------------
        Inout: l
            the link length, used to check whether there is a spillover. 
        Input:  fd
            fundamental diagram, the used parameters are fd.kj, fd.vf. 
            They are used to calculate the wave speed.
        Inout: spillovers
            list, spillovers = [(yku1, ykv1), (yku2,ykv2)...]
            each element is (yku,ykv), the onset and ending moment of a spillover. 
        ---------------------------------------
        Steps:
            - for each spillover interval yku ykv
                - stepwise compute the stopping wave and starting wave, until they intersect
                    - if the stoping wave bigger than l, self spillover happens
                    - 
        
        """
        
        #returned value.
        #   res['xy'][idx_rgp] = 2 dimentional array,
        #       first row is moments and 2nd row is locaiton
        res =  {}
        res['xy'] = {}
        res['startingwave'] = {}
        
        #Set two RouteDemand class to faciliate the following operations.
        #dmd1 is the raw demand, and dmd2 is the spillover influenced spillover.
        dmd1 = RouteDemand(T = 1000)
        dmd1.ts = copy.deepcopy(self.raw_ts)
        dmd1.N = copy.deepcopy(self.raw_N)
        dmd2 = RouteDemand(T = 1000)
        dmd2.ts = copy.deepcopy(self.ts)
        dmd2.N = copy.deepcopy(self.N)
        
        
        #Common moments for raw_N and new N
        #   after this operation, the dmd1.ts == dmd2.ts
        #   and include all ykus and ykvs
        common_moments = set(dmd1.ts + dmd2.ts)
        dmd1.InsertMoment(min(common_moments))
        dmd1.InsertMoment(max(common_moments))
        dmd2.InsertMoment(min(common_moments))
        dmd2.InsertMoment(max(common_moments))
        #   get the common moments, and make the directional moments set the same
        #       note that ykus_ykvs include all the moments.
        ykus_ykvs = [i[0] for i in spillovers]+[i[1] for i in spillovers]
        common_moments = set(dmd1.ts + dmd2.ts + ykus_ykvs)
        diff_moments1 = common_moments.difference(set(dmd1.ts))
        diff_moments2 = common_moments.difference(set(dmd2.ts))
        for t in diff_moments1:
            dmd1.InsertMoment(t)
        for t in diff_moments2:
            dmd2.InsertMoment(t)
        
        #debug
        #return dmd1,dmd2
        
        #for each spillover interval, compute the stopping wave 
        #   and startingwave.
        for idx_spillover, spillover in enumerate(spillovers):
            yku, ykv = spillover
            print(yku,ykv)
            #out of the time horizon, terminate.
            if yku > dmd1.ts[-1]:
                break
            #index of yku amd ykv
            idx_yku1    = np.where(np.array(dmd1.ts)==yku)[0][0]
            idx_ykv1    = np.where(np.array(dmd1.ts)==ykv)[0][0]
            idx_yku2    = np.where(np.array(dmd2.ts)==yku)[0][0]
            idx_ykv2    = np.where(np.array(dmd2.ts)==ykv)[0][0]
            
            #the initial moment and location of xy and startingwave
            xy_t = [yku];xy_x = [0]
            startingwave_t = [ykv]; startingwave_x = [0]
            
            #moment wise compute the xy and starting wave
            #1, get the potential xy and starting wave
            #2, determine they intersect within l or not, 
            #   if intersect within l,
            #       reformulate the raw_N
            #   if not intersec, then a self spillover
            #       reformulate the raw_N
            #1 get the potential xy
            for i in range(idx_yku1+1, len(dmd1.ts)):
                #if dmd1.N[i] == dmd1.N[i-1]
                if dmd1.N[i] == dmd1.N[i-1]:
                    xy_x.append(xy_x[-1])
                    xy_t.append(xy_t[-1]+dmd1.ts[i]-dmd1.ts[i-1])
                    continue
                
                #stopping wave speed, in m/s, positive
                #   flowrate is in veh/hour, density is in veh/km
                flowrate = ((dmd1.N[i]-dmd1.N[i-1])/(dmd1.ts[i]-dmd1.ts[i-1]))*3600.0
                density = flowrate/fd.vf
                stoppingwavespeed = (-flowrate/(density - fd.kj))/3.6
                
                #queue increment
                #   (dmd1.N[i]-dmd1.N[i-1])/(fd.kj/1000.0) 
                #       is the queue increment
                xy_x.append(xy_x[-1]+ (dmd1.N[i]-dmd1.N[i-1])/(fd.kj/1000.0))
                xy_t.append(xy_t[-1]+(dmd1.N[i]-dmd1.N[i-1])/(fd.kj/1000.0)/(stoppingwavespeed))
                
            #1 get the potantial startingwave
            for i in range(idx_ykv2+1, len(dmd2.ts)):
                #if dmd1.N[i] == dmd1.N[i-1]
                if dmd2.N[i] == dmd2.N[i-1]:
                    startingwave_x.append(startingwave_x[-1])
                    startingwave_t.append(startingwave_t[-1]+dmd2.ts[i]-dmd2.ts[i-1])
                    continue
                
                #startingwave speed, in m/s, positive
                #   flowrate is in veh/hour, density is in veh/km
                flowrate = ((dmd2.N[i]-dmd2.N[i-1])/(dmd2.ts[i]-dmd2.ts[i-1]))*3600.0
                density = flowrate/fd.vf
                startingwave = (-flowrate/(density - fd.kj))/3.6
                
                #startingwave increment
                #   (dmd1.N[i]-dmd1.N[i-1])/(fd.kj/1000.0) 
                #       is the queue increment
                startingwave_x.append(startingwave_x[-1]+ (dmd2.N[i]-dmd2.N[i-1])/(fd.kj/1000.0))
                startingwave_t.append(startingwave_t[-1] + (dmd2.N[i]-dmd2.N[i-1])/(fd.kj/1000.0)/(startingwave))
            
            #determine whether xy and startingwave_x and  intersect or not
            #   (startingwave_t,startingwave_x)
            #   (xy_t, xy_x)
            
            
            new_waves = TwoLinesIntersection_ReconstructQueueProfileFromN(xy_t,xy_x, startingwave_t, startingwave_x, l = l)
            res['xy'][idx_spillover] = new_waves['xy']
            res['startingwave'][idx_spillover] = new_waves['startingwave']
            
        return res
    
        
    def ReconstructQueueProfileFromN(self, l, spillovers, fd = FD()):
        """
        self is a RouteDemand class.
        re-construct the queue profile from self.spillovered_dmd and self.raw_N and self.raw_ts.
        self.spillovered_dm = {};
        self.spillovered_dmd[spillover_idx] = {'ts':ts, 'N':N} is the ts and N after the specific spillover;
        Hence len(self.spillovered_dmd) is the number of spillovers;
        
        len(self.raw_ts)==len(self.raw_N)
        len(self.ts)==len(self.N)
        
        len(self.spillovered_dmd[spillover_idx]['ts']) ==len(self.spillovered_dmd[spillover_idx]['N'])
        
        -------------------------------------
        Inout: l
            the link length, used to check whether there is a spillover. 
        Input:  fd
            fundamental diagram, the used parameters are fd.kj, fd.vf. 
            They are used to calculate the wave speed.
        Input: spillovers
            list, spillovers = [(yku1, ykv1), (yku2,ykv2)...]
            each element is (yku,ykv), the onset and ending moment of a spillover. 
        
        Output:
            res, a dict
            res[spillover_idx] = {'xy':xy, 'startingwave':startingwave, 'ideal_startingwave':ideal_startingwave}
            
            where: xy, startingwave and ideal_startingwave are both 2d np.array. 1st row is the moments and 2nd row is the locations.
        ---------------------------------------
        Steps:
            - for each spillover interval yku ykv
                - get the ts and N, before the spillover and after the spillover
                - compute the xy and startingwave
                - call TwoLinesIntersection_ReconstructQueueProfileFromN() to get the results.
        
        """
        
        #error debug
        #self.qdebug = mydebug()
        
        #returned value.
        #   res['xy'][idx_rgp] = 2 dimentional array,
        #       first row is moments and 2nd row is locaiton
        res =  {}
        res['xy'] = {}
        res['startingwave'] = {}
        
        #debug
        #return dmd1,dmd2
        
        #for each spillover interval, compute the stopping wave 
        #   and startingwave.
        for idx_spillover, spillover in enumerate(spillovers):
            yku, ykv = spillover
            #print(yku,ykv),'idx_spillover=',idx_spillover
            
            #get the ts and N before and after spillover.
            #   dmd1 is the demand before spillover 
            #   dmd2 is the demand after spillover. 
            dmd1 = RouteDemand(T = 1000)
            dmd2 = RouteDemand(T = 1000)
            if  idx_spillover==0:
                dmd1.ts = copy.deepcopy(self.raw_ts)
                dmd1.N = copy.deepcopy(self.raw_N)
            else:
                dmd1.ts = copy.deepcopy(self.spillovered_dmd[idx_spillover-1]['ts'])
                dmd1.N = copy.deepcopy(self.spillovered_dmd[idx_spillover-1]['N'])
            dmd2.ts = copy.deepcopy(self.spillovered_dmd[idx_spillover]['ts'])
            dmd2.N = copy.deepcopy(self.spillovered_dmd[idx_spillover]['N'])
            
            #Common moments for raw_N and new N
            #   after this operation, the dmd1.ts == dmd2.ts
            #   and include yku and ykv
            common_moments = set(dmd1.ts + dmd2.ts)
            dmd1.InsertMoment(min(common_moments))
            dmd1.InsertMoment(max(common_moments))
            dmd2.InsertMoment(min(common_moments))
            dmd2.InsertMoment(max(common_moments))
            common_moments = set(dmd1.ts + dmd2.ts + [yku,ykv])
            diff_moments1 = common_moments.difference(set(dmd1.ts))
            diff_moments2 = common_moments.difference(set(dmd2.ts))
            for t in diff_moments1:
                dmd1.InsertMoment(t)
            for t in diff_moments2:
                dmd2.InsertMoment(t)
            
            #out of the time horizon, terminate.
            if yku > dmd1.ts[-1]:
                break
            
            #index of yku amd ykv
            idx_yku1    = np.where(np.array(dmd1.ts)==yku)[0][0]
            idx_ykv1    = np.where(np.array(dmd1.ts)==ykv)[0][0]
            idx_yku2    = np.where(np.array(dmd2.ts)==yku)[0][0]
            idx_ykv2    = np.where(np.array(dmd2.ts)==ykv)[0][0]
            
            #the initial moment and location of xy and startingwave
            xy_t = [yku];xy_x = [0]
            startingwave_t = [ykv]; startingwave_x = [0]
            
            #moment wise compute the xy and starting wave
            #1, get the potential xy and starting wave
            #2, determine they intersect within l or not, 
            #   if intersect within l,
            #       reformulate the raw_N
            #   if not intersec, then a self spillover
            #       reformulate the raw_N
            #1 get the potential xy
            for i in range(idx_yku1+1, len(dmd1.ts)):
                #if dmd1.N[i] == dmd1.N[i-1]
                if dmd1.N[i] == dmd1.N[i-1]:
                    xy_x.append(xy_x[-1])
                    xy_t.append(xy_t[-1]+dmd1.ts[i]-dmd1.ts[i-1])
                    continue
                
                #if two instance too near, then may create error.
                # under this case, do nothing,
                if dmd1.ts[i] - dmd1.ts[i-1]<= 1e-8:
                    continue
                
                #stopping wave speed, in m/s, positive
                #   flowrate is in veh/hour, density is in veh/km
                flowrate = ((dmd1.N[i]-dmd1.N[i-1])/(dmd1.ts[i]-dmd1.ts[i-1]))*3600.0
                density = flowrate/fd.vf
                stoppingwavespeed = (-flowrate/(density - fd.kj))/3.6
                
                #queue increment
                #   (dmd1.N[i]-dmd1.N[i-1])/(fd.kj/1000.0) 
                #       is the queue increment
                xy_x.append(xy_x[-1]+ (dmd1.N[i]-dmd1.N[i-1])/(fd.kj/1000.0))
                
                xy_t.append(xy_t[-1]+((dmd1.N[i]-dmd1.N[i-1])/(fd.kj/1000.0))/stoppingwavespeed)
                
            #1 get the potantial startingwave
            for i in range(idx_ykv2+1, len(dmd2.ts)):
                #if dmd1.N[i] == dmd1.N[i-1]
                if dmd2.N[i] == dmd2.N[i-1]:
                    startingwave_x.append(startingwave_x[-1])
                    startingwave_t.append(startingwave_t[-1]+dmd2.ts[i]-dmd2.ts[i-1])
                    continue
                
                #if two instance too near, then may create error.
                # under this case, do nothing,
                if dmd2.ts[i] - dmd2.ts[i-1]<= 1e-7:
                    continue
                
                #startingwave speed, in m/s, positive
                #   flowrate is in veh/hour, density is in veh/km
                flowrate = ((dmd2.N[i]-dmd2.N[i-1])/(dmd2.ts[i]-dmd2.ts[i-1]))*3600.0
                density = flowrate/fd.vd()
                startingwave = (-flowrate/(density - fd.kj))/3.6
                
                #startingwave increment
                #   (dmd1.N[i]-dmd1.N[i-1])/(fd.kj/1000.0) 
                #       is the queue increment
                startingwave_x.append(startingwave_x[-1]+ (dmd2.N[i]-dmd2.N[i-1])/(fd.kj/1000.0))
                startingwave_t.append(startingwave_t[-1] + (dmd2.N[i]-dmd2.N[i-1])/(fd.kj/1000.0)/(startingwave))
            
            #determine whether xy and startingwave_x and  intersect or not
            #   (startingwave_t,startingwave_x)
            #   (xy_t, xy_x)
            
            #debug
            #self.qdebug.save(idx_spillover, xy_t,xy_x, startingwave_t, startingwave_x)
            
            #
            builtins.idx_spillover = idx_spillover
            
            new_waves = TwoLinesIntersection_ReconstructQueueProfileFromN(xy_t,xy_x, startingwave_t, startingwave_x, l = l)
                        
            print(new_waves['xy'])
            res['xy'][idx_spillover] = new_waves['xy']
            res['startingwave'][idx_spillover] = new_waves['startingwave']
            
        return res
    
        def GetIntCumulativeCurvesMoments(self):
                """
                Get the moment when the 1st vehicle, 2nd, 3rd vehicle enter the road. 
                ----------------------------------------------------------------------------------
                Output: ts
                        a list, the first element is the moment when 1st vehicle enter the road. ...                
                ---------------------------------------------------------------------------------
                Steps:
                        - Get the maximal N_max
                        - From 1 to N_max
                                - find the t and append it.
                
                """
                max_N = int(self.N[-1])
                
                ts = []
                for i in range(max_N + 1):
                        ts.append(self.InverseN(i))
                return ts, range(max_N + 1)
        
        def reset_demand_ts_N(self):
                """
                
                """
                
                
                pass
        
        def LaneFD_variation(self,LaneFD_variation,  ykv, gv, qm = FD.qm):
                """
                Variation of the lane FD. LaneFD_variation is used to calculate the new qm, vf don't change. 
                
                JUST change the self.N where t is with in (ykv,gv)
                
                Input: LaneFD_variation
                        from (0, infinit)
                Input:  ykv, gv
                        the onset and the termination of the moment where self.N changes.
                """
                
                if ykv<self.ts[0]:ykv=self.ts[0]
                if ykv>self.ts[-1]:gv=self.ts[-1]
                self.InsertMoment(ykv)
                self.InsertMoment(gv)
                #
                idx1 = self.idx_moment(ykv)
                idx2 = self.idx_moment(gv)
                for idx in range(idx1, idx2):
                        deltan = self.N[idx+1]-self.N[idx]
                        deltat = self.ts[idx+1]-self.ts[idx]
                        q = 3600.0*deltan/deltat
                        #adjust the flow rate
                        increment_q = (LaneFD_variation-1.0)*q*deltat/3600.0
                        self.fifo_adjust(t = self.ts[idx], delta_n = increment_q, qm = qm)
                
                self.ConvertedCumulativeCurve(qm=qm)
                
        def fifo_adjust(self, t, delta_n, qm = FD.qm):
                """
                TEST OK.
                in the multilane, if there is FIFO violation, the self.N should be adjusted. 
                self.ts[idx]=t
                and self.N[idx] will be changed. 
                
                No return value. self is changed.
                ----------------------------------------------------
                Input: t
                        both scalar, the onset and termination moment of the 
                Input: delta_n
                        delta_n maybe positive or negtive
                        self.N[some_idx+1] = self.N[some_idx]+ delta_n
                        
                        if delta_n>0, then self is faster then other demands
                Input: qm = FD.qm
                        make sure that the flow rate is small enough
                ---------------------------------------------------
                Steps:
                        
                """
                if delta_n==0:return
                if t>=self.ts[-1] or t <=self.ts[0]:return
                self.InsertMoment(t)
                #the idx of t in self.ts is np.searchsorted(self.ts,t)
                #therefore self.N[idx]+1 will be changed.
                idx = np.searchsorted(self.ts,t)-1
                
                #
                if delta_n<0:
                        #not decreasing
                        delta_n_0 = self.N[idx+1] - self.N[idx]
                        delta_n=max(delta_n, -delta_n_0)
                        #maintail the maximum qm
                        max_delta_n_qm = self.N[idx+1] - (self.N[idx+2]-(self.ts[idx+2] - self.ts[idx+1])*qm/3600.0)
                        delta_n=max(delta_n, -max_delta_n_qm)
                else:
                        #non decreasing
                        delta_n_0 = self.N[idx+2] - self.N[idx+1]
                        delta_n=min(delta_n, delta_n_0)
                        #maintail the maximum qm
                        max_delta_n_qm  = self.N[idx]+(self.ts[idx+1] - self.ts[idx])*qm/3600.0  - self.N[idx+1]
                        delta_n=min(delta_n, max_delta_n_qm)
                self.N[idx+1] = self.N[idx+1]+delta_n
                
                self.ConvertedCumulativeCurve(qm=qm)
                
                
        def idx_moment(self, t):
                if t<self.ts[0] or t>self.ts[-1]:raise ValueError('t not in domain')
                if t not in self.ts:self.InsertMoment(t)
                return np.searchsorted(self.ts,t)
        
    def reset_demand1(self,list_tuple_of_three_values, baseflow =10, T = 3600, deltat = 2, qm = FD.qm):
        """
        reset the self.ts self.N and self.CCC.
        ----------------------------------------
        input:  list_tuple_of_three_values:
            each element is (l,r,m), l means the left of the parabolic curve, r means right, m means maximum
        
        
        """
        
        #define the self.ts and self.q
        ts = list(np.arange(0,T,deltat))
        q  = list(np.arange(0,T,deltat))
        for i,t in enumerate(ts):
            res = -np.inf
            for j,parabolic in enumerate(list_tuple_of_three_values):
                l = parabolic[0]
                r = parabolic[1]
                maximum = parabolic[2]
                a = -4.0*maximum/((l-r)*(l-r))
                res = max([1.0*a*(t-l)*(t-r), res])
            q[i] = min([FD.qm,max([res,baseflow])])
        
        self.y = q
        self.ts = ts
        
        #compute the self.N and self.CCC
        self.CumulativeCurve()
        
        #conpute the converted cumulative curve.
        self.ConvertedCumulativeCurve()
        
        #store the raw
        self.raw_ts = copy.deepcopy(self.ts)
        self.raw_N = copy.deepcopy(self.N)
        self.raw_CCC = copy.deepcopy(self.CCC)
        
    def __init__(self,maximum=400, l=100, r=400, offset=600,baseflow =10, T=3600, deltat = 1):
        """
        生成流量, 模式是 常数-----抛物线-----常数
        
        --------------------------
        返回值:
            ts和value的列表,其中, ts必须是递增的, value是t时刻的demand,单位veh/h.
        
        --------------------------
        输入:maximum
            是抛物线最大值
        输入: l和r
            是抛物线经过的横轴两个点,(l,0)和(r,0)
            抛物线 用 f = a(x-l)(x-r) 来表示, 只需要求出a就可以
        输入: baseflow
            baseflow 是生成的结果都加上这个数字
        输入: offset
            是抛物线向后偏移多远的距离
        -----------------------------
        Initialized attributes:
            self.ts, 
            self.y
            self.N
            self.OverallVehicleN
        and the raw data, 
            self.raw_ts
            self.raw_y
            self.raw_N
        """
        
        #error debug
        #self.qdebug = mydebug()
        
        a = -4.0*maximum/((l-r)*(l-r))
        
        fun = lambda t:1.0*a*(t-l)*(t-r)
        
        ts = np.arange(0, T, deltat)
        y = []
        for t in ts:
            if t<l:
                y.append(1.0*baseflow)
            elif t>r:
                y.append(1.0*baseflow)
            else:
                y.append(1.0*fun(t)+baseflow)
        self.ts=list(1.0*ts)
        self.y=y
        
        self.raw_ts = copy.deepcopy(self.ts)
        self.raw_y=y
        self.CumulativeCurve()
        self.raw_N = copy.deepcopy(self.N)
        #Overall Vehicle Number
        self.ConvertedCumulativeCurve()
        self.raw_CCC = copy.deepcopy(self.CCC)
        
        #the following property is set just for lanes at U-section.
        #   where the reconstruction of queue is needed. 
        #used in the queue reconstruction, when self.N and self.demand_after_spillover are available. 
        #   by the method self.ReconstructQueueProfileFromN()
        #self.spillovered_dmd = {}
        
        self.OverallVehicleN = self.N[-1]
        
    def __init__1(self,fun=lambda t:-100.0/(3600**2)*t*(t-7200), startmoment=0, T=7200, deltat = 30):
        """
        Input a univariable function which denotes the demand along time
        
        default function fun go across three points:
            (0,0),(3600,100) and (7200,0)
        T is the sampling time length, or time domain duration, default 2 hours
        
        input:startmoment
            the start moment of the self.TimeIntervals
        input:deltat(seconds)
            the interval length of the demand, within which is stable
        input: T
            the time length of the demand, default 2 hours
        
        OUtput:
            self.TimeIntervals
            self.Demand
            self.UpperValues and self.LowerValues
            self.MinimalMoment and self.MaximalMoment
            self.IntervalLength 
        
        """
        GeneralContinuousIntervals.__init__(self)
        ######Set the intervals
        #set TimeIntervals and LowerValues and UpperValues
        
        #moments correspondent to each RGP, i.e. for non oversaturated condition
        # it is from characteristic moment a to next RGP's a.
        # the length is always equal to LaneDynamics.a
        #   i.e. for the 0-th rgp, the interval is self.RGP_moments[0], self.RGP_moments[1]
        self.RGP_moments = sllist()
        
        
        Moments= np.arange(startmoment, startmoment + T + deltat, deltat)
        
        self.TimeIntervals  =   pyinter.IntervalSet()
        for s,e in zip(Moments[0:-1],Moments[1:]):
            self.TimeIntervals.add(pyinter.interval.open(s,e))
        
        #Set the demand
        self.Demand =   sllist(fun(np.array(Moments[0:-1])))
        
        self.refreshAfterTimeIntervalsChanged()
        
    def DemandPickUp(self,s,e):
        """
        TESTED_OK
        
        Pick up some demand between time moments s and e
        
        Steps:
            1)Split the moments
            2)fine the start_idx and end_idx of s and e
            3)Output the results
        
        Input:
            s: starting moment,scalar
            e: ending moment, scalar
        Output:
            PartialDemand, RouteDemand instance. 
        
        """
        if s >= e:
            raise NameError('s should be smaller than e')
        
        #init the returning value and set the main property to null
        PartialDemand   =   RouteDemand(fun=lambda t:.0*t, T=e-s,deltat=e-s)
        PartialDemand.TimeIntervals =   pyinter.IntervalSet()#normally unit in seconds
        PartialDemand.Demand            =   sllist()#normally unit in veh/h
        
        #Split the time moment.
        self.DemandSplitTime(set([s,e]))
        
        #find the idx of s and e in self.TimeIntervals
        idx_s   =   self.idx_Left(s)
        idx_e   =   self.idx_Right(e)
        
        PartialDemand.TimeIntervals = pyinter.IntervalSet(sorted(self.TimeIntervals)[idx_s:idx_e+1]) 
        
        PartialDemand.Demand = sllist(list(self.Demand)[idx_s:idx_e+1])
        
        #refresh AFFILIATED properties
        PartialDemand.refreshAfterTimeIntervalsChanged()
        
        return PartialDemand
    
    def DemandAlignTimeIntervals_partial(self,AnotherRouteDemand):
        """
        Align TimeIntervals of two RouteDemand instance.
        the AnotherRouteDemand time domain (s,e) should be within the self time domain.
        After the running, the time intervals between (s,e) of self and AnotherRouteDemand are the same. 
        The AnotherRouteDemand time domain will NOT extended to be identical with self time domain.
        -------------------------------------
        input: AnotherRouteDemand
            default a zero demand RouteDemand intance
        output: no return
            the self.Demand is changed. 
        ------------------------------------
        Step:
            1) Get the CommonMoment within AnotherRouteDemand.MinimalMoment and AnotherRouteDemand.MaximalMoment
            
            2) Spilt the Demand using 
        
            
        """
        #the commons within AnotherRouteDemand.MinimalMoment and AnotherRouteDemand.MaximalMoment
        CommonMoments   =   set()
        CommonMoments.update(AnotherRouteDemand.LowerValues)
        CommonMoments.update(set([AnotherRouteDemand.MaximalMoment]))
        
        #function to filter the moments within AnotherRouteDemand.MinimalMoment and AnotherRouteDemand.MaximalMoment
        def f(x): return x<=AnotherRouteDemand.MaximalMoment and x>=AnotherRouteDemand.MinimalMoment 
        #   tmp is the moments set in self which are within the time domain
        tmp=copy.deepcopy(self.LowerValues)
        tmp.update(set([self.MaximalMoment]))
        CommonMoments.update( set(filter(f, tmp)) )
        
        #split the time moments
        self.DemandSplitTime(CommonMoments)
        AnotherRouteDemand.DemandSplitTime(CommonMoments)
        
    
    
    def DemandOverlap(self, AnotherRouteDemand):
        """
        TESTED_OK
        Overlap of two demands. Both are of the RouteDemand class.
        No return, 
        Before this method, the RouteDemand @@@@@@@@should be ALIGNED@@@@@@@@
        by self.DemandSplitTime method
        thus the TimeIntervals property should be the same. 
        
        input: AnotherRouteDemand
            default a zero demand RouteDemand intance
        output: no return
            the self.Demand is changed. 
        
        """
        
        #self.DemandAlignTimeIntervals(AnotherRouteDemand)
        
        #if the size of demand is not the same, then it is an error
        if self.Demand.size != AnotherRouteDemand.Demand.size:
            print(self.Demand.size,'-----',AnotherRouteDemand.Demand.size)
            raise IndexError('the length of the Demans are not the same')
            return
        
        #for demand in each interval
        for idx in range(self.Demand.size):
            self.Demand.nodeat(idx).value=self.Demand.nodeat(idx).value+AnotherRouteDemand.Demand.nodeat(idx).value
        
    def DemandAlignTimeIntervals(self, AnotherRouteDemand):
        """
        Align the time moments of all demand.
        After the alignment, TimeIntervals of Both the self and AnotherRouteDemand
        Should be the same
        
        Outputs:
            no outputs, both the self.TimeIntervals and self.Demand are changed
            self.TimeIntervals and AnotherRouteDemand.Demand are changed
        """
        
        #Step1: Get the common time moments in RouteDemand.TimeIntervals
        CommonMoments   =   set()
        CommonMoments.update(self.LowerValues)
        CommonMoments.update(set([self.MaximalMoment]))
        CommonMoments.update(AnotherRouteDemand.LowerValues)
        CommonMoments.update(set([AnotherRouteDemand.MaximalMoment]))
        
        #Step2: Align the TimeIntervals of all routes
        #after the alignment, the TimeIntervals for all routes are the same
        #thus can be operated by sum
        self.DemandSplitTime(CommonMoments)
        AnotherRouteDemand.DemandSplitTime(CommonMoments)
        
    def DemandRescale1(self, ratio=0):
        """
        decrease the demand of all intervals to (1-ratio)
        if ratio =1 it means that all demand is set ot zero.
        
        
        """
        
        
        
        
        self.refreshAfterTimeIntervalsChanged()
        pass
    
    def DemandOverlapPartial_extended(self,PartialDemand):
        """
        Different from self.DemandOverlapPartial(), the input is a dict
        Each key is an pyinter.interval, each value is respective flowrate
        ---------------------------------
        Input:PartialDemand
            RouteDemand instance, the PartialDemand.MinimalMoment and PartialDemand.MaximalMoment
            should be within the self.MinimalMoment and self.MaximalMoment
        Output:
            No output, but the time intervals and demand may be changed
        ---------------------------------
        Steps:
            1) self.DemandSplitTime() and RouteDemand.DemandSplitTime()
            2) find the index of the PartialDemand.MinimalMoment and PartialDemand.MaximalMoment, i.e. idx_s and idx_e in self.DemandSplitTime()
            3) for each range(idx_s,idx_e+1), 
                self.Demand.nodeat(idx).value=self.Demand.nodeat(idx).value+AnotherRouteDemand.Demand.nodeat(idx).value
        
        """
        while self.MinimalMoment >= PartialDemand.MinimalMoment:
            self.ExtendDemandBackward()
        while self.MaximalMoment <= PartialDemand.MaximalMoment:
            self.ExtendDemandForward()
        #Step 1:self.DemandSplitTime() and RouteDemand.DemandSplitTime()
        self.DemandAlignTimeIntervals_partial(PartialDemand)
        #Step 2:
        idx_s   = self.idx_Left(PartialDemand.MinimalMoment)
        idx_e   = self.idx_Right(PartialDemand.MaximalMoment)
        #Step 3:
        for idx,idy in zip(range(idx_s,idx_e+1),range(PartialDemand.Demand.size)):
            self.Demand.nodeat(idx).value=self.Demand.nodeat(idx).value+PartialDemand.Demand.nodeat(idy).value
        
    def DemandOverlapPartial(self,start_t ,end_t , flowrate):
        """
        This method just takes three parameters:
            starting moment of the demand, as start_t
            ending moment of the demand, as end_t
            and flow rate at this time domain.
        
        Steps:
            1) Align the self.TimeIntervals, therefore start_t and end_t are covered by
                self.uppervalues or self.lowervalues
            2) find the idx of start_t,
            3) from idx, add the flowrate, until the end_t
            
        Input:start_t ,end_t , flowrate
            start_t ,end_t unit is in sec
            flowrate unit is in veh/h
        Output:
            No output, but the time intervals and demand may be changed
        """
        if 1.0*start_t==1.0*end_t:
            return
        elif start_t>end_t:
            raise ValueError('start_t should be smaller than or equal to end_t')
        
        #Step 1: align the self.TimeIntervals
        self.DemandSplitTime(set([start_t,end_t]))
        
        #step 2:find the idx of the interval which start_t is in
        #   each interval is as [a,b) then if t is in [a,b) return this idx
        idx =   self.idx_Left(start_t)
        
        #step 3: from idx, add the flowrate, until the end_t
        for idxx in range(idx, self.Demand.size):
            #print("addddd")
            self.Demand.nodeat(idxx).value  = self.Demand.nodeat(idxx).value + flowrate
            #if condition satisfies, then idx is the last interval to be added
            if sorted(self.UpperValues)[idxx]==end_t:break
    def copy(self):
        """
        deep copy of the RouteDemand, since copy.deepcopy(sllitobject)
        do not work properly. 
        
        """
        res  = RouteDemand ()
        res.TimeIntervals = copy.deepcopy(self.TimeIntervals)
        res.Demand = sllist( list(self.Demand) )
        
        res.refreshAfterTimeIntervalsChanged()
        
        return res
    
    def GetFlowRate(self):
        """
        
        """
        idx_es  = np.diff(self.ts)<1.0e-5
        if sum(idx_es) != 0:
            new_ts = list(np.array(self.ts)[~idx_es])
            new_N = list(np.array(self.N)[~idx_es])
        else:
            new_ts = self.ts
            new_N = self.N
        
        q= list(np.around(3600.0*np.diff(new_N)/np.diff(new_ts),2))
        #plot(t,y)
        self.q = q
        
    def PlotFlowRate(self, ax = False, maxx = np.inf, minn = -np.inf, label = "Left-turn flow rate", filename = 'Temp.png',dpi=500):
        """
        plot the flow rate vs time.
        Since the self.ts and self.N is the same length, then the flow rate should be computed.
        The resulting q is with length len(self.ts)-1. 
        The corresponding moments is self.ts[1:].
        Note that the self.ts is in seconds
        and the self.N is in number of vehicles.
        --------------------------------
        Input:maxx and minn
            due to the error, some q may be greater than q, or some flow may be negative
        -----------------------------
        
        """
        
        #eliminate the error.
        idx_es = np.array(self.ts) > -np.inf
        tmp  = np.where(np.diff(self.ts)<1.0e-5)[0]
        if len(tmp) > 0:
            for i in tmp:idx_es[i]= False
            new_ts = list(np.array(self.ts)[idx_es])
            new_N = list(np.array(self.N)[idx_es])
        else:
            new_ts = np.array(self.ts)
            new_N = np.array(self.N)
        
        q= np.around(3600.0*np.diff(new_N)/np.diff(new_ts),2)
        q[q>maxx] = maxx
        q[q<minn] = minn
        #plot(t,y)
        if not ax:
                
                        fig, ax = plt.subplots()
    
        ax.plot(new_ts[1:],q, label = label);
        ax.set_xlabel("Time (in sec)")
        ax.set_ylabel("Flow rate (in veh/h)")
        ax.legend(loc=0)
        plt.savefig(filename, dpi=dpi)


        return ax
    
    
    def PlotFlowRate_backup(self):
        """
        plot the flow rate vs time.
        Since the self.ts and self.N is the same length, then the flow rate should be computed.
        The resulting q is with length len(self.ts)-1. 
        The corresponding moments is self.ts[1:].
        Note that the self.ts is in seconds
        and the self.N is in number of vehicles.
        
        """
        
        #eliminate the error.
        idx_es  = np.diff(self.ts)<1.0e-5
        if sum(idx_es) != 0:
            new_ts = list(np.array(self.ts)[~idx_es])
            new_N = list(np.array(self.N)[~idx_es])
        else:
            new_ts = self.ts
            new_N = self.N
        
        q= list(np.around(3600.0*np.diff(new_N)/np.diff(new_ts),2))
        #plot(t,y)
        fig, ax = plt.subplots()
    
        ax.plot(new_ts[1:],q);
        ax.set_xlabel("Time (in sec)")
        ax.set_ylabel("Flow rate (in veh/h)")
        
        plt.show()
        
    
    
    
    
    def PlotDemand1(self):
        """
        
        """
        #plot(t,y)
        fig, ax = plt.subplots()
        T = self.ts
        Y =self.y
        ax.plot(T,Y);
        ax.set_xlabel("Time (in sec)")
        ax.set_ylabel("Flow rate (in veh/h)")
        
        plt.show()
        
        #return T,Y


    def ClearRouteDemandAfter(self,s):
        """
        When self spillover, suppose the duration is (s,e), then the demand after s should be discarded, given that the lane is not run isolatedly. 
        
        Input s is the moment after which the demand should be cleared (NOT set to zero)
        ----------------------------------------------------------------
        Input: s
            the moment, scalar. 
            If the s is smaller than self.MinimalMoment, then firstly the self will be extended backward to make sure the self time domain cover s.
        ----------------------------------------------------------------
        Steps:
                1) Extend the demand if necessary;
                2) clear the time intervals using self.TimeIntervals.difference() method
                3) clear the demand by self.Demand.popright()
        
        """
        #no need to be cleared
        if s>=self.MaximalMoment:
            return
        
        if s<=self.MinimalMoment:
            #this means that all demand are cleared, thus here 1st interval and 1st demand is remained., and 1st demand is set to 0 either.
            
            tmp = pyinter.interval.open(self.MinimalMoment,self.MinimalMoment + self.IntervalLength[0])
            self.TimeIntervals  =   pyinter.IntervalSet()
            self.TimeIntervals.add(tmp)
            self.Demand = sllist([0])
            
            self.refreshAfterTimeIntervalsChanged()
            return
        
        #to make sure that the self time domain include moment s.
        self.DemandSplitTime(t=set([s]))
        #find the idx of s in self.TimeIntervals
        idx = self.idx_Left(s)
        #delete the time intervals.
        self.TimeIntervals = self.TimeIntervals.difference(pyinter.interval.open(s,self.MaximalMoment))
        #delete demand
        self.Demand = sllist(list(self.Demand)[0:idx])
        
        #for i in range(idx,self.Demand.size):_=self.Demand.popright()
        self.refreshAfterTimeIntervalsChanged()
    
    def DemandRescale2(self,saturationflowrate=1800):#saturationflowrate=LaneDemand.FD.qm
        """
        Due to the possibility that the demand will exceed the saturation flow rate
        the demand should be rescaled to accomodate this condition,
        i.e. matain the lane demand within the threshold(saturation flow rate)
        
        Input: saturationflowrate, unit in veh/hour
        
        Output:no return, but the self.Demand is changed (self.TImeIntervals remain unchanged if there is no remained vehicle in after the last interval)
        
        """
        #print('length of Interval length',self.IntervalLength.size, 'length of TimeIntervals: ',len(self.TimeIntervals))
        
        #for the first interval, set the accumulated remained flow as zero
        remainedvehiclenumber = 0
        for idx,interval in enumerate(sorted(self.TimeIntervals)):
            
            #interval duration, unit is in hours
            intervallength = 1.0*self.IntervalLength[idx]/3600.0
            
            #plus the remained flow to renew each interval demand, self.Demand.nodeat(idx) unit is veh/h
            self.Demand.nodeat(idx).value = self.Demand.nodeat(idx).value + remainedvehiclenumber*1.0/(intervallength)
            
            #determine whether the flowrate exceeds the threshold
            #flow rate which will be added to next interval, in veh. 
            if self.Demand.nodeat(idx).value > saturationflowrate:
                #note the order to update,first remainedvehiclesnumber
                remainedvehiclenumber = (self.Demand.nodeat(idx).value-saturationflowrate)*intervallength
                self.Demand.nodeat(idx).value = saturationflowrate
                
            else:
                #no vehicles for left interval
                remainedvehiclenumber = 0
        
        #if still remained for the whole time domain
        #, then extend the Demand Time orizon
        while remainedvehiclenumber>0:
            #First: extend the intervals. 
            #   the self.Demand and TimeIntervals and affliated variables
            #   are changed
            self.ExtendDemandForward()
            
            #compute the flow rate and remained flow rate
            #index as -1 means the last node
            intervallength = 1.0*self.IntervalLength.nodeat(-1).value/3600
            self.Demand.nodeat(-1).value = self.Demand.nodeat(-1).value + remainedvehiclenumber*1.0/(intervallength)
            
            #determine whether the flowrate exceeds the threshold
            #flow rate which will be added to next interval, in veh. 
            if self.Demand.nodeat(-1).value > saturationflowrate:
                remainedvehiclenumber = (self.Demand.nodeat(-1).value-saturationflowrate)*intervallength
                self.Demand.nodeat(-1).value = saturationflowrate
            else:
                remainedvehiclenumber = 0
        
        self.refreshAfterTimeIntervalsChanged()
    
    
    def PrecisionControl(self,precision = 1e-6):
        """
        
        
        """
        idx_es = np.array(self.ts) > -np.inf
        tmp  = np.where(np.diff(self.ts)<=precision)[0]
        if len(tmp)==0:return
        
        for i in tmp:
            idx_es[i] = False
        
        new_ts = list(np.array(self.ts)[idx_es])
        new_N = list(np.array(self.N)[idx_es])
        new_CCC = list(np.array(self.CCC)[idx_es])
        
        self.ts = new_ts
        self.N = new_N
        self.CCC = new_CCC
        
    
    def Plot_raw_current_N(self):
        """
        
        """
        
        fig, (ax1, ax2) = plt.subplots(nrows=2, ncols=1)
        ax1.plot(self.ts , self.N, 'r',label='new')
        ax1.plot(self.raw_ts , self.raw_N, 'b',label='raw')
        ax1.legend()
        ax1.set_xlabel('Time (sec)')
        ax1.set_ylabel('N (Veh)')
        
        ax2.plot(self.ts , self.CCC, 'r',label='new')
        ax2.plot(self.raw_ts , self.raw_CCC, 'b',label='raw')
        ax2.legend()
        ax2.set_xlabel('Time (sec)')
        ax2.set_ylabel('g(x) (sec)')
        
    def PlotDemand(self):
        """
        plot the demand. 
        
        """
        #construct the t axis, which is the middle moment of each interval
        T=[self.MinimalMoment]
        Y=[0]
        for interval,d in zip(sorted(self.TimeIntervals),self.Demand) :
            T.extend([interval.lower_value, interval.upper_value])
            Y.extend([d , d])
        
        T.append(self.MaximalMoment)
        Y.append(0)
        
        #plot(t,y)
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(T,Y);
        plt.show()
        
        return T,Y



class LaneDemand(RouteDemand):
    """
    definition of lane demand, which consists of many route demands. 
    Still the decendent of RouteDemand
    
    
    
    """

    
    def __init__(self, RouteLabels=[1] ,fun=lambda t:-100.0/(3600**2)*t*(t-7200), startmoment =0, T=7200, deltat=30):
        """
        Add the following properties:
            self.RoutesDemands as a dict;
            self.plm as a dict. 
        
        ----------------------------------------
        RouteLabels:
            is the labels for all routes. It shoud be iterable. 
        fun:
            the funnction on which the route demand is generated
        T:
            the time length for the lane demand. 
        -----------------------------------------
        """
        #set route demands attribute
        self.RoutesDemands  = {}
        for label in RouteLabels:
            self.RoutesDemands[label]=RouteDemand(fun=fun, startmoment =startmoment, T=T, deltat=deltat)
        
        #init the parement class. 
        RouteDemand.__init__(self,fun=lambda t:.0*t, startmoment =startmoment, T=T,deltat=deltat)
        
        
        #change the self.Demand and self.TimeIntervals
        self.RoutesDemandsAggregation()
        
        #Proportional Line Model implementation
        #   the array shape is (number of routes +1, N)
        #   N is the same as the returning value of self.VehicleN(), which returns (M,N) shape array
        #   the reason number of routes +2 is that the 1st row is moment
        #   the 2-nd row is the overall demand and 3rd to last is each route demand vehicle number. 
        tmp     =   self.VehiclesNumber()
        self.plm={}
        self.plm['moments']=tmp[0,:]
        self.plm['overall']=tmp[1,:]
        for label in RouteLabels:
            self.plm[label]=self.RoutesDemands[label].VehiclesNumber()[1,:]
            
    def RescaleLaneDemand_by_spillover(self,s,e,qm=FD.qm):
        """
        overload GeneralContinuousIntervals.RescaleDemand_by_spillover.
        The GeneralContinuousIntervals.RescaleDemand_by_spillover()
        do not change the self.RoutesDemands properties. 
        
        
        """
        
        #the method from GeneralContinuousIntervals class.
        #this just change the overall demand, i.e. self.Demand and self.TimeIntervals
        #   together with its affiliated variables.
        self.RescaleRouteDemand_by_spillover(s=s,e=e,qm=qm)
        
        #default NewDemandAdded = False
        #   will change self.plm 
        #   if false, plm will not be changed. 
        self.refreshAfterTimeIntervalsChanged(refreshroutes = False)
        
        #set route demand by self.TimeIntervals and plm
        self.ResetRoutesDemands_from_PLM()
    
    def ResetDemandGivenRoutesDemands(self,HardRoutesDemands):
        """
        Given routes Demands, reset the LandDemand. 
        
        The returning value is not a reference to input, but a new value.
        Thus at first the input value is copyed. 
        ----------------------------------------
        Input: HardRoutesDemands
            dict, HardRoutesDemands[routelabel] is a RouteDemand class. 
        -------------------------------------------
        Steps:
            1) align the intervals of all RouteDemand;
            2) overlap demand to overall demand;
            3) point each self.RoutesDemands[routelabel] to the RoutesDemands[routelabel]
        """
        
        #copy the input RoutesDemands.
        RoutesDemands = {}
        for label,rdemand in HardRoutesDemands.iteritems():
            RoutesDemands[label]= rdemand.copy()
        
        #===============1) Align the time moments of all routes--->commonmoments
        #Get the moments for each routedemand;
        #allmoments[routelael] is a set
        allmoments  =   {}
        for label,rdemand in RoutesDemands.iteritems():
            allmoments[label] = set()
            allmoments[label].update(rdemand.OverallMoments)
        #commonments among all routes, is a set. 
        commonmoments = set()
        for moments in allmoments.itervalues():
            commonmoments.update(moments)
        
        #=============2) For each route, align the Time intervals
        for label,rdemand in RoutesDemands.iteritems():
            residulemoments = commonmoments.difference(allmoments[label])
            #if the len is 0, no need to split
            if len(residulemoments)>0:rdemand.DemandSplitTime(residulemoments)
        
        #=============3) Overlap to overall demand; 
        somelabel = random.choice(allmoments.keys())
        #Set the TimeIntervals
        self.TimeIntervals = copy.deepcopy(RoutesDemands[somelabel].TimeIntervals)
        #set the demands
        #   first set demand to zero
        demand_np = 0 * np.array(list(RoutesDemands[somelabel].Demand))
        for label,rdemand in RoutesDemands.iteritems():
            demand_np = demand_np + np.array(list(rdemand.Demand))
        #tranform the demand from np.array to sllist.
        self.Demand = sllist(demand_np)
        
        #==============4) set each route demand. 
        for label,rdemand in RoutesDemands.iteritems():
            self.RoutesDemands[label] = rdemand
        
        self.refreshAfterTimeIntervalsChanged(NewDemandAdded = True)
        
    def ResetPLM(self):
        """
        when self.Demand and self.TimeIntervals are changed due to outside demand input, the plm need to be changed. This function of this method is also included in the 
        self.refrenshAfterTimeIntervalsChanged().
        ---------------------------
        Output:
            no output, the self.plm is changed.
        
        
        """
        tmp     =   self.VehiclesNumber()
        self.plm={}
        self.plm['moments']=tmp[0,:]
        self.plm['overall']=tmp[1,:]
        for label in self.RoutesDemands.keys():
            self.plm[label]=self.RoutesDemands[label].VehiclesNumber()[1,:]
        
    def ResetRoutesDemands_from_PLM(self):
        """
        When overall demand changed, then the routes demand shoule be also changed. 
        This method assumes that overall demand and time intervals are changed, and no outside demand input, thus the routedemand do not need to be re-calculated based on the self.plm property. 
        
        Not that the self.Demand and self.TimeIntervals should be consistent. 
        --------------------------------------------------
        Output:
            self.RoutesDemands will be changed
        --------------------------------------------------
        Steps:
            1) get the overall time intervals and respective cumu vehicle numbers
            2) assign each route the self.TimeIntervals and 
            3) calculate the cumu_N for each moment 
                and then set the self.RoutesDemands[routelabel].Demand
        """
        
        #===================== make sure the plm is refreshed.
        #self.ResetPLM()
        
        #=====================2) 
        #shape of tmp is (2,(self.Demand.size)+1)
        tmp=self.VehiclesNumber()
        routes_N    =   self.PLM_get_routes_N(tmp[1,:])
        
        for routelabel in self.RoutesDemands.keys():
            #1 Get the time intervals
            #       Set the time intervals
            self.RoutesDemands[routelabel].TimeIntervals = copy.deepcopy(self.TimeIntervals)
            #2 Set the demand based on plm, not that it should be veh/h.
            self.RoutesDemands[routelabel].Demand = sllist(3600.0*np.divide(routes_N[routelabel][1:]-routes_N[routelabel][0:-1],1.0*self.IntervalLength))
            
            self.RoutesDemands[routelabel].refreshAfterTimeIntervalsChanged()
    
    
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
        if t <= 0:
            raise ValueError('t should be positive')
        
        
        #Extend time intervals
        self.TimeIntervals.add(pyinter.interval.open(self.MaximalMoment,self.MaximalMoment + t))
        #Extend the demand as zero
        self.Demand.appendright(0)
        
        #Extend each route.
        for label in self.RoutesDemands.keys():
            self.RoutesDemands[label].TimeIntervals.add(pyinter.interval.open(self.MaximalMoment,self.MaximalMoment + t))
            self.RoutesDemands[label].Demand.appendright(0)
        
        #refresh
        self.refreshAfterTimeIntervalsChanged(NewDemandAdded=True)
    
    def ExtendDemandBackward(self,t=3600):
        """
        extend the demand backward by t.
        default is 3600 seconds, i.e. 2 hours.
        After the sxtension, the time intervals and Demand is changed. 
        
        Before this method operation, the time intervals of self.TimeIntervals and 
        self.RoutesDemands[someroute] should be identical. 
        
        -----------------------------------
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
        
        #Extend each route.
        for label in self.RoutesDemands.keys():
            self.RoutesDemands[label].TimeIntervals.add(pyinter.interval.open(self.MinimalMoment-t,self.MinimalMoment))
            self.RoutesDemands[label].Demand.appendleft(0)
        
        #refresh
        self.refreshAfterTimeIntervalsChanged(NewDemandAdded=True)
    
    def refreshAfterTimeIntervalsChanged(self, refreshroutes = True, NewDemandAdded=False):
        """
        since the self.TimeIntervals changed, other affiliated variables should be changed
        including:
            lower_values,
            upper_values,
            MinimalMoment,
            MaximalMoment
            plm
        --------------------------------------
        Input: NewDemandAdded
            whether there is new demand added to self. If yes, then the plm should also be changed.
        
        """
        #=================================== overall demand
        #lower_values and upper_values
        self.LowerValues = set([interval.lower_value for interval in self.TimeIntervals])
        self.UpperValues = set([interval.upper_value for interval in self.TimeIntervals])
        #minimal and maximal moment of the instance time domain
        self.MinimalMoment=sorted(self.TimeIntervals)[0].lower_value
        self.MaximalMoment=sorted(self.TimeIntervals)[-1].upper_value
        self.IntervalLength = np.array(sorted(self.UpperValues))-np.array(sorted(self.LowerValues))
        #a list
        self.OverallMoments = sorted(self.LowerValues.union([self.MaximalMoment]))
        
        self.OverallVehicleNumbers  =   self.VehiclesNumber()[1,-1]
        
        #=================================== each route demand
        if refreshroutes:
            for _,routedemand in self.RoutesDemands.iteritems():
                routedemand.LowerValues = set([interval.lower_value for interval in routedemand.TimeIntervals])
                routedemand.UpperValues = set([interval.upper_value for interval in routedemand.TimeIntervals])
                #minimal and maximal moment of the instance time domain
                routedemand.MinimalMoment=sorted(routedemand.TimeIntervals)[0].lower_value
                routedemand.MaximalMoment=sorted(routedemand.TimeIntervals)[-1].upper_value
                routedemand.IntervalLength = np.array(sorted(routedemand.UpperValues))-np.array(sorted(routedemand.LowerValues))
                        #a list
                routedemand.OverallMoments = sorted(routedemand.LowerValues.union([routedemand.MaximalMoment]))
                routedemand.OverallVehicleNumbers   =   routedemand.VehiclesNumber()[1,-1]
        
        if NewDemandAdded:
            self.ResetPLM()
        
        #check the timeinterval number and the size of self.Demand.
        if self.Demand.size != len(self.LowerValues):
            raise ValueError('zero interval length')
    
    def copy(self):
        """
        deep copy of the RouteDemand, since copy.deepcopy(sllitobject)
        do not work properly. 
        
        """
        res  = LaneDemand (RouteLabels=self.RoutesDemands.keys() ,fun=lambda t:.0*t, startmoment =0, T=20, deltat=10)
        
        res.TimeIntervals = copy.deepcopy(self.TimeIntervals)
        res.Demand = sllist( list(self.Demand) )
        
        for rl in self.RoutesDemands.keys():
            res.RoutesDemands[rl] = self.RoutesDemands[rl].copy()
        
        res.refreshAfterTimeIntervalsChanged(NewDemandAdded=True)
        
        return res
        
    def Check_Consistency(self):
        """
        check 
            * the time moments of self.Demand and self.RoutesDemand are aligned or not. 
            * the sum of all route demand is overall demand. (will cause truncate error.)
        
        If not, raise ValueError
        
        """
        #=========================check the time intervals of self and each route
        #Get all the Demand moments as set
        overallmoments = self.LowerValues.union(set([self.MaximalMoment]))
        for label in self.RoutesDemands.keys():
            #routemoments as set
            routemoments    =   self.RoutesDemands[label].LowerValues.union(set([self.RoutesDemands[label].MaximalMoment]))
            #check whether they are identical. 
            if len(routemoments.difference(overallmoments))>0 or len(overallmoments.difference(routemoments))>0:
                raise ValueError('Not Aligned between self.RoutesDemand and self.Demand')
        
        return
        
        #the following will cause truncate error!!!!!!!!!!!!!!
        #========================check the overall demand
        tmp = 0*self.plm['overall']
        for label in self.RoutesDemands.keys():
            tmp += self.plm[label]
        if not np.array_equal(self.plm['overall'], tmp):
            raise ValueError('Overall demand is not equal') 
    
    
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
            and the time moments for all the routes are also splited. 
        
        """
        
        #t is smaller than the minimal time horizon, extend the demand
        if min(t)<self.MinimalMoment:
            self.ExtendDemandBackward(t=self.MinimalMoment-min(t))
        #t is bigger than the maximal time horizon, extend the demand
        if max(t)>self.MaximalMoment:
            self.ExtendDemandForward(t=max(t)-self.MaximalMoment)
        
        #this iteration just change the self.OverallMoments and self.Demand, 
        #   after the interation, the self.TimeIntervals need to be re-constructured 
        #   from self.OverallMoments. 
        for moment in t:
            #if moment is NOT IN self.TimeIntervals, there is NO NEED to split
            #since each interval is OPEN both sides, it is necessary
            #if True, means that moment is either in LowerValues or in UpperValues
            if moment in self.OverallMoments:continue
            
            #step 1: fine the index, moment should be inserted before the index to maintain order
            idx =   bisect_left(self.OverallMoments, moment)
            #Step 2: change the properties, insert moment before idx
            self.OverallMoments.insert(idx, moment)
            #   split the demand, index in the sllist is idx
            #   just need to append the same value before node self.Demand.nodeat(idx)
            #   insertafter(x,node) insert x after node
            #   note the idx -1 .
            _=self.Demand.insertafter(self.Demand.nodeat(idx-1).value, self.Demand.nodeat(idx-1))
            #for each route
            for rdemand in self.RoutesDemands.itervalues():
                rdemand.Demand.insertafter(rdemand.Demand.nodeat(idx-1).value, rdemand.Demand.nodeat(idx-1))
        
        #Reset the self.TimeIntervals from self.OverallMoments
        self.Reset_TimeIntervals_by_OverallMoments()
        
        for rdemand in self.RoutesDemands.itervalues():
            rdemand.TimeIntervals = copy.deepcopy(self.TimeIntervals)

        #change all other affiliated variables
        self.refreshAfterTimeIntervalsChanged()
        
        
    
    def ClearLaneDemandAfter(self,s):
        """
        When self spillover, suppose the duration is (s,e), then the demand after s should be discarded, given that the lane is not run isolatedly. 
        
        Input s is the moment after which the demand should be cleared (NOT set to zero)
        ----------------------------------------------------------------
        Input: s
            the moment, scalar. 
            If the s is smaller than self.MinimalMoment, then firstly the self will be extended backward to make sure the self time domain cover s.
        
        """
        #no need to be cleared
        if s>=self.MaximalMoment:
            return
        
        if s<=self.MinimalMoment:
            #this means that all demand are cleared, thus here 1st interval and 1st demand is remained., and 1st demand is set to 0 either.
            tmp = pyinter.interval.open(self.MinimalMoment - self.IntervalLength[0], self.MinimalMoment)
            self.TimeIntervals  =   pyinter.IntervalSet();self.TimeIntervals.add(tmp)
            self.Demand = sllist([0])
            for rdemand in self.RoutesDemands.itervalues():
                rdemand.TimeIntervals = copy.deepcopy(self.TimeIntervals)
                rdemand.Demand = sllist([0])
            self.refreshAfterTimeIntervalsChanged(NewDemandAdded=True)
            return
        
        #clear the overall demand
        #   to make sure that the self time domain include moment s.
        self.DemandSplitTime(t=set([s]))
        #   find the idx of s in self.TimeIntervals
        idx = self.idx_Left(s)
        #   delete the time intervals.
        self.TimeIntervals = self.TimeIntervals.difference(pyinter.interval.open(s,self.MaximalMoment))
        #   delete demand
        self.Demand = sllist(list(self.Demand)[0:idx])
        
        #for each route
        for rdemand in self.RoutesDemands.itervalues():
            rdemand.TimeIntervals = copy.deepcopy(self.TimeIntervals)
            rdemand.Demand = sllist(list(rdemand.Demand)[0:idx])
        
        self.refreshAfterTimeIntervalsChanged(NewDemandAdded=True)
        
    def DemandAlignTimeIntervals_partial(self,AnotherLaneDemand):
        """
        Align TimeIntervals of two LaneDemand instance.
        
        The AnotherLaneDemand time domain (s,e) should be within the self time domain.
        However, if it is not the case, the function still works.
        
        After the running, the time intervals between (s,e) of self and AnotherLaneDemand are the same. 
        
        if both s and e are outside the self time domain. then the self.TimeIntervals are extended, either forward or backward by the GeneralClass.DemandSplitTime() method.
        
        -------------------------------------
        input: AnotherLaneDemand
            LaneDemand instance
        output: no return
            self.Demand and self.RoutesDemands are changed.
            AnotherLaneDemand.Demand and AnotherLaneDemand.RoutesDemands are changed.
        ------------------------------------
        Step:
            1) Get the CommonMoment within AnotherLaneDemand.MinimalMoment and AnotherLaneDemand.MaximalMoment
            2) Get the moments in self which is within the AnotherLaneDemand time domain.
            
            2) Spilt the Demand using 
        
            
        """
        #the commons within AnotherRouteDemand.MinimalMoment and AnotherRouteDemand.MaximalMoment
        CommonMoments   =   set()
        CommonMoments.update(AnotherLaneDemand.OverallMoments)
        
        #the moments of self which is within AnotherRouteDemand.MinimalMoment and AnotherRouteDemand.MaximalMoment
        partmoments = set([t for t in self.OverallMoments if t<=AnotherLaneDemand.MaximalMoment and t>=AnotherLaneDemand.MinimalMoment])
        
        CommonMoments.update( set(partmoments))
        
        #split the self time moments
        self.DemandSplitTime(CommonMoments)
        for label in self.RoutesDemands.keys():
            self.RoutesDemands[label].DemandSplitTime(CommonMoments)
        
        #split the AnotherLaneDemand time moments
        AnotherLaneDemand.DemandSplitTime(CommonMoments)
        for label in AnotherLaneDemand.RoutesDemands.keys():
            AnotherLaneDemand.RoutesDemands[label].DemandSplitTime(CommonMoments)
        
    def DemandPickup_qm(self, t_s, t_e, N_s, N_e, qm = FD.qm):
        """
        pick the upstream demand(i.e. self here) when the output is qm,and the starting and ending cumulative vehicles number are specified as N_s, N_e. The qm releasing dueation is from t_s to t_e. 
        N_e can be calculated based on the (t_s,t_e) of saturation release. 
        The resulting PartialLaneDemand is compressed and need to be transformed
        thus the resulting PartialLaneDemand.Demand are all qm, i.e. maximal flow rate.
        No need to SHIFT the self.TimeIntervls.
        
        It is assumed that the self.Demand is enough to sustain the saturational flow rate
        and it is assumed that the self.Demand is smaller than qm. 
        
        Note that there may be some numerical error, due to the computer. 
        Maybe the error between 1.0*3600*(N_s - N_e)/(t_s - t_e) are small enough. 
        -------------------------------------------------------------
        Input:t_s, t_e,
            starting moent and ending moment of the output flow which is qm, both are scalars
            
        Input:N_s, N_e
            Used to find the respective route cumulative vehicle number. 
            starting cumulative vehicle number and ending  cumulative vehicle number
            both are scalars. 
            
        Output: PartialLaneDemand
            LaneDemand instance 
        --------------------------------------------------------------
        Steps:
            1) Get the 1-dimentional np.array between starting vehicle number and ending vehicle number 
            2) 
        
        
        """
        if t_s >= t_e:
            raise ValueError('t_s should be smaller than t_e')
        if N_s >= N_e:
            raise ValueError('N_s should be smaller than N_e')
        
        # the flow rate between t_s and t_e must be qm.
        #   due to the computational numerical error
        #   the abs is applied here.
        if abs(1.0*3600*(N_s - N_e)/(t_s - t_e)-qm) >= 0.00002:
            print("the incorrect flowrate---->" + str(1.0*3600*(N_s - N_e)/(t_s - t_e)-qm))
            raise ValueError('flow rate between t_s and t_e is not qm!')
        
        if N_e - self.OverallVehicleNumbers > 0.00002:
            raise ValueError('N_e must be smaller than current overall vehicle number!')
        #consider the numerical error
        N_e = min([N_e, self.OverallVehicleNumbers])
        
        #Get the starting moment and ending moment in self.Demand, intert n_intervals data points
        #   thus each data point corresponds to a moment between (t_s,t_e)
        #       i.e. PartialLaneDemand.Demand and PartialLaneDemand.TimeIntervals 
        #       is determined by np.arange(1.0*N_s, 1.0*N_e, 1.0*(N_s-N_e)/10) and 
        #       np.arange(1.0*t_s, 1.0*t_e, 1.0*(t_s-t_e)/10)
        #   PartialLaneDemand.Demand--------->np.arange(1.0*N_s, 1.0*N_e, 1.0*(N_s-N_e)/10)
        #   PartialLaneDemand.TimeIntervals-->np.arange(1.0*t_s, 1.0*t_e, 1.0*(t_s-t_e)/10)
        # the following is to consider the numerical error.
        if (t_e-t_s)<=0.00002:
            n_intervals =   2.0#moments are just the [t_s,t_e]
        else:
            n_intervals =   5.0#len of resulting array is n_intervals.      
        
        #Get the respective route cumulative vehicle number
        moments =   np.linspace(1.0*t_s, 1.0*t_e, num= n_intervals)#1-dimentional np.array,
        cumu_N  =   np.linspace(1.0*N_s, 1.0*N_e, num= n_intervals)#1-dimentional np.array,
        
        #dict, routesN[routelabel] is a 1-dimentional np.array, the same size with cumu_N.
        routesN =   self.PLM_get_routes_N(N = cumu_N)
        
        #initialze the returning data
        #init the returning value and set the main property to zero
        PartialDemand   =   LaneDemand(RouteLabels=self.RoutesDemands.keys(),fun=lambda t:t*0.0, startmoment = t_s, T=t_e-t_s,deltat=t_e-t_s)
        
        #reset the data, note that the moments= moments, cumu_N = cumu_N assures that overall
        #   flow rate is qm. 
        PartialDemand.ResetData(moments= moments, cumu_N = cumu_N )
        
        for label in PartialDemand.RoutesDemands.keys():
            PartialDemand.RoutesDemands[label].ResetData(moments= moments, cumu_N = routesN[label])
        
        return PartialDemand
        
        
    def ShiftTimeIntervals(self,t=0.0):
        """
        Tested__OK
        shift the time domain by t. 
        i.e. each interval (s,e) becomes (s+t,e+t). t can be negative. 
        
        Other properties does not change. 
        
        """
        #tmp is the new shifted time intervals
        tmp =   pyinter.IntervalSet()
        for interval in sorted(self.TimeIntervals):
            s   =   interval.lower_value + t
            e   =   interval.upper_value + t
            tmp.add(pyinter.interval.open(s,e))
        
        self.TimeIntervals = tmp
        
        for routelabel in self.RoutesDemands.keys():
            self.RoutesDemands[routelabel].TimeIntervals=copy.deepcopy(tmp)
        
        self.refreshAfterTimeIntervalsChanged(NewDemandAdded=True)
        
    def DemandOverlapPartial(self,PartialDemand, NewDemandAdded = True):
        """
        Tested_OK
        over load the RouteDemand.DemandOverlapPartial() method. 
        
        Before the overlap, the time within self and PartialDemand should be considered as aligned.
        i.e. self.Demand and self.RoutesDemands[somelabel] is aligned.
        PartialDemand.Demand and PartialDemand.RoutesDemands[somelabel] is aligned.
        It does not matter if some of the routes of PartialDemand not in self routes. 
        @@@@@@but the self and PartialDemand does not need to be aligned.
        
        Input: PartialDemand
            LaneDemand instance. 
        Output: no output
            but the self.Demand and self.TimeIntervals are changed.
            
            
            each RoutesDemands are also changed either. 
            if the route in PartialDemand is not in self, then the route demand is not 
            added to self.Demand and self.RoutesDemand[someroute].Demand.
        
        --------------------------------------------------
        Steps:
            0) aligh the time domain and get the idx_s and idx_e, the starting and ending moment
            1) overlap the whole demand
            2) overlap each route demand
        
        """
        
        #if set(self.RoutesDemands.keys()) != set(PartialDemand.RoutesDemands.keys()):
        #   raise ValueError('routes label should be the same')
        
        #align the demand, after the operation, the time between (s,e)
        #   of the self and PartialDemand are the same
        #   even if (s,e) cover the self time domain fully. 
        #   (s,e) is the time domain of PartialDemand. 
        self.DemandAlignTimeIntervals_partial(PartialDemand)
        
        #find the idx of the s and e in self.Demand
        #   s = PartialDemand.MinimalMoment
        #   e = PartialDemand.MaximalMoment
        idx_s   = self.idx_Left(PartialDemand.MinimalMoment)
        idx_e   = self.idx_Right(PartialDemand.MaximalMoment)
        
        #Step 3: overlap demand---> change the self.Demand and self.RoutesDemands[label].Demand 
        #Since some routes of PartialDemand may not be in self, then just add the route demand
        #   to the overall demand self.Demand
        for idx,idy in zip(range(idx_s,idx_e+1),range(PartialDemand.Demand.size)):
            #Add each route demand to self.RoutesDemand and self.Demand. 
            for label in PartialDemand.RoutesDemands.keys():
                if label not in self.RoutesDemands.keys():continue
                self.RoutesDemands[label].Demand.nodeat(idx).value += PartialDemand.RoutesDemands[label].Demand.nodeat(idy).value
                self.Demand.nodeat(idx).value += PartialDemand.RoutesDemands[label].Demand.nodeat(idy).value
        
        #NewDemandAdded = True, then the self.plm will be re-calculated. 
        self.refreshAfterTimeIntervalsChanged(NewDemandAdded=NewDemandAdded)
        
    def DemandPickUp(self,s,e,ShiftOfTime = 0.0):
        """
        TESTED_OK
        Pick up some demand between time moments s and e.
        
        overload from RouteDemand.DemandPickUp(self,s,e)
        difference is that the returning is LaneDemand instance
        It is assumed that the self.Demand and each self.RoutesDemands are @@@@@@@time-aligned@@@@@.
        ShiftOfTime is the time needed to be adjusted to self.TimeIntervals. 
        ---------------------------------------------------------------------
        Steps:
            1)Split the moments of self and PartialDemand(the returning value) due to s and e;
            2)find the start_idx and end_idx of s and e
            3)Output the results
        ---------------------------------------------------------------------
        Input:
            s: starting moment,scalar
            e: ending moment, scalar
                
                note: the following statements enssure that the returning time domain is within self time domain:
                    s=max([s,self.MinimalMoment])
                    e=min([e,self.MaximalMoment])
        
        Output:
            PartialDemand, LaneDemand instance. 
            if (s,e) cover the whole time domain, it does not matter.
        
        """
        if s >= e:
            raise NameError('s should be smaller than e')
        
        #if s and e are both outside the time domain, then return 0 demand. 
        if s >=self.MaximalMoment or e<=self.MinimalMoment:
            return LaneDemand(RouteLabels=self.RoutesDemands.keys(),fun=lambda t:.0*t, startmoment = s, T=e-s,deltat=e-s)
        
        s=max([s,self.MinimalMoment])
        e=min([e,self.MaximalMoment])
        PartialDemand   =   LaneDemand(RouteLabels=self.RoutesDemands.keys(),fun=lambda t:.0*t, startmoment = s, T=e-s,deltat=e-s)
        #########split the route demand, based on the moments of self.TimeIntervals within [s,e]
        #Split the self
        self.DemandSplitTime(set([s,e]))
        for routelabel in self.RoutesDemands.keys():
            self.RoutesDemands[routelabel].DemandSplitTime(set([s,e]))
        #split the PartialDemand
        #   moments include the s and e. 
        moments = self.PickUp_moments(s,e)
        PartialDemand.DemandSplitTime(moments)
        for routelabel in PartialDemand.RoutesDemands.keys():
            PartialDemand.RoutesDemands[routelabel].DemandSplitTime(moments)
        
        #find the idx of s and e in self.TimeIntervals
        idx_s   =   self.idx_Left(s)
        idx_e   =   self.idx_Right(e)
        
        PartialDemand.TimeIntervals = pyinter.IntervalSet(sorted(self.TimeIntervals)[idx_s:idx_e+1])
        PartialDemand.Demand = sllist(list(self.Demand)[idx_s:idx_e+1])
        for routelabel in PartialDemand.RoutesDemands.keys():
            PartialDemand.RoutesDemands[routelabel].TimeIntervals = pyinter.IntervalSet(sorted(self.RoutesDemands[routelabel].TimeIntervals)[idx_s:idx_e+1])
            PartialDemand.RoutesDemands[routelabel].Demand = sllist(list(self.RoutesDemands[routelabel].Demand)[idx_s:idx_e+1])
        
        #shift the time moments
        if ShiftOfTime!=0.0:
            PartialDemand.ShiftTimeIntervals(ShiftOfTime)
        
        #refresh AFFILIATED properties
        PartialDemand.refreshAfterTimeIntervalsChanged()
        
        return PartialDemand

    def PLM_get_routes_N(self,N = 0):
        """
        implementation of Proportional Line model. 
        ---------------------------------------------------------
        Input: cumulative vehicle number, N
            N can be a scalar, list, 1-dimentional array
            if N is a float or int, it will be translated to 1-dimentional np.array. 
        Output:routeN
            dict, routeN[label] is the respective route's N when overal cumulative number 
            is N, routeN[label] is a 1-dimentional np.array, shape is (len(N),)
            
            If some value of N is smaller than minimal cumu vehicle number
            or greater than maximal cumu vehicle number, both raise ValueError. 
        -----------------------------------------------------------
        Methods:
            use the interplation
            if N  is smaller than the minimal value, the minimal moment is returned
            if N is greater than  the maximal value, the maximal moment is returned
        
        """
        
        #initialization of the return value, pandas.DataFrame
        routeN  =   {}
        
        #transform the input N to np.array, which is 1 dimentional array
        #   One shape dimension can be -1. In this case, the value is inferred from the length of the array and remaining dimensions.
        N = np.asarray(N).reshape(1, -1)[0,:]
        
        #due to the numerica error, set a threshold. 
        if N.max()-self.plm['overall'][-1] >= 0.002:
            raise ValueError('N is greater than maximal vehicle numer')
        if N.min()< self.plm['overall'][0]:
            raise ValueError('N is smaller than minimal vehicle numer') 
        
        #self.PLM_get_moment(N) return 1-dimantional 
        t = self.PLM_get_moments(N)
        xp=self.plm['moments']
        
        for label in self.RoutesDemands.keys():
            yp=self.plm[label]
            #routeN[label]=interpolate.interp1d(xp, yp)(t)
            
            routeN[label] =   np.interp(t,xp, yp)
            
        return routeN
    
    def PLM_get_moments(self,N=0):
        
        """
        implementation of Proportional Line model. 
        --------------------------------------------------------------------
        Input: cumulative vehicle number, N
            float or array, it will be transformed to np.array firstly,
            shape is (len(N),)
            
            If N is negative or smaller than minimal value, then ValueError. 
            If N is greater than maximal cumulative vehicle number, then ValueError. 
            
        Output:moment
            1-dimentional np.array, shale is (len(N),)
            moment when the overall demand cumulative vehicle number is N
        --------------------------------------------------------------
        Methods:
            use the interplation
            if N  is smaller than the minimal value, the minimal moment is returned
            if N is greater than  the maximal value, the maximal moment is returned
        
        """
        
        #   -1 means the length of this dimanetional depend on the number of values
        N = np.asarray(N).reshape(1, -1)[0,:]
        if N.max()-self.plm['overall'][-1] >= 0.002:
            raise ValueError('N is greater than maximal vehicle numer')
        if N.min()< self.plm['overall'][0]:
            raise ValueError('N is smaller than minimal vehicle numer') 
        
        #interploate to get the value       
        #f = interpolate.interp1d(self.plm['overall'], self.plm['moments'])(N)
        
        return np.interp(N,self.plm['overall'], self.plm['moments'])
        
        #return f(N)
    
    
    
    def RouteDemandAccept(self,UpstreamDemand):
        
        
        pass
    
    
    
    
    def Plot_plm(self):
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        
        T=self.plm['moments']
        Y=self.plm['overall']
        ax.plot(T,Y);
        ax.set_xlabel("Time (in sec)")
        ax.set_ylabel("Flow rate (in veh/h)")
        for label in self.RoutesDemands.keys():
            ax.plot(T,self.plm[label]);
        plt.show()
    
    

    
    def PlotDemand(self):
        """
        plot the demand. Override the RouteDemand.PlotDemand1 method. 
        This method draw demand for each route. 
        
        """
        
        self.Check_Consistency()
        
        #construct the t axis, which is the middle moment of each interval
        #T is the moments, and Y is the overall demand, in veh/h
        T=[self.MinimalMoment]
        Y=[0]
        for interval,d in zip(sorted(self.TimeIntervals),self.Demand) :
            T.extend([interval.lower_value, interval.upper_value])
            Y.extend([d , d])
        
        T.append(self.MaximalMoment)
        Y.append(0)
        
        yy = np.empty((2*(self.Demand.size+1) ,len(self.RoutesDemands.keys())))
        for label,idx in zip(self.RoutesDemands.keys(),range(len(self.RoutesDemands.keys()))):
            tmp=[0]
            for d in self.RoutesDemands[label].Demand:
                tmp.extend([d , d])
            tmp.append(0)
            yy[:,idx]= np.array(tmp)
        
        #plot(t,y)
        fig, ax = plt.subplots()
        ax.plot(T,Y);
        ax.plot(T,yy)
        ax.set_xlabel("Time (in sec)")
        ax.set_ylabel("Flow rate (in veh/h)")
        
        plt.show()
        
        return T,Y
        
    def RoutesDemandsAggregation(self,saturationflowrate=1800):
        """
        Overlap all routes demand to lane demand
        The time intervals should be aligned 
        using RouteDemand.DemandSplitTime method
        self.Demand is changed afterwards.
        Four steps:
            - get common moments of overalldemand and each route demand
            - Align the TimeIntervals for all routes
            - Aggregation(just sum up each route demand)
            - Checking the flow exceed the maximum
        
        Input: no input
        Output: no output
            but the self.Demand and self.TimeIntervals are changed
        """
        
        ###############Step1: Get the common time moments in RouteDemand.TimeIntervals
        CommonMoments   =   set()   
        #because RoutesDemands is a dict
        for routedemand in self.RoutesDemands.values():
            CommonMoments.update(routedemand.OverallMoments)
        
        ############Step2: Align the TimeIntervals of all routes
        #after the alignment, the TimeIntervals for all routes are the same
        #thus can be operated by sum
        self.DemandSplitTime(CommonMoments)
        for routedemand in self.RoutesDemands.values():
            routedemand.DemandSplitTime(CommonMoments)
        
        ##############Step3: Aggregation
        #this method is in RouteDemand.DemandOverlap()
        for routedemand in self.RoutesDemands.values():
            self.DemandOverlap(routedemand)
        
        ################refresh after demand and TimeIntervals are changed. 
        #self.refreshAfterTimeIntervalsChanged()
        
        #Step4: Checking whether the lane demand exceed the capacity
        #   note that the function is subclassed from RouteDemand
        self.DemandRescale2(saturationflowrate=saturationflowrate)
        
    #def PLM(self, OverallCumulativeNumber = 0):
        #"""
        #Reliazation of Proportional line method
        
        #input:  OverallCumulativeNumber
            #cumulative vehicle number of the whole lane
        #output: RespectiveRouteCumulativeNumber
            #dict, keys are the routes label, values are scalars. 
            #denote when the overall cumulative number is given, 
            #the respective cumulative vehicle number for each route
        
        #"""
        
        #RespectiveRouteVehicleNumber   =   {}
        ##
        
        
        
        
        
        #pass
        
    #def VehiclesNumberFrom_t(self,a):
        #"""
        #Overload the method of GeneralContinuousIntervals in that 
        #The degradation is considered. 
        
        #The GeneralContinuousIntervals.VehiclesNumberFrom_t() just compute the 
        #cumulativie vehicle number. Here the method consider the degradation by 
        #considering the 'self.IncomingV' property. 
        
        #Thus the cumulative vehicle number is computered for two types: without degradation and with degradation. Note that under the degradation sypt there are many possible InComingV values. Any InComingV which is smaller than FD.vf is considered degradation. 
        
        #Input:a
            #moment, is a Scalar. 
        
        #Output:Cumu_TuplesList
            #list of tuples. Each is (incomingv, CumulativeNumber). 
            #len(CumulativeNumber) is the number of different IncomingV
            #each incomingv is a scalr means speed,
            #each CumulativeNumber is a (2,N) np,array. 
                #1st row is moments, 2nd row is cumulative vehicle number. 
                #1st element at 1st row should be the starting moment of Differenct IncomingV
                #1st element at 2nd row should be zero.
            
        
        #"""
        
        
        
        
        #return Cumu_TuplesList



class MixDegradedLaneDemand(LaneDemand):
    """
    May includ Degraded demand. 
    
    """
    #the incoming velocity of the demand. This property is set to 
    #   reflect the spillover of the channelized section spillover.
    #   After the spillover, the inflow of channelized section is degraded
    #   and the respondent velocity is degraded to vd
    #   the length of InComingV is the same as Demand
    InComingV       =   sllist()#normally unit in veh/h
    
    def __init__(self, RouteLabels=[1] ,fun=lambda t:-100.0/(3600**2)*t*(t-7200), T=7200):
        """
        RouteLabels:
            is the labels for all routes. It shoud be iterable. 
        fun:
            the funnction on which the route demand is generated
        T:
            the time length for the lane demand. 
        """
        self.RoutesDemands  = {}
        for label in RouteLabels:
            self.RoutesDemands[label]=RouteDemand(fun,T)
        
        #change the self.Demand and self.TimeIntervals
        self.RoutesDemandsAggregation()
        
        #assignment of InComingV
        self.InComingV  =   sllist([FD.vf] * self.Demand.size)
    
    def VehiclesNumberFrom_t(self,a):
        """
        Overloading the GeneralContinuousIntervals.VVehiclesNumberFrom_t(a)
        COnsidering the self.InComingV. 
        
        """
        
        
        pass
    
class CumulativeLine():
    """
    The line that keeps increasing.
    """
    def __init__(self,ts,xs):
        
        #delete the repeat ts, keep the first repeated value.
        idx_es = np.array(ts)>-np.inf
        tmp = np.where(np.diff(ts)==0)[0]
        if len(tmp)>0:
            for i in tmp:idx_es[i]= False
            ts0 = list(np.array(ts)[idx_es])
            xs0 = list(np.array(xs)[idx_es])
        else:
            ts0 = ts
            xs0 = xs
        
        if not np.all(np.diff(ts0) > 0):
            builtins.ts = ts0
            builtins.xs = xs0
            raise ValueError('ts is not strict increasing')
            
        if not np.all(np.diff(xs0) >= 0):
            builtins.ts = ts0
            builtins.xs = xs0
            raise ValueError('xs is not increasing')
        
        self.ts = ts0
        self.xs = xs0
    
    def InsertN(self, l):
        """
        
        """
        if self.xs[-1]<l or self.xs[0]>l:
            raise ValueError('input N is greater than maximal. self.xs[-1] = ', self.xs[-1],', self.xs[0]=', self.xs[0], ', l = ',l)
        
        if l in self.xs:
            return
        
        #insert the t
        #   find the idx of t in self.ts such that
        #   self.ts[idx] < t < self.ts[idx+1]
        idx = np.searchsorted(self.xs,l)-1
        #   compute the N and the CCC
        delta_t = 1.0*self.ts[idx+1] - self.ts[idx]
        delta_x = 1.0*self.xs[idx+1] - self.xs[idx]
        new_t = self.ts[idx]+delta_t*(l-self.xs[idx])/delta_x
        #   insert the t in self.ts self.N and self.CCC
        self.ts.insert(idx+1, new_t)
        self.xs.insert(idx+1, l)
        
    
    def InsertMoment(self,t):
        """
        If t is outside the time domain
            at the left, insert the self.xs[0]
            at the right, insert the self.xs[-1].
        
        """
        if t<self.ts[0]:
            self.ts.insert(0,t)
            self.xs.insert(0,self.xs[0])
            return
        
        if t>self.ts[-1]:
            self.ts.append(t)
            self.xs.append(self.xs[-1])
            return
        
        #more than 1 (include 1) values exist that self.N == n
        if t in self.ts:
            #sum(np.array(self.ts) == t)>=1:
            return
            #if more than one value that N(t) =n
            idx = np.where(np.array(self.N) == n)[0]
        
        #insert the t
        #   find the idx of t in self.ts such that
        #   self.ts[idx] < t < self.ts[idx+1]
        idx = np.searchsorted(self.ts,t)-1
        #   compute the N and the CCC
        delta_t = 1.0*self.ts[idx+1] - self.ts[idx]
        delta_x = 1.0*self.xs[idx+1] - self.xs[idx]
        new_x = self.xs[idx]+delta_x/delta_t*(t-self.ts[idx])
        #   insert the t in self.ts self.N and self.CCC
        self.ts.insert(idx+1, t)
        self.xs.insert(idx+1, new_x)
    
    def Interpolate(self,t):
        """
        If t is outside the time domain
            at the left, return the self.xs[0]
            at the right, return the self.xs[-1].
        other interpolate the value and return it. 
        """
        if t<self.ts[0]:
            return self.xs[0]
        
        if t>self.ts[-1]:
            return self.xs[-1]
        
        #more than 1 (include 1) values exist that self.N == n
        if t in self.ts:
            idx = np.where(np.array(self.ts) == t)[0]
            return self.xs[idx]
        
        #insert the t
        #   find the idx of t in self.ts such that
        #   self.ts[idx] < t < self.ts[idx+1]
        idx = np.searchsorted(self.ts,t)-1
        #   compute the N and the CCC
        delta_t = 1.0*self.ts[idx+1] - self.ts[idx]
        delta_x = 1.0*self.xs[idx+1] - self.xs[idx]
        new_x = self.xs[idx]+delta_x/delta_t*(t-self.ts[idx])
        return new_x
    










def TwoLinesIntersection_ReconstructQueueProfileFromN(ts1,xs1,ts2,xs2, l):
    """
    Given two curves, get the intersection point. 
    The link length is l.
    It is designed for stoppingwave and starting wave. 
    (ts1,xs1) is the stopping wave. ts1[0]=yku
    (ts2,xs2) is the starting wave. ts2[0]=ykv
    ts2[0]>ts1[0]. 
    And that, xs1[0]=0, xs2[0]=0.
    
    There are several cases:
        - intersect within l, then return the waves.
        - intersect outside l, then it is a spillover case.
        - not intersect, but both within l
    
    
    --------------------------------------------
    Input: ts1, xs1
        moments and locations of line 1. len(ts1)==len(xs1)
    Input: ts2, xs2
        moments and locations of line 2. len(ts2)==len(xs2)
    Input: l
        length of the link.
        If the 
    ------------------------------------
    Output:
        dict, keys are 'xy' and 'startingwave'
    """
    #initialize the two lines. 
    #   line 1 is the stopping wave
    #   line 2 is the starting wave
    line1 = CumulativeLine(ts1,xs1)
    line2 = CumulativeLine(ts2,xs2)
    
    #make the moment common between line2.ts[0] and max([line1.ts[-1],line2.ts[-1]])
    for t in line2.ts:
        line1.InsertMoment(t)
    for t in line1.ts:
        if t>line2.ts[0]:
            line2.InsertMoment(t)
    
    #determine whether they intersect
    #   line2.xs[-1]>=line1.xs[-1], then they intersect
    #   else, not intersect. 
    tolerance = 1e-07
    if line1.xs[-1]-line2.xs[-1] <= tolerance:
        #intersect, get the intersection point (t,x)
        #   and check whether x>l
        #idx is the index in line1.ts that equal line2.ts[0]
        #   len(xs1 == xs2)
        idx = np.where(np.array(line1.ts)==line2.ts[0])[0][0]
        tmp_xs1 = np.array(line1.xs[idx:])
        tmp_xs2 = np.array(line2.xs)
        #delta_x[0] must be positive, len(delta_x)==len(line2.ts)
        #   tolerance is used to deal with numerical error.
        delta_x = tmp_xs1 - tmp_xs2
        delta_x[np.where(delta_x<=tolerance)] = 0
        delta_x = list(delta_x)
        
        if delta_x[0]<0:
            raise ValueError('The first value must be positive, but we have delta_x[0] = ',delta_x[0])
        elif delta_x[0]==0:
            #means that the spillover duration, flowrate ==0
            # hence there virtual signal is red too.
            xy = np.array([[ts1[0], ts2[0]],[.0,.0]])
            startingwave = np.array([[ts2[0],  ts2[0]],[.0,.0]])
            
        else:
            #no equal moment, then get the equal moment
            if len(np.where(np.array(delta_x)==0)[0])==0:
                #find the idx that delta_x[idx-1]>0 and delta_x[idx]<0
                
                #debug
                print(min(delta_x))
                
                idx = np.where(np.array(delta_x)<0)[0][0]
                #get the moment when the lines intersect.
                zeromoment = line2.ts[idx-1]+ (line2.ts[idx]-line2.ts[idx-1])*(delta_x[idx-1])/(delta_x[idx-1]-delta_x[idx])
                line1.InsertMoment(zeromoment)
                line2.InsertMoment(zeromoment)
            else:
                #have equal moment, get the moment.
                idx = np.where(np.array(delta_x)==0)[0][0]
                zeromoment = line2.ts[idx]
            
            #idx_in_line2 is the index in line2.ts that two line intersect.
            #idx_in_line1 is in the line1.ts
            idx_in_line2 = np.where(np.array(line2.ts)==zeromoment)[0][0]
            idx_in_line1 = np.where(np.array(line1.ts)==zeromoment)[0][0]
            
            #spillover or not?
            if tmp_xs2[idx_in_line2]<=l:
                #no spillover, get xy and starting wave
                startingwave = np.array([line2.ts[:idx_in_line2+1], line2.xs[:idx_in_line2+1]])
                xy = np.array([line1.ts[:idx_in_line1+1], line1.xs[:idx_in_line1+1]])
                pass
            else:
                #spillover, then cut off the queue tail, return the value
                line1.InsertN(l)
                line2.InsertN(l)
                #get the yku and ykv.
                idx_l_in_line1 = np.where(np.array(line1.xs)==l)[0][0]
                idx_l_in_line2 = np.where(np.array(line2.xs)==l)[0][0]
                #get the xy
                #   note that add the last point.
                xy = np.array([line1.ts[:idx_l_in_line1+1]+[line2.ts[idx_l_in_line2]], line1.xs[:idx_l_in_line1+1]+[l]])
                #get the startingwave
                startingwave = np.array([line2.ts[:idx_l_in_line2+1], line2.xs[:idx_l_in_line2+1]])
        
    else:
        #the two lines do not intersect, then considering the following
        #   - line1.xs[-1]>l or not
        #   - line2.xs[-1]>l or not
        #if both greater than l, then get the spillovered xy and starting eave
        #otherwise return None
        #
        if line1.xs[-1]<l or line2.xs[-1]<l:
            return None
        
        #cut off the line
        line1.InsertN(l)
        line2.InsertN(l)
        #get the yku and ykv.
        idx_l_in_line1 = np.where(np.array(line1.xs)==l)[0][0]
        idx_l_in_line2 = np.where(np.array(line2.xs)==l)[0][0]
        #get the xy
        #   note that add the last point.
        xy = np.array([line1.ts[:idx_l_in_line1+1]+[line2.ts[idx_l_in_line2]], line1.xs[:idx_l_in_line1+1]+[l]])
        #get the startingwave
        startingwave = np.array([line2.ts[:idx_l_in_line2+1], line2.xs[:idx_l_in_line2+1]])
        
    return {'xy':xy,'startingwave':startingwave}



def gmmfun(t):
    return Demandfun(t,name='GMM')

def zerofun(t):
    return D.Demandfun(t)

def Demandfun(t,saturationflowrate=FD.qm,**kwargs):
    """
    Example:   Demandfun(t,name="GMM"):
    
    Curve fnction for the demand. 
    **kwargs={'func':-----,'para':[,,,]}
    two key-value pairs, one is the type of the function, maybe GMM etc. 
    Another is the parameters of the function. 
    
    gmm: parameters is ([],[])
        example:
            kwargs['para']={}
            kwargs['para']['means']   = np.array([[-1], [0], [3]])
            kwargs['para']['covars']  = np.array([[1.5], [1], [0.5]]) ** 2
            kwargs['para']['weights'] = np.array([0.3, 0.5, 0.2])
    
    
    """
    funtype =   kwargs.pop('name', None)
    
    if funtype=="GMM":
        #guassiam
        para    =   kwargs.pop('para', {'means':np.array([[100], [800], [1500]]),\
                        'covars':np.array([[200], [20], [200]]) ** 2,\
                        'weights':np.array([0.3, 0.4, 0.3])})
        g = mixture.GMM(n_components=3)
        g.means_    = para['means']#np.array([[-1], [0], [3]]), shape is (3,1)
        g.covars_   = para['covars']#np.array([[1.5], [1], [0.5]]) ** 2, shape is (3,1)
        g.weights_  = para['weights']#np.array([0.3, 0.5, 0.2]),shape is (3,)
        
        #g.score_samples(t) is a tuple. length is 2.
        #g.score_samples(t)[0].shape --------> (1,)
        #g.score_samples(t)[1].shape --------> (1,3)
        return min([10000*np.exp(g.score_samples(t)[0][0]),saturationflowrate])

    elif funtype=='quadraic':
        para    =   kwargs.pop('para', [2000,100])
        #kwargs['para']=[m,n]
        #the function go across three points: (0,0), (m/2,n), (m,0)
        m,n = para
        return min([-m**2.0/(4*n)*t*(t-m),saturationflowrate])#-100.0/(3600**2)*t*(t-7200)
    else:
        return 0

    
    
    
    
