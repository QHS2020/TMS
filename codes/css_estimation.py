# -*- coding: utf-8 -*-

"""
This module contains the implementation of TMS reatime estimation, including the occurrence and duration estimation.

"""
#############################################################
from .RequiredModules import *
from .LaneFD import FD
from .Demand import RouteDemand,CumulativeLine
from .RGP import RGP#red-green class
from .LaneCapacity import LaneCapacity

import scipy

#meta variables
ApproachLabels = ['east', 'west', 'north', 'south']
TurningDIrectionLabels = ['left', 'through', 'right']
T_horizon = 7200#in seconds, used in the RGP settings


#config
def output_config(T_demand = 3600, T_signal = 3600):
        config = {}
        config['east'] = {'lu':300, 'ld':100,\
                        #signal settings
                       'rl':60.0, 'gl':40.0,  'rth':60.0, 'gth':40.0,  'oth':0,\
                        #flow rate
                       'ql':400.0,  'qth':800.0,  'qr':400.0, \
                       #lanes number
                        'n_lanes_u_section':2, 'n_lanes_through':1,\
                       'n_lanes_right':1, 'n_lanes_left':1,\
                       'n_lanes_out':2,\
                       #time horizon
                        'T_demand':T_demand, 'T_signal':T_signal, 'deltat':4.0, \
                       'qm':1.0*FD().qm}
        config ['north'] = {'lu':300, 'ld':100,\
                        #signal settings
                       'rl':60.0, 'gl':40.0,  'rth':60.0, 'gth':40.0,  'oth':0,\
                        #flow rate
                       'ql':300.0,  'qth':700.0,  'qr':400.0, \
                       #lanes number
                        'n_lanes_u_section':2, 'n_lanes_through':1,\
                       'n_lanes_right':1, 'n_lanes_left':1,'n_lanes_out':2,\
                       #time horizon
                        'T_demand':T_demand, 'T_signal':T_signal, 'deltat':4.0, \
                       'qm':1.0*FD().qm}
        config['west'] = {'lu':300, 'ld':100,\
                        #signal settings
                        'rl':60.0, 'gl':40.0,  'rth':60.0, 'gth':40.0,  'oth':0,\
                        #flow rate
                        'ql':300.0,  'qth':700.0,  'qr':400.0, \
                        #lanes number
                        'n_lanes_u_section':2, 'n_lanes_through':1,\
                        'n_lanes_right':1, 'n_lanes_left':1,'n_lanes_out':2,\
                        #time horizon
                        'T_demand':T_demand, 'T_signal':T_signal, 'deltat':4.0, \
                        'qm':1.0*FD().qm}
        config['south'] = {'lu':300, 'ld':100,\
                        #signal settings
                        'rl':60.0, 'gl':40.0,  'rth':60.0, 'gth':40.0,  'oth':0,\
                        #flow rate
                        'ql':300.0,  'qth':700.0,  'qr':400.0, \
                        #lanes number
                        'n_lanes_u_section':2, 'n_lanes_through':1,\
                        'n_lanes_right':1, 'n_lanes_left':1,'n_lanes_out':2,\
                        #time horizon
                        'T_demand':T_demand, 'T_signal':T_signal, 'deltat':4.0, \
                        'qm':1.0*FD().qm}
        return config
configs = output_config()


class CSS_withsignal():
    """
    TMS estimation using trajectory when signals info are given. 
    

    """
    #axis rotation. 
    sin_theta = (FD.w/3.6)/np.sqrt((FD.w/3.6)**2+1)
    cos_theta = 1/np.sqrt((FD.w/3.6)**2+1)

    @classmethod
    def VehicleTrajectory(self, Approach_TRs, Approach_signals, loc_stopline, l_tb,l_tb_direction =1, w = FD.w, vf =FD.vf):
        """
        Abstract the trajectories. 
        ----------------------------------------------------------------

        """


        pass

    @classmethod
    def slice_trajectory_full_cycle(self, approach_TRs, paired_reds, kwargs, completeORnot = True):
        """
        different from self.slice_trajectory1(). and self.slice_trajectory_with_corss_direction() in that, 
        slice_trajectory_full_cycle() will aggregate the turning directions vehicles for the full cycle. 
        ---------------------------------------------------
        input: approach_TRs
            dict. 
            approach_TRs['left'][vehicle_id] = (ts, xs).
            ts and xs are both list. 
            plt.plot(ts, xs)
        """
        loc_stopline = kwargs['loc_stopline']
        l_tb = kwargs['l_tb']
        w = kwargs['w']
        vf = kwargs['vf']
        direction_flag = kwargs['direction_flag']

        SlicedTRs_full = [{} for i in paired_reds]
        #turning_d may be 'left','through','right'
        for turning_d in approach_TRs.keys():
            for vid in approach_TRs[turning_d].keys():
                ts,xs = approach_TRs[turning_d][vid]

                #slice the data
                for idx,paired_red in enumerate(paired_reds):
                    directions = paired_red.keys()

                    #get the t0 and t2 of the whole cycle. 
                    t0 = np.inf
                    t2 = -np.inf
                    for d in paired_red.keys():
                        #t0,t1,t2 = paired_red[d]
                        t0 = min([paired_red[d][0], t0])
                        t2 = max([paired_red[d][2], t2])

                    #d may be either 'left' or right
                    for d in directions:
                        if not d==turning_d:continue
                        local_t0,local_t1,local_t2 = paired_red[d]
                        #if not intersect, t_w would be False
                        t_w,x_w = self.tsxsIntersectionPoint(ts=ts, xs=xs, t0 = local_t1, loc_stopline=loc_stopline, w=w)
                        if t_w==False:
                            #print(ts[0]-abs(xs[0]-loc_stopline)/(w/3.6), t1, ts[-1]-abs(xs[-1]-loc_stopline)/(w/3.6),'dddd');
                            continue
                        if abs(x_w-loc_stopline)<l_tb:continue

                        if not SlicedTRs_full[idx].has_key(d):
                            SlicedTRs_full[idx][d] = {}
                        #if not SlicedTRs_full[idx][d].has_key(turning_d):
                        #    SlicedTRs_full[idx][d][turning_d] = {}
                        #print(ts[0],ts[-1], t0,t1,t2)
                        
                        #slice. 
                        ts0,xs0,complete = self.tsxs_cycleslice(ts=copy.deepcopy(ts), xs=copy.deepcopy(xs), t0= t0, t1 = t2, loc_stopline=loc_stopline,w = w)
                        SlicedTRs_full[idx][d][vid] = {'tsxs':(ts0,xs0), 'intersectionpoint':(t_w, x_w), 'redduration':local_t1-local_t0, 'cyclestartmoment':local_t0, 't0t1t2':(local_t0,local_t1,local_t2)}
        return SlicedTRs_full


    @classmethod
    def slice_trajectory_with_corss_direction(self, approach_TRs, paired_reds, kwargs,completeORnot = True):
        """
        Slice the trajectories. Different from self.slice_trajectory1() in that:
            - slice_trajectory1() seperate the trajectories of each turning directions when sliceing
            - slice_trajectory_with_corss_direction() will slice the TRs corss different turning directions. 

        Note that, paired_reds[some_idx].keys() are part of approach_TRs keys. 

        ---------------------------------------------------------
        input: paired_reds, a list
            the paired reds. paired_reds[idx] = {'left':(t0,t1,t2), 'through':(t0,t1,t2))}
        input: approach_TRs
            dict. 
            approach_TRs['left'][vehicle_id] = (ts, xs).
            ts and xs are both list. 
            plt.plot(ts, xs)
        input: l_tb, unit is ,
            length of turning bay aea, or length of C-section. 
        input: w vf. units are km/h
            the parameters in the FD settings. 
        input: loc_stopline
            location of the stopline. 
        input: direction_flag
            either 1 or -1.
            If it is 1, then the distance of the road increase upstream. 
        OUTPUT: 
            SlicedTRs, a list. Each element corresponds to the element in paired_reds
            SlicedTRs[idx] is a dict. 
            SlicedTRs[idx].keys are the same as paired_reds[idx].keys.
            SlicedTRs[idx]['left']are dict. 
            SlicedTRs[idx]['left'].keys() = approach_TRs.keys()
            It means that using 'left' signal to slice the 'left' 'through' and 'right' TRs. 
            
            SlicedTRs[idx]['left']['through'][vehicle_id].keys() include:
                - 'tsxs':(ts,xs)
                - 't0t1t2':(t0,t1,t2)
                - 'intersectionpoint':(t_w, x_w)
                - 'redduration': redduration
                - 'twoboudnarys':((ta,xa),(tb,xb))
        -------------------------------------------------------
        Step:
            - for each turning_direction
                - for each vehicle, with id as vid
                    - 

        """
        loc_stopline = kwargs['loc_stopline']
        l_tb = kwargs['l_tb']
        w = kwargs['w']
        vf = kwargs['vf']
        direction_flag = kwargs['direction_flag']

        SlicedTRs = [{} for i in paired_reds]
        #turning_d may be 'left','through','right'
        for turning_d in approach_TRs.keys():
            for vid in approach_TRs[turning_d].keys():
                ts,xs = approach_TRs[turning_d][vid]
                for idx,paired_red in enumerate(paired_reds):
                    directions = paired_red.keys()
                    #d may be either 'left' or right
                    #   d is the signal direction
                    for d in directions:

                        if not SlicedTRs[idx].has_key(d):
                            SlicedTRs[idx][d] = {}
                        if not SlicedTRs[idx][d].has_key(turning_d):
                            SlicedTRs[idx][d][turning_d] = {}

                        t0,t1,t2 = paired_red[d]
                        #if not intersect, t_w would be False
                        t_w,x_w = self.tsxsIntersectionPoint(ts=ts, xs=xs, t0 = t1, loc_stopline=loc_stopline, w=w)
                        if t_w==False:
                            #print(ts[0]-abs(xs[0]-loc_stopline)/(w/3.6), t1, ts[-1]-abs(xs[-1]-loc_stopline)/(w/3.6),'dddd');
                            continue
                        if abs(x_w-loc_stopline)<l_tb:continue

                        #print(ts[0],ts[-1], t0,t1,t2)
                        #slice. 
                        ts0,xs0,complete = self.tsxs_cycleslice(ts=copy.deepcopy(ts), xs=copy.deepcopy(xs), t0=t0, t1=t2, loc_stopline=loc_stopline,w = w)
                        SlicedTRs[idx][d][turning_d][vid] = {'tsxs':(ts0,xs0), 'intersectionpoint':(t_w, x_w), 'redduration':t1-t0, 'cyclestartmoment':t0, 't0t1t2':(t0,t1,t2)}
        return SlicedTRs


    @classmethod
    def SpilloversIntervalsetSplit2GreenSplits(self, spillovers_at_interface, effective_cycles_idx, cycles_intervals_list):
        """
        Split the obtained spillover intervalset to each cycle. 
        --------------------------------------------------------------
        input: spillovers_at_interface
            a pyinter.InvervalSet
        input: effective_cycles_idx
            a list of int
        input: cycles_intervals_list
            a list of interval. 
        """
        splits_in_each_cycle = []
        for i,cycle_interval in enumerate(cycles_intervals_list):
            if not i in effective_cycles_idx:continue
            cycle_length = 1.0*(cycle_interval.upper_value-cycle_interval.lower_value)
            intersection_intervals = spillovers_at_interface.intersection([cycle_interval])
            intersection_durations = 0
            for i in sorted(intersection_intervals):
                intersection_durations=intersection_durations+i.upper_value-i.lower_value

            splits_in_each_cycle.append(1.0-intersection_durations/cycle_length)

        return splits_in_each_cycle

        pass

    @classmethod
    def SplitGreen_turning(self, paired_reds, tms_estimation_res_pd, direction = 'left'):
        """
        Get the green splits. 
        -----------------------------------------------------------
        input: tms_estimation_res_pd
            a pd.DataFrame. 
            The index are like 'cycle_0', 'cycle_1'.
            Some 'cycle_i' may missing due to the lacking of trajectories. 
            a pandas.DataFrame. 
            Used columns include:
                ['left TMS duration', 'through TMS duration', 'TMS_ocurence_left', TMS_ocurence_through']
            They are the estimated left TMS, through TMS, the estimated left TMS occurrence and estimated through TMS occurrence. 
        input: paired_reds
            a list. 
            paired_reds[cycle_index] = {'left': (24.0, 137.0, 161.0), 'through': (0.0, 24.0, 59.0)}.
            paired_reds[cycle_index]['left'] = t0,t1,t2. 
            They are red start, red end and green end.

        Output: 
            splits
            a list of float. len(splits)= len(tms_estimation_res_pd.index)
        -----------------------------------------------------------

        """
        def another_direction(direction):
            if direction=='left':
                return 'through'
            elif direction=='through':
                return 'left'
        splits = []
        effective_cycles_idx = []
        for row_idx in tms_estimation_res_pd.index:
            #the cycle index.
            cycle_idx = int(row_idx.split('_')[1])
            if not direction in paired_reds[cycle_idx].keys():splits.append(0)
            if not another_direction(direction) in paired_reds[cycle_idx].keys():splits.append(0)

            effective_cycles_idx.append(cycle_idx)
            if direction=='left':
                #signal of direction
                t0,t1,t2 = paired_reds[cycle_idx][direction]
                cyclelength = t2-t0
                red  = pyinter.interval.closed(t0,t1)

                #signal of another direction
                t0_another,t1_another,t2_another = paired_reds[cycle_idx][another_direction(direction)]

                if float(tms_estimation_res_pd.loc[row_idx, 'through TMS duration'])>0:
                    duration = float(tms_estimation_res_pd.loc[row_idx, 'through TMS duration'])
                    #print(t1_another,duration)
                    another_tms = pyinter.interval.closed(t1_another-duration,t1_another).intersect(pyinter.interval.closed(t0,t2))
                    if t1_another-duration>t2 or t1_another<t0:
                        another_tms = pyinter.interval.open(red.lower_value,red.lower_value)
                    red_overlapped = red.union(another_tms)
                    if isinstance(red_overlapped,pyinter.Interval):
                        splits.append(1.0-(red_overlapped.upper_value-red_overlapped.lower_value)/cyclelength)
                    elif isinstance(red_overlapped,pyinter.IntervalSet):
                        splits.append(1.0-sum([i.upper_value-i.lower_value for i in red_overlapped])/cyclelength)
                else:
                    splits.append(1.0-(t1-t0)/cyclelength)

            elif direction=='through':
                #signal of direction
                t0,t1,t2 = paired_reds[cycle_idx][direction]
                cyclelength = t2-t0
                red  = pyinter.interval.closed(t0,t1)

                #signal of another direction
                t0_another,t1_another,t2_another = paired_reds[cycle_idx][another_direction(direction)]

                if float(tms_estimation_res_pd.loc[row_idx, 'left TMS duration'])>0:
                    duration = float(tms_estimation_res_pd.loc[row_idx, 'left TMS duration'])
                    another_tms = pyinter.interval.closed(t1_another-duration,t1_another).intersect(pyinter.interval.closed(t0,t2))
                    if t1_another-duration>t2 or t1_another<t0:
                        another_tms = pyinter.interval.open(red.lower_value,red.lower_value)
                    red_overlapped = red.union(another_tms)
                    if isinstance(red_overlapped,pyinter.Interval):
                        splits.append(1.0-(red_overlapped.upper_value-red_overlapped.lower_value)/cyclelength)
                    elif isinstance(red_overlapped,pyinter.IntervalSet):
                        splits.append(1.0-sum([i.upper_value-i.lower_value for i in red_overlapped])/cyclelength)
                else:
                    splits.append(1.0-(t1-t0)/cyclelength)

        return splits,effective_cycles_idx


    @classmethod
    def Spillovers_Interface_Intervals_1(self, tms_estimation_res_pd, paired_reds, shift_time = 100.0/(40/3.6), directions_considered = ['left', 'through'], debug_cycle_idx = 36):
        """
        Different from self.Spillovers_Interface_Intervals():
            - self.Spillovers_Interface_Intervals_1() take into each duration into account. 
        Get the spillvoer invervals at the interface. 
        ----------------------------------------------
        input: shift_time
            a float. Used to denote the temporal shift of the spillover duration. 
        input: tms_estimation_res_pd
            a pd.DataFrame. 
            The index are like 'cycle_0', 'cycle_1'.
            Some 'cycle_i' may missing due to the lacking of trajectories. 
            a pandas.DataFrame. 
            Used columns include:
                ['left TMS duration', 'through TMS duration', 'TMS_ocurence_left', TMS_ocurence_through']
            They are the estimated left TMS, through TMS, the estimated left TMS occurrence and estimated through TMS occurrence. 
        input: paired_reds
            a list. 
            paired_reds[idx]['through'] = (start_of_red, end_of_red, end_of_green)
            It is used to determinethe t0,t1,t2
                t0 is the red onset moment.
                t1 is the red end moment.
                t2 is the green end moment.
        ------------------------------------------------------------
        Output: spilloverIntervalsSet


        """
        #effective_cycles_idx are those have spillovers. 
        effective_cycles_idx = []
        spillover_intervals = pyinter.IntervalSet()
        for row_idx in tms_estimation_res_pd.index:
            ############left
            if 'left' in directions_considered:
                if float(tms_estimation_res_pd.loc[row_idx, 'left TMS duration'])>0:
                    #t1-t0 is the red duration, and t2-t1 is the green duration. 
                    duration = float(tms_estimation_res_pd.loc[row_idx, 'left TMS duration'])
                    t0,t1,t2 = paired_reds[int(row_idx.split('_')[1])]['left']
                    spillover_intervals.add(pyinter.interval.open(t1 - shift_time - duration,t1-shift_time))
                    if int(row_idx.split('_')[1])==debug_cycle_idx:
                        pass
                        #print('debug: ', t1 - shift_time - duration,t1-shift_time)
                    effective_cycles_idx.append(int(row_idx.split('_')[1]))

            ############through
            if 'through' in directions_considered:
                if float(tms_estimation_res_pd.loc[row_idx, 'through TMS duration'])>0:
                    #t1-t0 is the red duration, and t2-t1 is the green duration. 
                    duration = float(tms_estimation_res_pd.loc[row_idx, 'through TMS duration'])
                    t0,t1,t2 = paired_reds[int(row_idx.split('_')[1])]['through']
                    spillover_intervals.add(pyinter.interval.open(t1 - shift_time - duration,t1-shift_time))
                    if int(row_idx.split('_')[1])==debug_cycle_idx:
                        print('debug: ', t1 - shift_time - duration,t1-shift_time)

                    #
                    if not int(row_idx.split('_')[1]) in effective_cycles_idx:
                        effective_cycles_idx.append(int(row_idx.split('_')[1]))


        return spillover_intervals,effective_cycles_idx

    @classmethod
    def Paired_reds_mapped_by_spillover(self, paired_reds, tms_estimation_res_pd):
        """
        Mapp the left-turn spillover to the through signal 
        and vice versa. 

        """
        paired_reds_mapped = []
        for paired_red in paired_reds:
            paired_red_mapped = {}
            for d in paired_red.keys():
                paired_red_mapped[d] = pyinter.interval.closed(paired_red[d][0] ,paired_red[d][1])
            paired_reds_mapped.append(paired_red_mapped)

        for row_idx in tms_estimation_res_pd.index:
            cycle_idx = int(row_idx.split('_')[1])

            ############left spillover map to through signal
            if float(tms_estimation_res_pd.loc[row_idx, 'left TMS duration'])>0:
                #t1-t0 is the red duration, and t2-t1 is the green duration. 
                duration = float(tms_estimation_res_pd.loc[row_idx, 'left TMS duration'])
                t0,t1,t2 = paired_reds[cycle_idx]['left']
                spilloverinterval_mapped = pyinter.interval.closed(t1 - duration,t1)
                if 'through' in paired_reds[cycle_idx]:
                    #tmp is the Interval instance of through red
                    t0,t1,t2 = paired_reds[cycle_idx]['through']
                    mapped_red = pyinter.interval.closed(t0,t1)
                    mapped_cycle = pyinter.interval.closed(t0,t2)
                    if not ((t0>spilloverinterval_mapped.upper_value) or (t2<spilloverinterval_mapped.lower_value)):
                        paired_reds_mapped[cycle_idx]['through'] = mapped_red.union(spilloverinterval_mapped.intersect(mapped_cycle))
                else:
                    paired_reds_mapped[cycle_idx]['through'] = spilloverinterval_mapped

            ############through spillover map to left turn signal
            if float(tms_estimation_res_pd.loc[row_idx, 'through TMS duration'])>0:
                #t1-t0 is the red duration, and t2-t1 is the green duration. 
                duration = float(tms_estimation_res_pd.loc[row_idx, 'through TMS duration'])
                t0,t1,t2 = paired_reds[cycle_idx]['through']
                spilloverinterval_mapped = pyinter.interval.closed(t1 - duration,t1)
                if 'left' in paired_reds[cycle_idx]:
                    t0,t1,t2 = paired_reds[cycle_idx]['left']
                    mapped_red = pyinter.interval.closed(t0,t1)
                    mapped_cycle = pyinter.interval.closed(t0,t2)
                    #print('---', t0,t2, spilloverinterval_mapped)
                    if not ((t0>spilloverinterval_mapped.upper_value) or (t2<spilloverinterval_mapped.lower_value)):
                        paired_reds_mapped[cycle_idx]['left'] = mapped_red.union(spilloverinterval_mapped.intersect(mapped_cycle))
                else:
                    paired_reds_mapped[cycle_idx]['left'] = spilloverinterval_mapped

        return paired_reds_mapped

    @classmethod
    def RedDurations_from_pairedreds_mapped(self, paired_reds_mapped):
        """

        """
        reds = []


        pass


    @classmethod
    def Capcity_with_TMS_from_overlapped_intervalset(self, paired_reds_mapped, paired_reds, SFR = 1800):
        """
        Capctity
        """
        Capctities_with_tms = []
        for paired_red_mapped,paired_red in zip(paired_reds_mapped,paired_reds):
            cycle_capacity = 0
            direction = 'left'
            if direction in paired_red_mapped:
                t0,t1,t2 = paired_red[direction]
                cycle_length = t2-t0
                redduration=0
                if isinstance(paired_red_mapped[direction], pyinter.Interval):
                    redduration = paired_red_mapped[direction].upper_value - paired_red_mapped[direction].lower_value
                elif isinstance(paired_red_mapped[direction], pyinter.IntervalSet):
                    redduration  = sum([i.upper_value-i.lower_value for i in paired_red_mapped[direction]])
                cycle_capacity = cycle_capacity + SFR*(cycle_length-redduration)/cycle_length

            direction = 'through'
            if direction in paired_red_mapped:
                t0,t1,t2 = paired_red[direction]
                cycle_length = t2-t0
                redduration=0
                if isinstance(paired_red_mapped[direction], pyinter.Interval):
                    redduration = paired_red_mapped[direction].upper_value - paired_red_mapped[direction].lower_value
                elif isinstance(paired_red_mapped[direction], pyinter.IntervalSet):
                    redduration  = sum([i.upper_value-i.lower_value for i in paired_red_mapped[direction]])
                cycle_capacity = cycle_capacity + SFR*(cycle_length-redduration)/cycle_length

            Capctities_with_tms.append(cycle_capacity/2.0)

        return Capctities_with_tms



    @classmethod
    def Capcity_without_considering_TMS(self, paired_reds, SFR = 1800):
        """
        Capctity
        """
        Capctities_without_tms = []
        for cycle_idx,paired_red in enumerate(paired_reds):
            cycle_capacity = 0
            if 'left' in paired_red.keys():
                t0,t1,t2  = paired_red['left']
                cycle_capacity = cycle_capacity + SFR*(t2-t1)/(t2-t0)
            else:
                cycle_capacity = cycle_capacity + SFR
            if 'through' in paired_red.keys():
                t0,t1,t2  = paired_red['through']
                cycle_capacity = cycle_capacity + SFR*(t2-t1)/(t2-t0)
            else:
                cycle_capacity = cycle_capacity + SFR
            cycle_capacity = cycle_capacity/2.0

            Capctities_without_tms.append(cycle_capacity)


        return Capctities_without_tms

    @classmethod
    def Spillovers_Interface_Intervals(self, tms_estimation_res_pd, paired_reds, shift_time = 100.0/(40/3.6), directions_considered = ['left', 'through']):
        """
        Get the spillvoer invervals at the interface. 
        ----------------------------------------------
        input: shift_time
            a float. Used to denote the temporal shift of the spillover duration. 
        input: tms_estimation_res_pd
            a pd.DataFrame. 
            The index are like 'cycle_0', 'cycle_1'.
            Some 'cycle_i' may missing due to the lacking of trajectories. 
            a pandas.DataFrame. 
            Used columns include:
                ['left TMS duration', 'through TMS duration', 'TMS_ocurence_left', TMS_ocurence_through']
            They are the estimated left TMS, through TMS, the estimated left TMS occurrence and estimated through TMS occurrence. 
        input: paired_reds
            a list. 
            paired_reds[idx]['through'] = (start_of_red, end_of_red, end_of_green)
        ------------------------------------------------------------
        Output: spilloverIntervalsSet


        """
        effective_cycles_idx = []
        spillover_intervals = pyinter.IntervalSet()
        for row_idx in tms_estimation_res_pd.index:
            ############left
            if 'left' in directions_considered:
                if tms_estimation_res_pd.loc[row_idx, 'TMS_ocurence_left']==1 and tms_estimation_res_pd.loc[row_idx, 'left TMS duration']>0:
                    #t1-t0 is the red duration, and t2-t1 is the green duration. 
                    duration = float(tms_estimation_res_pd.loc[row_idx, 'left TMS duration'])
                    t0,t1,t2 = paired_reds[int(row_idx.split('_')[1])]['left']
                    spillover_intervals.add(pyinter.interval.open(t1 - shift_time - duration,t1-shift_time))

                    effective_cycles_idx.append(int(row_idx.split('_')[1]))

            ############through
            if 'through' in directions_considered:
                if tms_estimation_res_pd.loc[row_idx, 'TMS_ocurence_through']==1 and tms_estimation_res_pd.loc[row_idx, 'through TMS duration']>0:
                    #t1-t0 is the red duration, and t2-t1 is the green duration. 
                    duration = float(tms_estimation_res_pd.loc[row_idx, 'through TMS duration'])
                    t0,t1,t2 = paired_reds[int(row_idx.split('_')[1])]['through']
                    spillover_intervals.add(pyinter.interval.open(t1 - shift_time - duration,t1-shift_time))

                    if not int(row_idx.split('_')[1]) in effective_cycles_idx:
                        effective_cycles_idx.append(int(row_idx.split('_')[1]))


        return spillover_intervals,effective_cycles_idx


    @classmethod
    def slice_trajectory1(self, approach_TRs, paired_reds, kwargs,completeORnot = True):
        """
        Slice the trajectories. Different from self.slice_trajectory2() in that:
            - slice_trajectory1() seperate the trajectories of each turning directions when sliceing
            - slice_trajectory2() aggregate slice left, throug, etc.
        The structure of Output SlicedTRs is the same as  paired_reds, i.e. a list of dict. The keys of each element (i.e. a dict) are the same. 

        NOTe that, if there is no feasible TRs for cycle_idx's turning d, then 
            SlicedTRs[cycleidx][d] = {} (It still has key d!!!!!!!!!!!!!!!!!!!!!!)
        ---------------------------------------------------------
        input: paired_reds, a list
            the paired reds. paired_reds[idx] = {'left':(t0,t1,t2), 'through':(t0,t1,t2))}
        input: approach_TRs
            dict. 
            approach_TRs['left'][vehicle_id] = (ts, xs).
            ts and xs are both list. 
            plt.plot(ts, xs)
        input: l_tb, unit is ,
            length of turning bay aea, or length of C-section. 
        input: w vf. units are km/h
            the parameters in the FD settings. 
        input: loc_stopline
            location of the stopline. 
        input: direction_flag
            either 1 or -1.
            If it is 1, then the distance of the road increase upstream. 
        OUTPUT: 
            SlicedTRs, a list. Each element corresponds to the element in paired_reds
            SlicedTRs[idx] is a dict. means the trajectory for cycle idx
            SlicedTRs[idx].keys are the same as paired_reds[idx].keys.
            SlicedTRs[idx]['left']are dict. 
            SlicedTRs[idx]['left'][vehicle_id].keys() include:
                - 'tsxs':(ts,xs)
                - 't0t1t2':(t0,t1,t2)
                - 'intersectionpoint':(t_w, x_w)
                - 'redduration': redduration
                - 'twoboudnarys':((ta,xa),(tb,xb))
        -------------------------------------------------------
        Step:

        """
        loc_stopline = kwargs['loc_stopline']
        l_tb = kwargs['l_tb']
        w = kwargs['w']
        vf = kwargs['vf']
        direction_flag = kwargs['direction_flag']


        SlicedTRs = [{} for i in paired_reds]
        #turning_d may be 'left','through','right'
        for turning_d in approach_TRs.keys():
            for vid in approach_TRs[turning_d].keys():

                ts,xs = approach_TRs[turning_d][vid]
                for idx,paired_red in enumerate(paired_reds):
                    directions = paired_red.keys()
                    #d may be either 'left' or right
                    for d in directions:#the d is for paired_red

                        if not SlicedTRs[idx].has_key(d):
                            SlicedTRs[idx][d] = {}

                        if not d==turning_d:continue
                        t0,t1,t2 = paired_red[d]
                        #if not intersect, t_w would be False
                        t_w,x_w = self.tsxsIntersectionPoint(ts=ts, xs=xs, t0 = t1, loc_stopline=loc_stopline, w=w)
                        if t_w==False:
                            #print(ts[0]-abs(xs[0]-loc_stopline)/(w/3.6), t1, ts[-1]-abs(xs[-1]-loc_stopline)/(w/3.6),'dddd');
                            continue
                        if abs(x_w-loc_stopline)<l_tb:continue

                        #print(ts[0],ts[-1], t0,t1,t2)
                        #slice. 
                        ts0,xs0,complete = self.tsxs_cycleslice(ts=copy.deepcopy(ts), xs=copy.deepcopy(xs), t0=t0, t1=t2, loc_stopline=loc_stopline,w = w)
                        SlicedTRs[idx][d][vid] = {'tsxs':(ts0,xs0), 'intersectionpoint':(t_w, x_w), 'redduration':t1-t0, 'cyclestartmoment':t0, 't0t1t2':(t0,t1,t2)}
        return SlicedTRs

    @classmethod
    def reading_sumo_files_data(self, probefile = "", signalstatefile =  '/home/qhs/Qhs_Files/Program/Python/difference_MFD/sumo_files/output_signalstatesswitchstates.xml'):
        """

        """
        


        pass


    @classmethod
    def rgp_end_moment(self, list_of_intervals, idx):
        """
        the end moment of the rgp. or the end moment of the green. 
        Usage:
            csswithsignal.rgp_end_moment(east_reds['left'],46)

        """
        if idx>=len(list_of_intervals)-1:
            return np.inf
        else:
            return list_of_intervals[idx+1].lower_value
        pass
    @classmethod
    def random_select_TRs(self, TRs_dict, N = 10, basedondelay = True):
        """
        ---------------------------
        input: basedondelay
            the trajectory would influence the quatlity. THus basedondelay means select based on delay It the vehicle TR display a clear stop-go pattern then it rank high for selection
        """
        if N>=len(TRs_dict):return TRs_dict
        res = {}
        if basedondelay:
            vids0 = TRs_dict.keys()
            speeds = []
            for vid in vids0:
                ts,xs = TRs_dict[vid]['tsxs']
                #speeds.append((xs[-1]-xs[0])/(ts[-1]-ts[0]))
                speeds.append(ts[0])
            #speed in ascending order, for output. 
            vids = pd.Series(vids0).iloc[np.array(speeds).argsort()][:N]
            for vid in vids:
                res[vid] = TRs_dict[vid]
        else:
            vids = np.random.choice(TRs_dict.keys(),min(N, len(TRs_dict.keys())))
            
            for vid in vids:
                res[vid] = TRs_dict[vid]
        return res

    @classmethod
    def get_paired_reds(self, approach_reds, paired_label = ['left', 'through']):
        """
        Get the reds overlapped or exactly adjecent. This is for spillover estimation. 
        This function assume that each red cannot overlap with more than two reds of other directions. 
        --------------------------------------------------------
        input: approach_reds
            a dict. Keys are ['right', 'through', 'left']
            approach_reds['left'] is a list, and each element is a pinter.interval.Interval instance. approach_reds['left'].lower_value is the onset of i-th red signal; upper_value give the red termination moment.  
        input: paired_label
            the turning direction of the required pair. 
        output: paired_reds
            a list. 
            Each element is a dict. paired_reds[i] = {'left': (t0,t1,t2), 'through': (t0,t1,t2)}.
            t0, t1 , t2 are the red start, red end and next red start. 
            It means that the left signal and thrugh signal are exact adjecent or partially overlap.
            If some element length is 1, then it means this red is single.  
        ----------------------------------------------------
        Steps: 
            - 1 find the earliest red on set moment
            - 
        """
        result = []

        #function for finding the other direction
        #   for instance, if current is 'left', then 
        #       other_direction('left', paired_label)
        #   will return 'through'
        other_direction = lambda current,paired_label:list(set(copy.deepcopy(paired_label)).difference([current]))[0]

        def TerminationCheck(indexes_track, approach_reds):
            for d in indexes_track.keys():
                if indexes_track[d]>=len(approach_reds[d]):
                    #means terminated and direction is d
                    return True,d
            #means not terminated
            return False,'a'
            

        #find the earliest red onset moment
        onset_ts = [approach_reds[d][0].lower_value for d in paired_label]
        current_direction = paired_label[onset_ts.index(min(onset_ts))]
        red_current = copy.deepcopy(approach_reds[current_direction][0])

        #the tracking index 
        indexes_track = {}
        for d in paired_label:indexes_track[d] = 0

        while True:

            #termination check
            if TerminationCheck(indexes_track, approach_reds)[0]:
                tmp_d = TerminationCheck(indexes_track, approach_reds)[1]
                #set all remaining reds
                current_direction = other_direction(tmp_d, paired_label)
                for idx in range(indexes_track[current_direction],len(approach_reds[current_direction])):
                    tmp = approach_reds[current_direction][idx]
                    tmp1 = self.rgp_end_moment(approach_reds[current_direction], idx)
                    result.append({current_direction:(tmp.lower_value, tmp.upper_value, tmp1)})
                return result

            #current red of current direction. 
            #   current index is indexes_track[current_direction]
            red_current = copy.deepcopy(approach_reds[current_direction][indexes_track[current_direction]])
            #will be used when: result.append(result_temp)
            rgp_end_current = self.rgp_end_moment(approach_reds[current_direction], indexes_track[current_direction])
            result_temp = {current_direction:(red_current.lower_value, red_current.upper_value, rgp_end_current)}

            #another direction, e.g.' left'
            other_d = other_direction(current_direction, paired_label)
            #red of other direction.
            red_other = copy.deepcopy(approach_reds[other_d][indexes_track[other_d]])
            rgp_end_other = self.rgp_end_moment(approach_reds[other_d], indexes_track[other_d])

            while True:

                #termination check
                #TerminationCheck(indexes_track, approach_reds)---> (True, direction)
                if TerminationCheck(indexes_track, approach_reds)[0]:
                    tmp_d = TerminationCheck(indexes_track, approach_reds)[1]
                    #set all remaining reds
                    current_direction = other_direction(tmp_d, paired_label)
                    for idx in range(indexes_track[current_direction],len(approach_reds[current_direction])):
                        tmp_red = approach_reds[current_direction][idx]
                        rgp_end = self.rgp_end_moment(approach_reds[current_direction], idx)
                        result.append({current_direction:(tmp_red.lower_value, tmp_red.upper_value, rgp_end)})
                    return result
                
                #no over lap
                if red_other.lower_value > red_current.upper_value:
                    result.append(result_temp)
                    indexes_track[current_direction] = indexes_track[current_direction] +1
                    #indexes_track[other_d] = indexes_track[other_d] +1
                    current_direction = other_d
                    break
                else:
                    if red_other.upper_value < red_current.upper_value:
                        result_temp[other_d] = (red_other.lower_value, red_other.upper_value, rgp_end_other)
                        indexes_track[other_d] = indexes_track[other_d] +1
                    elif red_other.upper_value > red_current.upper_value:
                        result_temp[other_d] = (red_other.lower_value, red_other.upper_value, rgp_end_other)
                        indexes_track[current_direction] = indexes_track[current_direction] +1
                        current_direction = other_d
                        result.append(result_temp)
                        break
                    elif red_other.upper_value == red_current.upper_value:

                        result_temp[other_d] = (red_other.lower_value, red_other.upper_value, rgp_end_other)
                        indexes_track[current_direction] = indexes_track[current_direction] +1
                        indexes_track[other_d] = indexes_track[other_d] +1
                        result.append(result_temp)
                        break


    @classmethod
    def Points_EDJ(self, vehicle_slice_data, tau, phi, redduration, loc_stopline, l_tb, l_tb_direction = 1, w = FD.w, vf =FD.vf):
        """
        Three points of the triangular. 
        """
        #first segment
        t_w,x_w = vehicle_slice_data['intersectionpoint']
        cyclestartmoment = vehicle_slice_data['cyclestartmoment']
        cycleendmoment = cyclestartmoment + redduration
        spillover_end_t = cycleendmoment + (l_tb)/(w/3.6)
        vstop_vector_t0 = spillover_end_t - tau
        vstop_vector_x0 = loc_stopline + l_tb_direction*l_tb
        vstop = 1.0*(np.abs(x_w - loc_stopline) - l_tb)/phi
        #   the end of the arrow of vstop, in (vstop_vector_t1, vstop_vector_x1)
        vstop_vector_t1 = vstop_vector_t0 + phi
        vstop_vector_x1 = x_w

        #the start of the theoretical trajectory, in (t_theoretical, x_theoretical)
        tmp_m = redduration - (t_w - vstop_vector_t1)
        tmp_h = tmp_m * (w/3.6 * vf/3.6)/(w/3.6 + vf/3.6)
        x_theoretical = x_w + l_tb_direction*tmp_h
        t_theoretical = vstop_vector_t1 - tmp_h/(vf/3.6)

        return [(vstop_vector_t1, vstop_vector_x1), (t_theoretical, x_theoretical), (t_w - redduration, x_w)]



    @classmethod
    def bokeh_polygon(self, vehicle_slice_data, tau, phi, redduration, loc_stopline, l_tb, l_tb_direction = 1, w = FD.w, vf =FD.vf):
        """
        --------------------------------------
        output: tss,xss
            the point series that surround the area, or the objective function. 
        """
        #first segment
        ts,xs = vehicle_slice_data['tsxs']
        t_w,x_w = vehicle_slice_data['intersectionpoint']
        cyclestartmoment = vehicle_slice_data['cyclestartmoment']
        cycleendmoment = cyclestartmoment + redduration
        spillover_end_t = cycleendmoment + (l_tb)/(w/3.6)
        vstop_vector_t0 = spillover_end_t - tau
        vstop_vector_x0 = loc_stopline + l_tb_direction*l_tb
        vstop = 1.0*(np.abs(x_w - loc_stopline) - l_tb)/phi
        #   the end of the arrow of vstop
        vstop_vector_t1 = vstop_vector_t0 + phi
        vstop_vector_x1 = x_w

        #the start of the theoretical trajectory
        tmp_m = redduration - (t_w - vstop_vector_t1)
        tmp_h = tmp_m * (w/3.6 * vf/3.6)/(w/3.6 + vf/3.6)
        x_theoretical = x_w + l_tb_direction*tmp_h
        t_theoretical = vstop_vector_t1 - tmp_h/(vf/3.6)

        return list(ts) + [t_w, vstop_vector_t1, t_theoretical, ts[0]], list(xs) + [x_w, vstop_vector_x1, x_theoretical, xs[0]]
        
    @classmethod
    def Get_inputdata_oneturning(self, reds, trajectory_turning_tsxs, loc_stopline=0, w = 20, completeORnot = True, l_tb = 100):
        """ 
        ------------------------------- 
        input: reds, 
            reds[i] = pyinterval reds[i].lower_value is the red onset moment reds[i].upper_value is the red termination
            is a list.  the i-th element is the duration of the i-th red signal. moment. 
        input: trajectory_turning_tsxs 
            dict, keys include 'l', 't' and 'r', corresponding to left, through and right respectively.  trajectory_turning_tsxs['l'][vehicle_id] = (ts,xs) input: w, unit is km/h. the backward wave speed. 
        input: loc_stopline, float, 
            unit is m the location of stopline.  
        input: completeORnot a bool, 
            indicate the return trajectory need to be complete or can be not complelte. 
            if completeORnot=True, then the trajecotry must occupy the whole time horizon of the cycle.  
        OUTPUT: cycles_sliceddata A DICT. keys are the cycle index. 
            cycles_sliceddata[cycle_idx].keys()= ['l','t','r'] cycles_sliceddata[cycle_idx]['l'][vehicle_id1] ={'tsxs':(ts,xs),'intersectionpoint':(t,x),'twoboundarys':((t0,x0), (t2,x2)), 'redduration':red,
            'cyclestartmoment':cyclestartmoment}.  'twoboundarys':  
            t0,x0 is the leftboundary and (t2,x2) is the right boundary.
            'redduration' and 'cyclestartmoment': 
                If the trajectory is totally cover the cycle time horizon, then redruation is the real duration, and the cyclestartmoment is the real cycle start moment. 
                However, if the trajectory just partially occupy the cycle time horizon, the red duration and the cycle startmoment depends on the trajectory (the first point within the cycle).  
                Suppose the first point is (t,x), the the cycle startmoment is calculated as: t-(x-loc_stopline)/(w/3.6) the red duration is calculated as  t-(x-loc_stopline)/(w/3.6)-t1
                    t1 is the termination moment of red. 

        ------------------------------------
        Steps:
            - 

        """
        w = w/3.6
        cycles_sliceddata = {}
        
        #Get cycle moments
        #   cycle_moments[i] = t0,t1,t2
        cycle_moments = self.signal_3moments(reds)
        for cycle_idx in range(len(cycle_moments)):
            #the three boundary moments at stopline
            t0,t1,t2 = cycle_moments[cycle_idx]
            for turningdirection in trajectory_turning_tsxs.keys():
                for v_id in trajectory_turning_tsxs[turningdirection].keys():
                    ts,xs = trajectory_turning_tsxs[turningdirection][v_id]
                    #check if the trajectory intersect with the line start from t1 with slope w
                    # if intersect, then intersection_t is a float, else intersection_t is False
                    intersection_t,intersection_x = self.tsxsIntersectionPoint(ts=ts, xs=xs, t0 = t1, loc_stopline=loc_stopline, w=w*3.6)
                    if intersection_t:
                        partial_ts,partial_xs,completeness = self.tsxs_cycleslice(ts=ts, xs=xs, t0=t0, t1=t2, loc_stopline=loc_stopline,w = w*3.6)
                        
                        #check the trajectory completeness within the cycle. 
                        if completeORnot:
                            if not completeness:continue
                        
                        #compute the redduration and the cyclestartmomnet 
                        cyclestartmoment = partial_ts[0] - (partial_xs[0]-loc_stopline )/w
                        redduration = t1 - cyclestartmoment
                        twoboudnarys= ((partial_ts[0], partial_xs[0]), (partial_ts[-1], partial_xs[-1]))
                        #result assignment
                        if not cycles_sliceddata.has_key(cycle_idx):
                            cycles_sliceddata[cycle_idx]={}
                        if not cycles_sliceddata[cycle_idx].has_key(turningdirection):
                            cycles_sliceddata[cycle_idx][turningdirection]={}
                        
                        cycles_sliceddata[cycle_idx][turningdirection][v_id] = {'tsxs':(partial_ts,partial_xs),'intersectionpoint':(intersection_t,intersection_x),'twoboudnarys':twoboudnarys, 'redduration':redduration, 'cyclestartmoment':cyclestartmoment}

                        #print cycles_sliceddata[cycle_idx][turningdirection][v_id].keys()
        return cycles_sliceddata

    @classmethod
    def plot_turning_TRs_with_X0(self, TurningTRs_dict, X, optimization_variables_sequence, kwargs,thickness_tsxs = 2, delta_loc = -4, polygon = False, outputpath = 'TMS/figs/', namee = 'Temp.html', tools="pan,box_zoom,reset,save",):
        """
        input: TurningTRs_dict
            a dict. 
            Keys are vids. 
            one_vehicle_TR = TurningTRs_dict[vid]
            one_vehicle_TR.keys() include 
                TR_info['tsxs']  = (ts,xs), 
                TR_info['redduration']  =  redduration, a float;
                TR_info['intersectionpoint']  =  (t_w, x_w), 
                TR_info['t0t1t2']  = (t0,t1,t2), 
        input: X
            the decision variables. 
            X[0] is TMS duration, 
        input: optimization_variables_sequence
            a list of vids. len(X) = len(optimization_variables_sequence) + 1
        """
        loc_stopline = kwargs['loc_stopline']
        l_tb = kwargs['l_tb']
        w = kwargs['w']
        vf = kwargs['vf']
        direction_flag = kwargs['direction_flag']
        tolerance = kwargs['tolerance']
        ufi = kwargs['ufi']

        if set(TurningTRs_dict.keys())!=set(optimization_variables_sequence):
            raise ValueError("sddzfsdfsvsdfjjkk")

        colors = self.Get_colors()
        bokeh_output_file(outputpath + namee)
        p = bokeh_figure(tools = tools,title="Sliced trajectories", x_axis_label='time', y_axis_label='space')

        #random color
        #any_palettes = np.random.choice(bokeh_palettes.__palettes__)
        #colors = getattr(bokeh_palettes, any_palettes)

        for idx,vid in enumerate(optimization_variables_sequence):
            color = np.random.choice(colors)
            ts,xs = TurningTRs_dict[vid]['tsxs']
            redduration = TurningTRs_dict[vid]['redduration']
            p.line(ts,xs,line_color = color, line_width = thickness_tsxs)

            t_w, x_w = TurningTRs_dict[vid]['intersectionpoint']
            #first segment of the theoretical TR
            #   firstly compute the endpoint.
            end_point_x = x_w
            end_point_t = t_w - (abs(x_w - loc_stopline)-l_tb)/(w/3.6)-X[0] + (abs(x_w - loc_stopline)-l_tb)/(X[idx+1]/3.6)
            #   second, compute the coordinate of starting point. 
            tmp_width = redduration - (t_w - end_point_t)#unit is sec
            tmp_height = tmp_width*w/3.6*vf/3.6/(w/3.6+vf/3.6)#unit is m.
            start_point_t = end_point_t - tmp_height/(vf/3.6)
            start_point_x = end_point_x + direction_flag*tmp_height
            p.line([end_point_t, start_point_t],[end_point_x, start_point_x], line_color = color, line_dash ='dashed')

            #second segment
            p.line([end_point_t, t_w], [end_point_x, x_w],line_color = color, line_dash ='dashed')

            #third segment
            greenduration = TurningTRs_dict[vid]['t0t1t2'][2]- TurningTRs_dict[vid]['t0t1t2'][1]
            end_point_t = t_w + greenduration*w/3.6/(w/3.6+ufi/3.6)
            end_point_x = x_w- direction_flag*greenduration*w/3.6*ufi/3.6/(w/3.6+ufi/3.6)
            p.line([t_w, end_point_t], [x_w, end_point_x], line_dash ='dashed',line_color = color)
        bokeh_show(p)


    @classmethod
    def plot_one_TR_without_polygon(self, TR_info, tau, vstop,loc_stopline, l_tb, w= FD.w, vf=FD.vf, ufi = FD.vf, direction_flag=1, outputpath = 'TMS/figs/', namee = 'Temp.html', thickness = 2,  ):
        """
        will plot the trajectory, the theoretical TR and the area. 
        --------------------------------------
        input: TR_info
            dict. 
            TR_info['tsxs']  = (ts,xs), 
            TR_info['redduration']  =  redduration, a float;
            TR_info['intersectionpoint']  =  (t_w, x_w), 
            TR_info['t0t1t2']  = (t0,t1,t2), 
        input: polygonn
            a shapely.Polygon instance, the area. 
            polygonn.boundary is a LineString. 
            aa = polygonn.boundary
            ts_bounds, xs_bounds = polygonn.boundary.xy
            then ts_bounds and xs_bounds are both np.array. 
        """
        bokeh_output_file(outputpath + namee)
        p = bokeh_figure(title="simple line example", x_axis_label='x', y_axis_label='y')

        #theoretical_tr['FirstSegment'] = ((t0,x0),(t1,x1))
        theoretical_tr = self.Theoretical_TR(TR_info=TR_info, tau=tau, vstop=vstop,loc_stopline=loc_stopline, l_tb=l_tb, w= w, vf=vf, ufi = vf, direction_flag=1)


        #Steps: first get the attributes
        ts,xs = TR_info['tsxs']
        t_w,x_w = TR_info['intersectionpoint']
        p.circle(t_w,x_w, radius = 1)

        #trajectory
        p.line(ts,xs)
        #
        p0,p1 = theoretical_tr['FirstSegment']
        p.line([p0[0], p1[0]], [p0[1], p1[1]])

        p0,p1 = theoretical_tr['SecondSegment']
        p.line([p0[0], p1[0]], [p0[1], p1[1]])

        p0,p1 = theoretical_tr['ThirdSegment']
        p.line([p0[0], p1[0]], [p0[1], p1[1]])

        #plot boundary
        p0,p1 = theoretical_tr['LT_bound_w']
        p.line([p0[0], p1[0]], [p0[1], p1[1]], line_dash = 'dashed')
        p0,p1 = theoretical_tr['RT_bound_w']
        p.line([p0[0], p1[0]], [p0[1], p1[1]], line_dash = 'dashed')
        bokeh_show(p)

    @classmethod
    def plot_one_TR_with_ploygon(self, TR_info, polygonn, tau, vstop,loc_stopline, l_tb, thickness = 2, delta_loc = -4, sequence = ['left','through'], w= FD.w, vf=FD.vf, ufi = FD.vf, direction_flag=1, outputpath = 'TMS/figs/', namee = 'Temp.html'):
        """
        will plot the trajectory, the theoretical TR and the area. 
        --------------------------------------
        input: TR_info
            dict. 
            TR_info['tsxs']  = (ts,xs), 
            TR_info['redduration']  =  redduration, a float;
            TR_info['intersectionpoint']  =  (t_w, x_w), 
            TR_info['t0t1t2']  = (t0,t1,t2), 
        input: polygonn
            a shapely.Polygon instance, the area. 
            polygonn.boundary is a LineString. 
            aa = polygonn.boundary
            ts_bounds, xs_bounds = polygonn.boundary.xy
            then ts_bounds and xs_bounds are both np.array. 
        """
        bokeh_output_file(outputpath + namee)
        p = bokeh_figure(title="simple line example", x_axis_label='x', y_axis_label='y')

        #Steps: first get the attributes
        ts,xs = TR_info['tsxs']
        t_w, x_w = TR_info['intersectionpoint']
        redduration = TR_info['redduration']
        greenduration = TR_info['t0t1t2'][2] - TR_info['t0t1t2'][1]

        if not np.all(np.diff(ts) > 0):
            builtins.tmp = ts
            raise ValueError("The input ts must be increasing!!!ts is stored in builtins.tmp")

        #trajectory
        p.line(ts,xs)

        #tao

        #area patch
        ts_bounds,xs_bounds = polygonn.boundary.xy
        p.patch(ts_bounds, xs_bounds, alpha=0.5, line_width=0)

        #first segment of theoretical TR
        #   first compute the corrdinate of end point
        startmoment_TMS = t_w - tau
        end_point_x = x_w
        end_point_t = startmoment_TMS + (abs(x_w - loc_stopline)-l_tb)/(vstop/3.6)
        if end_point_t<startmoment_TMS or end_point_t>t_w:
            raise ValueError("sdfsdfsdfsadfdfwef")
        end_point = (end_point_t, end_point_x)
        #   second, compute the coordinate of starting point. 
        tmp_width = redduration - (t_w - end_point_t)#unit is sec
        tmp_height = tmp_width*w/3.6*vf/3.6/(w/3.6+vf/3.6)#unit is m.
        start_point = (end_point_t - tmp_height/(vf/3.6), end_point_x + direction_flag*tmp_height)
        p.line([start_point[0], end_point[0]], [start_point[1], end_point[1]])
        

        bokeh_show(p)

    @classmethod
    def plot_sliced_TRs_decisionvariables(self, cycle_sliceddata_turning, X0, optimization_variables_sequence_turning, redduration, loc_stopline, l_tb, l_tb_direction = 1, w = FD.w, vf =FD.vf, plot_point_radius = 1.5, TRs_width=2, outputpath = 'TMS/figs/', namee = 'Temp.html'):
        """
        plot the sliced trajectories and the decision variables, includeing the spillover duration and the 'vstop'. (the vector 'vstop' starts from )
        -------------------------------------------------
        input: cycle_sliceddata_turning
            the trajectory info for each cycle. cycle_sliceddata_turning is a dict, keys are vehicle_id. 
            cycle_sliceddata_turning[vehicle_id].keys()=['tsxs', 'intersectionpoint', 'twoboudnarys', 'redduration', 'cyclestartmoment']

            cycle_sliceddata_turning[vehicle_id]['tsxs']  =(ts,xs)
            cycle_sliceddata_turning[vehicle_id]['intersectionpoint'] = (t_w,x_w)
            cycle_sliceddata_turning[vehicle_id]['twoboudnarys'] = ((t0,x0),(t2,x2))
            cycle_sliceddata_turning[vehicle_id]['redduration'] = r
            cycle_sliceddata_turning[vehicle_id]['cyclestartmoment'] = cyclestartmoment
        input: X0
            the decision variables, X0[0] is the tau, i.e. spillover duration; the remaining elements are phi, which is calculated as phi = (x_w - l_tb)/v_stop. 
            Thus all elements in X0 have unit in sec. 
        input: optimization_variables_sequence
            indicate the sequence of the decision variable. 
            optimization_variables_sequence[i] = vehicle_id
            It means that i-th+1 variable in X is for vehicle_id.
            Plus 1 is because that the X[0] is the spillover duration. 

            optimization_variables_sequence should be identical to cycle_sliceddata_turning.keys(). 
        input: w
            the backward wave speed.
        input: loc_stopline
            the location of the stopline. x-loc_stopline is the distance of the point from the stopline.
        input: l_tb
            the length of turning bay area.
        input: l_tb_direction, either 1 or -1
            When calculating the location of interface, loc_stopline + l_tb_direction*l_tb
        ---------------------------------------------
        Steps: 
            - first 
        """
        colors = self.Get_colors()
        bokeh_output_file(outputpath + namee)
        p = bokeh_figure(title="Sliced trajectories & decision variables", x_axis_label='time', y_axis_label='space')
        #plot the red duration. 
        anyvid = np.random.choice(cycle_sliceddata_turning.keys())
        cyclestartmoment = cycle_sliceddata_turning[anyvid]['cyclestartmoment']
        cycleendmoment = cyclestartmoment + redduration
        #red signal
        p.rect(x = (cyclestartmoment + cycleendmoment)/2.0, y = loc_stopline, width = redduration, height = 2, color = 'red')
        #tau, or spillover duration
        spillover_end_t = cycleendmoment + (l_tb)/(w/3.6)
        p.rect(x = spillover_end_t-X0[0]/2.0, y = loc_stopline + l_tb_direction*l_tb, width = X0[0], height = .5, color = 'red')
        #   
        vstop_vector_t0 = spillover_end_t - X0[0]
        vstop_vector_x0 = loc_stopline + l_tb_direction*l_tb
        #for each trajectory, plot the trajectory, vstop
        for i,vid in enumerate(optimization_variables_sequence_turning):
            color = np.random.choice(colors)

            ts,xs = cycle_sliceddata_turning[vid]['tsxs']
            t_w,x_w = cycle_sliceddata_turning[vid]['intersectionpoint']
            #add trajectory
            p.line(ts,xs, color = color, line_width= TRs_width)
            #add intersection point
            p.circle(x=t_w, y=x_w, radius = plot_point_radius)
            
            #add vstop, (vstop_vector_t0, vstop_vector_x0) and (vstop_vector_t1, vstop_vector_x1) are two ends of the vector
            phi = X0[i+1]
            vstop = 1.0*(np.abs(x_w - loc_stopline) - l_tb)/phi
            vstop_vector_t1 = vstop_vector_t0 + phi
            vstop_vector_x1 = x_w
            p.line([vstop_vector_t0, vstop_vector_t1], [vstop_vector_x0, vstop_vector_x1],  color = color, line_dash = 'dashed')

            #add theoretical trajectory
            p.line([vstop_vector_t1, t_w], [vstop_vector_x1, x_w],  color = color, line_dash = 'solid')
            p.circle(x=vstop_vector_t1, y=vstop_vector_x1, radius = plot_point_radius)
            #   the start of the theoretical trajectory
            tmp_m = redduration - (t_w - vstop_vector_t1)
            tmp_h = tmp_m * (w/3.6 * vf/3.6)/(w/3.6 + vf/3.6)
            x_theoretical = x_w + l_tb_direction*tmp_h
            t_theoretical = vstop_vector_t1 - tmp_h/(vf/3.6)
            p.line([t_theoretical, vstop_vector_t1], [x_theoretical, vstop_vector_x1],  color = color, line_dash = 'solid')
            #p.add_layout(Arrow(end=VeeHead(size=35), line_color=color,x_start= vstop_vector_t0, y_start = vstop_vector_x0, x_end=vstop_vector_t1, y_end=vstop_vector_x1))

        bokeh_show(p)

    @classmethod
    def plot_cycles_sliceddata(self, cycles_sliceddata, colors = {'l':'black','t':'blue','r':'green'}, outputpath = 'TMS/figs/', namee = 'Temp.html'):
        """
        plot the cycle sliced trajectory data. 
        ------------------------------------------
        input: cycles_sliceddata
            cycles_sliceddata[2]['l']['type_east_left_360.2'].keys() will gives:
                ['tsxs',
                 'intersectionpoint',
                 'twoboudnarys',
                 'redduration',
                 'cyclestartmoment']
        '2' is cycle index
        --------------------------------------------
        """
        bokeh_output_file(outputpath + namee)
        p = bokeh_figure(title="Sliced trajectories", x_axis_label='time', y_axis_label='space')

        #random color
        #any_palettes = np.random.choice(bokeh_palettes.__palettes__)
        #colors = getattr(bokeh_palettes, any_palettes)

        for cycleidx in cycles_sliceddata.keys():
            #turningdirections contain the existing turning directions, 
            #   i.e.  'l','t' and 'r'
            turningdirections = cycles_sliceddata[cycleidx].keys()
            for d in turningdirections:
                for vid in cycles_sliceddata[cycleidx][d].keys():
                    ts,xs = cycles_sliceddata[cycleidx][d][vid]['tsxs']
                    #print(ts,xs,d)
                    #print(d)
                    p.line(ts,xs,line_color = colors[d])
        bokeh_show(p)

    @classmethod
    def plot_approach_tsxs_with_paired_reds(self, approach_tsxs, paired_reds, outputpath = 'TMS/figs/', namee = 'Temp.html', loc_stopline = 421, thickness = 2, delta_loc = -4, sequence = ['left','through'], w= FD.w):
        """

        """
        bokeh_output_file(outputpath + namee)
        p = bokeh_figure(title="simple line example", x_axis_label='x', y_axis_label='y')
        delta_x_max = 0
        for td in approach_tsxs.keys():
            for vid in approach_tsxs[td].keys():
                ts,xs = approach_tsxs[td][vid]
                delta_x_max = max([delta_x_max, max(np.array(xs)-loc_stopline)])
                p.line(ts,xs)

        for paired_red in paired_reds:
            for td in paired_red.keys():
                t0,t1,t2 = paired_red[td]
                T = t1 + delta_x_max/(w/3.6)
                p.line([t1,T], [loc_stopline,loc_stopline+delta_x_max], line_color = 'black')

        bokeh_show(p)
        pass

    @classmethod
    def plot_approach_tsxs_with_signal(self, approach_tsxs, approach_reds, outputpath = 'TMS/figs/', namee = 'Temp.html', loc_stopline = 421, thickness = 2, delta_loc = -4, sequence = ['left','through']):
        """
        
        """
        bokeh_output_file(outputpath + namee)
        p = bokeh_figure(title="simple line example", x_axis_label='x', y_axis_label='y')
        
        for td in approach_tsxs.keys():
            for vid in approach_tsxs[td].keys():
                ts,xs = approach_tsxs[td][vid]
                p.line(ts,xs)

        x= 0
        for td in sequence:
            for red in approach_reds[td]:
                s  = red.lower_value
                e = red.upper_value
                p.line([s,e], [loc_stopline+delta_loc*x,loc_stopline+delta_loc*x], line_width = thickness ,line_color = 'red')
            x = x+1

        bokeh_show(p)

    @classmethod
    def plot_approach_tsxs(self, approach_tsxs, outputpath = 'TMS/figs/', namee = 'Temp.html'):
        """

        """
        bokeh_output_file(outputpath + namee)
        p = bokeh_figure(title="simple line example", x_axis_label='x', y_axis_label='y')
        for td in approach_tsxs.keys():
            for vid in approach_tsxs[td].keys():
                ts,xs = approach_tsxs[td][vid]
                p.line(ts,xs)

        bokeh_show(p)


    @classmethod
    def samplesize_cross(self, durations_accuracy, TRs_paired_reds_corss):
        """

        """
        samples_size = pd.DataFrame()
        for cycleidx in durations_accuracy.index:
            samples_size.loc[cycleidx, 'left'] = len(TRs_paired_reds_corss[int(cycleidx[6:])]['left']['left'])
            samples_size.loc[cycleidx, 'through'] = len(TRs_paired_reds_corss[int(cycleidx[6:])]['through']['through'])

        return samples_size


    @classmethod
    def delete_some(self, occurrence_accuracy, duration_accuracy, thh = .6):
        """
        
        """

        #
        cycles = []
        for cycleidx in occurrence_accuracy.index:
            #print((duration_accuracy.loc[cycleidx, :]>thh).sum())
            if not (duration_accuracy.loc[cycleidx, ['left','through']]>thh).sum()>0:
                cycles.append(cycleidx)

        return occurrence_accuracy.loc[cycles,:], duration_accuracy.loc[cycles,:]

    @classmethod
    def plot_PI_durations_pd(self, duration_accuracy,  outputpath = 'TMS/figs/', namee = 'Temp.html', title="simple line example", x_axis_label='x', y_axis_label='y',plot_height=400, plot_width = 400, sizee = 6):
        """
        input: duration_accuracy
            pd.DataFrame data. 
            duration_accuracy.index = ['cycle_0', 'cycle_1'....]
            duration_accuracy.columns = 'left' and 'through'
        """
        colors = self.Get_colors( randomize = False)

        bokeh_output_file(outputpath + namee)
        p = bokeh_figure(title=title, x_axis_label=x_axis_label, y_axis_label=y_axis_label, plot_height=plot_height, plot_width=plot_width)
        p.line(range(len(duration_accuracy['left'].values)), duration_accuracy['left'].values, line_color = np.random.choice(colors))
        p.circle(range(len(duration_accuracy['left'].values)), duration_accuracy['left'].values, size=sizee, legend = 'Left')
        #p.xaxis.ticker = range(len(duration_accuracy['left'].values))
        #p.xaxis.major_label_overrides = dict(pd.Series(duration_accuracy.index))


        p.line(range(len(duration_accuracy['through'].values)), duration_accuracy['through'].values, line_color = np.random.choice(colors))
        p.square(range(len(duration_accuracy['through'].values)), duration_accuracy['through'].values,size = sizee, fill_alpha=False,  legend = 'Through')

        bokeh_show(p)


    @classmethod
    def plot_TRs_dict(self, TRs_dict,  outputpath = 'TMS/figs/', namee = 'Temp.html'):
        """
        ---------------------------
        input: sliced_TRs, a list
            sliced_TRs[cycle_idx][turingdirection_signal][vehicle_id].keys() include:
        input: 


        """

        colors = self.Get_colors( randomize = False)

        bokeh_output_file(outputpath + namee)
        p = bokeh_figure(title="simple line example", x_axis_label='x', y_axis_label='y')
        for vid in TRs_dict.keys():
            ts,xs = TRs_dict[vid]['tsxs']
            t_w,x_w = TRs_dict[vid]['intersectionpoint']
            p.line(ts,xs,line_color = np.random.choice(colors))
            p.circle(t_w,x_w, radius = 1)
        bokeh_show(p)


    @classmethod
    def plot_TRs_with_X(self, TRs_dict,  outputpath = 'TMS/figs/', namee = 'Temp.html'):
        """
        ---------------------------
        input: sliced_TRs, a list
            sliced_TRs[cycle_idx][turingdirection_signal][vehicle_id].keys() include:
        input: 


        """

        colors = self.Get_colors( randomize = False)

        bokeh_output_file(outputpath + namee)
        p = bokeh_figure(title="simple line example", x_axis_label='x', y_axis_label='y')
        for vid in TRs_dict.keys():
            ts,xs = TRs_dict[vid]['tsxs']
            t_w,x_w = TRs_dict[vid]['intersectionpoint']
            p.line(ts,xs,line_color = np.random.choice(colors))
            p.circle(t_w,x_w, radius = 1)
        bokeh_show(p)


    @classmethod
    def TRs_sliced_corss_delete_right(self, sliced_TRs_corss):
        """

        """
        for cycle_TRs_corss in sliced_TRs_corss:
            for signal_d in cycle_TRs_corss.keys():
                if cycle_TRs_corss[signal_d].has_key('right'):
                    del cycle_TRs_corss[signal_d]['right']

    @classmethod
    def TRs_numbers_with_cross(self, sliced_TRs_corss, delete_right = False):
        """
        The difference between self.TRs_numbers_without_cross() and self.TRs_numbers_with_cross() are that:
            - The input of self.TRs_numbers_without_cross() is sliced_TRs[cycle_idx][sliced_signal_turning][vehicle_id] = {'tsxs':..}
            - The input of self.TRs_numbers_with_cross() is sliced_TRs[cycle_idx][sliced_signal_turning][TR_turning][vehicle_id] = {'tsxs':..}
        ------------------------------------------------------------------
        input:

        input:delete_right
            as the right turning vehicles is less, then 
        """
        sliced_TRs_corss1 = copy.deepcopy(sliced_TRs_corss)
        if delete_right:
            for cycle_TRs_corss in sliced_TRs_corss1:
                for signal_d in cycle_TRs_corss.keys():
                    if cycle_TRs_corss[signal_d].has_key('right'):
                        del cycle_TRs_corss[signal_d]['right']
        N = pd.DataFrame()
        for idx,cycle_TRs in enumerate(sliced_TRs_corss1):
            for signal_d in cycle_TRs.keys():
                for turning_d in cycle_TRs[signal_d].keys():
                    N.loc[idx, signal_d+'_'+turning_d] = len(cycle_TRs[signal_d][turning_d])

        return N

    @classmethod
    def TRs_numbers_without_cross(self, sliced_TRs):
        """
        The difference between self.TRs_numbers_without_cross() and self.TRs_numbers_with_cross() are that:
            - The input of self.TRs_numbers_without_cross() is sliced_TRs[cycle_idx][sliced_signal_turning][vehicle_id] = {'tsxs':..}
            - The input of self.TRs_numbers_with_cross() is sliced_TRs[cycle_idx][sliced_signal_turning][TR_turning][vehicle_id] = {'tsxs':..}
        """
        N = [{} for i in sliced_TRs]
        for idx,cycle_TRs in enumerate(sliced_TRs):
            for slice_signal in cycle_TRs.keys():
                N[idx][slice_signal] = len(sliced_TRs[idx][slice_signal])

        return N

    @classmethod
    def plot_TRs_dict_sppeds(self, TRs_dict, outputpath = 'TMS/figs/', namee = 'Temp.html'):
        """

        """
        colors = self.Get_colors( randomize = False)

        bokeh_output_file(outputpath + namee)
        p = bokeh_figure(title="simple line example", x_axis_label='x', y_axis_label='y')
        for vid in TRs_dict[slice_signal][turning_d].keys():
            ts,xs = TRs_dict[slice_signal][turning_d][vid]['tsxs']
            vs = np.diff(xs)/np.diff(ts)
            p.line(ts,vs,line_color = np.random.choice(colors))
        bokeh_show(p)
        pass

    @classmethod
    def plot_TRs_sliced_using_paired_reds(self, TRs,  outputpath = 'TMS/figs/', namee = 'Temp.html'):
        """
        ---------------------------
        input: sliced_TRs, a list
            sliced_TRs[cycle_idx][turingdirection_signal][turning_direction][vehicle_id].keys() include:



        """

        colors = self.Get_colors( randomize = False)

        bokeh_output_file(outputpath + namee)
        p = bokeh_figure(title="simple line example", x_axis_label='x', y_axis_label='y')
        for cycle_TRs in TRs:
            for slice_signal in cycle_TRs.keys():
                for turning_d in cycle_TRs[slice_signal].keys():
                    for vid in cycle_TRs[slice_signal][turning_d].keys():
                        #print(vid)
                        ts,xs = cycle_TRs[slice_signal][turning_d][vid]['tsxs']
                        t_w,x_w = cycle_TRs[slice_signal][turning_d][vid]['intersectionpoint']
                        p.line(ts,xs,line_color = np.random.choice(colors))
                        p.circle(t_w,x_w, radius = 1)
        bokeh_show(p)

    @classmethod
    def plot_batchopt_res_and_TRs(self, sliced_TRs,kwargs,batch_optimization_results, optimization_variables_sequence = False, plot_directions = ['left', 'through'], outputpath = 'TMS/figs/', namee = 'Temp.html', title="Trajectories", x_axis_label='Time (sec)', y_axis_label='Location (m)', spillovers_paired_reds = False, plot_height=400, plot_width = 400, markers = {'left':'circle_cross','through':'square'}, area_size={'left':10,'through':10}, benchamrksize = {'left':6,'through':10}):
        """
        plot the optimization results and the traejctories. 
        --------------------------------------------
        input: batch_optimization_results
            a list. batch_optimization_results[idx] is a dict, keys include 'left', 'through'. Note that some cycles, there may be only one or even none elements. 
            batch_optimization_results[idx]['left'] is a dict. The keys include:
                ['Y0', 'X0', 'cons', 'optimization_variables_sequence', 'res']
            Y0 is the initial objective, cons are constrainte.
            batch_optimization_results[idx]['res'].x will give the optimimal decision variables. 
        input: sliced_TRs
            a list. The sturcture should be the same as batch_optimization_results.
            sliced_TRs[cycle_idx][turingdirection_signal][vehicle_id].keys() include:

        input: plot_directions
            the directions that need to be ploted. 
        """
        if len(sliced_TRs)!=len(batch_optimization_results):
            raise ValueError("sdsdfsdfsadfag")

        loc_stopline = kwargs['loc_stopline']
        l_tb = kwargs['l_tb']
        w = kwargs['w']
        vf = kwargs['vf']
        direction_flag = kwargs['direction_flag']

        colors = self.Get_colors( randomize = False)

        bokeh_output_file(outputpath + namee)
        ps = {}
        for d in plot_directions:
            ps[d] = bokeh_figure(title=d, x_axis_label= x_axis_label, y_axis_label=y_axis_label,plot_height=plot_height,plot_width=plot_width)

        #get all possible directions, a sorted list. 
        directions0 = []
        for i in batch_optimization_results:directions0.extend(i.keys())
        directions = sorted(set(directions0))

        #plot the TRs
        for idx,cycle_TRs in enumerate(sliced_TRs):
            for turning_d in cycle_TRs.keys():
                if turning_d not in plot_directions:continue
                for vid in cycle_TRs[turning_d].keys():
                    ts,xs = cycle_TRs[turning_d][vid]['tsxs']
                    t_w,x_w = cycle_TRs[turning_d][vid]['intersectionpoint']
                    t0,t1,t2 = cycle_TRs[turning_d][vid]['t0t1t2']

                    #plot trejactory
                    ps[turning_d].line(ts,xs,line_color = np.random.choice(colors))

                    #plot intersection point
                    ps[turning_d].circle(t_w,x_w, radius = 1)

                    ####plot optimization results
                    #      batch_optimization_results[idx]['left'].keys() = ['cons', 'res', 'Y0', 'X0', 'optimization_variables_sequence', 'optimal_Y']
                    #x[0] is the duration. 
                    if not batch_optimization_results[idx].has_key(turning_d):continue
                    
                    X  = batch_optimization_results[idx][turning_d]['res']['x']
                    TMS_end = t1 + l_tb/(w/3.6)
                    TMS_start = TMS_end - X[0]
                    ps[turning_d].rect(x = (TMS_start + TMS_end)/2.0, y = loc_stopline + direction_flag*l_tb, width = X[0], height = 2, color = 'red')

        #plot the benchark and optimized spillovers
        p_spillover = []
        if not isinstance(spillovers_paired_reds, bool):
            p_spillover = [bokeh_figure(title='spillover duration', x_axis_label= 'Cycle index', y_axis_label='Duration (sec)', plot_height=plot_height,plot_width=plot_width)]
            for d in directions:
                #Benchmark duration
                tmp_d = [];Xs= []
                for idx,spillovers_paired_red in enumerate(spillovers_paired_reds):
                    if spillovers_paired_red.has_key(d):
                        tmp_d.append(spillovers_paired_red[d])
                        Xs.append(idx)
                #p_spillover[0].line(range(len(tmp_d)), tmp_d,  line_color = np.random.choice(colors))
                #p_spillover[0].circle(Xs, tmp_d,legend = 'Benchmark '+d, color = np.random.choice(colors), size = benchamrksize[d])
                p_spillover[0].line(Xs, tmp_d)
                p_spillover[0].scatter(Xs, tmp_d, marker=markers[d],legend = 'Benchmark '+ d,size = benchamrksize[d])

                #Theoretical duration
                tmp_d = [];Xs= []
                for idx,batch_optimization_result in enumerate(batch_optimization_results):
                    if batch_optimization_result.has_key(d):
                        tmp_d.append(batch_optimization_result[d]['res'].x[0])
                        Xs.append(idx)
                #p_spillover[0].line(range(len(tmp_d)), tmp_d, legend = 'Optimized '+d, line_color = np.random.choice(colors))
                #p_spillover[0].circle(Xs, tmp_d, color = np.random.choice(colors), size = benchamrksize[d])
                p_spillover[0].line(Xs, tmp_d)
                p_spillover[0].scatter(Xs, tmp_d, marker=markers[d] ,legend = 'Optimized '+d,alpha=0.5, size = benchamrksize[d])
        
        #plot optimal area
        p_optimal = [bokeh_figure(title='Optimal average area', x_axis_label= 'Cycle index', y_axis_label='Area', plot_height=plot_height,plot_width=plot_width)]
        for d in directions:
            tmp_d = [];Xs= []
            for idx,batch_optimization_result in enumerate(batch_optimization_results):
                if batch_optimization_result.has_key(d):
                    tmp_d.append(batch_optimization_result[d]['optimal_Y'])
                    Xs.append(idx)
            p_optimal[0].scatter(Xs, tmp_d,marker=markers[d],legend = 'Optimal area: '+d, color = np.random.choice(colors),size = area_size[d])
            p_optimal[0].line(Xs, tmp_d)

        bokeh_show(bokeh_row(ps.values() + p_spillover + p_optimal))

    @classmethod
    def plot_batch_optimization_results(self, batch_optimization_results, spillover_durations, outputpath = 'TMS/figs/', namee = 'Temp.html', title="simple line example", x_axis_label='x', y_axis_label='y'):
        """
        plot the optimization results. 
        --------------------------------------------
        input: batch_optimization_results
            a list. batch_optimization_results[idx] is a dict, keys include 'left', 'through'. Note that some cycles, there may be only one or even none elements. 
            batch_optimization_results[idx]['left'] is a dict. The keys include:
                ['Y0', 'X0', 'cons', 'optimization_variables_sequence', 'res']
            Y0 is the initial objective, cons are constrainte.
            batch_optimization_results[idx]['res'].x will give the optimimal decision variables. 
        input: spillover_durations
            the struct are the same as batch_optimization_results
        """
        builtins.tmp = []
        colors = self.Get_colors( randomize = False)

        bokeh_output_file(outputpath + namee)
        p = bokeh_figure(title=title, x_axis_label= x_axis_label, y_axis_label=y_axis_label)

        #get all possible directions, a sorted list. 
        directions0 = []
        for i in batch_optimization_results:directions0.extend(i.keys())
        directions = sorted(set(directions0))

        #plot optimal decision variables. 
        #   opt_res[direction] = a list.
        opt_res = {}
        for d in directions:
            opt_res[d] = [np.nan for i in range(len(batch_optimization_results))]
            for index, cycle_opt_res in enumerate(batch_optimization_results):
                if cycle_opt_res.has_key(d):
                    opt_res[d][index] = cycle_opt_res[d]['res']['x'][0]
        for d in directions:
            #print(opt_res[d])
            p.line(range(len(opt_res[d])), opt_res[d])
            #builtins.tmp.append((range(len(opt_res[d])), opt_res[d]))
            p.circle(range(len(opt_res[d])), opt_res[d], size=10, color=np.random.choice(colors), alpha=1,legend = d +' optimization')

        #plot benchmark results. 
        #   opt_res[direction] = a list.
        benchmark = {}
        for d in directions:
            benchmark[d] = [np.nan for i in range(len(spillover_durations))]
            for index, cycle_sd in enumerate(spillover_durations):
                if cycle_opt_res.has_key(d):
                    benchmark[d][index] = cycle_sd[d]
        for d in directions:
            #print(benchmark[d])
            p.line(range(len(benchmark[d])), benchmark[d])
            p.square(range(len(benchmark[d])), benchmark[d], size=10, color=np.random.choice(colors), alpha=1,legend = d +' Benchmark')
        #print(type(p))

        #plot optimal function value, or Y.
        #   which is stored in batch_optimization_results[cycle_idx][direction]['optimal_Y']
        p0 = bokeh_figure(title=title, x_axis_label= x_axis_label, y_axis_label=y_axis_label)
        opt_Y = {}
        for d in directions:
            opt_Y[d] = [np.nan for i in range(len(batch_optimization_results))]
            for index, cycle_opt_res in enumerate(batch_optimization_results):
                if cycle_opt_res.has_key(d):
                    opt_Y[d][index] = cycle_opt_res[d]['optimal_Y']
        for d in directions:
            p0.line(range(len(opt_Y[d])), opt_Y[d], color=np.random.choice(colors))
            #builtins.tmp.append((range(len(opt_res[d])), opt_res[d]))
            p0.diamond(range(len(opt_Y[d])), opt_Y[d], size=10, color=np.random.choice(colors), alpha=1, legend = d +' optimal Area')
        bokeh_show(bokeh_row(p,p0))


    @classmethod
    def plot_TRs_theoretical_TRs(self, TRs0, theoretical_trs0, outputpath = 'TMS/figs/', namee = 'Temp.html'):
        """
        
    
        input: theoretical_trs0
            either a dict (the keys are vehicles ids) or a dict with keys as follows:
                ['FirstSegment', 'SecondSegment', 'RT_bound_w', 'ThirdSegment', 'LT_bound_w']
            In this cast, theoretical_trs0['FirstSegment'] = ()
        """

        colors = self.Get_colors( randomize = False)
        bokeh_output_file(outputpath + namee)
        p = bokeh_figure(title="Trajectories", x_axis_label='x', y_axis_label='y')

        if theoretical_trs0.has_key('FirstSegment'):
            #means the keys are vehicle ids. 
            TRs = {'type_east_left_300.3':TRs0}
            theoretical_trs =  {'type_east_left_300.3':theoretical_trs0}
        else:
            TRs = TRs0
            theoretical_trs = theoretical_trs0
        for vid in TRs.keys():
            line_color = np.random.choice(colors)
            #plot ts,xs
            ts,xs = TRs[vid]['tsxs']
            p.line(ts,xs, line_color=line_color)

            #plot theoretical tr
            #   tr['FirstSegment'] = ((t0,x0), (t1,x1))
            #   theoretical_tr = (ts,xs)
            tr = theoretical_trs[vid]
            theoretical_tr = ((tr['FirstSegment'][0][0], tr['SecondSegment'][0][0], tr['ThirdSegment'][0][0], tr['ThirdSegment'][1][0]), (tr['FirstSegment'][0][1], tr['SecondSegment'][0][1], tr['ThirdSegment'][0][1], tr['ThirdSegment'][1][1]))
            p.line(theoretical_tr[0],theoretical_tr[1], line_color=line_color)

        bokeh_show(p)


    @classmethod
    def plot_TRs(self, TRs_dict, kwargs, X=False, Speed = False, optimization_variables_sequence = False, outputpath = 'TMS/figs/', namee = 'Temp.html', paired_lanes = {'left':['east_c_2', 'east_u_1'], 'through':['east_c_1', 'east_u_0']}, direction_colors = {'left':'blue', 'through':'black','right':'red'}, plot_point_radius = 1, TRs_width=2, plot_seed_directions = ['left','right'], title = 'simple line example', x_axis_label='x', y_axis_label = 'y',plot_width=300, plot_height = 300):
        """
        ---------------------------
        input: sliced_TRs, a list
            sliced_TRs[cycle_idx][turingdirection_signal][vehicle_id].keys() include:
        input: X
            the decision variables. If X==False, means no decision variables
        input: Speed, a dict, the speed arround the interface. 
            a dict. Speed.keys() = ['east_c_0', 'east_c_1', 'east_u_0', 'east_u_1', 'east_c_2']
            Speed['east_c_0'] = (ts,vs)


        """
        colors = self.Get_colors( randomize = False)

        loc_stopline = kwargs['loc_stopline']
        l_tb = kwargs['l_tb']
        w = kwargs['w']
        vf = kwargs['vf']
        direction_flag = kwargs['direction_flag']

        bokeh_output_file(outputpath + namee)
        p = bokeh_figure(title=title, x_axis_label=x_axis_label, y_axis_label=y_axis_label,plot_width=plot_width, plot_height = plot_height)

        #plot the red duration. 
        anyvid = np.random.choice(TRs_dict.keys())
        cyclestartmoment = TRs_dict[anyvid]['t0t1t2'][0]
        cycleendmoment = TRs_dict[anyvid]['t0t1t2'][1]
        redduration = cycleendmoment - cyclestartmoment
        #red signal
        p.rect(x = (cyclestartmoment + cycleendmoment)/2.0, y = loc_stopline, width = redduration, height = 2, color = 'red')
        #   decision variable
        if not isinstance(X,bool):
            #plot tau, or spillover duration
            spillover_end_t = cycleendmoment + (l_tb)/(w/3.6)
            p.rect(x = spillover_end_t-X[0]/2.0, y = loc_stopline + direction_flag*l_tb, width = X[0], height = .5, color = 'red')
            #   start point of vector vstop
            vstop_vector_t0 = spillover_end_t - X[0]
            vstop_vector_x0 = loc_stopline + direction_flag*l_tb
            #for each trajectory, plot the trajectory, vstop

        #speed
        if not isinstance(Speed,bool):
            shifted_end_t = cycleendmoment + (l_tb)/(w/3.6)
            shifted_start_t = cycleendmoment + (l_tb)/(w/3.6) - redduration
            for d in plot_seed_directions:
                for lane in paired_lanes[d]:
                    ts,vs = Speed[lane]
                    idxes = np.where((np.array(ts)>=shifted_start_t) & (np.array(shifted_start_t)<=t1))[0]
                    ts0 = np.array(ts)[idxes]
                    vs0  = np.array(vs)[idxes]
                    p.line(ts0,vs0, line_color = direction_colors[d])

        if isinstance(optimization_variables_sequence,bool):
            optimization_variables_sequence = TRs_dict.keys()
        for i,vid in enumerate(optimization_variables_sequence):
            color = np.random.choice(colors)

            ts,xs = TRs_dict[vid]['tsxs']
            t_w,x_w = TRs_dict[vid]['intersectionpoint']
            #add trajectory
            p.line(ts,xs, color = color, line_width= TRs_width)
            #add intersection point
            p.circle(x=t_w, y=x_w, radius = plot_point_radius)
            
            if isinstance(X,bool):continue
            #keys include 'FirstSegment', 'SecondSegment', 'ThirdSegment'
            #   tr['FirstSegment'] = ((t0,x0),(t1,x1)).
            tr = self.Theoretical_TR(TR_info = TRs_dict[vid], tau=X[0], vstop=X[i+1],boundary=False, kwargs=kwargs)
            #three segments.
            p.line([tr['FirstSegment'][0][0], tr['FirstSegment'][1][0]], [tr['FirstSegment'][0][1], tr['FirstSegment'][1][1]],  color = color, line_dash = 'dashed')
            p.line([tr['SecondSegment'][0][0], tr['SecondSegment'][1][0]], [tr['SecondSegment'][0][1], tr['SecondSegment'][1][1]],  color = color, line_dash = 'dashed')
            p.line([tr['ThirdSegment'][0][0], tr['ThirdSegment'][1][0]], [tr['ThirdSegment'][0][1], tr['ThirdSegment'][1][1]],  color = color, line_dash = 'dashed')

        bokeh_show(p)


    @classmethod
    def plot_several_paired_reds_opt_res(self, TRs_turnings_paired_reds, kwargs, Optimal_paired_reds=False,velocities = False, outputpath = 'TMS/figs/', namee = 'Temp.html', title = 'Speed around the interface', x_axis_label='Time', y_axis_label='Speed (km/h)', line_width = {'left':2, 'through':4}, text_loc = {'left':0, 'through':-10}, text_size = 10,plot_point_radius=2, alpha = .5, paired_lanes = {'left':['east_c_2', 'east_u_1'], 'through':['east_c_1', 'east_u_0']}, velocity_line_width = {'left':1, 'through':2}, velocity_height = 100, buffer_velocity_axis=3, redsignal_location_shift = {'left':2, 'through':-2}, velocity_line_dash = {'left':'solid','through':'dashed'}, plot_height=400, plot_width = 400):
        """
        combine several plots (each one is like the plot generated by self.plot_single_OptRes_paired_red())

        """
        loc_stopline = kwargs['loc_stopline']
        l_tb = kwargs['l_tb']
        w = kwargs['w']
        vf = kwargs['vf']
        direction_flag = kwargs['direction_flag']
        tolerance = kwargs['tolerance']
        method = kwargs['method']
        lu = kwargs['lu']
        spillover_threshold = kwargs['spillover_threshold']

        colors = self.Get_colors( randomize = False)
        velocity_line_color = {}
        for d in paired_lanes.keys():
            velocity_line_color[d] = np.random.choice(colors)

        bokeh_output_file(outputpath + namee)
        ps = []
        for idx in range(len(TRs_turnings_paired_reds)):
            ps.append(bokeh_figure(title='cycle '+str(idx), x_axis_label=x_axis_label, y_axis_label=y_axis_label, plot_height=plot_height, plot_width= plot_width))

        for idx,TRs_turnings_paired_red in enumerate(TRs_turnings_paired_reds):
            for d in TRs_turnings_paired_red.keys():
                anyvid = TRs_turnings_paired_red[d].keys()[0]
                t0,t1,t2 = TRs_turnings_paired_red[d][anyvid]['t0t1t2']
                
                #plot TMS duration in Optimal_paired_red
                if not isinstance(Optimal_paired_reds,bool):
                    Optimal_paired_red = Optimal_paired_reds[idx]
                    if Optimal_paired_red.has_key(d):
                        X = Optimal_paired_red[d]['res']['x']

                        TMS_end = t1 + l_tb/(w/3.6)
                        TMS_start = TMS_end - X[0]
                        ps[idx].rect(x = (TMS_start + TMS_end)/2.0, y = loc_stopline + direction_flag*l_tb, width = X[0], height = 2, color = np.random.choice(colors), alpha = alpha)
                        #
                        tmp = bokeh_Label(x = TMS_end, y = loc_stopline + direction_flag*l_tb+text_loc[d], text = str(round(Optimal_paired_red[d]['optimal_Y'],2)))
                        ps[idx].add_layout(tmp)

                #plot red signal
                ps[idx].rect(x = (t0 + t1)/2.0, y = loc_stopline+redsignal_location_shift[d], width = t1-t0, height = 2, color = 'red', alpha = alpha)
                #plot green
                ps[idx].rect(x = (t1 + t2)/2.0, y = loc_stopline+redsignal_location_shift[d], width = t2-t1, height = 2, color = 'green', alpha = alpha)

                #plot the ts and xs
                for vid in TRs_turnings_paired_red[d].keys():
                    #plot trajectories.
                    ts,xs = TRs_turnings_paired_red[d][vid]['tsxs']
                    t_w,x_w = TRs_turnings_paired_red[d][vid]['intersectionpoint']
                    t0,t1,t2 =  TRs_turnings_paired_red[d][vid]['t0t1t2']
                    ps[idx].line(ts,xs, line_width=line_width[d], legend = d)
                    ps[idx].line([t1, t_w],[loc_stopline, x_w], line_dash='dotted')
                    ps[idx].circle(x=t_w, y=x_w, radius = plot_point_radius)

        bokeh_show(bokeh_row(ps))
        return


        









        #plot the velocities. 
        if not isinstance(velocities, bool):
            
            #paired_lanes_1['east_c_1'] = 'left' means this lane speed is used to judge the spillover of left turn movement. 
            paired_lanes_1 = {}
            for i in paired_lanes.keys():
                for j in paired_lanes[i]:paired_lanes_1[j] = i

            #######################################
            #Get the spatial_temporal domain for velocity plot
            #   plot_min_x, plot_max_x, plot_start_t and plot_end_t are the plot area.
            plot_min_x = loc_stopline + direction_flag*l_tb
            plot_max_x = loc_stopline + direction_flag*l_tb + direction_flag*velocity_height
            delta_x = plot_max_x - plot_min_x
            plot_start_t = np.inf
            plot_end_t = -np.inf
            for d in TRs_turnings_paired_red.keys():
                anyvid = TRs_turnings_paired_red[d].keys()[0]
                t0,t1,t2 = TRs_turnings_paired_red[d][anyvid]['t0t1t2']
                plot_start_t = min([plot_start_t, t0 + l_tb/(w/3.6)])
                plot_end_t = max([plot_end_t, t2 + l_tb/(w/3.6)])
            
            #######################################
            ##############plot the velocity of each lane. 
            for lane in velocities.keys():
                #get partial ts and vs
                if not paired_lanes_1.has_key(lane):continue
                ts,vs = velocities[lane]
                tmp_idx = self.ts_filter_return_idxs(ts, plot_start_t, plot_end_t)
                
                #filter the ts vs for this paired red. 
                partial_ts = np.array(ts)[tmp_idx]
                partial_vs0 = np.array(vs)[tmp_idx]
                tmp_delta_v = max(partial_vs0)-min(partial_vs0)
                #   spillover_lines = [(ts,vs), (ts,vs), (ts,vs)...]
                #   non_spillover_lines = [(ts,vs), (ts,vs), (ts,vs)...]
                spillover_lines, non_spillover_lines = self.tsvs_threshold_lines(ts = partial_ts, vs = partial_vs0, threshold = spillover_threshold)
                #   transforme the y-axis coordinate.
                for i in spillover_lines:
                    i[1] = [direction_flag*buffer_velocity_axis+plot_min_x+delta_x*(tmpv - min(partial_vs0))/(tmp_delta_v)  for tmpv in i[1]]
                for i in non_spillover_lines:
                    i[1] = [direction_flag*buffer_velocity_axis+plot_min_x+delta_x*(tmpv - min(partial_vs0))/(tmp_delta_v)  for tmpv in i[1]]

                #plot the spillover lines and non spillover lanes. 
                for ts,vs in spillover_lines:
                    #paired_lanes_1[lane] is 'left' or 'through'
                    p.line(ts, vs, line_dash = velocity_line_dash[paired_lanes_1[lane]], line_width = velocity_line_width[paired_lanes_1[lane]], legend = 'velocity '+paired_lanes_1[lane], line_color = 'red')
                for ts,vs in non_spillover_lines:
                    #paired_lanes_1[lane] is 'left' or 'through'
                    p.line(ts, vs, line_dash=velocity_line_dash[paired_lanes_1[lane]], line_width=velocity_line_width[paired_lanes_1[lane]], legend = 'velocity '+paired_lanes_1[lane], line_color = 'black')
        bokeh_show(p)
        pass


    @classmethod
    def plot_single_OptRes_paired_red(self, TRs_turnings_paired_red, kwargs, Optimal_paired_red=False,velocities = False, velocities_differenciate = False, outputpath = 'TMS/figs/', namee = 'Temp.html', title = 'Speed around the interface', x_axis_label='Time', y_axis_label='Speed (km/h)', line_width = {'left':3, 'through':8}, text_loc = {'left':0, 'through':-10}, text_size = 10,plot_point_radius=2, alpha = .5, paired_lanes = {'left':['east_c_2', 'east_u_1'], 'through':['east_c_1', 'east_u_0']}, velocity_line_width = {'left':1, 'through':2}, velocity_height = 100, buffer_velocity_axis=3, redsignal_location_shift = {'left':2, 'through':-2}, velocity_line_dash = {'left':'solid','through':'dashed'}, plot_height=400, plot_width = 400, plot_theoretical_tr= True, show_area = False, patch =True):
        """
        plot single cycle of a paired red. 

        ----------------------------------------
        input: TRs_turnings_paired_red
            keys include 'left' and 'through'
            Note that the keys may only include 'left'.
        input: Optimal_paired_red
            Optimal_paired_red['left'] is a dict and keys() include:
                ['cons', 'res', 'Y0', 'X0', 'optimization_variables_sequence', 'optimal_Y']
        input: velocities and paired_lanes
            velocities is a dict. keys include the lanes id
            velocities.keys() will return ['east_c_0', 'east_c_1', 'east_u_0', 'east_u_1', 'east_c_2'].

            paired_lanes is a dict.
        input: velocity_height
            the height of the velocity
        input: theoretical_tr
            whether plot the 
        """
        loc_stopline = kwargs['loc_stopline']
        l_tb = kwargs['l_tb']
        w = kwargs['w']
        vf = kwargs['vf']
        direction_flag = kwargs['direction_flag']
        tolerance = kwargs['tolerance']
        method = kwargs['method']
        lu = kwargs['lu']
        spillover_threshold = kwargs['spillover_threshold']

        colors = self.Get_colors( randomize = False)
        velocity_line_color = {}
        for d in paired_lanes.keys():
            velocity_line_color[d] = np.random.choice(colors)

        bokeh_output_file(outputpath + namee)
        p = bokeh_figure(title=title, x_axis_label=x_axis_label, y_axis_label=y_axis_label, plot_height=plot_height, plot_width= plot_width)

        for d in TRs_turnings_paired_red.keys():
            if 'left' in TRs_turnings_paired_red[d].keys():
                TRs_turnings = TRs_turnings_paired_red[d][d]
            else:
                TRs_turnings = TRs_turnings_paired_red[d]
            #find the three moment. 
            anyvid = TRs_turnings.keys()[0]
            t0,t1,t2 = TRs_turnings[anyvid]['t0t1t2']
            
            #plot TMS duration in Optimal_paired_red
            if not isinstance(Optimal_paired_red,bool):
                if Optimal_paired_red.has_key(d):
                    X = Optimal_paired_red[d]['res']['x']

                    #plot TMS
                    TMS_end = t1 + l_tb/(w/3.6)
                    TMS_start = TMS_end - X[0]
                    p.rect(x = (TMS_start + TMS_end)/2.0, y = loc_stopline + direction_flag*l_tb, width = X[0], height = 2, color = np.random.choice(colors), alpha = alpha)
                    #
                    tmp = bokeh_Label(x = TMS_end, y = loc_stopline + direction_flag*l_tb+text_loc[d], text = str(round(Optimal_paired_red[d]['optimal_Y'],2)))
                    if show_area:
                        p.add_layout(tmp)

            #plot red signal
            p.rect(x = (t0 + t1)/2.0, y = loc_stopline+redsignal_location_shift[d], width = t1-t0, height = 2, color = 'red', alpha = alpha)
            #plot green
            p.rect(x = (t1 + t2)/2.0, y = loc_stopline+redsignal_location_shift[d], width = t2-t1, height = 2, color = 'green', alpha = alpha)

            #plot the ts and xs
            for vid in TRs_turnings.keys():
                #plot trajectories.
                ts,xs = TRs_turnings[vid]['tsxs']
                t_w,x_w = TRs_turnings[vid]['intersectionpoint']
                p.line(ts,xs, line_width=line_width[d], legend = d)
                p.circle(x=t_w, y=x_w, radius = plot_point_radius)

                #plot the theoretical tr
                if isinstance(Optimal_paired_red,bool):continue
                if plot_theoretical_tr:
                    #theoretical_tr = 
                    #print Optimal_paired_red[d]['optimization_variables_sequence'], TRs_turnings.keys()
                    idx_temp = Optimal_paired_red[d]['optimization_variables_sequence'].index(vid)
                    theoretical_tr = self.Theoretical_TR(TR_info = TRs_turnings[vid], tau = Optimal_paired_red[d]['res']['x'][0], \
                        vstop = Optimal_paired_red[d]['res']['x'][idx_temp+1], kwargs = kwargs)

                    #
                    ts = (theoretical_tr['FirstSegment'][0][0], theoretical_tr['FirstSegment'][1][0], theoretical_tr['SecondSegment'][1][0], theoretical_tr['ThirdSegment'][1][0])
                    xs = (theoretical_tr['FirstSegment'][0][1], theoretical_tr['FirstSegment'][1][1], theoretical_tr['SecondSegment'][1][1], theoretical_tr['ThirdSegment'][1][1])
                    p.line(ts,xs, line_dash = 'dashed')
                    #p.patch(ts,xs, fill_color = np.random.choice(colors), alpha = .2)

        #plot the velocities. 
        if not isinstance(velocities, bool):
            #paired_lanes_1['east_c_1'] = 'left' means this lane speed is used to judge the spillover of left turn movement. 
            paired_lanes_1 = {}
            for i in paired_lanes.keys():
                for j in paired_lanes[i]:paired_lanes_1[j] = i

            #######################################
            #Get the spatial_temporal domain for velocity plot
            #   plot_min_x, plot_max_x, plot_start_t and plot_end_t are the plot area.
            plot_min_x = loc_stopline + direction_flag*l_tb
            plot_max_x = loc_stopline + direction_flag*l_tb + direction_flag*velocity_height
            delta_x = plot_max_x - plot_min_x
            plot_start_t = np.inf
            plot_end_t = -np.inf
            for d in TRs_turnings_paired_red.keys():
                if 'left' in TRs_turnings_paired_red[d].keys():
                    TRs_turnings = TRs_turnings_paired_red[d][d]
                else:
                    TRs_turnings = TRs_turnings_paired_red[d]
                
                anyvid = TRs_turnings.keys()[0]
                t0,t1,t2 = TRs_turnings[anyvid]['t0t1t2']
                plot_start_t = min([plot_start_t, t0 + l_tb/(w/3.6)])
                plot_end_t = max([plot_end_t, t2 + l_tb/(w/3.6)])

            if velocities_differenciate:
                #######################################
                ##############plot the velocity of each lane. 
                for lane in velocities.keys():
                    #get partial ts and vs
                    if not paired_lanes_1.has_key(lane):continue
                    ts,vs = velocities[lane]
                    tmp_idx = self.ts_filter_return_idxs(ts, plot_start_t, plot_end_t)
                    if len(tmp_idx)==0:continue
                    #filter the ts vs for this paired red. 
                    partial_ts = np.array(ts)[tmp_idx]
                    partial_vs0 = np.array(vs)[tmp_idx]

                    tmp_delta_v = max(partial_vs0)-min(partial_vs0)
                    #   spillover_lines = [(ts,vs), (ts,vs), (ts,vs)...]
                    #   non_spillover_lines = [(ts,vs), (ts,vs), (ts,vs)...]
                    spillover_lines, non_spillover_lines = self.tsvs_threshold_lines(ts = partial_ts, vs = partial_vs0, threshold = spillover_threshold)
                    #   transforme the y-axis coordinate.
                    for i in spillover_lines:
                        i[1] = [direction_flag*buffer_velocity_axis+plot_min_x+delta_x*(tmpv - min(partial_vs0))/(tmp_delta_v)  for tmpv in i[1]]
                    for i in non_spillover_lines:
                        i[1] = [direction_flag*buffer_velocity_axis+plot_min_x+delta_x*(tmpv - min(partial_vs0))/(tmp_delta_v)  for tmpv in i[1]]

                    #plot the spillover lines and non spillover lanes. 
                    for ts,vs in spillover_lines:
                        #paired_lanes_1[lane] is 'left' or 'through'
                        p.line(ts, vs, line_dash = velocity_line_dash[paired_lanes_1[lane]], line_width = velocity_line_width[paired_lanes_1[lane]], legend = 'velocity '+paired_lanes_1[lane], line_color = 'red')
                    for ts,vs in non_spillover_lines:
                        #paired_lanes_1[lane] is 'left' or 'through'
                        p.line(ts, vs, line_dash=velocity_line_dash[paired_lanes_1[lane]], line_width=velocity_line_width[paired_lanes_1[lane]], legend = 'velocity '+paired_lanes_1[lane], line_color = 'black')
            else:
                for lane in velocities.keys():
                    #get partial ts and vs
                    if not paired_lanes_1.has_key(lane):continue
                    ts,vs = velocities[lane]
                    tmp_idx = self.ts_filter_return_idxs(ts, plot_start_t, plot_end_t)
                    if len(tmp_idx)==0:continue
                    #print(tmp_idx, plot_start_t, plot_end_t)
                    #filter the ts vs for this paired red. 
                    partial_ts = np.array(ts)[tmp_idx]
                    partial_vs0 = np.array(vs)[tmp_idx]
                    tmp_delta_v = max(partial_vs0)-min(partial_vs0)
                    partial_vs = [direction_flag*buffer_velocity_axis+plot_min_x+delta_x*(tmpv - min(partial_vs0))/(tmp_delta_v)  for tmpv in partial_vs0]

                    p.line(partial_ts, partial_vs, line_width=velocity_line_width[paired_lanes_1[lane]], legend = 'velocity '+paired_lanes_1[lane], line_color = 'black')

                pass
        bokeh_show(p)

    @classmethod
    def plot_single_OptRes_paired_red_corss(self, TRs_turnings_paired_red_corss, kwargs, Optimal_paired_red_corss=False,velocities = False, velocities_differenciate = False, outputpath = 'TMS/figs/', namee = 'Temp.html', title = 'Speed around the interface', x_axis_label='Time', y_axis_label='Speed (km/h)', line_width = {'left':3, 'through':8}, text_loc = {'left':0, 'through':-10}, text_size = 10,plot_point_radius=2, alpha = .5, paired_lanes = {'left':['east_c_2', 'east_u_1'], 'through':['east_c_1', 'east_u_0']}, velocity_line_width = {'left':1, 'through':2}, velocity_height = 100, buffer_velocity_axis=3, redsignal_location_shift = {'left':2, 'through':-2}, velocity_line_dash = {'left':'solid','through':'dashed'}, plot_height=400, plot_width = 400, plot_theoretical_tr= True, show_area = False, patch =True):
        """
        plot single cycle of a paired red. 

        ----------------------------------------
        input: TRs_turnings_paired_red_corss
            keys include 'left' and 'through'. TRs_turnings_paired_red_corss['left']['through']
            Note that the keys may only include 'left'.
        input: Optimal_paired_red_corss
            Optimal_paired_red['left']['left'] is a dict and keys() include:
                ['cons', 'res', 'Y0', 'X0', 'optimization_variables_sequence', 'optimal_Y']
        input: velocities and paired_lanes
            velocities is a dict. keys include the lanes id
            velocities.keys() will return ['east_c_0', 'east_c_1', 'east_u_0', 'east_u_1', 'east_c_2'].

            paired_lanes is a dict.
        input: velocity_height
            the height of the velocity
        input: theoretical_tr
            whether plot the 
        """
        loc_stopline = kwargs['loc_stopline']
        l_tb = kwargs['l_tb']
        w = kwargs['w']
        vf = kwargs['vf']
        direction_flag = kwargs['direction_flag']
        tolerance = kwargs['tolerance']
        method = kwargs['method']
        lu = kwargs['lu']
        spillover_threshold = kwargs['spillover_threshold']

        colors = self.Get_colors( randomize = False)
        velocity_line_color = {}
        for d in paired_lanes.keys():
            velocity_line_color[d] = np.random.choice(colors)

        bokeh_output_file(outputpath + namee)
        p = bokeh_figure(title=title, x_axis_label=x_axis_label, y_axis_label=y_axis_label, plot_height=plot_height, plot_width= plot_width)

        for d in TRs_turnings_paired_red_corss.keys():
            #find the three moment. 
            if len(TRs_turnings_paired_red_corss[d][d].keys())==0:continue
            anyvid = TRs_turnings_paired_red_corss[d][d].keys()[0]
            t0,t1,t2 = TRs_turnings_paired_red_corss[d][d][anyvid]['t0t1t2']
            
            #plot TMS duration in Optimal_paired_red
            if not isinstance(Optimal_paired_red_corss,bool):
                if Optimal_paired_red_corss.has_key(d):
                    if not np.isnan(Optimal_paired_red_corss[d][d]['res']['x'][0]):
                        X = Optimal_paired_red_corss[d][d]['res']['x']

                        #plot TMS
                        TMS_end = t1 + l_tb/(w/3.6)
                        TMS_start = TMS_end - X[0]
                        p.rect(x = (TMS_start + TMS_end)/2.0, y = loc_stopline + direction_flag*l_tb, width = X[0], height = 2, color = np.random.choice(colors), alpha = alpha)
                        #
                        if show_area:
                            tmp = bokeh_Label(x = TMS_end, y = loc_stopline + direction_flag*l_tb+text_loc[d], text = str(Optimal_paired_red_corss[d][d]['optimal_Y']))
                            p.add_layout(tmp)

            #plot red signal
            #print 'red signal',(t0 + t1)/2.0, loc_stopline+redsignal_location_shift[d]
            p.rect(x = (t0 + t1)/2.0, y = loc_stopline+redsignal_location_shift[d], width = t1-t0, height = 2, color = 'red', alpha = alpha)
            #plot green
            p.rect(x = (t1 + t2)/2.0, y = loc_stopline+redsignal_location_shift[d], width = t2-t1, height = 2, color = 'green', alpha = alpha)

            #plot the ts and xs
            for vid in TRs_turnings_paired_red_corss[d][d].keys():
                #plot trajectories.
                ts,xs = TRs_turnings_paired_red_corss[d][d][vid]['tsxs']
                t_w,x_w = TRs_turnings_paired_red_corss[d][d][vid]['intersectionpoint']
                p.line(ts,xs, line_width=line_width[d], legend = d)
                p.circle(x=t_w, y=x_w, radius = plot_point_radius)

                #plot the theoretical tr
                if isinstance(Optimal_paired_red_corss,bool):continue
                if plot_theoretical_tr:
                    #theoretical_tr = 
                    if not vid in Optimal_paired_red_corss[d][d]['optimization_variables_sequence']:
                        continue
                    idx_temp = Optimal_paired_red_corss[d][d]['optimization_variables_sequence'].index(vid)
                    theoretical_tr = self.Theoretical_TR(TR_info = TRs_turnings_paired_red_corss[d][d][vid], tau = Optimal_paired_red_corss[d][d]['res']['x'][0], \
                        vstop = Optimal_paired_red_corss[d][d]['res']['x'][idx_temp+1], kwargs = kwargs)
                    ts = (theoretical_tr['FirstSegment'][0][0], theoretical_tr['FirstSegment'][1][0], theoretical_tr['SecondSegment'][1][0], theoretical_tr['ThirdSegment'][1][0])
                    xs = (theoretical_tr['FirstSegment'][0][1], theoretical_tr['FirstSegment'][1][1], theoretical_tr['SecondSegment'][1][1], theoretical_tr['ThirdSegment'][1][1])
                    p.line(ts,xs, line_dash = 'dashed')
                    #p.patch(ts,xs, fill_color = np.random.choice(colors), alpha = .2)

        #plot the velocities. 
        if not isinstance(velocities, bool):
            #paired_lanes_1['east_c_1'] = 'left' means this lane speed is used to judge the spillover of left turn movement. 
            paired_lanes_1 = {}
            for i in paired_lanes.keys():
                for j in paired_lanes[i]:paired_lanes_1[j] = i

            #######################################
            #Get the spatial_temporal domain for velocity plot
            #   plot_min_x, plot_max_x, plot_start_t and plot_end_t are the plot area.
            plot_min_x = loc_stopline + direction_flag*l_tb
            plot_max_x = loc_stopline + direction_flag*l_tb + direction_flag*velocity_height
            delta_x = plot_max_x - plot_min_x
            plot_start_t = np.inf
            plot_end_t = -np.inf
            for d in TRs_turnings_paired_red_corss.keys():
                anyvid = TRs_turnings_paired_red_corss[d][d].keys()[0]
                t0,t1,t2 = TRs_turnings_paired_red_corss[d][d][anyvid]['t0t1t2']
                plot_start_t = min([plot_start_t, t0 + l_tb/(w/3.6)])
                plot_end_t = max([plot_end_t, t2 + l_tb/(w/3.6)])

            if velocities_differenciate:
                #######################################
                ##############plot the velocity of each lane. 
                for lane in velocities.keys():
                    #get partial ts and vs
                    if not paired_lanes_1.has_key(lane):continue
                    ts,vs = velocities[lane]
                    tmp_idx = self.ts_filter_return_idxs(ts, plot_start_t, plot_end_t)
                    if len(tmp_idx)==0:continue
                    #filter the ts vs for this paired red. 
                    partial_ts = np.array(ts)[tmp_idx]
                    partial_vs0 = np.array(vs)[tmp_idx]

                    tmp_delta_v = max(partial_vs0)-min(partial_vs0)
                    #   spillover_lines = [(ts,vs), (ts,vs), (ts,vs)...]
                    #   non_spillover_lines = [(ts,vs), (ts,vs), (ts,vs)...]
                    spillover_lines, non_spillover_lines = self.tsvs_threshold_lines(ts = partial_ts, vs = partial_vs0, threshold = spillover_threshold)
                    #   transforme the y-axis coordinate.
                    for i in spillover_lines:
                        i[1] = [direction_flag*buffer_velocity_axis+plot_min_x+delta_x*(tmpv - min(partial_vs0))/(tmp_delta_v)  for tmpv in i[1]]
                    for i in non_spillover_lines:
                        i[1] = [direction_flag*buffer_velocity_axis+plot_min_x+delta_x*(tmpv - min(partial_vs0))/(tmp_delta_v)  for tmpv in i[1]]

                    #plot the spillover lines and non spillover lanes. 
                    for ts,vs in spillover_lines:
                        #paired_lanes_1[lane] is 'left' or 'through'
                        p.line(ts, vs, line_dash = velocity_line_dash[paired_lanes_1[lane]], line_width = velocity_line_width[paired_lanes_1[lane]], legend = 'velocity '+paired_lanes_1[lane], line_color = 'red')
                    for ts,vs in non_spillover_lines:
                        #paired_lanes_1[lane] is 'left' or 'through'
                        p.line(ts, vs, line_dash=velocity_line_dash[paired_lanes_1[lane]], line_width=velocity_line_width[paired_lanes_1[lane]], legend = 'velocity '+paired_lanes_1[lane], line_color = 'black')
            else:
                for lane in velocities.keys():
                    #get partial ts and vs
                    if not paired_lanes_1.has_key(lane):continue
                    ts,vs = velocities[lane]
                    tmp_idx = self.ts_filter_return_idxs(ts, plot_start_t, plot_end_t)
                    if len(tmp_idx)==0:continue
                    #print(tmp_idx, plot_start_t, plot_end_t)
                    #filter the ts vs for this paired red. 
                    partial_ts = np.array(ts)[tmp_idx]
                    partial_vs0 = np.array(vs)[tmp_idx]
                    tmp_delta_v = max(partial_vs0)-min(partial_vs0)
                    partial_vs = [direction_flag*buffer_velocity_axis+plot_min_x+delta_x*(tmpv - min(partial_vs0))/(tmp_delta_v)  for tmpv in partial_vs0]

                    p.line(partial_ts, partial_vs, line_width=velocity_line_width[paired_lanes_1[lane]], legend = 'velocity '+paired_lanes_1[lane], line_color = 'black')

                pass
        bokeh_show(p)

    @classmethod
    def ts_filter_return_idxs(self, ts,t0,t1):
        """
        filter the moment within [t0,t1].

        """
        if t1<t0:raise ValueError("sdfsdfsdf")


        return np.where((np.array(ts)>=t0) & (np.array(ts)<=t1))[0]

    paired_lanes = {'left':['east_c_2', 'east_u_1'], 'through':['east_c_1', 'east_u_0']}
    @classmethod
    def plot_velocities(self, velocities,kwargs,paired_lanes=paired_lanes, outputpath = 'TMS/figs/', namee = 'Temp.html', title = 'Speed around the interface', x_axis_label='Time', y_axis_label='Speed (km/h)', plot_width=600, plot_height = 600, direction_colors = {'left':'blue', 'through':'black','right':'red'}):
        """

        """
        loc_stopline = kwargs['loc_stopline']
        l_tb = kwargs['l_tb']
        w = kwargs['w']
        vf = kwargs['vf']
        direction_flag = kwargs['direction_flag']
        tolerance = kwargs['tolerance']
        method = kwargs['method']
        lu = kwargs['lu']

        colors = self.Get_colors( randomize = False)
        bokeh_output_file(outputpath + namee)
        p = bokeh_figure(title=title, x_axis_label=x_axis_label, y_axis_label=y_axis_label)

        #paired_lanes_1[lan_label] = 'left'
        paired_lanes_1 = {}
        for i in paired_lanes.keys():
            for j in paired_lanes[i]:paired_lanes_1[j] = i

        for lane in velocities.keys():
            if lane not in paired_lanes_1.keys():continue
            ts,vs = velocities[lane]
            p.line(ts, vs, legend = paired_lanes_1[lane], line_color = direction_colors[paired_lanes_1[lane]])

        bokeh_show(p)
        #bokeh_export_png(p, filename= outputpath+ namee+'.png')



    paired_lanes = {'left':['east_c_2', 'east_u_1'], 'through':['east_c_1', 'east_u_0']}
    @classmethod
    def plot_velocities_with_spilloverduration(self, velocities_dict, kwargs, spillovers_paired_reds = False, paired_reds = False, paired_lanes = paired_lanes, outputpath = 'TMS/figs/', namee = 'Temp.html', title = 'Speed around the interface', x_axis_label='Time', y_axis_label='Speed (km/h)'):
        """
        -----------------------------------------------
        input: velocities_dict
            a dict, keys are the lane ids. 
            Include: ['east_c_0', 'east_c_1', 'east_u_0', 'east_u_1', 'east_c_2']
        input: paired_reds
            a list. Each element corresponding to a cycle. paired_reds[idx] = {'left':()}
        input: spillovers_paired_reds
            the structure of spillovers_paired_reds is the same as paired_reds.
        input: paired_lanes
            {'left':['east_c_2', 'east_u_1'], 'through':['east_c_1', 'east_u_0']}
        input: kwargs
        """
        loc_stopline = kwargs['loc_stopline']
        l_tb = kwargs['l_tb']
        w = kwargs['w']
        vf = kwargs['vf']
        direction_flag = kwargs['direction_flag']
        tolerance = kwargs['tolerance']
        method = kwargs['method']
        lu = kwargs['lu']

        colors = self.Get_colors( randomize = False)
        bokeh_output_file(outputpath + namee)
        p = bokeh_figure(title=title, x_axis_label=x_axis_label, y_axis_label=y_axis_label)

        for lane in velocities_dict.keys():
            ts,vs = velocities_dict[lane]
            p.line(ts,vs, line_color = np.random.choice(colors), legend = lane)

        if not isinstance(paired_reds, bool):
            for spillovers_paired_red, paired_red in zip(spillovers_paired_reds, paired_reds):
                for d in paired_red.keys():
                    t0,t1,t2 = paired_red[d]
                    xs = [t0 + l_tb/(w/3.6), t1 + l_tb/(w/3.6), t1 + l_tb/(w/3.6), t0 + l_tb/(w/3.6)]
                    ys = [-3, -3, vf, vf]
                    #print(xs,ys)
                    p.patch(xs,ys, fill_color = np.random.choice(colors), alpha = .2)
                    #plot paralel

        #p.x_range = bokeh_Range1d(0,1000)
        #p.y_range = bokeh_Range1d(loc_stopline,loc_stopline+direction_flag*l_tb)

        bokeh_show(p)
        #bokeh_export_png(p, filename= outputpath+ namee+'.png')


    @classmethod
    def plot_areas_TMS_types(self, areas_TMS_types, outputpath = 'TMS/figs/', namee = 'Temp.html', title = 'Speed around the interface', x_axis_label='Time', y_axis_label='Speed (km/h)', plot_height=500, plot_width = 500, markers = ['circle', 'square', 'triangle', 'circle_x', 'square_x'], marker_size = 4):
        """

        ---------------------------------------------------------
        input: areas_TMS_types
            a dict. 
            areas_TMS_types['np_spillovers'] is a list, containing the areas.
            areas_TMS_types['spillover_self_area'] is a list, containing the areas.
            areas_TMS_types['spillover_other_area'] is a list, containing the areas.
            areas_TMS_types['twospillovers'] is a list, containing the areas.
        """
        colors = self.Get_colors( randomize = False)
        bokeh_output_file(outputpath + namee)
        p = bokeh_figure(title=title, x_axis_label=x_axis_label, y_axis_label=y_axis_label, plot_height=plot_height, plot_width = plot_width)
        for idx,typee in enumerate(areas_TMS_types.keys()):
            if len(areas_TMS_types[typee])==0:continue
            xs = range(1, len(areas_TMS_types[typee])+1)
            p.line(xs, areas_TMS_types[typee])
            p.scatter(xs, areas_TMS_types[typee], marker = markers[idx],legend = typee, color = np.random.choice(colors) , size = marker_size)

        bokeh_show(p)

    @classmethod
    def plot_sliced_TRs_with_pairedreds(self, sliced_TRs, paired_reds, outputpath = 'TMS/figs/', namee = 'Temp.html'):
        """
        ---------------------------
        input: paired_reds

        input: sliced_TRs
            a list. 

        """

        colors = self.Get_colors( randomize = False)
        bokeh_output_file(outputpath + namee)
        p = bokeh_figure(title="simple line example", x_axis_label='x', y_axis_label='y')
        for TRs_dict in sliced_TRs:
            for td in TRs_dict.keys():
                for vid in TRs_dict[td].keys():
                    ts,xs = TRs_dict[td][vid]['tsxs']
                    
                    p.line(ts,xs,line_color = np.random.choice(colors))

        bokeh_show(p)

    @classmethod
    def plot_single_cycleslicedata(self, cycle_sliceddata, colors = {'l':'k','t':'r','r':'b'}):
        """
        plot the sliced trajectory data for single cycle. 

        -----------------------------------------------
        input: cycle_sliceddata
            A DICT.
            cycle_sliceddata.keys()= ['l','t','r']
            cycle_sliceddata['l'][vehicle_id1] = {'tsxs':(ts,xs),'intersectionpoint':(t,x)}
        """
        fig,ax = plt.subplots()
        for turningdirection in cycle_sliceddata.keys():
            for v_id in cycle_sliceddata[turningdirection].keys():
                #print cycle_sliceddata[turningdirection][v_id].keys()
                ts,xs = cycle_sliceddata[turningdirection][v_id]['tsxs']
                point_t,point_x = cycle_sliceddata[turningdirection][v_id]['intersectionpoint']
                #print type(ax)
                ax.plot(ts,xs,color = colors[turningdirection])
                ax.plot([point_t],[point_x],'o')
        plt.show()
        return ax

        pass
    @classmethod
    def plot_cycleslicedata_tau_phi(self, cycle_sliceddata, X, optimization_variables_sequence, loc_stopline, l_tb,redduration=100, w = FD.w, vf = FD.vf, lw_trajectory = 2):
        """
        ---------------------------------------------------
        input: X
            the decision variable. 
            len(X) = 1 + N. first '1' means spillover duration; N is the number of trajectories. the sequence is given by optimization_variables_sequence.
        input: optimization_variables_sequence
            indicate the sequence of the decision variable. 
            optimization_variables_sequence[i] = vehicle_id
            It means that i-th+1 variable in X is for vehicle_id.
            Plus 1 is because that the X[0] is the spillover duration. 
        input: cycle_sliceddata, a dict
            the trajectory info for each cycle of the all turning. keys are 'l','t' and 'r', meaning three turning directions respectively. 

            cycle_sliceddata['l'][vehicle_id]['tsxs']  =(ts,xs)
            cycle_sliceddata['l'][vehicle_id]['intersectionpoint'] = (t_w,x_w)
            cycle_sliceddata['l'][vehicle_id]['twoboudnarys'] = ((t0,x0),(t2,x2)), they are on the trajectory.
            cycle_sliceddata['l'][vehicle_id]['redduration'] = r
            cycle_sliceddata['l'][vehicle_id]['cyclestartmoment'] = cyclestartmoment, the moment at stop line correspond to (t0,x0).
        """
        #the spilloverr duration. unit is second. 
        tau = X[0]
        #firstly plot the trajectory
        fig,ax = plt.subplots()
        for turningdirection in cycle_sliceddata.keys():
            for v_id in cycle_sliceddata[turningdirection].keys():
                ts,xs = cycle_sliceddata[turningdirection][v_id]['tsxs']
                point_t,point_x = cycle_sliceddata[turningdirection][v_id]['intersectionpoint']
                #print type(ax)
                ax.plot(ts,xs,lw = lw_trajectory)
                ax.plot([point_t],[point_x],'o')
        
        #then plot the expected trajectory
        for turningdirection in cycle_sliceddata.keys():
            for v_id in cycle_sliceddata[turningdirection].keys():
                t_w,x_w =  cycle_sliceddata[turningdirection][v_id]['intersectionpoint']
                #boundary points in t0 x0 and t2 x2.
                P0,P2 = cycle_sliceddata[turningdirection][v_id]['twoboudnarys']
                t0,x0 = P0;t2,x2 = P2;
                #the phi and the stopping wave speed, v_stop
                idxx = optimization_variables_sequence.index(v_id)+1
                phi = X[idxx]
                v_stop = (x_w - loc_stopline- l_tb)/phi#unit is meters/s.
                #fitst switch point at the trajectory
                t_M = t_w - (tau+(x_w -loc_stopline-l_tb)/(w/3.6) - (x_w-loc_stopline-l_tb)/v_stop)
                x_M = x_w
                #the horizon part
                ax.plot([t_M, t_w], [x_M, x_w])

                if t_w-t_M>redduration:
                    print t_w,t_M,t_w-t_M,redduration
                    raise ValueError('Thedfsdf')
                #the part before joining the queue
                #   x = vf*(t-t_M)+x_w
                #   t = (x -x_w)/(vf/3.6)+t_M
                t_LW = (vf/3.6*t_M+x_M + w/3.6*t0-x0)/(w/3.6+vf/3.6)
                x_LW = w/3.6*(t_LW-t0)+x0
                #t_LW = (x0 -x_w)/(-vf/3.6)+t_M
                #x_LW = x0
                ax.plot([t_LW, t_M], [x_LW, x_M])

                #the part after joinin the queue.
                t_UP = (vf/3.6*t_w + x_w + w/3.6*t2-x2)/(w/3.6+vf/3.6)
                x_UP = w/3.6*(t_UP-t2)+x2
                #x_UP = (-vf/3.6)*(t2-t_w)+x_w
                #t_UP = t2
                ax.plot([t_UP, t_w], [x_UP, x_w])
        plt.show()
        return ax
                



    @classmethod
    def plot_cycleslicedata(self, cycle_sliceddata,  colors = {'l':'b','t':'r', 'r':'k'}, ax =None):
        """
        --------------------------------------------
        input: cycle_sliceddata
            A DICT. keys are the cycle index. 
            cycle_sliceddata[cycle_idx].keys()= ['l','t','r']
            cycle_sliceddata[cycle_idx]['l'][vehicle_id1] = {'tsxs':(ts,xs),'intersectionpoint':(t,x)}
        input: colors
            the color of the trajectories of each turning directions. 
        """
        if ax==None:
            fig,ax = plt.subplots()

        for cycle_idx in cycle_sliceddata.keys():
            for turningdirection in cycle_sliceddata[cycle_idx].keys():
                for v_id in cycle_sliceddata[cycle_idx][turningdirection].keys():
                    ts,xs = cycle_sliceddata[cycle_idx][turningdirection][v_id]['tsxs']
                    point_t,point_x = cycle_sliceddata[cycle_idx][turningdirection][v_id]['intersectionpoint']
                    #print type(ax)
                    ax.plot(ts,xs, color = colors[turningdirection])
                    ax.plot([point_t],[point_x],'o')
        #plt.show()
        return ax

    @classmethod
    def Get_inputdata_oneapproach(reds_turning, trajectory_turning_tsxs, w = 20):
        """
        -------------------------------
        input: reds_turning
            dict, keys include 'left','through' and 'right'.
            reds_turning['left'] is a list. 
                reds_turning['left'][i] = pyinterval
                reds_turning['left'][i].lower_value is the red onset moment
                reds_turning['left'][i].upper_value is the red termination moment.
        input: trajectory_turning_tsxs
            dict, keys include 'l', 't' and 'r', corresponding to left, through and right respectively. 
            trajectory_turning_tsxs['l'][vehicle_id] = (ts,xs)
        input: w, unit is km/h.
            the backward wave speed. 
        output: cycle_sliceddata
            A DICT. keys are the cycle index. 
            cycle_sliceddata[cycle_idx] = {'vehicle_id1':(ts,xs),'vehicle_id2':(ts,xs)...}
        ------------------------------------
        Steps:
            - 

        """
        w = w/3.6
        cycle_sliceddata = {}
        

        pass

    @classmethod
    def tsxs_cycleslice(self, ts, xs, t0, t1, loc_stopline=0,w = 20):
        """
        OK. 
        Slice the trajectory based on wave boundary. 
        t0 and t1  are two moments:
            rgp start, rgp end
        ts and xs are list. 
        w is the backward wave speed.
        If outof boundary, return [],[]
        If only part are within, then return partial.  
        ---------------------------------------------------
        input: ts xs
            len(ts)=len(xs)
            both are lists.
        input: 
            t0 t1 
            all are float
        input: loc_stopline
            the location of the stopline. 
        input: w, unit is km/h
            backward wave speed
        output: partial_ts,partial_xs, completeORnot
            partial_ts and partial_xs are list. 
            completeORnot is a bool, indicating whether the trajectory is complete within the cycle horizon.
        -------------------------------------------------
        Steps:
        """
        if t1<=t0:raise ValueError('t1 must be greater than t0')

        #check the ts is increasing.
        if not np.all(np.diff(ts) > 0):
            builtins.tmp = ts
            raise ValueError("The input ts must be increasing!!!ts is stored in builtins.tmp")

        w = w/3.6
        #the corresponding moment of ts at the stopline
        ts_stopline = [t-abs(x-loc_stopline)/w for t,x in zip(ts,xs)]

        #find the moment t that t0<ts_stopline[idxx]<t1
        #   if out of the horizon, then return null
        if ts_stopline[-1]<=t0 or ts_stopline[0]>=t1:
            return [],[],False

        #check the completeORnot
        if ts_stopline[0]<=t0 and ts_stopline[-1]>=t1:
            completeORnot = True
        else:
            completeORnot = False
        
        idx_t0 = np.where(np.array(ts_stopline) >= t0)[0][0]
        idx_t1 = np.where(np.array(ts_stopline) <= t1)[0][-1]
        #print idx_t0,idx_t1
        return ts[idx_t0:idx_t1+1],xs[idx_t0:idx_t1+1],completeORnot

    @classmethod
    def tsxsIntersection_check(self, ts, xs, t0, loc_stopline=0, w=20, direction_flag = 1):
        """
        just check whether the trajectory (in the format of ts and xs) intersect with the line with slope w or not. 
        If intersect, return True, else return False
        ---------------------------------------
        input: ts and xs
            the trajectory of the vehicle. 
            Both are list. ts must increasing. 
            len(xs)=len(ts)
        input: w    
            the slope of the linear line. 
        input: t0
            the intersection point of the linear line with temporal axis. 
        input: loc_stopline
            the location of the stop line. 
        input: direction_flag
            either 1 or -1. it is 1 when distance increase to upstream from stopline. 
            -1 otherwise.
        ---------------------------------------
        Steps: 
            - For each t and x, find the moment at the stop line
            - if 
        """
        #check the ts is increasing.
        if not np.all(np.diff(ts) > 0):
            builtins.tmp = ts
            raise ValueError("The input ts must be increasing!!!ts is stored in builtins.tmp")
        #convert the unit of w into m/s.
        w = w/3.6

        #find the intersection betwwenn the line and time-axis.
        #   for any point (t,x) in trajectory, we set a line with slope w
        ts0 = [t-abs(x-loc_stopline)/w for t,x in zip(ts,xs)]
        #if the t0 is outof the time horizon, then 
        if ts0[0]>t0 or ts0[-1]<t0:
            return False
        else:
            return True
    @classmethod
    def tsxsIntersectionPoint_fill(self, ts, xs, t0, loc_stopline=0, w=20):
        """
        if the line started from t0 with slope w does not intersec, then interplate it. 
        --------------------------------------------------------
        input: ts and xs
            the trajectory of the vehicle. 
            Both are list. ts must increasing. 
            len(xs)=len(ts)
        input: w    
            the slope of the linear line. 
        input: t0
            the intersection point of the linear line with temporal axis. 
        output: new_ts,new_xs
            if t0 is outof the time domain, then new_ts is False

        ---------------------------------------
        Steps: 
            - If the t0 is out of the boundar, return ValueError
            - if there is some t that equal t0, then 
        ---------------------------------------------------------
        """
        #check the ts is increasing.
        if not np.all(np.diff(ts) > 0):
            builtins.tmp = ts
            raise ValueError("The input ts must be increasing!!!ts is stored in builtins.tmp")

        #the corresponding moment of ts at the stopline
        ts_stopline = [t-abs(x-loc_stopline)/(w/3.6) for t,x in zip(ts,xs)]

        #find the moment t that t0<ts_stopline[idxx]<t1
        #   if out of the horizon, then return null
        if ts_stopline[-1]<=t0 or ts_stopline[0]>=t0:
            return ts,xs

        #if there is some moment that equals t0
        #   then does not change anything. 
        idx = np.where(np.array(ts_stopline) == t0)[0]
        if len(idx)>0:return ts,xs
        
        #   means there is no moment that equals t0, then interp it. 
        #   find the idx that satisfy ts0[idx]<t0<ts0[idx+1]
        idx = np.where(np.array(ts_stopline) < t0)[0][-1]
        returned_t = ts[idx]+(ts[idx+1]-ts[idx])*(t0-ts_stopline[idx])/(ts_stopline[idx+1]-ts_stopline[idx])
        returned_x = xs[idx]+(xs[idx+1]-xs[idx])*(t0-ts_stopline[idx])/(ts_stopline[idx+1]-ts_stopline[idx])

        return copy.deepcopy(list(ts[:idx+1])+[returned_t]+list(ts[idx+1:])),copy.deepcopy(list(xs[:idx+1])+[returned_x]+list(xs[idx+1:]))
    @classmethod
    def tsxsIntersectionPoint(self, ts, xs, t0, loc_stopline=0, w=20):
        """
        OK
        Get the coordinate of the intersection point between trajectory and linear line. 
        The trajectory is given by ts and xs, 
        The linear line is given by the slope w and the starting point at (t0, 0).
        ---------------------------------------
        input: ts and xs
            the trajectory of the vehicle. 
            Both are list. ts must increasing. 
            len(xs)=len(ts)
        input: w    
            the slope of the linear line. 
        input: t0
            the intersection point of the linear line with temporal axis. 
        ---------------------------------------
        Steps: 
            - If the t0 is out of the boundar, return ValueError
            - if there is some t that equal t0, then 
        """
        #check the ts is increasing.
        if not np.all(np.diff(ts) > 0):
            builtins.tmp = ts
            raise ValueError("The input ts must be increasing!!!ts is stored in builtins.tmp")
        
        line0 = LineString([(t,x) for t,x in zip(ts,xs)])
        #compute another end of the starting wave. 
        tmp1 = max(abs(np.array(xs)-loc_stopline))
        if min(xs)>loc_stopline:
            tmp2 = max(xs)
        else:
            tmp2 = min(xs)
        line1 = LineString([(t0, loc_stopline), (t0 + tmp1/(w/3.6), tmp2)])
        intersection = line1.intersection(line0)
        if intersection.is_empty:
            return False,False
        else:
            return intersection.x,intersection.y

    @classmethod
    def tsxsIntersectionPoint0(self, ts, xs, t0, loc_stopline=0, w=20):
        """
        OK
        Get the coordinate of the intersection point between trajectory and linear line. 
        The trajectory is given by ts and xs, 
        The linear line is given by the slope w and the starting point at (t0, 0).
        ---------------------------------------
        input: ts and xs
            the trajectory of the vehicle. 
            Both are list. ts must increasing. 
            len(xs)=len(ts)
        input: w    
            the slope of the linear line. 
        input: t0
            the intersection point of the linear line with temporal axis. 
        ---------------------------------------
        Steps: 
            - If the t0 is out of the boundar, return ValueError
            - if there is some t that equal t0, then 
        """
        #check the ts is increasing.
        if not np.all(np.diff(ts) > 0):
            builtins.tmp = ts
            raise ValueError("The input ts must be increasing!!!ts is stored in builtins.tmp")

        #convert the unit of w into m/s.
        w = w/3.6

        #find the intersection betwwenn the line and time-axis.
        #   for any point (t,x) in trajectory, we set a line with slope w
        ts_stopline = [t-abs(x-loc_stopline)/w for t,x in zip(ts,xs)]
        #check the ts is increasing.
        if not np.all(np.diff(ts_stopline) > 0):
            builtins.tmp = ts_stopline
            raise ValueError("The input ts must be increasing!!!ts is stored in builtins.tmp")
        
        #if the t0 is outof the time horizon, then 
        if ts_stopline[0]>t0 or ts_stopline[-1]<t0:return False,False
        
        #if there is some moment that equals t0
        idx = np.where(np.array(ts_stopline) == t0)[0]
        if len(idx)>0:return (ts[idx[0]],xs[idx[0]])
        
        #   means there is no moment that equals t0, then interp it. 
        #   find the idx that satisfy ts0[idx]<=t0<=ts0[idx+1]
        idx = np.where(np.array(ts_stopline) < t0)[0][-1]
        returned_t = ts[idx]+(ts[idx+1]-ts[idx])*(t0-ts_stopline[idx])/(ts_stopline[idx+1]-ts_stopline[idx])
        returned_x = xs[idx]+(xs[idx+1]-xs[idx])*(t0-ts_stopline[idx])/(ts_stopline[idx+1]-ts_stopline[idx])

        return returned_t,returned_x
    
    @classmethod
    def GetCycleBoundaryOfTrajectory(self,ts,xs,w, t0, t1, t2):
        """
        Get the boundary of the trajectories. 
        The results are given by three tuples: 
        ---------------------------------------------
        Input: t0,t1,t2
            t0 is the onset moment of cycle
            t1 is the termination moment of red signal
            t2 is the termination of green signal. 
        input: ts and xs
            the trajectory of the vehicle. 
            Both are list. ts must increasing. 
            len(xs)=len(ts)
        input: w
            the backward wave speed.
        -------------------------------------------
        output: (t_LW,x_LW),(t_w,x_W),(t_UP,x_UP)
            three coordinates of the points on the trajectory. 
                (t_LW,x_LW) is the intersection point between the backward wave line (starts from the onset of red signal) and the trajectory. 
                (t_w,x_w) is the intersection point between the starting wave line and the trajectory. 
                (t_UP,x_LW) is the intersection point between the backward wave line (starts from the green end) and the trajectory. 
        -------------------------------------------
        Steps:
            - for each line. 
        """
        point0 =  self.GetIntersectionPoint(ts = ts, xs=xs, w=w, t0=t0)
        point1 =  self.GetIntersectionPoint(ts = ts, xs=xs, w=w, t0=t1)
        point2 =  self.GetIntersectionPoint(ts = ts, xs=xs, w=w, t0=t2)
        return point0,point1,point2

    @classmethod
    def translate_point_x(self,t,x, sin_theta, cos_theta ,flag=1):
        """
        get the translated coordinate of x
        If flag is 1, the axis rotate clockwise  pi/2-theta, where np.sin(theta)=self.sin_theta
        If flag is -1, then 
        ---------------------------------------------
        """

        if flag==1:
            #
            return x*sin_theta+t*cos_theta
            
        else:
            return x*cos_theta-t*sin_theta

    @classmethod
    def translate_point_t(self,t,x,sin_theta, cos_theta ,flag=1):
        """
        get the translated coordinate of t
        If flag is 1, the translate the 
        ---------------------------------------------
        """

        if flag==1:
            #
            return -x*cos_theta+t*sin_theta
            
        else:
            return x*sin_theta+t*cos_theta

    @classmethod
    def translate_point(self,t, x, sin_theta, cos_theta ,flag=1):
        """
        get the translated coordinate of point (t,x).
        If flag is 1, the translate the 
        ---------------------------------------------
        """

        if flag==1:
            #return -x*sin_theta+t*cos_theta,x*cos_theta+t*sin_theta
            return -x*cos_theta+t*sin_theta,x*sin_theta+t*cos_theta
            
        else:
            return -x*sin_theta+t*cos_theta,x*cos_theta+t*sin_theta

    @classmethod
    def translate_curve(self, ts, xs, sin_theta, cos_theta ,flag=1):
        """
        get the translated coordinate of point (t,x).
        If flag is 1, the translate, eitherwise inverse-translate.
        ---------------------------------------------
        """
        #get cos theta
        new_t = []
        new_x = []
        for t0,x0 in zip(ts,xs):
            t,x = self.translate_point(t=t0, x=x0, sin_theta=sin_theta, cos_theta=cos_theta ,flag=flag)
            new_t.append(t)
            new_x.append(x)

        return new_t,new_x


    @classmethod
    def Theoretical_TRs(self, TRs_dict, optimization_variables_sequence, X, kwargs):
        """
        --------------------------------------------------------
        input: TRs_dict
            keys are the vehicle ids. 
        input: kwargs
            a dict. 

        """
        loc_stopline = kwargs['loc_stopline']
        l_tb = kwargs['l_tb']
        w = kwargs['w']
        vf = kwargs['vf']
        direction_flag = kwargs['direction_flag']
        tolerance = kwargs['tolerance']

        results = {}
        for idx,vid in enumerate(optimization_variables_sequence):
            TR = TRs_dict[vid]
            results[vid] = self.Theoretical_TR(TR_info = TR, tau = X[0], vstop = X[idx+1],loc_stopline=loc_stopline, l_tb = l_tb, w= w, vf= vf, ufi = vf, direction_flag=direction_flag, boundary=True)

        return results

    @classmethod
    def Theoretical_TR_without_tau(self, TR_info, tau_k, kwargs, ufi = FD.vf, boundary=True):
        """
        --------------------------------------
        input: tolerance
            error tolerance. Used to check the respective point
        input: TR_info
            dict. 
            TR_info['tsxs']  = (ts,xs), 
            TR_info['redduration']  =  redduration, a float;
            TR_info['intersectionpoint']  =  (t_w, x_w), 
            TR_info['t0t1t2']  = (t0,t1,t2), 
        input: tau_k
            time duration from stopping state to accelerating state for theoretical trajectory. 
        input: boundary, a bool
            whether compute the boundary.If False, then returned value (a dict) doesnot have the key of 'LT_bound_w' and 'RT_bound_w'
        -----------------------------------------
        Output: theoretical_tr
            a dict. Keys include 'FirstSegment', 'SecondSegment', 'ThirdSegment', 'LT_bound_w', 'RT_bound_w' (if arg boundary==False, then 'LT_bound_w' and 'RT_bound_w' don't appear.)

        """
        loc_stopline = kwargs['loc_stopline']
        l_tb = kwargs['l_tb']
        w = kwargs['w']
        vf = kwargs['vf']
        direction_flag = kwargs['direction_flag']
        tolerance = kwargs['tolerance']

        theoretical_tr = {}

        #Steps: first get the attributes
        ts,xs = TR_info['tsxs']
        t_w, x_w = TR_info['intersectionpoint']
        redduration = TR_info['redduration']
        greenduration = TR_info['t0t1t2'][2] - TR_info['t0t1t2'][1]

        if tau_k>redduration:
            builtins.tmp = (tau_k, redduration)
            raise ValueError("fsdfsdfsdf")

        #first segment
        #   the 'first_' prefix means the first segment.
        first_end_point_x = x_w
        first_end_point_t = t_w - tau_k

        """
        if vstop>w+tolerance or first_end_point_t>t_w+tolerance:
            builtins.tmp = (startmoment_TMS, first_end_point_t, t_w, tau, vstop)
            raise ValueError("sdfsdfsdfsadfdfwef")
        """

        first_end_point = (first_end_point_t, first_end_point_x)
        #   second, compute the coordinate of starting point. 
        #       because the ts and xs may not complete. then from (ts[0],xs[0]), we draw a line with slop w and intersect with point say A at stopline. Then the difference between A and red termination point is the virtual_redduration.
        #           ts[0] - abs(xs[0] - loc_stopline)/(w/3.6) is the intersection moment
        virtual_redduration = TR_info['t0t1t2'][1] - (ts[0] - abs(xs[0] - loc_stopline)/(w/3.6))
        tmp_width = abs(virtual_redduration - (t_w - first_end_point_t))#unit is sec

        tmp_height = tmp_width*w/3.6*vf/3.6/(w/3.6+vf/3.6)#unit is m.
        first_start_point = (first_end_point_t - tmp_height/(vf/3.6), first_end_point_x + direction_flag*tmp_height)
        theoretical_tr['FirstSegment'] = (first_start_point, first_end_point)

        #second segment
        theoretical_tr['SecondSegment'] = (first_end_point, (t_w,x_w))

        #ThirdSegment
        #       because the ts and xs may not complete. then from (ts[-1],xs[-1]), we draw a line with slop w and intersect with point say A at stopline. Then the difference between A and green termination point is the virtual_greenduration.
        #           ts[-1] - abs(xs[-1] - loc_stopline)/(w/3.6) is the intersection moment
        virtual_greenduration = (ts[-1] - abs(xs[-1] - loc_stopline)/(w/3.6)) - TR_info['t0t1t2'][1]
        third_end_point = (t_w + virtual_greenduration*w/3.6/(w/3.6+ufi/3.6), x_w- direction_flag*virtual_greenduration*w/3.6*ufi/3.6/(w/3.6+ufi/3.6))
        theoretical_tr['ThirdSegment'] = ((t_w,x_w), third_end_point)

        #LT_bound_w and RT_bound_w
        if boundary:
            if abs(first_start_point[1]-loc_stopline)>abs(xs[0] - loc_stopline):
                LT_bound_w = ((TR_info['t0t1t2'][1]-virtual_redduration, loc_stopline), first_start_point)
            else:
                LT_bound_w = ((TR_info['t0t1t2'][1]-virtual_redduration, loc_stopline), (ts[0],xs[0]))
            #RT_bound_w
            if abs(third_end_point[1]-loc_stopline)>abs(xs[-1] - loc_stopline):
                RT_bound_w = ((TR_info['t0t1t2'][1]+virtual_greenduration, loc_stopline), third_end_point)
            else:
                RT_bound_w = ((TR_info['t0t1t2'][1]+virtual_greenduration, loc_stopline), (ts[-1],xs[-1]))

            theoretical_tr['LT_bound_w'] = LT_bound_w
            theoretical_tr['RT_bound_w'] = RT_bound_w

        return theoretical_tr

    @classmethod
    def Theoretical_TR(self, TR_info, tau, vstop, kwargs, ufi = FD.vf, boundary=True):
        """
        --------------------------------------
        input: tolerance
            error tolerance. Used to check the respective point
        input: TR_info
            dict. 
            TR_info['tsxs']  = (ts,xs), 
            TR_info['redduration']  =  redduration, a float;
            TR_info['intersectionpoint']  =  (t_w, x_w), 
            TR_info['t0t1t2']  = (t0,t1,t2), 
        input: tau and vstop
            TMS duration (unit is sec) and stopping wave (unit is km/h)
        input: boundary, a bool
            whether compute the boundary.If False, then returned value (a dict) doesnot have the key of 'LT_bound_w' and 'RT_bound_w'
        -----------------------------------------
        Output: theoretical_tr
            a dict. Keys include 'FirstSegment', 'SecondSegment', 'ThirdSegment', 'LT_bound_w', 'RT_bound_w' (if arg boundary==False, then 'LT_bound_w' and 'RT_bound_w' don't appear.)

        """
        loc_stopline = kwargs['loc_stopline']
        l_tb = kwargs['l_tb']
        w = kwargs['w']
        vf = kwargs['vf']
        direction_flag = kwargs['direction_flag']
        tolerance = kwargs['tolerance']

        theoretical_tr = {}

        #Steps: first get the attributes
        ts,xs = TR_info['tsxs']
        t_w, x_w = TR_info['intersectionpoint']
        redduration = TR_info['redduration']
        greenduration = TR_info['t0t1t2'][2] - TR_info['t0t1t2'][1]

        #first segment
        #   the 'first_' prefix means the first segment.
        startmoment_TMS = TR_info['t0t1t2'][1] + l_tb/(w/3.6) -tau
        first_end_point_x = x_w
        first_end_point_t = startmoment_TMS + (abs(x_w - loc_stopline)-l_tb)/(vstop/3.6)
        
        """
        if vstop>w+tolerance or first_end_point_t>t_w+tolerance:
            builtins.tmp = (startmoment_TMS, first_end_point_t, t_w, tau, vstop)
            raise ValueError("sdfsdfsdfsadfdfwef")
        """

        first_end_point = (first_end_point_t, first_end_point_x)
        #   second, compute the coordinate of starting point. 
        #       because the ts and xs may not complete. then from (ts[0],xs[0]), we draw a line with slop w and intersect with point say A at stopline. Then the difference between A and red termination point is the virtual_redduration.
        #           ts[0] - abs(xs[0] - loc_stopline)/(w/3.6) is the intersection moment
        virtual_redduration = TR_info['t0t1t2'][1] - (ts[0] - abs(xs[0] - loc_stopline)/(w/3.6))
        tmp_width = abs(virtual_redduration - (t_w - first_end_point_t))#unit is sec

        tmp_height = tmp_width*w/3.6*vf/3.6/(w/3.6+vf/3.6)#unit is m.
        first_start_point = (first_end_point_t - tmp_height/(vf/3.6), first_end_point_x + direction_flag*tmp_height)
        theoretical_tr['FirstSegment'] = (first_start_point, first_end_point)

        #second segment
        theoretical_tr['SecondSegment'] = (first_end_point, (t_w,x_w))

        #ThirdSegment
        #       because the ts and xs may not complete. then from (ts[-1],xs[-1]), we draw a line with slop w and intersect with point say A at stopline. Then the difference between A and green termination point is the virtual_greenduration.
        #           ts[-1] - abs(xs[-1] - loc_stopline)/(w/3.6) is the intersection moment
        virtual_greenduration = (ts[-1] - abs(xs[-1] - loc_stopline)/(w/3.6)) - TR_info['t0t1t2'][1]
        third_end_point = (t_w + virtual_greenduration*w/3.6/(w/3.6+ufi/3.6), x_w- direction_flag*virtual_greenduration*w/3.6*ufi/3.6/(w/3.6+ufi/3.6))
        theoretical_tr['ThirdSegment'] = ((t_w,x_w), third_end_point)

        #LT_bound_w and RT_bound_w
        if boundary:
            if abs(first_start_point[1]-loc_stopline)>abs(xs[0] - loc_stopline):
                LT_bound_w = ((TR_info['t0t1t2'][1]-virtual_redduration, loc_stopline), first_start_point)
            else:
                LT_bound_w = ((TR_info['t0t1t2'][1]-virtual_redduration, loc_stopline), (ts[0],xs[0]))
            #RT_bound_w
            if abs(third_end_point[1]-loc_stopline)>abs(xs[-1] - loc_stopline):
                RT_bound_w = ((TR_info['t0t1t2'][1]+virtual_greenduration, loc_stopline), third_end_point)
            else:
                RT_bound_w = ((TR_info['t0t1t2'][1]+virtual_greenduration, loc_stopline), (ts[-1],xs[-1]))

            theoretical_tr['LT_bound_w'] = LT_bound_w
            theoretical_tr['RT_bound_w'] = RT_bound_w

        return theoretical_tr



    @classmethod
    def TRs_average_speed(self, TRs_dict, ufi_consider = False):
        """
        compute the average speed of the TRs

        """
        vmean =[]
        for vid in TRs_dict.keys():
            if ufi_consider:
                ts,xs  = TRs_dict[vid]['tsxs']
                t_w, x_w = TRs_dict[vid]['intersectionpoint']
            else:
                ts0,xs0  = TRs_dict[vid]['tsxs']
                t_w, x_w = TRs_dict[vid]['intersectionpoint']
                #use only the 
                ts = np.array(ts0)[np.where(np.array(ts0)<=t_w)[0]]
                xs = np.array(xs0)[np.where(np.array(ts0)<=t_w)[0]]
            vmean.append(abs(3.6*(xs[-1]-xs[0])/(ts[-1]-ts[0])))
        #builtins.tmp = vmean
        return np.nanmean(vmean)

    @classmethod
    def tsvs_threshold_lines(self, ts,vs, threshold  =3):
        """
        ----------------------------------
        output: res
            res['spillover'] is a list.
            ts0,vs0  = res['spillover'][idx]
        """
        res = {}
        res['spillover'] = []
        res['nonspillover'] = []
        for i,j in zip(range(len(ts)-1), range(1,len(ts))):
            t0 = ts[i];t1 = ts[j]
            v0 = vs[i];v1 = vs[j]
            if v0<threshold and v1<threshold and v0>=0 and v1>=0:
                res['spillover'].append([[t0,t1],[v0,v1]])
            else:
                res['nonspillover'].append([[t0,t1],[v0,v1]])
        return res['spillover'],res['nonspillover']

    @classmethod
    def Polygon_shapely_one_TR_using_vstop(self, TR_info, tao, loc_stopline, vstop, l_tb, w = FD.w, vf = FD.vf, direction_flag = 1, ufi=  FD.vf):
        """
        calculate the area using shapely for only one trajectory. 
        THe area is surrounded by the theoretical TR and the observed TR. 
        Note that there are three segments of the theoretical TR. 
        ----------------------------------------------------------
        input: tao vstop
            tao is the TMS duration ,unit is sec
            vstop is the stoping wave, unit is km/h
        input: TR_info
            dict. 
            TR_info['tsxs']  = (ts,xs), 
            TR_info['redduration']  =  redduration, a float;
            TR_info['intersectionpoint']  =  (t_w, x_w), 
            TR_info['t0t1t2']  = (t0,t1,t2), 
        input: loc_stopline, l_tb
            the location of stop line and the length of the turning bay. 
        input: direction_flag, either 1 or -1
            if east and north, it should be 1. Otherwise it should be -1.
        input: 
        output: area
        ------------------------------------------------------
        """

        #theoretical_tr['FirstSegment'] = ((t0,x0),(t1,x1))
        theoretical_tr = self.Theoretical_TR(TR_info=TR_info, tau=tau, vstop=vstop,loc_stopline=loc_stopline, l_tb=l_tb, w= w, vf=vf, ufi = vf, direction_flag=1)
        #Steps: first get the attributes
        ts,xs = TR_info['tsxs']

        p0,p1 = theoretical_tr['FirstSegment']
        p.line([p0[0], p1[0]], [p0[1], p1[1]])

        p0,p1 = theoretical_tr['SecondSegment']
        p.line([p0[0], p1[0]], [p0[1], p1[1]])

        p0,p1 = theoretical_tr['ThirdSegment']
        p.line([p0[0], p1[0]], [p0[1], p1[1]])





        #Steps: first get the attributes
        ts,xs = TR_info['tsxs']
        t_w, x_w = TR_info['intersectionpoint']
        redduration = TR_info['redduration']
        greenduration = TR_info['t0t1t2'][2] - TR_info['t0t1t2'][1]

        if not np.all(np.diff(ts) > 0):
            builtins.tmp = ts
            raise ValueError("The input ts must be increasing!!!ts is stored in builtins.tmp")
        
        #truncate the theoretical trajectory. 
        #   Truncate the first segment 
        #       first segment of theoretical TR
        #       firstly compute the corrdinate of end points, start_point and endpoint.
        startmoment_TMS = TR_info['t0t1t2'][1] + l_tb/(w/3.6) -tao
        end_point_x = x_w
        end_point_t = startmoment_TMS + (abs(x_w - loc_stopline)-l_tb)/(vstop/3.6)
        if vstop>w or end_point_t>t_w:
            builtins.tmp = (startmoment_TMS, end_point_t, t_w, tao, vstop)
            raise ValueError("sdfsdfsdfsadfdfwef")
        end_point = (end_point_t, end_point_x)
        #   second, compute the coordinate of starting point. 
        tmp_width = redduration - (t_w - end_point_t)#unit is sec
        tmp_height = tmp_width*w/3.6*vf/3.6/(w/3.6+vf/3.6)#unit is m.
        start_point = (end_point_t - tmp_height/(vf/3.6), end_point_x + direction_flag*tmp_height)
        FirstSegment = LineString([start_point, end_point])

        virtual_t0 = ts[0] - abs(xs[0]-loc_stopline)/(w/3.6)
        virtual_x0 = loc_stopline
        tmp_width = redduration#unit is sec
        tmp_height = tmp_width*w/3.6*vf/3.6/(w/3.6+vf/3.6)#unit is m.
        maximal_virtual_t0 = t_w - tmp_height/(vf/3.6)
        maximal_virtual_x0 = x_w + direction_flag*tmp_height

        intersection = FirstSegment.intersection( LineString([(maximal_virtual_t0, maximal_virtual_x0), (ts[0], xs[0]), (virtual_t0, virtual_x0)]))
        if intersection.is_empty:
            builtins.tmp = (start_point, end_point, (ts[0], xs[0]), (virtual_t0, virtual_x0), tao, vstop)
            raise ValueError("sdfsdfsdfsdf")
        else:
            Leftpoint_TR = (intersection.x,intersection.y)

        #Truncate the third segment 
        #   the two ends of the tihird segmeent are start_point,end_point.
        virtual_t1 = ts[-1] - abs(xs[-1]-loc_stopline)/(w/3.6)
        virtual_x1 = loc_stopline
        start_point = (t_w, x_w)
        end_point = (t_w + greenduration*w/3.6/(w/3.6+ufi/3.6), x_w- direction_flag*greenduration*w/3.6*ufi/3.6/(w/3.6+ufi/3.6))
        maximal_point_t = virtual_t1 + max([abs(x-loc_stopline) for x in xs])/(w/3.6)
        maximal_point_x = virtual_x1 + direction_flag*(maximal_point_t - virtual_t1)*(w/3.6)
        ThirdSegment = LineString([start_point, end_point])
        intersection = ThirdSegment.intersection( LineString([(maximal_point_t, maximal_point_x), (virtual_t1, virtual_x1)]))
        if intersection.is_empty:
            builtins.tmp = (start_point, end_point, (maximal_point_t, maximal_point_x), (virtual_t1, virtual_x1))
            raise ValueError("sdfsdfsdfsdf")
        else:
            Rightpoint_TR = (intersection.x,intersection.y)


        polygonn = Polygon([Leftpoint_TR]+[(end_point_t, end_point_x)]+[(t_w , x_w)] + [Rightpoint_TR]+ [(t,x) for t,x in zip(ts[::-1], xs[::-1])])

        return polygonn

    @classmethod
    def Area_Curve_with_translation(self,tss, xss):
        """
        Compute the area surrounded by the curve and the axis. 
        Different from self.Area_Curve(), the self.Area_Curve_with_translation() will translate the coordinate first 
        ------------------------------------------
        input: tss xss
            both are lists. 
            len(tss)=len(xss)
        """
        #translate
        translated_ts = []
        translated_xs = []
        for t,x in zip(tss,xss):
            new_t,new_x = self.translate_point(t= t, x= x, sin_theta= self.sin_theta,cos_theta= self.cos_theta)
            translated_ts.append(new_t)
            translated_xs.append(new_x)
        return np.array(translated_xs[0:-1]).dot(np.diff(translated_ts))

    @classmethod
    def Area_Curve(self,tss, xss):
        """
        Compute the area surrounded by the curve and the axis. 
        The method 
        ------------------------------------------
        input: tss xss
            both are lists. 
            len(tss)=len(xss)
        """
        return np.array(xss[0:-1]).dot(np.diff(tss))
    
    @classmethod
    def speed_vector_TR(self, TR):
        """

        """
        ts,xs = TR['tsxs']
        np.diff(xs)/np.diff(ts)





        pass


    @classmethod
    def Stopwavespeed2PHI(self,v_stop, l_tb, x_w):
        """
        convert the stop to the PHI, which is used in optimization.
        Because v_stop \in [0,w], then PHI in [(x_w - l_tb)/(w/3.6),+\infty]
        -------------------------------------
        input: v_stop, unit is km/h. 
            the stopping speed, 
        input: l_tb, unit is m.
            the lengh of the bay area. 
        input: x_w. unit is m. 
            the spatial coordinate of 
        """
        return (x_w - l_tb)/(v_stop/3.6)

    @classmethod
    def Area_EJD_using_stopwave(self,r,tao,x_w,l_tb,v_stop,w=FD.w,vf=FD.vf):
        """
        The area of the triangulal. When the stoping wave variable is used. 
        -------------------------------------------------
        input: w vf. units are km/h
            the parameters in the FD settings. 
        input: r, unit is sec
            the red duration
        input: tao, unit is sec.
            the assumed css duration at the interface between the C-section and U-section
        input: l_tb, unit is ,
            length of turning bay aea, or length of C-section. 
        input: x_w, unit is m.
            the spatial coordinate, shoule be greater than l_tb
        input: v_stop, unit is km/h. 
            the assumed stop wave speed, the vector starts from the onset of tao, and terminate at...
        """
        #convert to m/s
        w=w/3.6
        vf=vf/3.6
        v_stop = v_stop/3.6

        PHI = self.Stopwavespeed2PHI(v_stop=v_stop*3.6, l_tb=l_tb, x_w=x_w)

        #compute the area
        S = w*vf/(2.0*w+vf)*((r-tao-(x_w-l_tb)/w+PHI)**2)

        return S

    @classmethod
    def Area_EJD_using_PHI(self,PHI,r,tao,x_w,l_tb,loc_stopline ,w=FD.w,vf=FD.vf):
        """
        The area of the triangulal. When the stoping wave variable is used. 
        -------------------------------------------------
        input: w vf. units are km/h
            the parameters in the FD settings. 
        input: r, unit is sec
            the red duration
        input: tao, unit is sec.
            the assumed css duration at the interface between the C-section and U-section
        input: l_tb, unit is ,
            length of turning bay aea, or length of C-section. 
        input: x_w, unit is m.
            the spatial coordinate, shoule be greater than l_tb
        input: PHI 
            PHI = self.Stopwavespeed2PHI(v_stop=v_stop*3.6, l_tb=l_tb, x_w=x_w)
        """
        #convert to m/s
        w=w/3.6
        vf=vf/3.6

        #compute the area
        S = w*vf/(2.0*w+2*vf)*((r-tao-(x_w-loc_stopline-l_tb)/w+PHI)**2)
        return S
    
    @classmethod
    def Area_JOMC(self, w, t_w, x_w, r, Cycle_start):
        """
        the area of the Trapezoid
        ---------------------------------------
        input: w
            the backward wave speed, unit is km/h
        input: t_w and x_w
            the coordinate of the intersection point by the starting wave and the trajectory. 
        input: Cycle_start, unit is sec
            the start moment of current cycle. 
        input: r
            the red duration.
        """
        w = w/3.6
        #transplate the interection point between the starting wave and trajectory
        x_w_11 = self.translate_point_x(t=t_w, x=x_w, sin_theta = self.sin_theta, cos_theta = self.cos_theta)
        S  = r*w/(np.sqrt((w)**2+1))*(x_w_11+(w*Cycle_start+2*x_w)/(np.sqrt((w)**2+1)))/2.0
        return S

    @classmethod
    def Area_PI12_using_vstop(self,r, tao, t_w, x_w, l_tb,v_stop,Cycle_start , w=FD.w,vf=FD.vf):
        """
        Compute the area defined by the first and second part. 
        """
        S1 = self.Area_EJD_using_stopwave(r=r, tao = tao, x_w = x_w, l_tb = l_tb, v_stop = v_stop,w=w, vf=vf)

        S2 = self.Area_JOMC(w=w, t_w = t_w, x_w = x_w, r=r,Cycle_start=Cycle_start)

        return S1+S2

    @classmethod
    def USELESS_objectivefunction(self, tau, v_stops, t0,t1,t2, cycle_sliceddata):
        """
        compute the objective function, 
        -------------------------------------
        input: tau, a float
            the assumed spillover duration
        input: v_stops, a list.
            the stoppingwave speed. The length is the same as number of trajectories. 
        input: cycle_sliceddata, a dict
            NOTE that it is only one cycle data. 
            the trajectory and the intersectionpoint info (between the trajectory and the starting wave).
            cycle_sliceddata['l'][vehicle_id]={'tsxs':(ts,xs),'intersectionpoint':(t,x)}
            ts and xs are list with the same length.
            t and x are float. (t,x) should locate at the trajectory.
        input: t0 t1 t2
            all are float. They are cycle onset moment, red termination moment, cycle end moment. 
        """
        #for easy ordered reference
        turningdirection = ['l','t','r']
        pass
    
    @classmethod
    def numberoftrajectories_cycles(self, cycles_sliceddata):
        """
        count the number of trajectories in one cycle. 
        ---------------------------------
        input: cycle_sliceddata
            the trajectory info for all cycles. cycles_sliceddata is a dict, keys include 'l', 't' and 'r'. 
            cycle_sliceddata['l'][vehicle_id].keys()=['tsxs', 'intersectionpoint', 'twoboudnarys', 'redduration', 'cyclestartmoment']

            cycle_sliceddata['l'][vehicle_id]['tsxs']  =(ts,xs)
            cycle_sliceddata['l'][vehicle_id]['intersectionpoint'] = (t_w,x_w)
            cycle_sliceddata['l'][vehicle_id]['twoboudnarys'] = ((t0,x0),(t2,x2))
            cycle_sliceddata['l'][vehicle_id]['redduration'] = r
            cycle_sliceddata['l'][vehicle_id]['cyclestartmoment'] = cyclestartmoment
        """
        count = 0
        for cycle_idx in cycles_sliceddata.keys():
            count = count + self.numberoftrajectories(cycles_sliceddata[cycle_idx])
        return count


    @classmethod
    def numberoftrajectories_paired_reds(self, TRs_paired_reds):
        """
        count the number of trajectories in one cycle. 
        ---------------------------------
        input: cycle_sliceddata
            the trajectory info for each cycle. cycle_sliceddata is a dict, keys include 'l', 't' and 'r'. 
            cycle_sliceddata['l'][vehicle_id].keys()=['tsxs', 'intersectionpoint', 'twoboudnarys', 'redduration', 'cyclestartmoment']

            cycle_sliceddata['l'][vehicle_id]['tsxs']  =(ts,xs)
            cycle_sliceddata['l'][vehicle_id]['intersectionpoint'] = (t_w,x_w)
            cycle_sliceddata['l'][vehicle_id]['twoboudnarys'] = ((t0,x0),(t2,x2))
            cycle_sliceddata['l'][vehicle_id]['redduration'] = r
            cycle_sliceddata['l'][vehicle_id]['cyclestartmoment'] = cyclestartmoment
        """
        counts = []
        for TRs_paired_red in TRs_paired_reds:
            tmp = {}
            for d in TRs_paired_red.keys():
                tmp[d] = len(TRs_paired_red[d])
            counts.append(tmp)
        return counts

    @classmethod
    def numberoftrajectories(self, cycle_sliceddata):
        """
        count the number of trajectories in one cycle. 
        ---------------------------------
        input: cycle_sliceddata
            the trajectory info for each cycle. cycle_sliceddata is a dict, keys include 'l', 't' and 'r'. 
            cycle_sliceddata['l'][vehicle_id].keys()=['tsxs', 'intersectionpoint', 'twoboudnarys', 'redduration', 'cyclestartmoment']

            cycle_sliceddata['l'][vehicle_id]['tsxs']  =(ts,xs)
            cycle_sliceddata['l'][vehicle_id]['intersectionpoint'] = (t_w,x_w)
            cycle_sliceddata['l'][vehicle_id]['twoboudnarys'] = ((t0,x0),(t2,x2))
            cycle_sliceddata['l'][vehicle_id]['redduration'] = r
            cycle_sliceddata['l'][vehicle_id]['cyclestartmoment'] = cyclestartmoment
        """
        count = 0
        for td in cycle_sliceddata.keys():
            count = count + len(cycle_sliceddata[td].keys())
        return count
    
    @classmethod
    def optimize_one_TR(self, one_TR, loc_stopline, l_tb, w=FD.w, vf=FD.vf, post_constraint_check = True, direction_flag = 1, ufi = FD.vf, method = 'COBYLA', tolerance = 1):
        """
        --------------------------------------
        input: one_TR
            dict. 
            one_TR['tsxs']  = (ts,xs), 
            one_TR['redduration']  =  redduration, a float;
            one_TR['intersectionpoint']  =  (t_w, x_w), 
            one_TR['t0t1t2']  = (t0,t1,t2), 


        """
        #constraints. 
        #   constraint_one_TR_vstop( TR_info,  loc_stopline, l_tb, w = FD.w, vf = FD.vf, direction_flag = 1, ufi=  FD.vf)
        cons_one_TR = self.constraint_one_TR_vstop(TR_info = one_TR, loc_stopline=loc_stopline,l_tb=l_tb, w = w, vf = vf, direction_flag = direction_flag, ufi=ufi, tolerance=tolerance)

        #initial guess
        #   X = [tau, vstop]
        X0 = self.initial_guess_one_TR(TR_info = one_TR,  loc_stopline=loc_stopline , l_tb=l_tb, w=w, vf=vf, direction_flag=direction_flag, ufi = ufi,tolerance=tolerance)

        #
        if not self.constraints_satistied(cons = cons_one_TR, X=X0, tolerance = tolerance):
            builtins.tmp ={}
            builtins.tmp['X0']=X0;builtins.tmp['cons']=cons_one_TR;
            raise ValueError('Constraints not satisfied. ')
        
        #args in the scipy.optimization
        #   the objective function is as follows:
        #       objective_one_turning(self, X, redduration,cycle_sliceddata_turning, \
        #       optimization_variables_sequence, loc_stopline, l_tb, w=FD.w, vf = FD.vf)
        #args = (redduration,cycle_sliceddata_turning, optimization_variables_sequence_turning,loc_stopline,l_tb, w, vf)

        #res is the scipy.optimize.optimize.OptimizeResult instance 
        #   res['x'] gives the solution.
        #   res['fun'] gives the objective function. 
        #self.objective_one_TR(self, X, TR_info, loc_stopline, l_tb, w = FD.w, vf = FD.vf, direction_flag = 1, ufi= FD.vf, tolerance = 1)
        args = (one_TR, loc_stopline, l_tb, w, vf, direction_flag, ufi,tolerance)
        res = scipy.optimize.minimize(fun = self.objective_one_TR, x0=X0,constraints=cons_one_TR,args = args,method = method)
        
        #
        if post_constraint_check and (not self.constraints_satistied(cons = cons_one_TR,X=res['x'])):
            builtins.tmp ={}
            builtins.tmp['X0']=res['x'];builtins.tmp['cons']=constraints_turning;
            raise ValueError('Constraints not satisfied. ')
        
        return X0,res,cons_one_TR

    @classmethod
    def benchamrk_spillover_types(self, spillovers_paired_reds, res_paired_reds, TRs_paired_reds):
        """
        Get the indexes (of the paired red) of each spillover types: no spillvoer, only one turning sillover, both spillovers 
        ------------------------------------------------
        input: len(spillovers_paired_reds)==len(res_paired_reds)==len(TRs_paired_reds) 
        input: res_paired_reds, a list.
            res_paired_reds[cycle_idx][direction].keys() are 
                ['cons', 'res', 'Y0', 'X0', 'optimization_variables_sequence', 'optimal_Y']
        input: spillovers_paired_reds, a list. 
            spillovers_paired_reds[cycle_idx][direction] = a scalar. The simulated spillover duration. 
        output: 
            no_spillover_idxs, one_spillover_idxs, both_spillover_idxs

            All are lists. 
        -----------------------------------------
        """

        no_spillover_idxs = []
        one_spillover_idxs = []
        both_spillover_idxs = []
        for idx, (spillover_dict, opt_res_dict, TRs_dict) in enumerate(zip(spillovers_paired_reds, res_paired_reds, TRs_paired_reds)):
            #typee==0 means no spillover
            #   == 1 means only one turning direction spillover
            #   == 2 
            typee = 0
            for d in spillover_dict.keys():
                typee = typee + (spillover_dict[d]>0)
            #typee = typee-1
            #print('typee=====', typee)
            if typee==0:
                #check the trajectory 
                n_trs = False
                for d in TRs_dict.keys():
                    if len(TRs_dict[d])>0:n_trs=True;break
                if n_trs:
                    no_spillover_idxs.append(idx)
                pass
            elif typee==1:
                #check the trajectory 
                n_trs = False
                for d in TRs_dict.keys():
                    if len(TRs_dict[d])>0:n_trs=True;break
                if n_trs:
                    one_spillover_idxs.append(idx)
            elif typee==2:
                #check the trajectory 
                n_trs = False
                for d in TRs_dict.keys():
                    if len(TRs_dict[d])>0:n_trs=True;break
                if n_trs:
                    both_spillover_idxs.append(idx)
        return no_spillover_idxs, one_spillover_idxs, both_spillover_idxs

    @classmethod
    def aggregate_areas_different_TMS_types(self, spillovers_paired_reds, res_paired_reds):
        """
        statistics aboud the area of different spillover types:
            no spillover, single spillover and multiple spillovers
        --------------------------------------------------------------
        input: spillovers_paired_reds
            a list. 
            spillovers_paired_reds[cycle_idx][turning_direction] = scalar. 0 means there is no spillover. 
        input: res_paired_reds
            a list. 
            res_paired_reds[cycle_idx][turning_direction] is a dict. 
            keys include:
                ['cons', 'res', 'Y0', 'X0', 'optimization_variables_sequence', 'optimal_Y']
            If the TRs exhibit no delay, then res_paired_reds[cycle_idx][turning_direction] is:
                {'X0':np.nan, 'res':{'x':[0]},'Y0':np.nan, 'cons':np.nan,'optimization_variables_sequence':np.nan, 'optimal_Y':np.nan}
        output: areas_aggregations
            a dict. 
            areas_aggregations['np_spillovers'] is a list
            areas_aggregations['spillover_self_area'] is a list
            areas_aggregations['spillover_other_area'] is a list
            areas_aggregations['twospillovers'] is a list


        """
        areas_aggregations = {}
        areas_aggregations['np_spillovers'] = []
        areas_aggregations['spillover_self_area'] = []
        areas_aggregations['spillover_other_area'] = []
        areas_aggregations['twospillovers'] = []

        for idx,(spillovers_paired_red,opt_res_paired_red) in enumerate(zip(spillovers_paired_reds, res_paired_reds)):
            #determine the spillover types, 
            typee_spillover = 0
            for d in spillovers_paired_red.keys():
                typee_spillover = typee_spillover + (spillovers_paired_red[d]>0)

            #store the area result of each spillover type. 
            #   without spillover
            if typee_spillover==0:
                
                for d in spillovers_paired_red.keys():
                    areas_aggregations['np_spillovers'].append(opt_res_paired_red[d]['res']['x'][0])

            #   one spillover
            elif typee_spillover==1:
                #self spillover
                for d in spillovers_paired_red.keys():
                    if spillovers_paired_red[d]>0:
                        areas_aggregations['spillover_self_area'].append(opt_res_paired_red[d]['res']['x'][0])
                    else:
                        areas_aggregations['spillover_other_area'].append(opt_res_paired_red[d]['res']['x'][0])
            elif typee_spillover==2:
                for d in spillovers_paired_red.keys():
                    areas_aggregations['twospillovers'].append(opt_res_paired_red[d]['res']['x'][0])
        return areas_aggregations

    @classmethod
    def aggregate_durations(self, spillovers_paired_reds, res_paired_reds):
        """
        
        --------------------------------------------------------------
        input: spillovers_paired_reds
            a list. 
            spillovers_paired_reds[cycle_idx][turning_direction] = scalar. 0 means there is no spillover. 
        input: res_paired_reds
            a list. 
            res_paired_reds[cycle_idx][turning_direction] is a dict. 
            keys include:
                ['cons', 'res', 'Y0', 'X0', 'optimization_variables_sequence', 'optimal_Y']
            If the TRs exhibit no delay, then res_paired_reds[cycle_idx][turning_direction] is:
                {'X0':np.nan, 'res':{'x':[0]},'Y0':np.nan, 'cons':np.nan,'optimization_variables_sequence':np.nan, 'optimal_Y':np.nan}
        output: 
            tau_es_aggregation
            pd.DataFrame instance. index are cycle_idx, columns include
                'benchamrk LT', 'benchamrk TH', 'optimized LT', 'optimized TH'

        """
        if len(spillovers_paired_reds)!=len(res_paired_reds):
            raise ValueError("sdfsdfsdfsfasf")
        result = pd.DataFrame()
        for idx in range(len(spillovers_paired_reds)):
            for d in spillovers_paired_reds[idx].keys():
                result.loc['cycle_'+str(idx), 'benchamrk '+ d] = spillovers_paired_reds[idx][d]
            for d in spillovers_paired_reds[idx].keys():
                if res_paired_reds[idx].has_key(d):
                    result.loc['cycle_'+str(idx), 'optimized '+ d] = round(res_paired_reds[idx][d]['res']['x'][0],3)

            for d in spillovers_paired_reds[idx].keys():
                if res_paired_reds[idx].has_key(d):
                    result.loc['cycle_'+str(idx), 'Area '+ d] = round(res_paired_reds[idx][d]['optimal_Y'],3)
        return result


    @classmethod
    def aggregate_durations_cross(self, spillovers_paired_reds, opt_res_paired_reds_corss):
        """
        
        --------------------------------------------------------------
        input: spillovers_paired_reds
            a list. 
            spillovers_paired_reds[cycle_idx][turning_direction] = scalar. 0 means there is no spillover. 
        input: opt_res_paired_reds_corss
            a list. 
            opt_res_paired_reds_corss[cycle_idx][signal_d][turning_direction] is a dict. 
            keys include:
                ['cons', 'res', 'Y0', 'X0', 'optimization_variables_sequence', 'optimal_Y']
            If the TRs exhibit no delay, then res_paired_reds[cycle_idx][turning_direction] is:
                {'X0':np.nan, 'res':{'x':[0]},'Y0':np.nan, 'cons':np.nan,'optimization_variables_sequence':np.nan, 'optimal_Y':np.nan}
        output: 
            tau_es_aggregation
            pd.DataFrame instance. index are cycle_idx, columns include
                'benchamrk left', 'benchamrk through', 'optimized left', 'optimized through'

        """
        if len(spillovers_paired_reds)!=len(opt_res_paired_reds_corss):
            raise ValueError("sdfsdfsdfsfasf")

        result = pd.DataFrame()

        for idx in range(len(spillovers_paired_reds)):

            #optimized tau and benchamrk tau
            for d in spillovers_paired_reds[idx].keys():
                result.loc['cycle_'+str(idx), 'benchamrk:'+ d] = spillovers_paired_reds[idx][d]

            #the duration, column name is optimized left-left
            #   signal_d[:2] = 'le' or 'th'
            for signal_d in opt_res_paired_reds_corss[idx].keys():
                for turning_d in opt_res_paired_reds_corss[idx][signal_d].keys():
                    result.loc['cycle_'+str(idx), 'optimized:'+ signal_d[:2]+'-'+turning_d[:2]] = str(opt_res_paired_reds_corss[idx][signal_d][turning_d]['res']['x'][0])
                    
            #area, the columns name is "Area left"
            for signal_d in opt_res_paired_reds_corss[idx].keys():
                for turning_d in opt_res_paired_reds_corss[idx][signal_d].keys():
                    result.loc['cycle_'+str(idx), 'Area:'+ signal_d[:2]+'-'+turning_d[:2]] = str(opt_res_paired_reds_corss[idx][signal_d][turning_d]['optimal_Y'])
        return result

    @classmethod
    def aggregate_areas(self, spillovers_paired_reds, res_paired_reds, TRs_paired_reds):
        """
        ---------------------------------------------------------
        input: len(spillovers_paired_reds) ==len(res_paired_reds) ==len(TRs_paired_reds)
        input: 
        output: areas
            a dict. 
            areas['no_spillover_cases'] = pd.Dataframe, index include the 'left' and 'through'; columns include 'cycle_0', 'cycle_1'...
            areas['one_spillover_cases'] = pd.Dataframe, index include the 'left' and 'through'; columns include 'cycle_0', 'cycle_1'...
            areas['both_spillover_cases'] = pd.Dataframe, index include the 'left' and 'through'; columns include 'cycle_0', 'cycle_1'...
        """
        #initialize the output
        results = {'no_spillover_cases':{}, 'one_spillover_cases':{}, 'both_spillover_cases':{}}

        no_spillover_idxs, one_spillover_idxs, both_spillover_idxs = self.benchamrk_spillover_types(spillovers_paired_reds = spillovers_paired_reds,res_paired_reds = res_paired_reds, TRs_paired_reds = TRs_paired_reds)

        for idx in no_spillover_idxs:
            for d in res_paired_reds[idx].keys():
                if not results['no_spillover_cases'].has_key(d):results['no_spillover_cases'][d]={}
                results['no_spillover_cases'][d]['cycle_'+str(idx)] = res_paired_reds[idx][d]['optimal_Y']

        for idx in one_spillover_idxs:
            for d in res_paired_reds[idx].keys():
                if not results['one_spillover_cases'].has_key(d):results['one_spillover_cases'][d]={}
                results['one_spillover_cases'][d]['cycle_'+str(idx)] = res_paired_reds[idx][d]['optimal_Y']

        for idx in both_spillover_idxs:
            for d in res_paired_reds[idx].keys():
                if not results['both_spillover_cases'].has_key(d):results['both_spillover_cases'][d]={}
                results['both_spillover_cases'][d]['cycle_'+str(idx)] = res_paired_reds[idx][d]['optimal_Y']

        results['no_spillover_cases'] = pd.DataFrame(results['no_spillover_cases'])
        results['one_spillover_cases'] = pd.DataFrame(results['one_spillover_cases'])
        results['both_spillover_cases'] = pd.DataFrame(results['both_spillover_cases'])

        return results

    @classmethod
    def aggregate_batch_opt_res_spillover_non_spillover_areas(self, spillovers_paired_reds, res_paired_reds):
        """
        statistics about the area for (simulated) spillover scanarios.
        -------------------------------
        input: res_paired_reds, a list.
            res_paired_reds[cycle_idx][direction].keys() are 
                ['cons', 'res', 'Y0', 'X0', 'optimization_variables_sequence', 'optimal_Y']
        input: spillovers_paired_reds, a list. 
            spillovers_paired_reds[cycle_idx][direction] = a scalar. The simulated spillover duration. 
        output: Areas, a dict
        """
        #A means the area of 
        Areas = {'spillovercase':{'A':[],'A_':[]}, 'nospillovercase':[]}
        for idx,spillovers_paired_red in enumerate(spillovers_paired_reds):
            for d in spillovers_paired_red.keys():
                if not res_paired_reds[idx].has_key(d):continue
                if spillovers_paired_red[d]==0:
                    Areas['nospillovercase'].append(res_paired_reds[idx][d]['optimal_Y'])
                else:
                    for k in spillovers_paired_red.keys():
                        if k==d:
                            Areas['spillovercase']['A'].append({'cycle_'+str(idx):res_paired_reds[idx][k]['optimal_Y']})
                        else:
                            if not res_paired_reds[idx].has_key(k):
                                Areas['spillovercase']['A_'].append({'cycle_'+str(idx):'NODATA'})
                            else:
                                Areas['spillovercase']['A_'].append({'cycle_'+str(idx):res_paired_reds[idx][k]['optimal_Y']})
        return Areas

    @classmethod
    def aggregate_opt_res_paired_reds(self, spillovers_paired_reds, res_paired_reds):
        """
        NOTe the difference between self.aggregate_opt_res_paired_red() and self.aggregate_opt_res_paired_reds().
        If one cycle is assumed no spillover, then it returns:
            {'X0':np.nan, 'res':{'x':[0]},'Y0':np.nan, 'cons':np.nan,'optimization_variables_sequence':np.nan, 'optimal_Y':np.nan}
        -------------------------------
        input: res_paired_reds, a list.
            res_paired_reds[cycle_idx][direction].keys() are 
                ['cons', 'res', 'Y0', 'X0', 'optimization_variables_sequence', 'optimal_Y']
        input: spillovers_paired_reds, a list. 
            spillovers_paired_reds[cycle_idx][direction] = a scalar. The simulated spillover duration. 
        -------------------------------------------------
        output: aggregation
            columns include '1_left' '1_through', i.e. '1_left' means the first cycle and left turning direction. 
            index include 'objective', 'spillover duration'
        """
        output = {}
        for idx,res_paired_red in enumerate(res_paired_reds):

            spillovers_paired_red = spillovers_paired_reds[idx]

            for d in res_paired_red.keys():
                label = str(idx)+'_'+d
                if math.isnan(res_paired_red[d]['optimal_Y']):
                    output[label] = {'objective':np.nan, \
                            'optimalDuration':0,\
                            'SpilloverDuration':spillovers_paired_red[d]
                            }
                else:
                    output[label] = {'objective':res_paired_red[d]['optimal_Y'], \
                            'optimalDuration':res_paired_red[d]['res'].x[0],\
                            'SpilloverDuration':spillovers_paired_red[d]
                            }
        return pd.DataFrame(output)

    @classmethod
    def aggregate_opt_res_paired_red(self, spillovers_paired_red, res_paired_red):
        """
        -------------------------------
        input: res_paired_red
            res_paired_red[direction].keys are 
                ['cons', 'res', 'Y0', 'X0', 'optimization_variables_sequence', 'optimal_Y']
        input: spillovers_paired_reds
            spillovers_paired_reds[direction] = a scalar. The simulated spillover duration. 
        -------------------------------------------------
        output: aggregation
            columns include 'left' 'through'
            index include 'objective', 'spillover duration'
        """
        output = {}
        for d in res_paired_red.keys():
            if math.isnan(res_paired_red[d]['optimal_Y']):
                output[d] = {'objective':np.nan, \
                        'optimalDuration':0,\
                        'SpilloverDuration':spillovers_paired_red[d]
                        }
            else:
                output[d] = {'objective':res_paired_red[d]['optimal_Y'], \
                        'optimalDuration':res_paired_red[d]['res'].x[0],\
                        'SpilloverDuration':spillovers_paired_red[d]
                        }
        return pd.DataFrame(output)

    @classmethod 
    def occurrence_paired_red_relative_threshold(self, opt_res_paired_red_corss, spillovers_paired_red,relative_threshold = 3):
        """

        If the (max(area))/min(area)>relative_threshold, then the TMS is considered. 

        """




        pass

    @classmethod
    def occurrence_effective_row_labels(self, aggregation_res_pd):
        """
        FIlter the effective rows. Uneffective means:
            If there is no trajectory, the optimization results (the column "optimized:th-th" and "optimized:le-le") will be null
            If there are trajectories and the pattern shows no TMS, then the (the column "optimized:th-th" and "optimized:le-le") will be nan 
        ------------------------------------------------------
        input: aggregation_res_pd
            the DataFrame type data. 
            inddex is "cycle_0", 'cycle_1'.....
            columns include:
                columnsTitles = ['benchamrk:through',  'benchamrk:left', 'optimized:th-th', 'optimized:le-le',  'optimized:th-le','optimized:le-th','Area:th-th', 'Area:le-le','Area:th-le',  'Area:le-th']
                "benchamrk:through" is the simulation results. 
        output: effective_types,
            a pd.Series data. index are 'cycle_0', ....
            Note that if certain cycle "cycle_i" is not effective, it will be deleted. 
        """
        def effective(data):
            #print(type(data))
            if data=='null' or data=='nan' or pd.isna(data):
            #if data=='null' or data=='nan' or (not isinstance(data, str) and np.isnan(data)):
                return 0
            else:
                return 1
        effective_types = pd.Series()
        for index in aggregation_res_pd.index:
            check = effective(aggregation_res_pd.loc[index, 'Area:th-th']) + effective(aggregation_res_pd.loc[index, 'Area:le-le'])
            if check==2:
                effective_types[index] = check
            elif check==1:
                if effective(aggregation_res_pd.loc[index, 'Area:th-th'])==1:
                    effective_types[index] = 'Area:th-th'
                else:
                    effective_types[index] = 'Area:le-le'
        return effective_types

    @classmethod
    def PI_duration(self, paired_reds,spillovers_paired_reds, aggregation_corss, threshold_LW = 1, abss = True):
        """
        alculate the duration performances
        Rold of paired_reds is to calculate the red duration
        -----------------------------------------------
        input: 
        """
        durations_accuracy = pd.DataFrame()

        #occurrence inferrences results. index are 'cycle_2',....means the 2nd cycle (coupling red duration)
        #   columns include ['Area:through', 'Area:left', 'Left TMS duration', 'Through TMS duration', 'TMS_ocurence_left', 'TMS_ocurence_through']
        #      occurrence_infer_res.loc['cycle_9','TMS_ocurence_through']=1 means we make a occurrence inferrences. 
        occurrence_infer_res = self.occurrence_paired_red_absolute_threshold_pd(aggregation_res_pd = aggregation_corss, threshold_LW=threshold_LW)

        #accuracy.loc['cycle_9','left'] =0 means correnct occurrence inference; ==1 means false negative; ==-1 means false positive. 
        #
        occurrence_accuracy = self.PI_occcurrence(occur_infer_res=occurrence_infer_res, spillovers_paired_reds =spillovers_paired_reds)
        
        for cycleidx in occurrence_accuracy.index:
            for d in occurrence_accuracy.columns:
                if occurrence_accuracy.loc[cycleidx, d]==0:
                    True_dura = spillovers_paired_reds[int(cycleidx[6:])][d]
                    if d=='left':
                        esti_dur = occurrence_infer_res.loc[cycleidx, 'left TMS duration']
                    else:
                        esti_dur = occurrence_infer_res.loc[cycleidx, 'through TMS duration']
                    #red duration
                    t0,t1,t2 = paired_reds[int(cycleidx[6:])][d]
                    r = t1-t0
                    if abss:
                        durations_accuracy.loc[cycleidx,d] = abs(True_dura - esti_dur)/r
                    else:
                        durations_accuracy.loc[cycleidx,d] = (True_dura - esti_dur)/r

        return durations_accuracy


    @classmethod
    def PI_duration0(self, paired_reds,spillovers_paired_reds, aggregation_corss, threshold_LW = 1, abss = True):
        """
        alculate the duration performances
        Rold of paired_reds is to calculate the red duration
        -----------------------------------------------
        input: 
        """
        errors = []
        #effective_types is a Series. 
        #   index are cycle_1, values are Area:th-th
        effective_types = self.occurrence_effective_row_labels(aggregation_corss)

        for cycleidx in aggregation_corss.index:
            #means not 'null' and not 'nan'
            if aggregation_corss.loc[cycleidx, 'optimized:th-th'][0]!='n':
                t0,t1,t2 = paired_reds[int(cycleidx[6:])]['through']
                r = t1-t0
                errors.append(abs(float(aggregation_corss.loc[cycleidx, 'optimized:th-th'])-float(aggregation_corss.loc[cycleidx, 'benchamrk:through']))/r)

            if aggregation_corss.loc[cycleidx, 'optimized:le-le'][0]!='n':
                t0,t1,t2 = paired_reds[int(cycleidx[6:])]['left']
                r = t1-t0
                errors.append(abs(float(aggregation_corss.loc[cycleidx, 'optimized:le-le'])-float(aggregation_corss.loc[cycleidx, 'benchamrk:left']))/r)


        return np.mean(errors)

    @classmethod
    def PI_cycle_occurrence_correctness(self, occurrencess_estimation_pd):
        """
        They are obtained via:
            occurrencess_estimation_pd = csswithsignal.occurrence_paired_red_absolute_threshold_pd(aggregation_corss_replaced)
            aggregation_corss = csswithsignal.aggregate_durations_cross(spillovers_paired_reds = partial_spillovers_paired_reds, \
                                        opt_res_paired_reds_corss = partial_optimization_results_cross)

        --------------------------------------------
        input: occurrencess_estimation_pd
            a pd.DataFrame. 
            index are 'cycle_0', 'cycle_1', etc
            columns are:
                Index([u'Area:through', u'Area:left', u'left TMS duration',
                       u'through TMS duration', u'TMS_ocurence_left', u'TMS_ocurence_through',
                       u'benchamrk:through', u'benchamrk:left'],
                      dtype='object')

        """
        res = pd.DataFrame(index = occurrencess_estimation_pd.index, columns = ['left_correct','through_correct'])
        for row_index in occurrencess_estimation_pd.index:
            
            ####################left
            if (occurrencess_estimation_pd.loc[row_index,'TMS_ocurence_left']==1 and occurrencess_estimation_pd.loc[row_index,'benchamrk:left']>0) or (occurrencess_estimation_pd.loc[row_index,'TMS_ocurence_left']==0.0 and occurrencess_estimation_pd.loc[row_index,'benchamrk:left']==0.0):
                res.loc[row_index, 'left_correct'] = 1
            elif (occurrencess_estimation_pd.loc[row_index,'TMS_ocurence_left']==1 and occurrencess_estimation_pd.loc[row_index,'benchamrk:left']==0) or (occurrencess_estimation_pd.loc[row_index,'TMS_ocurence_left']==0 and occurrencess_estimation_pd.loc[row_index,'benchamrk:left']>0):
                res.loc[row_index, 'left_correct'] = -1
            
            #################through
            if (occurrencess_estimation_pd.loc[row_index,'TMS_ocurence_through']==1 and occurrencess_estimation_pd.loc[row_index,'benchamrk:through']>0) or (occurrencess_estimation_pd.loc[row_index,'TMS_ocurence_through']==0.0 and occurrencess_estimation_pd.loc[row_index,'benchamrk:through']==0.0):
                res.loc[row_index, 'through_correct'] = 1
            elif (occurrencess_estimation_pd.loc[row_index,'TMS_ocurence_through']==1 and occurrencess_estimation_pd.loc[row_index,'benchamrk:through']==0) or (occurrencess_estimation_pd.loc[row_index,'TMS_ocurence_through']==0 and occurrencess_estimation_pd.loc[row_index,'benchamrk:through']>0):
                res.loc[row_index, 'through_correct'] = -1

        return res










        pass

    @classmethod
    def PI_occcurrence(self, occur_infer_res, spillovers_paired_reds):
        """

        ------------------------------------------------------
        input: 

        input: occur_infer_res, pd.DataFrame data. 
            columns include 'TMS_ocurence_left', 'TMS_ocurence_through', 
            occur_infer_res.loc['cycle_1', 'TMS_ocurence_left'] = 1 means a occurrence estimation.
            occur_infer_res.loc['cycle_1', 'TMS_ocurence_left'] = 0 means a non-occurrence estimation.
        input: spillovers_paired_reds, it is obtained from the SUMO simulation. 
            spillovers_paired_reds[cycle_idx]['left'] = 0 means no spillover 

        output: correctness
            correctness.loc['cycle_9', 'left'] = -1 means False positive
            correctness.loc['cycle_9', 'left'] = 0 means Success 
            correctness.loc['cycle_9', 'left'] = 1 means positive. 

        ---------------------------------------------------

        """
        correctness = pd.DataFrame()
        for cycleidx in occur_infer_res.index:
            #cycleidx = 'cycle_10', then int(cycleidx[6:]) is 10

            ################LEFT PI
            spillover_list_idx = int(cycleidx[6:])
            if (occur_infer_res.loc[cycleidx, 'TMS_ocurence_left']==1 and spillovers_paired_reds[spillover_list_idx]['left']>0) or (occur_infer_res.loc[cycleidx, 'TMS_ocurence_left']==0 and spillovers_paired_reds[spillover_list_idx]['left']==0):
                correctness.loc[cycleidx, 'left'] = 0
            elif occur_infer_res.loc[cycleidx, 'TMS_ocurence_left']==0 and spillovers_paired_reds[spillover_list_idx]['left']>0:
                correctness.loc[cycleidx, 'left'] = 1
            elif occur_infer_res.loc[cycleidx, 'TMS_ocurence_left']==1 and spillovers_paired_reds[spillover_list_idx]['left']==0:
                correctness.loc[cycleidx, 'left'] = -1

            ##################THROUGH
            spillover_list_idx = int(cycleidx[6:])
            if (occur_infer_res.loc[cycleidx, 'TMS_ocurence_through']==1 and spillovers_paired_reds[spillover_list_idx]['through']>0) or (occur_infer_res.loc[cycleidx, 'TMS_ocurence_through']==0 and spillovers_paired_reds[spillover_list_idx]['through']==0):
                correctness.loc[cycleidx, 'through'] = 0
            elif occur_infer_res.loc[cycleidx, 'TMS_ocurence_through']==0 and spillovers_paired_reds[spillover_list_idx]['through']>0:
                correctness.loc[cycleidx, 'through'] = 1
            elif occur_infer_res.loc[cycleidx, 'TMS_ocurence_through']==1 and spillovers_paired_reds[spillover_list_idx]['through']==0:
                correctness.loc[cycleidx, 'through'] = -1

        return correctness

    @classmethod
    def occurrence_paired_red_absolute_threshold_pd(self, aggregation_res_pd,threshold_LW = 1):
        """
        ------------------------------------------------------
        input: aggregation_res_pd
            the DataFrame type data. 
            inddex is "cycle_0", 'cycle_1'.....
            columns include:
                columnsTitles = ['benchamrk:through',  'benchamrk:left', 'optimized:th-th', 'optimized:le-le',  'optimized:th-le','optimized:le-th','Area:th-th', 'Area:le-le','Area:th-le',  'Area:le-th']
                "benchamrk:through" is the simulation results. 
        inut: threshold_LW
            the area threshold used to check the TMS occurrence, 
        output: occur_infer_res, pd.DataFrame data. 
            columns include ['Area:through', 'Area:left', 'TMS_ocurence_left', 'TMS_ocurence_through', 'TMS_duration_left', 'TMS_duration_through']

            occur_infer_res.loc['cycle_1', 'TMS_ocurence_left'] = 1 means a correct occurrence estimation.
        Steps;

        ----------------------------------------------
        Steps:
            For each cycle, find the types:
        """
        occur_infer_res = pd.DataFrame()

        #effective_types is a pd.Series, the values are only two cases:
        #   effective_types['cycle_idx'] = 2 means both turning directions can be optimized. 
        #   effective_types['cycle_idx'] = 'Area:le-le' means only left-turn area is obtained 
        effective_types = self.occurrence_effective_row_labels(aggregation_res_pd)
        builtins.tmp = effective_types

        for effective_idx in effective_types.index:
            if effective_types[effective_idx] == 2:
                #effective_types['cycle_idx'] == 2 means both turning directions are optimized
                #   and hence aggregation_res_pd.loc[effective_idx, 'Area:le-le'] and aggregation_res_pd.loc[effective_idx, 'Area:th-th'] exist and are str (converted dfrom float).
                occur_infer_res.loc[effective_idx, 'Area:through'] = float(aggregation_res_pd.loc[effective_idx, 'Area:th-th'])
                occur_infer_res.loc[effective_idx, 'Area:left'] = float(aggregation_res_pd.loc[effective_idx, 'Area:le-le'])
                occur_infer_res.loc[effective_idx, 'left TMS duration'] = float(aggregation_res_pd.loc[effective_idx, 'optimized:le-le'])
                occur_infer_res.loc[effective_idx, 'through TMS duration'] = float(aggregation_res_pd.loc[effective_idx, 'optimized:th-th'])

                if float(aggregation_res_pd.loc[effective_idx, 'Area:th-th'])<=threshold_LW and float(aggregation_res_pd.loc[effective_idx, 'Area:le-le'])<=threshold_LW:
                    occur_infer_res.loc[effective_idx, 'TMS_ocurence_left'] = 1
                    occur_infer_res.loc[effective_idx, 'TMS_ocurence_through'] = 1
                elif float(aggregation_res_pd.loc[effective_idx, 'Area:th-th'])<=threshold_LW and (aggregation_res_pd.loc[effective_idx, 'Area:le-le'])>threshold_LW:
                    occur_infer_res.loc[effective_idx, 'TMS_ocurence_left'] = 0
                    occur_infer_res.loc[effective_idx, 'TMS_ocurence_through'] = 1
                elif float(aggregation_res_pd.loc[effective_idx, 'Area:th-th'])>threshold_LW and float(aggregation_res_pd.loc[effective_idx, 'Area:le-le'])<=threshold_LW:
                    occur_infer_res.loc[effective_idx, 'TMS_ocurence_left'] = 1
                    occur_infer_res.loc[effective_idx, 'TMS_ocurence_through'] = 0
                elif float(aggregation_res_pd.loc[effective_idx, 'Area:th-th'])>threshold_LW and float(aggregation_res_pd.loc[effective_idx, 'Area:le-le'])>threshold_LW:
                    occur_infer_res.loc[effective_idx, 'TMS_ocurence_left'] = 1
                    occur_infer_res.loc[effective_idx, 'TMS_ocurence_through'] = 1
                #########corss compare
                elif float(aggregation_res_pd.loc[effective_idx, 'Area:th-le'])>threshold_LW and float(aggregation_res_pd.loc[effective_idx, 'Area:le-th'])>threshold_LW:
                    occur_infer_res.loc[effective_idx, 'TMS_ocurence_left'] = 1
                    occur_infer_res.loc[effective_idx, 'TMS_ocurence_through'] = 1
                elif  float(aggregation_res_pd.loc[effective_idx, 'Area:th-le'])>threshold_LW and float(aggregation_res_pd.loc[effective_idx, 'Area:le-th'])<threshold_LW:
                    occur_infer_res.loc[effective_idx, 'TMS_ocurence_left'] = 0
                    occur_infer_res.loc[effective_idx, 'TMS_ocurence_through'] = 1
                elif  float(aggregation_res_pd.loc[effective_idx, 'Area:th-le'])<=threshold_LW and float(aggregation_res_pd.loc[effective_idx, 'Area:le-th'])>threshold_LW:
                    occur_infer_res.loc[effective_idx, 'TMS_ocurence_left'] = 1
                    occur_infer_res.loc[effective_idx, 'TMS_ocurence_through'] = 0
                elif  float(aggregation_res_pd.loc[effective_idx, 'Area:th-le'])>threshold_LW and float(aggregation_res_pd.loc[effective_idx, 'Area:le-th'])>threshold_LW:
                    occur_infer_res.loc[effective_idx, 'TMS_ocurence_left'] = 0
                    occur_infer_res.loc[effective_idx, 'TMS_ocurence_through'] = 0


            else:
                #only one turning direction estimation is available. 
                #satistfy the requirement
                if aggregation_res_pd.loc[effective_idx, effective_types[effective_idx]]<=threshold_LW:
                    if 'le' in effective_types[effective_idx]:
                        occur_infer_res.loc[effective_idx, 'TMS_ocurence_left'] = 1
                        occur_infer_res.loc[effective_idx, 'Area:left'] = float(aggregation_res_pd.loc[effective_idx, 'Area:le-le'])
                        occur_infer_res.loc[effective_idx, 'Area:through'] = 'NAN'
                        occur_infer_res.loc[effective_idx, 'left TMS duration'] = float(aggregation_res_pd.loc[effective_idx, 'optimized:le-le'])
                    else:
                        occur_infer_res.loc[effective_idx, 'TMS_ocurence_through'] = 1
                        occur_infer_res.loc[effective_idx, 'Area:through'] = float(aggregation_res_pd.loc[effective_idx, 'Area:th-th'])
                        occur_infer_res.loc[effective_idx, 'Area:left'] = 'NAN'
                        occur_infer_res.loc[effective_idx, 'through TMS duration'] = float(aggregation_res_pd.loc[effective_idx, 'optimized:th-th'])
                else:
                    #@@@@@@@@@@@@@@@@@@@@@@
                    if 'le' in effective_types[effective_idx]:
                        occur_infer_res.loc[effective_idx, 'TMS_ocurence_through'] = 1
                        occur_infer_res.loc[effective_idx, 'Area:through'] = aggregation_res_pd.loc[effective_idx, 'Area:th-le']
                        occur_infer_res.loc[effective_idx, 'Area:through'] = 'NAN'
                        occur_infer_res.loc[effective_idx, 'through TMS duration'] = aggregation_res_pd.loc[effective_idx, 'optimized:th-le']
                    else:
                        occur_infer_res.loc[effective_idx, 'TMS_ocurence_left'] = 1
                        occur_infer_res.loc[effective_idx, 'Area:left'] = aggregation_res_pd.loc[effective_idx, 'Area:le-th']
                        occur_infer_res.loc[effective_idx, 'Area:through'] = 'NAN'
                        occur_infer_res.loc[effective_idx, 'left TMS duration'] = aggregation_res_pd.loc[effective_idx, 'optimized:le-th']

        occur_infer_res.loc[occur_infer_res.index, 'benchamrk:through'] = aggregation_res_pd.loc[occur_infer_res.index, 'benchamrk:through']
        occur_infer_res.loc[occur_infer_res.index, 'benchamrk:left'] = aggregation_res_pd.loc[occur_infer_res.index, 'benchamrk:left']
        return occur_infer_res

    @classmethod 
    def occurrence_paired_red_absolute_threshold(self, opt_res_paired_red, spillovers_paired_red,threshold_LW = 1):
        """
        Using absolute 
        if the area is smaller than threshold_LW, then TMS occurs. 
        -------------------------------------------------------------
        input: opt_res_paired_red
            opt_res_paired_red['left'] means the optimization results using the through left-turn trajectories which is sliced by left-turn signal. 
            If the TRs exhibit no delayyed pattern then the results are:
                {'X0':np.nan, 'res':{'x':[0]},'Y0':np.nan, 'cons':np.nan,'optimization_variables_sequence':np.nan, 'optimal_Y':np.nan}
            If no TRs avaailale, then it is: 
                {'X0':'null', 'res':{'x':['null']},'Y0':'null', 'cons':'null','optimization_variables_sequence':['null'], 'optimal_Y':'null'}

        input: threshold_LW
            the area threshold that used to check the occurrence of TMS. no unit
            if opt_res_paired_red_corss[singal_direction][turning_direction]['optimal_Y']<threshold_LW, then TMS of direction singal_direction occurs. 
        output: TMS, dict, the structrue is the same as opt_res_paired_red
            TMS is the duration of each direction. 
            TMS[d]=0 means

        """
        TMS = {}
        directions_set = set(opt_res_paired_red.keys())

        #check the availablility of the optimization results. 
        #   if only one turning direction is estimatied, 
        #   if it is estimated, then opt_res_paired_red[d]['res'][]
        N_opt_res = 0
        for d in opt_res_paired_red.keys():
            if isinstance(opt_res_paired_red[d]['res']['x'][0], str) or np.isnan(opt_res_paired_red[d]['res']['x'][0]):continue
            N_opt_res = N_opt_res+1
        
        if N_opt_res==0:
            for d in opt_res_paired_red.keys():TMS[d] = 0
        elif N_opt_res==1:
            #find the original direction for spillover, stored in original_tms_d
            for d in opt_res_paired_red.keys():
                if not (isinstance(opt_res_paired_red[d]['res']['x'][0], str) or np.isnan(opt_res_paired_red[d]['res']['x'][0])):
                   original_tms_d = d;break
            new_tms_d = list(directions_set.difference([original_tms_d]))[0]
            if opt_res_paired_red[original_tms_d]['res']['x'][0]<=threshold_LW:
                TMS[original_tms_d] = opt_res_paired_red[original_tms_d]['res']['x'][0]
                TMS[new_tms_d] = 0
            else:
                TMS[new_tms_d] = opt_res_paired_red[original_tms_d]['res']['x'][0]
                TMS[original_tms_d] = 0
        elif N_opt_res==2:
            #if both smaller than 
            pass




            pass

            pass





        pass


    @classmethod
    def TRs_paired_red_trim_samplesize(self, TRs_paired_red, UP_LIMIT = np.inf, shuffle = False):
        """
        Trim the Trajectories and make the trajectories sample size the same. 
        -----------------------------------------------------
        input: TRs_paired_red, a dict.
            keys() may empty, one key or two keys. The keys are the turning directions. including 'left', 'through'
            TRs_paired_red['left']are dict. 
            TRs_paired_red['left'][vehicle_id].keys() include:
                - 'tsxs':(ts,xs)
                - 't0t1t2':(t0,t1,t2)
                - 'intersectionpoint':(t_w, x_w)
                - 'redduration': redduration
                - 'twoboudnarys':((ta,xa),(tb,xb))
        """
        #total number of trajectories is zero, then return
        TRs_N = sum([len(TRs_paired_red[d]) for d in TRs_paired_red.keys()])
        if TRs_N==0:return TRs_paired_red

        #returned value.
        TRs_paired_red_trimed = {}

        #find the 
        common_n = UP_LIMIT
        for d in TRs_paired_red.keys():
            if len(TRs_paired_red[d])==0:continue
            common_n = min([len(TRs_paired_red[d]),common_n])

        for d in TRs_paired_red.keys():
            if len(TRs_paired_red[d])>common_n:
                TRs_paired_red_trimed[d] = {}
                vids = self.get_vids_t_w_ascending_TRs_dict(TRs_paired_red[d], shuffle=shuffle)
                for vid in vids[:common_n]:
                    TRs_paired_red_trimed[d][vid] = TRs_paired_red[d][vid]
            else:
                TRs_paired_red_trimed[d] = copy.deepcopy(TRs_paired_red[d])
        return TRs_paired_red_trimed


    @classmethod
    def TRs_paired_red_trim_samplesize_cross(self, TRs_paired_red_cross, UP_LIMIT = np.inf, shuffle= False):
        """
        Trim the Trajectories and make the trajectories sample size the same. 
        -----------------------------------------------------
        input: TRs_paired_red, a dict.
            keys() may empty, one key or two keys. The keys are the turning directions. including 'left', 'through'
            TRs_paired_red['left']['through']are dict, means the trajectories of through vehicles sliced by the left-turn vehicle. 
            TRs_paired_red['left']['through'][vehicle_id].keys() include:
                - 'tsxs':(ts,xs)
                - 't0t1t2':(t0,t1,t2)
                - 'intersectionpoint':(t_w, x_w)
                - 'redduration': redduration
                - 'twoboudnarys':((ta,xa),(tb,xb))
        input: UP_LIMIT
            the sample size for single direction of single cycle. 
        """
        #total number of trajectories is zero, then return
        TRs_N = 0
        for signal_d in TRs_paired_red_cross.keys():
            for turning_d in TRs_paired_red_cross[signal_d].keys():
                TRs_N = TRs_N + len(TRs_paired_red_cross[signal_d][turning_d])
        if TRs_N==0:return TRs_paired_red_cross

        #returned value.
        TRs_paired_red_trimed_cross = {}

        #find the common n
        common_n = UP_LIMIT
        for signal_d in TRs_paired_red_cross.keys():
            for turning_d in TRs_paired_red_cross[signal_d].keys():
                if len(TRs_paired_red_cross[signal_d][turning_d])>0:
                    #print(signal_d,turning_d,'len(TRs_paired_red_cross[signal_d][turning_d])=', len(TRs_paired_red_cross[signal_d][turning_d]))
                    common_n = min([len(TRs_paired_red_cross[signal_d][turning_d]),common_n])
        #print('**********************',common_n)

        for signal_d in TRs_paired_red_cross.keys():
            TRs_paired_red_trimed_cross[signal_d] = {}
            for turning_d in TRs_paired_red_cross[signal_d].keys():
                TRs_paired_red_trimed_cross[signal_d][turning_d] = {}
                if len(TRs_paired_red_cross[signal_d][turning_d])>common_n:
                    vids = self.get_vids_t_w_ascending_TRs_dict(TRs_paired_red_cross[signal_d][turning_d], shuffle=shuffle)
                    for vid in vids[:common_n]:
                        TRs_paired_red_trimed_cross[signal_d][turning_d][vid] = TRs_paired_red_cross[signal_d][turning_d][vid]
                else:
                    TRs_paired_red_trimed_cross[signal_d][turning_d] = TRs_paired_red_cross[signal_d][turning_d]

        return TRs_paired_red_trimed_cross


    @classmethod
    def TRs_paired_reds_trim_samplesize_cross(self, sliced_TRs_corss, UP_LIMIT = np.inf, shuffle=False):
        """
        Trim the Trajectories and make the trajectories sample size the same. 
        -----------------------------------------------------
        input: sliced_TRs_corss, a list. , a dict.
            = sliced_TRs_corss[cycle_idx]
            keys() may empty, one key or two keys. The keys are the turning directions. including 'left', 'through'
            TRs_paired_red['left']['through']are dict, means the trajectories of through vehicles sliced by the left-turn vehicle. 
            TRs_paired_red['left']['through'][vehicle_id].keys() include:
                - 'tsxs':(ts,xs)
                - 't0t1t2':(t0,t1,t2)
                - 'intersectionpoint':(t_w, x_w)
                - 'redduration': redduration
                - 'twoboudnarys':((ta,xa),(tb,xb))
        input: UP_LIMIT
            the sample size for single direction of single cycle. 
        """
        TRs_paired_reds_trimed_cross = [self.TRs_paired_red_trim_samplesize_cross(TRs_paired_red_trimed_cross, UP_LIMIT=UP_LIMIT, shuffle=shuffle) for TRs_paired_red_trimed_cross in sliced_TRs_corss]

        return TRs_paired_reds_trimed_cross

    @classmethod 
    def occurrence_paired_red_cross(self, opt_res_paired_red_corss, threshold_LW = 1):
        """
        find the triggering movements of the spillover.
        If the objective is smaller than threshold_LW, then it is considered as TMS. 
        -------------------------------------------------------------
        input: opt_res_paired_red_corss
            opt_res_paired_red_corss['left']['through'] means the optimization results using the through vehicles trajectories which is sliced by left-turn signal. 
            If the TRs exhibit no delayyed pattern then the results are:
                {'X0':np.nan, 'res':{'x':[0]},'Y0':np.nan, 'cons':np.nan,'optimization_variables_sequence':np.nan, 'optimal_Y':np.nan}


            opt_res_paired_red_corss[singal_direction][turning_direction] = {'X0':X0, 'res':res,'Y0':Y0, 'cons':cons,'optimization_variables_sequence':optimization_variables_sequence, 'optimal_Y':bestY}

            Thus opt_res_paired_red_corss[singal_direction][turning_direction]['optimal_Y'] give the objective function, and 
            opt_res_paired_red_corss[singal_direction][turning_direction]['res']['x'][0] gives the optimal TMS duration, unit is sec. 
        input: threshold_LW
            the threshold that used to check the occurrence of TMS. 
            if opt_res_paired_red_corss[singal_direction][turning_direction]['optimal_Y']<threshold_LW, then TMS of direction singal_direction occurs. 
        ---------------------------------------------------------------------

        """




        pass


    @classmethod
    def optimization_paired_red(self, TRs_paired_red_cross, kwargs,iterations = 10):
        """
        optimize the trajectorys using corss method, i.e.:
            using left TRs sliced by left signal to optimize
            using through TRs sliced by through signal to optimize
            using left TRs sliced by through signal to optimize
            using through TRs sliced by left signal to optimize
        -----------------------------------------------
        input: TRs_paired_red_cross
            a dict. TRs_paired_red_cross[signal_d][trajectory_d][vid] is a dict. 
        input: TRs_paired_red_cross.keys() = ['left','through']
            note that some turning directions have no 
                    keys are vehicle id.
                    dict. 
                    TR_info = TRs_dict[vid]
                    TR_info['tsxs']  = (ts,xs), 
                    TR_info['redduration']  =  redduration, a float;
                    TR_info['intersectionpoint']  =  (t_w, x_w), 
                    TR_info['t0t1t2']  = (t0,t1,t2), 
        input: kwargs
            loc_stopline = kwargs['loc_stopline']
            l_tb = kwargs['l_tb']
            w = kwargs['w']
            vf = kwargs['vf']
            direction_flag = kwargs['direction_flag']
            tolerance = kwargs['tolerance']
            method = kwargs['method']
        --------------------------------------------------
        output: opt_res_paired_red_corss
            opt_res_paired_red_corss[signal_d][trajectory_d] = {'X0':X0, 'res':res,'Y0':Y0, 'cons':cons,'optimization_variables_sequence':optimization_variables_sequence, 'optimal_Y':bestY}

            X0 is the initial solution
            Y0 is the initial objective
            cons are contreaints
            optimization_variables_sequence are list of vids
            optimal_Y is the bese value. 
            res is the result returned by scipy.minimize() function.
        ----------------------------------------------------------

        """
        opt_res_paired_red_corss = {}
        for signal_d in TRs_paired_red_cross.keys():
            opt_res_paired_red_corss[signal_d] = {}
            for trajectory_d in TRs_paired_red_cross[signal_d].keys():
                res = self.optimization_TRs(TRs_dict = TRs_paired_red_cross[signal_d][trajectory_d], kwargs = kwargs, iterations = iterations)
                opt_res_paired_red_corss[signal_d][trajectory_d] = res

        return opt_res_paired_red_corss


    @classmethod
    def optimization_paired_red_cross(self, TRs_paired_red_cross, kwargs,iterations = 10):
        """
        optimize the trajectorys using corss method, i.e.:
            using left TRs sliced by left signal to optimize
            using through TRs sliced by through signal to optimize
            using left TRs sliced by through signal to optimize
            using through TRs sliced by left signal to optimize
        -----------------------------------------------
        input: TRs_paired_red_cross
            a dict. TRs_paired_red_cross[signal_d][trajectory_d][vid] is a dict. 
            signal_d and trajectory-d include 'left' and 'through'.
            It means using the signal of signal_d to slice the trajectory belonging to the direction trajectory_d. 
                TRs_paired_red_cross[signal_d][trajectory_d][vid]['tsxs']  = (ts,xs), 
                TRs_paired_red_cross[signal_d][trajectory_d][vid]['redduration']  =  redduration, a float;
                TRs_paired_red_cross[signal_d][trajectory_d][vid]['intersectionpoint']  =  (t_w, x_w), 
                TRs_paired_red_cross[signal_d][trajectory_d][vid]['t0t1t2']  = (t0,t1,t2), 
        input: kwargs
            loc_stopline = kwargs['loc_stopline']
            l_tb = kwargs['l_tb']
            w = kwargs['w']
            vf = kwargs['vf']
            direction_flag = kwargs['direction_flag']
            tolerance = kwargs['tolerance']
            method = kwargs['method']
        --------------------------------------------------
        output: opt_res_paired_red_corss
            opt_res_paired_red_corss[signal_d][trajectory_d] = {'X0':X0, 'res':res,'Y0':Y0, 'cons':cons,'optimization_variables_sequence':optimization_variables_sequence, 'optimal_Y':bestY}

            X0 is the initial solution
            Y0 is the initial objective
            cons are contreaints
            optimization_variables_sequence are list of vids
            optimal_Y is the bese value. 
            res is the result returned by scipy.minimize() function.
        ----------------------------------------------------------

        """
        opt_res_paired_red_corss = {}
        for signal_d in TRs_paired_red_cross.keys():
            opt_res_paired_red_corss[signal_d] = {}
            for trajectory_d in TRs_paired_red_cross[signal_d].keys():
                res = self.optimization_TRs(TRs_dict = TRs_paired_red_cross[signal_d][trajectory_d], kwargs = kwargs, iterations = iterations)
                opt_res_paired_red_corss[signal_d][trajectory_d] = res

        return opt_res_paired_red_corss

    @classmethod
    def get_vids_t_w_ascending_TRs_dict(self, TRs_dict, shuffle = False):
        """

        """
        t_w_es = []
        vids = list(TRs_dict.keys())
        for vid in vids:t_w_es.append(TRs_dict[vid]['intersectionpoint'][0])
        if shuffle:
            np.random.shuffle(t_w_es)
            sorted_idxx = np.argsort(t_w_es)
        else:
            sorted_idxx = np.argsort(np.array(t_w_es))

        #
        return [vids[idx] for idx in sorted_idxx]


    @classmethod 
    def optimization_TRs(self, TRs_dict, kwargs, iterations = 10, post_adjust_tau = False):
        """
        different from self.optimization_TRs() in that, self.optimization_TRs_iters() will execute the optimization many times (specified by iterations). 

        If there is no trajectories available, then res['optimal_Y']='null'
        If the trajectory display no delay pattern, then then res['optimal_Y']=np.nan.

        ----------------------------------------------
        input: TRs_dict
            keys are vehicle id.
            dict. 
            TR_info = TRs_dict[vid]
            TR_info['tsxs']  = (ts,xs), 
            TR_info['redduration']  =  redduration, a float;
            TR_info['intersectionpoint']  =  (t_w, x_w), 
            TR_info['t0t1t2']  = (t0,t1,t2), 
        input: post_adjust_tau
            as there are many local minima of tau, i.e. many tau values corresponds to one objective function, thus the tau will be set to the minimal one if post_adjust_tau==True.
        output: 
            {'X0':X0, 'res':res,'Y0':Y0, 'cons':cons,'optimization_variables_sequence':optimization_variables_sequence, 'optimal_Y':bestY}

            selected from the best among all iterations. 
        ------------------------------------------
        Steps:  
            - get constraints and optimization_variable_sequence
            - get initial guess, i.e. X0
            - optimize
        """
        #no trajectories samples, then reuturn NULL
        if len(TRs_dict.keys())==0:
            return {'X0':'null', 'res':{'x':['null']},'Y0':'null', 'cons':'null','optimization_variables_sequence':['null'], 'optimal_Y':'null'}

        if not self.TRs_TMS_TMS_or_NOT(TRs_dict, vf = kwargs['vf'], delay_tolerance = kwargs['delay_check_threshold']):
            #print('No spillover. ')
            return {'X0':np.nan, 'res':{'x':[0]},'Y0':np.nan, 'cons':np.nan,'optimization_variables_sequence':[np.nan], 'optimal_Y':np.nan}

        #the optimization will begin from several initial points x0. 
        #   Ys store the optimal y and res store the whole optimization results. 
        Ys = []
        res = []
        for i in range(iterations):
            #res_temp = {'X0':X0, 'res':res,'Y0':Y0, 'cons':cons,'optimization_variables_sequence':optimization_variables_sequence, 'optimal_Y':bestY}
            res_temp = self.optimization_TRs_single_iter(TRs_dict = TRs_dict, kwargs = kwargs)
            Ys.append(res_temp['optimal_Y'])
            res.append(res_temp)
            #print('-------------------iter ',i,'in total iterations:',iterations,'-------samplesize=',len(TRs_dict))
        best_idx = Ys.index(min(Ys))
        #print Ys
        # res[best_idx] = {'X0':X0, 'res':res,'Y0':Y0, 'cons':cons,'optimization_variables_sequence':optimization_variables_sequence, 'optimal_Y':bestY}
        if post_adjust_tau:
            #adjust the tau, or just change the res[best_idx]['res'][x][0]
            return self.optimization_TRs_post_adjust_tau(TRs_dict = TRs_dict, opt_res = res[best_idx], kwargs=kwargs)
        else:
            return res[best_idx]


    @classmethod
    def optimization_TRs_post_adjust_tau(self, TRs_dict, opt_res, kwargs):
        """
        as there are many tau values correspond to the same objective (or area surrounded by the theoretical trajectory and the observed trajectory), it is reasonable to set the tau or TMS duration to the minimal one. 

        Thus this function only changes opt_res['res']['x'][0]
        --------------------------------------------------------------
        input: TRs_dict
            keys are vehicle id.
            dict. 
            TR_info = TRs_dict[vid]
            TR_info['tsxs']  = (ts,xs), 
            TR_info['redduration']  =  redduration, a float;
            TR_info['intersectionpoint']  =  (t_w, x_w), 
            TR_info['t0t1t2']  = (t0,t1,t2), 
        input: opt_res
            a dict. {'X0':X0, 'res':res,'Y0':Y0, 'cons':cons,'optimization_variables_sequence':optimization_variables_sequence, 'optimal_Y':bestY}

            opt_res['optimization_variables_sequence'] is a list of vehicle vids. 
            len(opt_res['res']['x']) = 1+ len(opt_res['optimization_variables_sequence'])

            opt_res['res']['x'][0] is the tau, or TMS turation. 

        """
        loc_stopline = kwargs['loc_stopline']
        l_tb = kwargs['l_tb']
        w = kwargs['w']
        vf = kwargs['vf']
        direction_flag = kwargs['direction_flag']
        tolerance = kwargs['tolerance']
        method = kwargs['method']


        opt_res1 = copy.deepcopy(opt_res)

        #TMS duration, sec
        tau = opt_res['res']['x'][0]
        candidates_tau_es = []
        #print opt_res['optimization_variables_sequence'], type(opt_res['optimization_variables_sequence']),opt_res['optimization_variables_sequence']==float('nan')
        #if np.isnan(opt_res['optimization_variables_sequence']):return opt_res
        for idx,vid in enumerate(opt_res['optimization_variables_sequence']):

            if not isinstance(vid, str):return opt_res
            if opt_res['res']['x'][0]=='null' or np.isnan(opt_res['res']['x'][0]):
                return opt_res
            vstop = opt_res['res']['x'][idx+1]#km/h
            if vstop>=vf:continue
            t_w,x_w = TRs_dict[vid]['intersectionpoint']
            t0,t1,t2 = TRs_dict[vid]['t0t1t2']

            #compute the stopping duration for the theoretical trajectory. 
            tmp = t_w -  (t1 + l_tb/(w/3.6) - tau + (abs(x_w - loc_stopline)-l_tb)/(vstop/3.6))
            #print 't_w=',t_w,'---', (t1 + l_tb/(w/3.6) - tau + (abs(x_w - loc_stopline)-l_tb)/(vstop/3.6))
            candidates_tau_es.append(max(0, min(tmp, t1-t0)))

        if len(candidates_tau_es)>0:
            opt_res1['res']['x'][0] = min(candidates_tau_es)
        return opt_res1

    @classmethod
    def optimization_TRs_cross_post_adjust_tau(self, TRs_dict_cross, opt_res, kwargs):
        """
        as there are many tau values correspond to the same objective (or area surrounded by the theoretical trajectory and the observed trajectory), it is reasonable to set the tau or TMS duration to the minimal one. 

        Thus this function only changes opt_res['res']['x'][0]
        --------------------------------------------------------------
        input: TRs_dict
            keys are vehicle id.
            dict. 
            TR_info = TRs_dict[vid]
            TR_info['tsxs']  = (ts,xs), 
            TR_info['redduration']  =  redduration, a float;
            TR_info['intersectionpoint']  =  (t_w, x_w), 
            TR_info['t0t1t2']  = (t0,t1,t2), 
        input: opt_res
            a dict. {'X0':X0, 'res':res,'Y0':Y0, 'cons':cons,'optimization_variables_sequence':optimization_variables_sequence, 'optimal_Y':bestY}

            opt_res['optimization_variables_sequence'] is a list of vehicle vids. 
            len(opt_res['res']['x']) = 1+ len(opt_res['optimization_variables_sequence'])

            opt_res['res']['x'][0] is the tau, or TMS turation. 

        """
        loc_stopline = kwargs['loc_stopline']
        l_tb = kwargs['l_tb']
        w = kwargs['w']
        vf = kwargs['vf']
        direction_flag = kwargs['direction_flag']
        tolerance = kwargs['tolerance']
        method = kwargs['method']


        opt_res1 = copy.deepcopy(opt_res)

        #TMS duration, sec
        tau = opt_res['res']['x'][0]
        candidates_tau_es = []
        #print opt_res['optimization_variables_sequence'], type(opt_res['optimization_variables_sequence']),opt_res['optimization_variables_sequence']==float('nan')
        #if np.isnan(opt_res['optimization_variables_sequence']):return opt_res
        for idx,vid in enumerate(opt_res['optimization_variables_sequence']):

            if not isinstance(vid, str):return opt_res

            vstop = opt_res['res']['x'][idx+1]#km/h
            if vstop>=vf:continue
            t_w,x_w = TRs_dict[vid]['intersectionpoint']
            t0,t1,t2 = TRs_dict[vid]['t0t1t2']

            #compute the stopping duration for the theoretical trajectory. 
            tmp = t_w -  (t1 + l_tb/(w/3.6) - tau + (abs(x_w - loc_stopline)-l_tb)/(vstop/3.6))
            #print 't_w=',t_w,'---', (t1 + l_tb/(w/3.6) - tau + (abs(x_w - loc_stopline)-l_tb)/(vstop/3.6))
            candidates_tau_es.append(max(0, min(tmp, t1-t0)))

        if len(candidates_tau_es)>0:
            opt_res1['res']['x'][0] = min(candidates_tau_es)
        return opt_res1

    @classmethod 
    def optimization_TRs_single_iter(self, TRs_dict, kwargs):
        """
        input: TRs_dict
            keys are vehicle id.
            dict. 
            TR_info = TRs_dict[vid]
            TR_info['tsxs']  = (ts,xs), 
            TR_info['redduration']  =  redduration, a float;
            TR_info['intersectionpoint']  =  (t_w, x_w), 
            TR_info['t0t1t2']  = (t0,t1,t2), 
        ------------------------------------------
        Steps:  
            - get constraints and optimization_variable_sequence
            - get initial guess, i.e. X0
            - optimize
        """
        loc_stopline = kwargs['loc_stopline']
        l_tb = kwargs['l_tb']
        w = kwargs['w']
        vf = kwargs['vf']
        direction_flag = kwargs['direction_flag']
        tolerance = kwargs['tolerance']
        method = kwargs['method']

        #constraints. 
        #   cons is tuple, optimization_variables_sequence is a list
        cons, optimization_variables_sequence = self.constraints(TRs_dict = TRs_dict, kwargs=kwargs)
        X0 = self.initial_guess_vstop(TRs_dict = TRs_dict, constraints=cons,optimization_variables_sequence = optimization_variables_sequence , kwargs=kwargs)

        #If there is no feasible initial guess even tried many times. 
        if X0=='no_feasible_initial_guess':
            return {'X0':'null', 'res':{'x':['null']},'Y0':'null', 'cons':'null','optimization_variables_sequence':['null'], 'optimal_Y':'null'}


        Y0 = self.objective_TRs(X=X0, TRs_dict=TRs_dict,optimization_variables_sequence=optimization_variables_sequence,  kwargs=kwargs)

        #optimization

        #   self.objective_TRs(self, X, TRs_dict, optimization_variables_sequence, kwargs)
        args = (TRs_dict, optimization_variables_sequence, kwargs)
        res = scipy.optimize.minimize(fun = self.objective_TRs, x0=X0,constraints=cons, args = args, method = method)

        bestY = self.objective_TRs(X=res.x, TRs_dict=TRs_dict,optimization_variables_sequence=optimization_variables_sequence,  kwargs=kwargs)

        return {'X0':X0, 'res':res,'Y0':Y0, 'cons':cons,'optimization_variables_sequence':optimization_variables_sequence, 'optimal_Y':bestY}

    @classmethod
    def optimization(self, redduration, cycle_sliceddata_turning, loc_stopline, l_tb, w=FD.w, vf=FD.vf, post_constraint_check = True):
        """
        the total process of optimziation of the css. 
        The estimator estimate the spillover duration of one cycle, using trajectorys of ONLY one turning movement (i.e. left vehicles trajectories. )
        -------------------------------------------------------
        input: cycle_sliceddata_turning
            the trajectory info for each cycle. cycle_sliceddata_turning is a dict, keys are vehicle_id. 
            cycle_sliceddata_turning[vehicle_id].keys()=['tsxs', 'intersectionpoint', 'twoboudnarys', 'redduration', 'cyclestartmoment']

            cycle_sliceddata_turning[vehicle_id]['tsxs']  =(ts,xs)
            cycle_sliceddata_turning[vehicle_id]['intersectionpoint'] = (t_w,x_w)
            cycle_sliceddata_turning[vehicle_id]['twoboudnarys'] = ((t0,x0),(t2,x2))
            cycle_sliceddata_turning[vehicle_id]['redduration'] = r
            cycle_sliceddata_turning[vehicle_id]['cyclestartmoment'] = cyclestartmoment
        input: redduration
            the red duration of this cycle. Used to set the constraints. 
        input: loc_stopline, l_tb
            loc_stopline is the location of the stopline. l_tb is the length of the turning bay area. 
        input: post_constraint_check
            whether check the constraint after the optimization

        """
        #obtain the constraints and the optimization variables sequences. 
        #   constraints_turning is a tuple
        #   optimization_variables_sequence_turning is a list, containing the vehicle_id. 
        #       the order specify the variable sequences
        constraints_turning,optimization_variables_sequence_turning =self.constraints_turning(cycle_sliceddata_turning=cycle_sliceddata_turning,redduration=redduration, loc_stopline=loc_stopline, l_tb=l_tb, w = w)
        
        #initial guess
        X0 = self.initial_guess(redduration=redduration, cycle_sliceddata_turning=cycle_sliceddata_turning,constraints=constraints_turning, optimization_variables_sequence=optimization_variables_sequence_turning,l_tb=l_tb, loc_stopline=loc_stopline, w= w)

        #
        if not self.constraints_satistied(cons = constraints_turning,X=X0):
            builtins.tmp ={}
            builtins.tmp['X0']=X0;builtins.tmp['cons']=constraints_turning;
            raise ValueError('Constraints not satisfied. ')
        
        #args in the scipy.optimization
        #   the objective function is as follows:
        #       objective_one_turning(self, X, redduration,cycle_sliceddata_turning, \
        #       optimization_variables_sequence, loc_stopline, l_tb, w=FD.w, vf = FD.vf)
        args = (redduration,cycle_sliceddata_turning, optimization_variables_sequence_turning,loc_stopline,l_tb, w, vf)

        #res is the scipy.optimize.optimize.OptimizeResult instance 
        #   res['x'] gives the solution.
        #   res['fun'] gives the objective function. 
        res = scipy.optimize.minimize(fun = self.objective_one_turning, x0=X0,constraints=constraints_turning,args = args,method = 'COBYLA')
        
        #
        if post_constraint_check and (not self.constraints_satistied(cons = constraints_turning,X=res['x'])):
            builtins.tmp ={}
            builtins.tmp['X0']=res['x'];builtins.tmp['cons']=constraints_turning;
            raise ValueError('Constraints not satisfied. ')
        
        return X0,res,constraints_turning,optimization_variables_sequence_turning

    @classmethod
    def constraint_satisfied_one_TR(self, cons, X):
        """
        -------------------------------
        input: cons
            a list. Each element is like 
                cons[i] =  {'type': 'ineq', 'fun': lambda x:  x[0]}
        input: X
            a list. X[0] is the tau, or TMS duration;
            X[1] is the stopping wave, unit is km/h.
        """
        for con in cons:
            if con['fun'](X)<0:
                #print con
                return False
        return True

        pass

    @classmethod
    def constraint_one_TR_vstop(self, TR_info,  loc_stopline, l_tb, w = FD.w, vf = FD.vf, direction_flag = 1, ufi=  FD.vf, tolerance = 1):
        """
        constraints for only one TR. 
        returned value is cons, a list. 
        Each element is a dict and like:
            cons[i] =  {'type': 'ineq', 'fun': lambda x:  x[0]}
        'ineq' specify the type of the constraints.

        Note that vstop unit is km/h
        ----------------------------------------------
        input: TR_info
            dict. 
            TR_info['tsxs']  = (ts,xs), 
            TR_info['redduration']  =  redduration, a float;
            TR_info['intersectionpoint']  =  (t_w, x_w), 
            TR_info['t0t1t2']  = (t0,t1,t2), 
        OUTPUT: cons, optimization_variables_sequence
            cons is a dict. 
            cons[i] =  {'type': 'ineq', 'fun': lambda x:  x[0]}

            it means that the constraint is x[0]>0
        """

        cons = []
        ts,xs = TR_info['tsxs']
        t_w, x_w = TR_info['intersectionpoint']
        redduration = TR_info['redduration']

        #unit is sec
        tau = np.random.uniform( redduration)

        #constraint of TMS duration
        con_tau_LW = {'type': 'ineq', 'fun': lambda X:  X[0]}
        #   note that constraint_buffer is used to avoid the overflow of red duration.
        con_tau_UP = {'type': 'ineq', 'fun': lambda X:  redduration-X[0]}
        cons.append(con_tau_LW)
        cons.append(con_tau_UP)

        #unit is km
        con_vstop_UP = {'type': 'ineq', 'fun': lambda X:  w-tolerance-X[1]}
        cons.append(con_vstop_UP)

        #constraint of the vstop
        tmp_distance = abs(x_w - loc_stopline)-l_tb
        tmp_time = tmp_distance/(w/3.6)
        con_vstop_LW = {'type': 'ineq', 'fun': lambda X:  X[1]-tmp_distance/(X[0]+tmp_time)}
        cons.append(con_vstop_LW)

        return tuple(cons)

    @classmethod
    def constraints_turning_vstop(self,cycle_sliceddata_turning, redduration, loc_stopline, l_tb, w = FD.w, constraint_buffer = .05, direction_flag = 1):
        """
        different from self.constraints_turning_phi(), this function is for vstop, and vstop unit is km/h
        return the constraints as a dict. 
            cons[i] =  {'type': 'ineq', 'fun': lambda x:  x[0]}
        'ineq' specify the type of the constraints.

        different from self.constraints() in that, the self.constraints_turning() only considers the trajectories of a specific turning. 
        NOTE that the constraints are based on phi, not stopping wave. 

        The decision variable include:
            - the tau, which is the spillover duration
            - the phi, each trajectory have a phi
        There are the following constraints:
            - tau is in [0, r]
        Constraints include
            - tau >0 
            - tau < redduration
            - each phi > (x_w - l_tb)/(w/3.6)
            - each phi: tau+(x_w - l_tb)/(w/3.6)>= phi
            - any two trajectories: phi_j-phi_i>(x_w_j - x_w_i)/(w/3.6)
        Thus number of consraints are 2+N+N+combinations(N,2)
        ------------------------------------
        input: cycle_sliceddata_turning
            the trajectory info for each cycle. cycle_sliceddata_turning is a dict, keys are vehicle_id. 
            cycle_sliceddata_turning[vehicle_id].keys()=['tsxs', 'intersectionpoint', 'twoboudnarys', 'redduration', 'cyclestartmoment']

            cycle_sliceddata_turning[vehicle_id]['tsxs']  =(ts,xs)
            cycle_sliceddata_turning[vehicle_id]['intersectionpoint'] = (t_w,x_w)
            cycle_sliceddata_turning[vehicle_id]['twoboudnarys'] = ((t0,x0),(t2,x2))
            cycle_sliceddata_turning[vehicle_id]['redduration'] = r
            cycle_sliceddata_turning[vehicle_id]['cyclestartmoment'] = cyclestartmoment
        input: direction_flag
            either 1 or -1. If the coordinate increase to upstream, then it is 1. 
        input: r
            the red duration
        input: w
            the backward wave speed.
        input: loc_stopline
            the location of the stopline. x-loc_stopline is the distance of the point from the stopline.
        input: l_tb
            the length of turning bay area.
        input: constraint_buffer
            used to add the constraints,  to avoid the infeasibility of the solution.
        OUTPUT: cons, optimization_variables_sequence
            cons is a dict. 
            cons[i] =  {'type': 'ineq', 'fun': lambda x:  x[0]}

            it means that the constraint is x[0]>0
        """
        cons = []
        #optimization_variables_sequence[i] = (vehicle_id')
        #   the optimization_variables_sequence specifies the order of variable in x.
        optimization_variables_sequence = sorted(cycle_sliceddata_turning.keys())

        #constraint of tau
        con_tau_LW = {'type': 'ineq', 'fun': lambda X:  X[0]}
        #   note that constraint_buffer is used to avoid the overflow of red duration.
        con_tau_UP = {'type': 'ineq', 'fun': lambda X:  max(0,redduration-constraint_buffer)-X[0]}
        cons.append(con_tau_LW)
        cons.append(con_tau_UP)

        #single phi constraint
        for i,v_id in enumerate(optimization_variables_sequence):
            variable_idx = i+1
            t_w, x_w = cycle_sliceddata_turning[v_id]['intersectionpoint']
            #vstop<=w
            cons.append({'type': 'ineq', 'fun': lambda X: w-X[variable_idx],'vid_phi_big_than':v_id})
            #vstop>=tmp_distance/(X[0]+tmp_time)
            tmp_distance = abs(t_w - loc_stopline)-l_tb#unit is m.
            tmp_time = tmp_distance/(w/3.6)#unit is sec.
            cons.append({'type': 'ineq', 'fun': lambda X: X[variable_idx]/3.6- tmp_distance/(tmp_time+X[0]),'vid_phi_small_than':v_id})
        
        #more then two trajectories, there are extra constraints of stopping wave. 
        #   slope of VECTOR difference is in [0,w]
        if len(optimization_variables_sequence)>1:
            for i,j in itertools.combinations(range(len(optimization_variables_sequence)),2):
                v_id_i = optimization_variables_sequence[i]
                v_id_j = optimization_variables_sequence[j]
                t_w_i, x_w_i = cycle_sliceddata_turning[v_id_i]['intersectionpoint']
                t_w_j, x_w_j = cycle_sliceddata_turning[v_id_j]['intersectionpoint']
                delta_x = x_w_j - x_w_i
                i=i+1;j=j+1#because X[0] is tau.

                #constraint of slope of VECTOR difference>0
                #   VECTOR difference t = tmp_tj-tmp_ti
                #   tmp_tj = (abs(x_w_j-loc_stopline)-l_tb)/(X[j]/3.6)
                #   tmp_ti = (abs(x_w_j-loc_stopline)-l_tb)/(X[i]/3.6)
                cons.append({'type': 'ineq', 'fun': lambda X: delta_x/((abs(x_w_j-loc_stopline)-l_tb)/(X[j]/3.6) - (abs(x_w_j-loc_stopline)-l_tb)/(X[i]/3.6)),'pair_phis':v_id_i + ':'+v_id_j})
                #constraint that slope of VECTOR difference<w
                cons.append({'type': 'ineq', 'fun': lambda X: w/3.6-delta_x/((abs(x_w_j-loc_stopline)-l_tb)/(X[j]/3.6) - (abs(x_w_j-loc_stopline)-l_tb)/(X[i]/3.6)),'pair_phis':v_id_i + ':'+v_id_j})
        return tuple(cons),optimization_variables_sequence

    @classmethod
    def constraints_turning_phi(self,cycle_sliceddata_turning, redduration, loc_stopline, l_tb, w = FD.w, constraint_buffer = .5, direction_flag = 1):
        """
        return the constraints as a dict. 
            cons[i] =  {'type': 'ineq', 'fun': lambda x:  x[0]}
        'ineq' specify the type of the constraints.

        different from self.constraints() in that, the self.constraints_turning() only considers the trajectories of a specific turning. 
        NOTE that the constraints are based on phi, not stopping wave. 

        The decision variable include:
            - the tau, which is the spillover duration
            - the phi, each trajectory have a phi
        There are the following constraints:
            - tau is in [0, r]
        Constraints include
            - tau >0 
            - tau < redduration
            - each phi > (x_w - l_tb)/(w/3.6)
            - each phi: tau+(x_w - l_tb)/(w/3.6)>= phi
            - any two trajectories: phi_j-phi_i>(x_w_j - x_w_i)/(w/3.6)
        Thus number of consraints are 2+N+N+combinations(N,2)
        ------------------------------------
        input: cycle_sliceddata_turning
            the trajectory info for each cycle. cycle_sliceddata_turning is a dict, keys are vehicle_id. 
            cycle_sliceddata_turning[vehicle_id].keys()=['tsxs', 'intersectionpoint', 'twoboudnarys', 'redduration', 'cyclestartmoment']

            cycle_sliceddata_turning[vehicle_id]['tsxs']  =(ts,xs)
            cycle_sliceddata_turning[vehicle_id]['intersectionpoint'] = (t_w,x_w)
            cycle_sliceddata_turning[vehicle_id]['twoboudnarys'] = ((t0,x0),(t2,x2))
            cycle_sliceddata_turning[vehicle_id]['redduration'] = r
            cycle_sliceddata_turning[vehicle_id]['cyclestartmoment'] = cyclestartmoment
        input: direction_flag
            either 1 or -1. If the coordinate increase to upstream, then it is 1. 
        input: r
            the red duration
        input: w
            the backward wave speed.
        input: loc_stopline
            the location of the stopline. x-loc_stopline is the distance of the point from the stopline.
        input: l_tb
            the length of turning bay area.
        input: constraint_buffer
            used to add the constraints,  to avoid the infeasibility of the solution.
        OUTPUT: cons, optimization_variables_sequence
            cons is a dict. 
            cons[i] =  {'type': 'ineq', 'fun': lambda x:  x[0]}

            it means that the constraint is x[0]>0
        """
        cons = []
        optimization_variables_sequence = []
        #constraint of tau
        con_tau_LW = {'type': 'ineq', 'fun': lambda X:  X[0]}
        #   note that constraint_buffer is used to avoid the overflow of red duration.
        con_tau_UP = {'type': 'ineq', 'fun': lambda X:  max(0,redduration-constraint_buffer)-X[0]}
        cons.append(con_tau_LW)
        cons.append(con_tau_UP)

        #constraints of the stopping waves or phi
        #   optimization_variables_sequence[i] = (vehicle_id')
        #   the optimization_variables_sequence specifies the order of variable in x.
        optimization_variables_sequence = sorted(cycle_sliceddata_turning.keys())

        #single phi constraint
        for i,v_id in enumerate(optimization_variables_sequence):
            variable_idx = i+1
            t_w, x_w = cycle_sliceddata_turning[v_id]['intersectionpoint']
            #phi>(x_w -loc_stopline- l_tb)/(w/3.6)
            cons.append({'type': 'ineq', 'fun': lambda X:  X[variable_idx]-(abs(x_w -loc_stopline)- l_tb)/(w/3.6),'vid_phi_big_than':v_id})
            #phi< tau + (x_w -loc_stopline- l_tb)/(w/3.6)
            cons.append({'type': 'ineq', 'fun': lambda X:  X[0]+(abs(x_w -loc_stopline)- l_tb)/(w/3.6)-X[variable_idx],'vid_phi_small_than':v_id})
        
        #more then two trajectories, there are extra constraints of stopping wave. 
        if len(optimization_variables_sequence)>1:
            for i,j in itertools.combinations(range(len(optimization_variables_sequence)),2):
                v_id_i = optimization_variables_sequence[i]
                v_id_j = optimization_variables_sequence[j]
                _, x_w_i = cycle_sliceddata_turning[v_id_i]['intersectionpoint']
                _, x_w_j = cycle_sliceddata_turning[v_id_j]['intersectionpoint']
                i=i+1;j=j+1
                cons.append({'type': 'ineq', 'fun': lambda X: X[j]-X[i]-(x_w_j-x_w_i)/(w/3.6),'pair_phis':v_id_i + ':'+v_id_j})
        return tuple(cons),optimization_variables_sequence

    @classmethod
    def kwargs_parser(self, kwargs):
        """

        """
        loc_stopline = kwargs['loc_stopline']
        l_tb = kwargs['l_tb']
        w = kwargs['w']
        vf = kwargs['vf']
        direction_flag = kwargs['direction_flag']
        tolerance = kwargs['tolerance']
        return loc_stopline,l_tb,w,vf,direction_flag

    @classmethod
    def constraints(self, TRs_dict, kwargs):
        """
        return the constraints as a dict. The decitin variable is X:
            X[0] is tau, i.e. TMS duration
            X[1:] are vstops, unit is km/h.
        Thus if there is N trajectories, the X should be len(X)=N+1.
        The constraint number include
            - 2, 0<TMS duration<r
            - 2*N, LW<vstop<w
            - C(N,2): difference vstop. 
        The decition variable include:
            - the tau, which is the spillover duration
            - the phi, each trajectory have a phi
        There are the following constraints:
            - tau is in [0, r]
        ------------------------------------
        input: TRs_dict
            keys are vids
            TRs_dict[vehicle_id]['tsxs']  =(ts,xs)
            TRs_dict[vehicle_id]['intersectionpoint'] = (t_w,x_w)
            TRs_dict[vehicle_id]['t0t1t2'] = (t0,t1,t2), red onset, red end, green end.
        input: w
            the backward wave speed.
        input: loc_stopline
            the location of the stopline. x-loc_stopline is the distance of the point from the stopline.
        input: l_tb
            the length of turning bay area.
        input:direction_flag, either 1 or -1
            if the coordinate increase with upstream, then it is 1. else it is -1
        OUTPUT: cons, optimization_variables_sequence
            cons is a dict. 
            cons[i] =  {'type': 'ineq', 'fun': lambda x:  x[0]}

            it means that the constraint is x[0]>0
        """
        loc_stopline = kwargs['loc_stopline']
        l_tb = kwargs['l_tb']
        w = kwargs['w']
        vf = kwargs['vf']
        direction_flag = kwargs['direction_flag']

        #compute the redduration
        somevid = TRs_dict.keys()[0]
        redduration = TRs_dict[somevid]['t0t1t2'][1] - TRs_dict[somevid]['t0t1t2'][0]

        cons = []
        optimization_variables_sequence = []
        #constraint of tau
        con_tau_LW = {'type': 'ineq', 'fun': lambda X:  X[0]}
        con_tau_UP = {'type': 'ineq', 'fun': lambda X:  redduration-X[0]}
        cons.append(con_tau_LW)
        cons.append(con_tau_UP)

        #constraints of the stopping waves or phi
        #   optimization_variables_sequence[i] = 'vehicle_id'
        #   the optimization_variables_sequence specifies the order of variable in x.
        v_ids = sorted(TRs_dict.keys())
        for v_id in v_ids:optimization_variables_sequence.append(v_id)

        #vstop constaints. 
        for idx,v_id in enumerate(optimization_variables_sequence):
            t_w, x_w = TRs_dict[v_id]['intersectionpoint']
            #vstop<w
            cons.append({'type': 'ineq', 'fun': lambda X:  w - X[idx+1]})
            #vstop>something
            #   (abs(x_w - loc_stopline)-l_tb)/(w/3.6)
            cons.append({'type': 'ineq', 'fun': lambda X: X[idx+1] - 3.6*(abs(x_w - loc_stopline)-l_tb)/((abs(x_w - loc_stopline)-l_tb)/(w/3.6) + X[0])})
        
        #more then two trajectories, there are extra constraints of stopping wave. 
        if len(optimization_variables_sequence)>1:
            #  
            for i,j in itertools.combinations(range(len(optimization_variables_sequence)),2):
                v_id_i = optimization_variables_sequence[i]
                v_id_j = optimization_variables_sequence[j]
                _, x_w_i = TRs_dict[v_id_i]['intersectionpoint']
                _, x_w_j = TRs_dict[v_id_j]['intersectionpoint']
                delta_x = x_w_j-x_w_i
                #   delta_t = (abs(x_w_j-loc_stopline)-l_tb)/(X[j+1]/3.6) - (abs(x_w_i-loc_stopline)-l_tb)/(X[i+1]/3.6)
                #   difference vstop>0 constraint.
                cons.append({'type': 'ineq', 'fun': lambda X: ((abs(x_w_j-loc_stopline)-l_tb)/(X[j+1]/3.6) - (abs(x_w_i-loc_stopline)-l_tb)/(X[i+1]/3.6))/delta_x})
                #   difference vstop<w constaint. 
                cons.append({'type': 'ineq', 'fun': lambda X: w/3.6-((abs(x_w_j-loc_stopline)-l_tb)/(X[j+1]/3.6) - (abs(x_w_i-loc_stopline)-l_tb)/(X[i+1]/3.6))/delta_x})
        return tuple(cons),optimization_variables_sequence

    @classmethod
    def constraints_all_turning(self, cycle_sliceddata, redduration,  l_tb, w = FD.w, direction_flag = 1):
        """
        return the constraints as a dict. 
        The decition variable include:
            - the tau, which is the spillover duration
            - the phi, each trajectory have a phi
        There are the following constraints:
            - tau is in [0, r]
        ------------------------------------
        input: cycle_sliceddata
            the trajectory info for each cycle. cycle_sliceddata is a dict, keys include 'l', 't' and 'r'. 
            cycle_sliceddata['l'][vehicle_id].keys()=['tsxs', 'intersectionpoint', 'twoboudnarys', 'redduration', 'cyclestartmoment']

            cycle_sliceddata['l'][vehicle_id]['tsxs']  =(ts,xs)
            cycle_sliceddata['l'][vehicle_id]['intersectionpoint'] = (t_w,x_w)
            cycle_sliceddata['l'][vehicle_id]['twoboudnarys'] = ((t0,x0),(t2,x2))
            cycle_sliceddata['l'][vehicle_id]['redduration'] = r
            cycle_sliceddata['l'][vehicle_id]['cyclestartmoment'] = cyclestartmoment
        input: r
            the red duration
        input: w
            the backward wave speed.
        input: loc_stopline
            the location of the stopline. x-loc_stopline is the distance of the point from the stopline.
        input: l_tb
            the length of turning bay area.
        input:direction_flag, either 1 or -1
            if the coordinate increase with upstream, then it is 1. else it is -1
        OUTPUT: cons, optimization_variables_sequence
            cons is a dict. 
            cons[i] =  {'type': 'ineq', 'fun': lambda x:  x[0]}

            it means that the constraint is x[0]>0
        """
        #compute the redduration

        cons = []
        optimization_variables_sequence = []
        #constraint of tau
        con_tau_LW = {'type': 'ineq', 'fun': lambda X:  X[0]}
        con_tau_UP = {'type': 'ineq', 'fun': lambda X:  redduration-X[0]}
        cons.append(con_tau_LW)
        cons.append(con_tau_UP)

        #constraints of the stopping waves or phi
        #   optimization_variables_sequence[i] = ('l','vehicle_id')
        #   the optimization_variables_sequence specifies the order of variable in x.
        tds = ['l','t', 'r']#turning directions
        for td in tds:
            if cycle_sliceddata.has_key(td):
                #optimization variable sequence
                v_ids = sorted(cycle_sliceddata[td].keys())
                for v_id in v_ids:optimization_variables_sequence.append((td,v_id))
        #single phi constraint
        variable_idx = 1
        for td,v_id in optimization_variables_sequence:
            t_w, x_w = cycle_sliceddata[td][v_id]['intersectionpoint']
            cons.append({'type': 'ineq', 'fun': lambda X:  X[variable_idx]-(x_w -loc_stopline- l_tb)/(w/3.6)})
            #phi< tau + (x_w -loc_stopline- l_tb)/(w/3.6)
            cons.append({'type': 'ineq', 'fun': lambda X:  X[0]+(x_w -loc_stopline- l_tb)/(w/3.6)-X[variable_idx]})

            variable_idx = variable_idx + 1
        
        #more then two trajectories, there are extra constraints of stopping wave. 
        if len(optimization_variables_sequence)>1:
            for i,j in itertools.combinations(range(len(optimization_variables_sequence)),2):
                td_i, v_id_i = optimization_variables_sequence[i]
                td_j, v_id_j = optimization_variables_sequence[j]
                _, x_w_i = cycle_sliceddata[td_i][v_id_i]['intersectionpoint']
                _, x_w_j = cycle_sliceddata[td_j][v_id_j]['intersectionpoint']
                i=i+1;j=j+1
                cons.append({'type': 'ineq', 'fun': lambda X: X[j]-X[i]-(x_w_j-x_w_i)/(w/3.6) })
        return tuple(cons),optimization_variables_sequence
        


    @classmethod
    def constraints_satistied(self, cons, X, tolerance = 1):
        """
        
        check whether the constraints are satisfied.
        ------------------------------------------
        input: cons
            a tuple, the constraints. 
            cons[i].keys() = ['fun', 'type']
            It is generated by self.constraints_turning().
        input: X
            the decision variables. X[0] is the tau, the remaining are the phi, which is converted from stoppingwave speed by phi = (x_w - l_tb)/v_stop
        OUTPUT: 
            True or False. All constraints are satisfied, then return True, else False
        -----------------------------------------
        """

        #
        for con in cons:
            if con['fun'](X)<-tolerance:
                #print con
                return False
        return True

    @classmethod
    def Area_CMPF(self, ufi, t_w, x_w, t_UP, x_UP, w):
        """
        
        -------------------------------------
        input: ufi, unit is km/h
            the assumed vehicle speed after the spillover.
        input: t_w, x_w, t_UP, x_UP
            points on the trajectory. 
            t_w and x_w are the intersection points by the trajectory and 
        """
        w=w/3.6
        ufi = ufi/3.6

        #the hgight of the trapezoid.
        translated_t_UP = self.translate_point_t(t = t_UP, x = x_UP, sin_theta = self.sin_theta, cos_theta = self.cos_theta)
        translated_t_w = self.translate_point_t(t = t_w, x = x_w, sin_theta = self.sin_theta, cos_theta = self.cos_theta)
        TrapezoidHeight = translated_t_UP-translated_t_w

        #the spatial coordinate of point C
        
        width1 = self.translate_point_x(t = t_w, x=x_w, sin_theta=self.sin_theta,cos_theta=self.cos_theta)
        width2 = (w-ufi)/(1+ufi*w)*translated_t_UP + (ufi*t_w+x_w)*(np.sqrt((w)**2+1))/(1+ufi*w)
        
        return (width1+width2)*TrapezoidHeight/2.0

    @classmethod
    def signal_3moments(self, reds):
        """
        Get three moments of one signal. THey are 
            onset moment of cycle (or equivalently onset moment of red)
            onset moment of green (or equivalently the red termination)
            termination of green (or equivalently the cycle termination)
        ------------------------------------------
        input: reds
            a list. 
            reds[i]=pyinterval, t0 is the red start moment and t1 is the red termination moment. 
        output: cycle_moments
            a list. 
            cycle_moments[i]=(t0,t1,t2)
        -------------------------------------------
        Steps:
            - 
        """
        cycle_moments = []
        for i in range(len(reds)-1):
            t0 = reds[i].lower_value
            t1 = reds[i].upper_value
            t2 = reds[i+1].lower_value
            cycle_moments.append((t0,t1,t2))

        return cycle_moments




    @classmethod
    def modeldatainput(self, TR_es, redsignals, k_LT, k_TH, w= FD.w):
        """
        Get the input of the model.
        -----------------------------------------------
        input: TR_es
            a list, containing the trajectoryies. 
            Each trajectory is likke ((t1,x1),(t2,x2)...)
        input: redsignals
            a dict. redsignals['LT']=((s1,e1),(s2,e2).....)
            s1 e1 are the onset and end moment of the red signal. 
        input: k_LT, k_TH
            the cycle index of left-turn and through signal. Both are integers, start from 0. 
            means k_LT red signal and k_TH through red. 
        -------------------------------------------------
        output: part_TR_es
            part_TR_es is part of the trajectories. part_TR_es is a list. 
            part_TR_es[i] = ((t1,x1), (t2,x2), (t3,x3)..... )
        output: ((t0_LT,t1_LT,t2_LT),(t0_TH,t1_TH,t2_TH))
            t0 is the onset of cycle time
            t1 is the termination moment of red signal
            t2 is the termination moment of green signal.
            LT and TH means left-turn and through.
        """

        pass


    @classmethod
    def objective_one_turning(self, X, redduration,cycle_sliceddata_turning, optimization_variables_sequence, loc_stopline, l_tb, w=FD.w, vf = FD.vf):
        """
        compute the objective for one cycle. 
        When used in scipy.optimization, the constraints are given by:
            
            constraints_turning,optimization_variables_sequence_turning = csswithsignal.constraints_turning_tsxs(cycle_sliceddata_turning=cycle_sliceddata['l'], redduration=82, loc_stopline=loc_stopline, l_tb=100, w = FD.w)
        
        -----------------------------------------------
        input: X
            the decision variable. 
            len(X) = 1 + N. first '1' means spillover duration; N is the number of trajectories. the sequence is given by optimization_variables_sequence.
        input: optimization_variables_sequence
            indicate the sequence of the decision variable. 
            optimization_variables_sequence[i] = vehicle_id
            It means that i-th+1 variable in X is for vehicle_id.
            Plus 1 is because that the X[0] is the spillover duration. 
        input: cycle_sliceddata_turning
            the trajectory info for each cycle of the specific turning. cycle_sliceddata_turning is a dict
            cycle_sliceddata_turning[vehicle_id].keys()=['tsxs', 'intersectionpoint', 'twoboudnarys', 'redduration', 'cyclestartmoment']

            cycle_sliceddata_turning[vehicle_id]['tsxs']  =(ts,xs)
            cycle_sliceddata_turning[vehicle_id]['intersectionpoint'] = (t_w,x_w)
            cycle_sliceddata_turning[vehicle_id]['twoboudnarys'] = ((t0,x0),(t2,x2))
            cycle_sliceddata_turning[vehicle_id]['redduration'] = r
            cycle_sliceddata_turning[vehicle_id]['cyclestartmoment'] = cyclestartmoment
        input: redduration
            the red duration
        input: X
            the decision variable. 
            X[0] is the assumed spillover duration
            X[1] is the stopping wave of the first vehicle trajectory (specified in optimization_variables_sequence[0]);
            X[2] is the stopping wave of the first vehicle trajectory (specified in optimization_variables_sequence[1]);

        """
        S = 0.0

        for idxx,v_id in enumerate(optimization_variables_sequence):
            #
            t_w,x_w = cycle_sliceddata_turning[v_id]['intersectionpoint']
            TR = cycle_sliceddata_turning[v_id]['tsxs']
            cyclestartmoment = cycle_sliceddata_turning[v_id]['cyclestartmoment']

            S=S+self.objective_one_trajectory_without_ufi_using_phi(tao=X[0],\
                phi=X[idxx+1],TR=TR, r=redduration,\
                    cyclestartmoment=cyclestartmoment,\
                        loc_stopline=loc_stopline, t_w=t_w,x_w=x_w,\
                            l_tb=l_tb,w=FD.w, vf= FD.vf)
        return S/len(optimization_variables_sequence)

    @classmethod
    def initial_guess_one_TR(self, TR_info,  loc_stopline, l_tb, w = FD.w, vf = FD.vf, direction_flag = 1, ufi=  FD.vf,tolerance=1):
        """
        rando guess the TMS duration (i.e. tao) and stopping wave speped. 
        ------------------------------
        input: TR_info
            dict. 
            TR_info['tsxs']  = (ts,xs), 
            TR_info['redduration']  =  redduration, a float;
            TR_info['intersectionpoint']  =  (t_w, x_w), 
            TR_info['t0t1t2']  = (t0,t1,t2), 
        input: 
        output: tao,vstop
            tao (in sec), vstop (in km/h)

        """
        #Steps: first get the attributes
        ts,xs = TR_info['tsxs']
        t_w, x_w = TR_info['intersectionpoint']
        redduration = TR_info['redduration']

        #unit is sec
        tau = np.random.uniform(redduration)

        tmp_distance = abs(x_w - loc_stopline)-l_tb#unit is meter
        tmp_time = tmp_distance/(w/3.6)+tau#unit is sec
        vstop_min = tmp_distance/tmp_time#unit is m/s

        return tau,np.random.uniform(vstop_min*3.6, w-tolerance)


    @classmethod
    def initial_guess_vstop(self,TRs_dict, constraints, optimization_variables_sequence, kwargs, max_infeasible_iter = 10000):
        """

        randomly generate a solution.
        The decision variables include spillover duration (as in tau, unit is sec) and vstop (unit is km/h)
        ---------------------------------------------------
        input: redduration
            a float, the red duration. 
        input: TRs_dict
            the trajectory info for each cycle of the specific turning. 
            TRs_dict is a dict
            TRs_dict[vehicle_id].keys()=['tsxs', 'intersectionpoint', 'twoboudnarys', 'redduration', 'cyclestartmoment']

            TRs_dict[vehicle_id]['tsxs']  =(ts,xs)
            TRs_dict[vehicle_id]['intersectionpoint'] = (t_w,x_w)
            TRs_dict[vehicle_id]['redduration'] = r
            TRs_dict[vehicle_id]['t0t1t2'] = (t0,t1,t2)
        input: constraints
        input: kwargs
            'loc_stopline'
            'l_tb'
            'w'
            'vf'
            'direction_flag'
        output: optimization_variables_sequence
            a list, specify the order of variables in X[1:]. len(X[1:])=len(optimization_variables_sequence_turning).
        -----------------------------------------------
        Steps:
            - the tau, i.e. the spillover duration is sampled uniformly from [0, r]
            - the vstop (each trajectory have its own vstop)
        """
        loc_stopline = kwargs['loc_stopline']
        l_tb = kwargs['l_tb']
        w = kwargs['w']
        vf = kwargs['vf']
        direction_flag = kwargs['direction_flag']

        if len(TRs_dict)!=len(optimization_variables_sequence):
            #print len(TRs_dict),len(optimization_variables_sequence)
            raise ValueError('The length shoulbe the same')
        
        #compute the redduration
        somevehicle = TRs_dict.keys()[0]
        redduration = TRs_dict[somevehicle]['t0t1t2'][1]-TRs_dict[somevehicle]['t0t1t2'][0]

        #decision variable, 
        #   X[0] is tau, i.e. the spillover duration
        #   X[1:] is vstop. The vstop vector starts from onset of TMS. 
        X = np.zeros((1+len(TRs_dict),))

        #generaate the X[0]
        X[0] = np.random.uniform(redduration)
        #   generate the X[1:]
        for i,v_id in enumerate(optimization_variables_sequence):
            #intersection point by the starting wave and the trajectory
            t_w,x_w = TRs_dict[v_id]['intersectionpoint']
            #    LW is lower bound of vstop, unit is km/h
            LW = (abs(x_w - loc_stopline) - l_tb)/((abs(x_w - loc_stopline) - l_tb)/(w/3.6) + X[0])*3.6
            if LW<0:
                #print x_w,loc_stopline,l_tb,LW
                raise ValueError("sdfsdf")
            #    UP is the upper bound of phi, i.e. phi = (x_w - l_tb)/v_stop
            UP = w
            X[i+1] = np.random.uniform(LW, UP)
        
        #when the condition is not feasible, the loop will begin from a new x0. a new iter begins. 
        feasible_iter = 1
        while not self.constraints_satistied(X=X,cons=constraints):
            feasible_iter=feasible_iter+1
            #generate X[0]
            X[0] = np.random.uniform(redduration)
            #generate X[1:]
            for i,v_id in enumerate(optimization_variables_sequence):
                t_w,x_w = TRs_dict[v_id]['intersectionpoint']
                #   LW unit is km/h.
                LW = (abs(x_w - loc_stopline) - l_tb)/((abs(x_w - loc_stopline) - l_tb)/(w/3.6) + X[0])*3.6
                if LW<0:
                    #print x_w,loc_stopline,l_tb,LW
                    raise ValueError("sdfsdf")
                #print('X[0]-----', X[0],'LW-----', LW, 'w------',w)
                X[i+1] = np.random.uniform(LW, w)
            if feasible_iter>=max_infeasible_iter:
                #
                #
                return 'no_feasible_initial_guess'
                builtins.tmp = (TRs_dict, constraints, optimization_variables_sequence, kwargs, X)

                #TRs_dict, constraints, optimization_variables_sequence, kwargs,X = builtins.tmp

                raise ValueError('Two may infeasible initial guess. ')

        return X


    @classmethod
    def initial_guess_phi(self,redduration, cycle_sliceddata_turning, constraints, optimization_variables_sequence, l_tb, loc_stopline, w= FD.w):
        """
        randomly generate a solution.
        The decision variables include spillover duration (as in tau, unit is sec) and phi (unit is also sec, phi = (x_w - l_tb)/v_stop) for each trajectory. 
        ---------------------------------------------------
        input: redduration
            a float, the red duration. 
        input: cycle_sliceddata_turning
            the trajectory info for each cycle of the specific turning. cycle_sliceddata_turning is a dict
            cycle_sliceddata_turning[vehicle_id].keys()=['tsxs', 'intersectionpoint', 'twoboudnarys', 'redduration', 'cyclestartmoment']

            cycle_sliceddata_turning[vehicle_id]['tsxs']  =(ts,xs)
            cycle_sliceddata_turning[vehicle_id]['intersectionpoint'] = (t_w,x_w)
            cycle_sliceddata_turning[vehicle_id]['twoboudnarys'] = ((t0,x0),(t2,x2))
            cycle_sliceddata_turning[vehicle_id]['redduration'] = r
            cycle_sliceddata_turning[vehicle_id]['cyclestartmoment'] = cyclestartmoment
        input: constraints

        output: optimization_variables_sequence
            a list, specify the order of variables in X[1:]. len(X[1:])=len(optimization_variables_sequence_turning).
        -----------------------------------------------
        Steps:
            - the tau, i.e. the spillover duration is sampled uniformly from [0, r]
            - the vstop (each trajectory have its own vstop)
        """
        if len(cycle_sliceddata_turning)!=len(optimization_variables_sequence):
            #print len(cycle_sliceddata_turning),len(optimization_variables_sequence)
            raise ValueError('The length shoulbe the same')
        
        #decision variable, 
        #   X[0] is tau, i.e. the spillover duration
        #   X[1:] is phi, which is calculated from stoppingwave speed by phi = (x_w - l_tb)/v_stop, unit is sec
        X = np.zeros((1+len(cycle_sliceddata_turning),))
        #generaate the X[0]
        X[0] = np.random.uniform(redduration)
        #   generate the X[1:]
        for i,v_id in enumerate(optimization_variables_sequence):
            #intersection point by the starting wave and the trajectory
            t_w,x_w = cycle_sliceddata_turning[v_id]['intersectionpoint']
            #    LW is lower bound of phi, i.e. phi = (x_w - l_tb)/v_stop
            LW = (x_w - loc_stopline - l_tb)/(w/3.6)
            if LW<0:
                #print x_w,loc_stopline,l_tb,LW
                raise ValueError("sdfsdf")
            #    UP is the upper bound of phi, i.e. phi = (x_w - l_tb)/v_stop
            UP = X[0]+(x_w - loc_stopline - l_tb)/(w/3.6)
            X[i+1] = np.random.uniform(LW, UP)
        
        while not self.constraints_satistied(X=X,cons=constraints):
            #generate X[0]
            X[0] = np.random.uniform(redduration)
            #generate X[1:]
            for i,v_id in enumerate(optimization_variables_sequence):
                t_w,x_w = cycle_sliceddata_turning[v_id]['intersectionpoint']
                LW = (x_w - loc_stopline - l_tb)/(w/3.6)
                UP = X[0]+(x_w - loc_stopline - l_tb)/(w/3.6)
                X[i+1] = np.random.uniform(LW, UP)

        return X

    @classmethod
    def batch_preprocess_select_cycles(self, TRs_paired_reds, kwargs, cycles = [0],  samplesize_limit = 30, spillovers_paired_reds=False):
        """
        Select the cycles for batch optimization. 
        Note that, the paired reds' directions should be the same for each cycle (or each idx)
        ------------------------------------------
        input: cycles
            a list. Containing the indx in TRs_paired_reds.
        input: TRs_paired_reds
            a list. Each elements corresponding to a cycle. 
        input: spillovers_paired_reds, a list
            spillovers_paired_reds[cycle_idx]['left'] = a scalar, the TMS duration.
        output: optimization_inputdata, a dict.
            optimization_inputdata['lists_dictTRs'], a list. Element is dict, keys are vehicle ids.
            optimization_inputdata['spillover_durations'], a list, Each element is a scalar. The structure of optimization_inputdata['spillover_durations'] is the same as optimization_inputdata['lists_dictTRs'].
            optimization_inputdata['sequence'] = [(idxx,direction),(idxx,direction)....]
        """
        #
        if max(cycles)>len(TRs_paired_reds)-1:raise ValueError("sdfzdsadfasdfasd")
        loc_stopline = kwargs['loc_stopline']
        l_tb = kwargs['l_tb']
        w = kwargs['w']
        vf = kwargs['vf']
        direction_flag = kwargs['direction_flag']

        #returned value
        lists_dictTRs = []
        sequence = []

        #get all possible directions, a sorted list. 
        directions0 = []
        for i in TRs_paired_reds:directions0.extend(i.keys())
        directions = sorted(set(directions0))

        #
        durations = []
        for idx in cycles:
            for d in directions:
                if not TRs_paired_reds[idx].has_key(d):continue
                #TRs_paired_reds[idx] may not have the key of d.
                lists_dictTRs.append(TRs_paired_reds[idx][d])
                sequence.append((idx,d))
                if not isinstance(spillovers_paired_reds, bool):
                    durations.append(spillovers_paired_reds[idx][d])

        return {'lists_dictTRs':lists_dictTRs, 'spillover_durations':durations, 'sequence':sequence}

    @classmethod
    def batch_optimization_select_cycles_data(self, TRs_paired_reds, spillovers, sliced_TRs_full, select_cycles = range(10), samplesize_limit = 30):
        """
        Select part of the cycles to optimize the TMS duration.
        -------------------------------------
        input: select_cycles
            a list. THe indexs list. 
        input: TRs_paired_reds
            TRs_paired_reds[cycle_index][turningdirection][vid].keys() = ..
        input: sliced_TRs_full
            a litt. sliced_TRs_full[cycle_idx][direction][vid] = {'tsxs':...}
        input: spillovers
            a list. 
            spillovers[cycle_idx]['left'] = 50
            spillovers[cycle_idx]['through'] = 0
        """
        partial_TRs_paired_reds =[]
        partial_spillovers = []
        partial_TRs_pairedred_full = []
        for i in select_cycles:
            partial_spillovers.append(spillovers[i])
            tmp = {};tmp_4full = {}
            for d in TRs_paired_reds[i].keys():
                tmp[d] = {};tmp_4full[d] = {}
                vids = np.random.choice(TRs_paired_reds[i][d].keys(),min(samplesize_limit, len(TRs_paired_reds[i][d])))
                for vid in vids:
                    tmp[d][vid] = TRs_paired_reds[i][d][vid]
                    tmp_4full[d][vid] = sliced_TRs_full[i][d][vid]

            partial_TRs_paired_reds.append(tmp)
            partial_TRs_pairedred_full.append(tmp_4full)
        return partial_TRs_paired_reds,partial_spillovers,partial_TRs_pairedred_full

    @classmethod
    def TRs_TMS_TMS_or_NOT(self, TRs_dict, vf = FD.vf, delay_tolerance =3, ufi = True):
        """
        if the average speed is below the vf some extend, then it is considered as TMS occurs 
        ----------------------------------------
        input: delay_tolerance
            unit is km/h, compared with vf. 
        input: TRs_dict
            a dict. Keys are vids. 
        input: ufi
            when computing the average speed, whether consider the third segment of the trajectory
        """
        vmean =[]
        for vid in TRs_dict.keys():
            if ufi:
                ts,xs  = TRs_dict[vid]['tsxs']
            else:
                ts0,xs0  = TRs_dict[vid]['tsxs']
                t_w, x_w = TRs_dict[vid]['intersectionpoint']
                #use only the 
                ts = np.array(ts0)[np.where(np.array(ts0)<=t_w)[0]]
                xs = np.array(xs0)[np.where(np.array(ts0)<=t_w)[0]]
            vmean.append(abs(3.6*(xs[-1]-xs[0])/(ts[-1]-ts[0])))
        builtins.tmp = vmean
        if vf-np.nanmean(vmean)>delay_tolerance:
            return True
        else:
            return False

    @classmethod
    def test(self,):
        """

        """

        pass

    @classmethod
    def area_practical_theoretical_ts_xs(self, practical_tr, theoretical_tr, kwargs):
        """
        compute the area by using difference bettwen practical_tr and theoretical_tr. 
        -----------------------------------------
        input: practical_tr
            practical_tr = (ts,xs)
            theoretical_tr = (ts,xs)
        output: scalar. the area
        --------------------------------------
        Steps:
            - translate the coordinate, then compute. 
        """
        #translate
        #   new_theoretical_ts...are both list 
        new_theoretical_ts,new_theoretical_xs = self.translate_curve(theoretical_tr[0],theoretical_tr[1], sin_theta=self.sin_theta,cos_theta = self.cos_theta)
        new_practical_ts,new_practical_xs = self.translate_curve(practical_tr[0],practical_tr[1], sin_theta=self.sin_theta,cos_theta = self.cos_theta)

        #compute the theoretical area.
        lenth_theoretical_ts = len(new_theoretical_ts)
        bin_areas_theoretical = []
        for i,j in zip(range(lenth_theoretical_ts-1), range(1,lenth_theoretical_ts)):
            bin_area = (new_theoretical_ts[j]-new_theoretical_ts[i])*(new_theoretical_xs[i]+new_theoretical_xs[j])/2.0
            bin_areas_theoretical.append(bin_area)

        #compute the practical area.
        length_practical_ts = len(new_practical_ts)
        bin_areas_practical = []
        for i,j in zip(range(length_practical_ts-1), range(1,length_practical_ts)):
            bin_area = (new_practical_ts[j]-new_practical_ts[i])*(new_practical_xs[i]+new_practical_xs[j])/2.0
            bin_areas_practical.append(bin_area)

        return abs(sum(bin_areas_practical) - sum(bin_areas_theoretical))


    @classmethod
    def batch_occurrence_TMS(self, batch_optimization_results, ):
        """
        Even there is no TMS, still an estimation can be made. 
        Thus after 
        ----------------------------------------------------
        input: batch_optimization_results
            result returned by self.batch_optimization().
            batch_optimization_results[cycle_idx][turning_diretion] is a dict and keys inlucde:
                - X0
                - Y0
                - res
                - cons
                - optimization_variables_sequence
                - optimal_Y
        """


        pass


    @classmethod
    def batch_optimization(self, TRs_paired_reds, kwargs, max_cycles_N = np.inf, samplesize_limit = 30):
        """
        optimization for many scenarioes. 
        -------------------------------------
        input: delay_tolerance
            when estimate the TMS, 
        input: TRs_paired_reds
            a list. Each elements corresponding to a cycle. Tde 
            TRs_paired_reds[idx] is a dict. 
            TRs_paired_reds[idx].keys are the same as paired_reds[idx].keys.
            TRs_paired_reds[idx]['left']are dict. 
            TRs_paired_reds[idx]['left'][vehicle_id].keys() include:
                - 'tsxs':(ts,xs)
                - 't0t1t2':(t0,t1,t2)
                - 'intersectionpoint':(t_w, x_w)
                - 'redduration': redduration
                - 'twoboudnarys':((ta,xa),(tb,xb))

        input: samplesize_limit
            the uplimit of sample size. 
        input: cycles_N
            the desired optimization cycles. 
            Note that if cycles_N>len(TRs_paired_reds), then it is truncated. 
        input: kwargs
            loc_stopline = kwargs['loc_stopline']
            l_tb = kwargs['l_tb']
            w = kwargs['w']
            vf = kwargs['vf']
            direction_flag = kwargs['direction_flag']
            tolerance = kwargs['tolerance']
            method = kwargs['method']
        Output: results, a list. 
            the same structure as TRs_paired_reds.
            results[idx].keys() are 'left' or through
            results[idx][direction] is a dict and keys inlucde:
                - X0
                - Y0
                - res
                - cons
                - optimization_variables_sequence
                - optimal_Y
        """
        #used to check whether the trajectories experience delay or not.
        vf = kwargs['vf']

        #get the desired optimization cycles. 
        cycles = range(min([max_cycles_N, len(TRs_paired_reds)]))
        #get all possible directions, a sorted list. 
        directions0 = []
        for i in TRs_paired_reds:directions0.extend(i.keys())
        directions = sorted(set(directions0))

        #results[idx][direction].keys() include X0, Y0, res, cons, optimization_variables_sequence, optimal_Y
        results = []
        for idx in cycles:
            res_temp = {}

            #trim the sample size to make the sample size the same for different 
            #   after time, len(TRs_paired_reds_trimed[d]) is the same for all d, if there is any.
            if kwargs['trim_same_samplesize']:
                TRs_paired_reds_trimed = self.TRs_paired_red_trim_samplesize(TRs_paired_reds[idx])
            else:
                TRs_paired_reds_trimed = TRs_paired_reds[idx]

            for d in directions:
                if not TRs_paired_reds_trimed.has_key(d):continue
                #no TRs, then return null
                if len(TRs_paired_reds_trimed[d].keys())==0:
                    res_temp[d] = {'X0':'null', 'res':{'x':['null']},'Y0':'null', 'cons':'null','optimization_variables_sequence':['null'], 'optimal_Y':'null'}
                    continue

                #Check the average speed is lower than vf how much. 
                #   if not too much, then it is no TMS
                #   TRs_paired_reds_trimed[idx][d] is a dict. keys are vids. 
                #TRs_TMS_TMS_or_NOT(self, TRs_dict, vf = FD.vf, delay_tolerance =3):
                if not self.TRs_TMS_TMS_or_NOT(TRs_paired_reds_trimed[d], vf = vf, delay_tolerance = kwargs['delay_check_threshold']):
                    res_temp[d]  = {'X0':np.nan, 'res':{'x':[np.nan]},'Y0':np.nan, 'cons':np.nan,'optimization_variables_sequence':[np.nan], 'optimal_Y':np.nan}
                    continue

                #basedondelay means select the vehicles that enter the road earlier.
                TRs_dict_partial = self.random_select_TRs(TRs_paired_reds_trimed[d], N = samplesize_limit, basedondelay = True)
                #TRs_dict_partial = TRs_paired_reds_trimed[idx][d]
                
                #res_temp[d] = {'X0':X0, 'res':res,'Y0':Y0, 'cons':cons,'optimization_variables_sequence':optimization_variables_sequence, 'optimal_Y':bestY}
                res_temp[d] = self.optimization_TRs(TRs_dict = TRs_dict_partial,kwargs=kwargs)

            print('Complete the optimization of cycle ',idx+1,'---- in ', min([max_cycles_N, len(TRs_paired_reds)]), 'cycles')

            results.append(res_temp)
        
        return results


    @classmethod
    def batch_optimization_corss(self, TRs_paired_reds_cross, kwargs, max_cycles_N = np.inf, samplesize_limit = 30):
        """
        optimization for many paired cycles. 
        Note that, 
        -------------------------------------
        input: delay_tolerance
            when estimate the TMS, 
        input: TRs_paired_reds_cross
            a list. Each elements corresponding to a cycle. TRs_paired_reds_cross[cycle_idx]['left']['through'] means the trajectories using left-turn signal to slice the through turning vehicles. 
            TRs_paired_reds_cross[cycle_idx]['left']['through']['vid'] = 
                - 'tsxs':(ts,xs)
                - 't0t1t2':(t0,t1,t2)
                - 'intersectionpoint':(t_w, x_w)
                - 'redduration': redduration
                - 'twoboudnarys':((ta,xa),(tb,xb))
        input: samplesize_limit
            the uplimit of sample size. 
        input: cycles_N
            the desired optimization cycles. 
            Note that if cycles_N>len(TRs_paired_reds), then it is truncated. 
        input: kwargs
            loc_stopline = kwargs['loc_stopline']
            l_tb = kwargs['l_tb']
            w = kwargs['w']
            vf = kwargs['vf']
            direction_flag = kwargs['direction_flag']
            tolerance = kwargs['tolerance']
            method = kwargs['method']
        Output: results, a list. 
            the same structure as TRs_paired_reds.
            results[idx].keys() are 'left' or through
            results[idx][direction] is a dict and keys inlucde:
                - X0
                - Y0
                - res
                - cons
                - optimization_variables_sequence
                - optimal_Y
        """
        #used to check whether the trajectories experience delay or not.
        vf = kwargs['vf']

        #get the desired optimization cycles. 
        cycles = range(min([max_cycles_N, len(TRs_paired_reds_cross)]))

        #get all possible signal directions, a sorted list. 
        directions0 = []
        for i in TRs_paired_reds_cross:directions0.extend(i.keys())
        directions = sorted(set(directions0))

        #results[idx][direction].keys() include X0, Y0, res, cons, optimization_variables_sequence, optimal_Y
        results = []
        for idx in cycles:
            res_temp = {}
            for signal_d in directions:
                res_temp[signal_d] = {}
                if not TRs_paired_reds_cross[idx].has_key(signal_d):continue
                for turning_d in TRs_paired_reds_cross[idx][signal_d].keys():
                    
                    #exclude right turning trajectory. 
                    if 'right' in turning_d:continue

                    #there is no trajectory, thus optimization will not be implemented. 
                    if len(TRs_paired_reds_cross[idx][signal_d][turning_d].keys())==0:
                        res_temp[signal_d][turning_d] = {'X0':'null', 'res':{'x':['null']},'Y0':'null', 'cons':'null','optimization_variables_sequence':['null'], 'optimal_Y':'null'}
                        continue

                    #determine whether the TMS occurs or not. 
                    if not self.TRs_TMS_TMS_or_NOT(TRs_paired_reds_cross[idx][signal_d][turning_d], vf = vf, delay_tolerance = kwargs['delay_check_threshold']):
                        res_temp[signal_d][turning_d]  = {'X0':np.nan, 'res':{'x':[np.nan]},'Y0':np.nan, 'cons':np.nan,'optimization_variables_sequence':[np.nan], 'optimal_Y':np.nan}
                        continue

                    #optimization given the directional TRs. 
                    res_temp[signal_d][turning_d] = self.optimization_TRs(TRs_dict = TRs_paired_reds_cross[idx][signal_d][turning_d],kwargs=kwargs)

            print('Complete the optimization of cycle ',idx + 1,'---- in ', min([max_cycles_N, len(TRs_paired_reds_cross)]), 'cycles')

            results.append(res_temp)
        
        return results

    @classmethod
    def batch_optimization_TRs_list(self, lists_of_TRs_dict, kwargs, samplesize_limit = 30):
        """
        optimization for many scenarioes. 
        -------------------------------------
        input: lists_of_TRs_dict
            lists_of_TRs_dict[idx] is a dict. The keys are vehicles ids
        input: samplesize_limit
            the uplimit of sample size. 
        input: kwargs
            loc_stopline = kwargs['loc_stopline']
            l_tb = kwargs['l_tb']
            w = kwargs['w']
            vf = kwargs['vf']
            direction_flag = kwargs['direction_flag']
            tolerance = kwargs['tolerance']
            method = kwargs['method']
        Output: results, a list. 
            the same structure as lists_of_TRs_dict.
            results[idx].keys():

        """
        results = []
        #X0, Y0, res, cons, optimization_variables_sequence = csswithsignal.optimization_TRs(TRs_dict = TRs_dict_partial,wargs=kwargs)
        for idx,TRs_dict in enumerate(lists_of_TRs_dict):
            res_temp = {}
            TRs_dict_partial = self.random_select_TRs(TRs_dict, N=samplesize_limit)
            X0, Y0, res, cons, optimization_variables_sequence = self.optimization_TRs(TRs_dict = TRs_dict_partial,kwargs=kwargs)
            res_temp['X0'] = X0
            res_temp['Y0'] = Y0
            res_temp['res'] = res
            res_temp['cons'] = cons
            res_temp['optimization_variables_sequence'] = optimization_variables_sequence
            #print('Complete the optimization of ',idx,'---- in ', len(lists_of_TRs_dict))

            results.append(res_temp)
        
        return results

    @classmethod
    def objective_TR_without_tau(self, TR, tau_k, kwargs, ufi = False, boundary = False):
        """
        input: TR, a dict. 
            TR['tsxs']  = (ts,xs), 
            TR['redduration']  =  redduration, a float;
            TR['intersectionpoint']  =  (t_w, x_w), 
            TR['t0t1t2']  = (t0,t1,t2), 
        input: tau_k
            scalar
        input: ufi
            whether the computed area include the latter part.

        """
        loc_stopline = kwargs['loc_stopline']
        l_tb = kwargs['l_tb']
        w = kwargs['w']
        vf = kwargs['vf']
        direction_flag = kwargs['direction_flag']
        tolerance = kwargs['tolerance']
        use_shapely = kwargs['use_shapely']#whether use shaely to compute the area
        lu = kwargs['lu']

        ts,xs = TR['tsxs']
        t_w,x_w = TR['intersectionpoint']
        cycle_length = TR['t0t1t2'][2] - TR['t0t1t2'][0]
        redduration = TR['t0t1t2'][1] - TR['t0t1t2'][0]

        #keys include 'FirstSegment', 'SecondSegment', 'ThirdSegment'
        #   tr['FirstSegment'] = ((t0,x0),(t1,x1)).
        tr = self.Theoretical_TR_without_tau(TR_info = TR, tau_k=tau_k, boundary=boundary,kwargs = kwargs)

        if ufi:
            practical_tr = (ts,xs)
            #      theoretical_tr = (ts,xs)
            theoretical_tr = ((tr['FirstSegment'][0][0], tr['SecondSegment'][0][0], tr['ThirdSegment'][0][0], tr['ThirdSegment'][1][0]), (tr['FirstSegment'][0][1], tr['SecondSegment'][0][1], tr['ThirdSegment'][0][1], tr['ThirdSegment'][1][1]))
            tmp_ploygon = Polygon(list(tr['FirstSegment'])+list(tr['SecondSegment'])+list(tr['ThirdSegment'])+[(t,x) for t,x in zip(ts[::-1],xs[::-1])])
                #       the temporal width of the trajectory
            tmp_width = ts[-1]-abs(xs[-1]-loc_stopline)/(w/3.6) - (ts[0]-abs(xs[0]-loc_stopline)/(w/3.6))
            #objective is the difference between two areas. 
        else:
            #compute the practial ts,xs and the polygon respectively
            ts0 = np.array(ts)[np.where(np.array(ts)<=t_w)[0]]
            xs0 = np.array(xs)[np.where(np.array(ts)<=t_w)[0]]
            theoretical_tr = ((tr['FirstSegment'][0][0], tr['SecondSegment'][0][0], tr['ThirdSegment'][0][0], tr['ThirdSegment'][1][0]), (tr['FirstSegment'][0][1], tr['SecondSegment'][0][1], tr['ThirdSegment'][0][1], tr['ThirdSegment'][1][1]))
            practical_tr = (ts0,xs0)
            tmp_ploygon = Polygon(list(tr['FirstSegment'])+list(tr['SecondSegment'])+[(t,x) for t,x in zip(ts0[::-1],xs0[::-1])])
            tmp_width = t_w-abs(x_w-loc_stopline)/(w/3.6) - (ts[0]-abs(xs[0]-loc_stopline)/(w/3.6))

        if use_shapely:
            return tmp_ploygon.area/(redduration*lu)
        else:
            temp1 = (w/3.6)*(vf/3.6)/((w/3.6) + (vf/3.6))
            return self.area_practical_theoretical_ts_xs(practical_tr=practical_tr, theoretical_tr=theoretical_tr,kwargs=kwargs)/(redduration*lu)

    @classmethod
    def objective_TRs_without_tau(self, TRs_dict, tau_ks, optimization_variables_sequence, kwargs, ufi = False):
        """
        input: TRs_dict
            keys are vehicle id.
            dict. 
            TR_info = TRs_dict[vid]
            TR_info['tsxs']  = (ts,xs), 
            TR_info['redduration']  =  redduration, a float;
            TR_info['intersectionpoint']  =  (t_w, x_w), 
            TR_info['t0t1t2']  = (t0,t1,t2), 
        input: tau_ks
            the tau of each trajectory. 
            len(tau_ks)==len(TRs_dict)
        input: ufi
            whether the computed area include the latter part.

        """
        loc_stopline = kwargs['loc_stopline']
        l_tb = kwargs['l_tb']
        w = kwargs['w']
        vf = kwargs['vf']
        direction_flag = kwargs['direction_flag']
        tolerance = kwargs['tolerance']
        use_shapely = kwargs['use_shapely']#whether use shaely to compute the area
        lu = kwargs['lu']

        tau = X[0]
        AREAs  = []
        for idx,vid in enumerate(optimization_variables_sequence):
            vstop = X[idx+1]
            ts,xs = TRs_dict[vid]['tsxs']
            t_w,x_w = TRs_dict[vid]['intersectionpoint']
            cycle_length = TRs_dict[vid]['t0t1t2'][2] - TRs_dict[vid]['t0t1t2'][0]
            redduration = TRs_dict[vid]['t0t1t2'][1] - TRs_dict[vid]['t0t1t2'][0]
            #keys include 'FirstSegment', 'SecondSegment', 'ThirdSegment'
            #   tr['FirstSegment'] = ((t0,x0),(t1,x1)).
            tr = self.Theoretical_TR(TR_info = TRs_dict[vid], tau=tau, vstop=vstop,boundary=False,kwargs = kwargs)
            if ufi:
                #compute the practial ts,xs and the polygon respectively
                practical_tr = (ts,xs)
                theoretical_tr = ((tr['FirstSegment'][0][0], tr['SecondSegment'][0][0], tr['ThirdSegment'][0][0], tr['ThirdSegment'][1][0]), (tr['FirstSegment'][0][1], tr['SecondSegment'][0][1], tr['ThirdSegment'][0][1], tr['ThirdSegment'][1][1]))
                tmp_ploygon = Polygon(list(tr['FirstSegment'])+list(tr['SecondSegment'])+list(tr['ThirdSegment'])+[(t,x) for t,x in zip(ts[::-1],xs[::-1])])
                #       the temporal width of the trajectory
                tmp_width = ts[-1]-abs(xs[-1]-loc_stopline)/(w/3.6) - (ts[0]-abs(xs[0]-loc_stopline)/(w/3.6))
            else:
                #compute the practial ts,xs and the polygon respectively
                ts0 = np.array(ts)[np.where(np.array(ts)<=t_w)[0]]
                xs0 = np.array(xs)[np.where(np.array(ts)<=t_w)[0]]
                theoretical_tr = ((tr['FirstSegment'][0][0], tr['SecondSegment'][0][0], tr['ThirdSegment'][0][0], tr['ThirdSegment'][1][0]), (tr['FirstSegment'][0][1], tr['SecondSegment'][0][1], tr['ThirdSegment'][0][1], tr['ThirdSegment'][1][1]))
                practical_tr = (ts0,xs0)
                tmp_ploygon = Polygon(list(tr['FirstSegment'])+list(tr['SecondSegment'])+[(t,x) for t,x in zip(ts0[::-1],xs0[::-1])])
                tmp_width = t_w-abs(x_w-loc_stopline)/(w/3.6) - (ts[0]-abs(xs[0]-loc_stopline)/(w/3.6))
            if use_shapely:
                AREAs.append(tmp_ploygon.area/tmp_width*tmp_width)
            else:
                temp1 = (w/3.6)*(vf/3.6)/((w/3.6) + (vf/3.6))
                AREAs.append(self.area_practical_theoretical_ts_xs(practical_tr=practical_tr, theoretical_tr=theoretical_tr,kwargs=kwargs)/(redduration*lu)*tmp_width/redduration)
            #builtins.tmp = tmp_ploygon
            #AREAs.append(tmp_ploygon.area/(tmp_width*tmp_width))
        return np.mean(AREAs)



    @classmethod
    def objective_TRs(self, X, TRs_dict, optimization_variables_sequence, kwargs):
        """
        input: TRs_dict
            keys are vehicle id.
            dict. 
            TR_info = TRs_dict[vid]
            TR_info['tsxs']  = (ts,xs), 
            TR_info['redduration']  =  redduration, a float;
            TR_info['intersectionpoint']  =  (t_w, x_w), 
            TR_info['t0t1t2']  = (t0,t1,t2), 
        input: ufi
            whether the computed area include the latter part.

        """
        loc_stopline = kwargs['loc_stopline']
        l_tb = kwargs['l_tb']
        w = kwargs['w']
        vf = kwargs['vf']
        direction_flag = kwargs['direction_flag']
        tolerance = kwargs['tolerance']
        use_shapely = kwargs['use_shapely']#whether use shaely to compute the area
        lu = kwargs['lu']
        area_ufi = kwargs['area_ufi']
        second_average = kwargs['second_average']#whether the objectives need to averaged the second time.

        #find the vid that most near the turning bay area. 
        #   min_weight is used to weight the areas. 
        min_weight = np.inf
        for idx,vid in enumerate(optimization_variables_sequence):
            t_w,x_w = TRs_dict[vid]['intersectionpoint']
            min_weight = min(abs(x_w - loc_stopline), min_weight)

        tau = X[0]
        AREAs  = []
        for idx,vid in enumerate(optimization_variables_sequence):
            vstop = X[idx+1]
            ts,xs = TRs_dict[vid]['tsxs']
            t_w,x_w = TRs_dict[vid]['intersectionpoint']
            cycle_length = TRs_dict[vid]['t0t1t2'][2] - TRs_dict[vid]['t0t1t2'][0]
            redduration = TRs_dict[vid]['t0t1t2'][1] - TRs_dict[vid]['t0t1t2'][0]
            #keys include 'FirstSegment', 'SecondSegment', 'ThirdSegment'
            #   tr['FirstSegment'] = ((t0,x0),(t1,x1)).
            tr = self.Theoretical_TR(TR_info = TRs_dict[vid], tau=tau, vstop=vstop,boundary=False,kwargs = kwargs)
            if area_ufi:
                #compute the practial ts,xs and the polygon respectively
                practical_tr = (ts,xs)
                theoretical_tr = ((tr['FirstSegment'][0][0], tr['SecondSegment'][0][0], tr['ThirdSegment'][0][0], tr['ThirdSegment'][1][0]), (tr['FirstSegment'][0][1], tr['SecondSegment'][0][1], tr['ThirdSegment'][0][1], tr['ThirdSegment'][1][1]))
                tmp_ploygon = Polygon(list(tr['FirstSegment'])+list(tr['SecondSegment'])+list(tr['ThirdSegment'])+[(t,x) for t,x in zip(ts[::-1],xs[::-1])])
                #       the temporal width of the trajectory
                tmp_width = ts[-1]-abs(xs[-1]-loc_stopline)/(w/3.6) - (ts[0]-abs(xs[0]-loc_stopline)/(w/3.6))
            else:
                #compute the practial ts,xs and the polygon respectively
                ts0 = np.array(ts)[np.where(np.array(ts)<=t_w)[0]]
                xs0 = np.array(xs)[np.where(np.array(ts)<=t_w)[0]]
                theoretical_tr = ((tr['FirstSegment'][0][0], tr['SecondSegment'][0][0], tr['ThirdSegment'][0][0], tr['ThirdSegment'][1][0]), (tr['FirstSegment'][0][1], tr['SecondSegment'][0][1], tr['ThirdSegment'][0][1], tr['ThirdSegment'][1][1]))
                practical_tr = (ts0,xs0)
                tmp_ploygon = Polygon(list(tr['FirstSegment'])+list(tr['SecondSegment'])+[(t,x) for t,x in zip(ts0[::-1],xs0[::-1])])
                tmp_width = t_w-abs(x_w-loc_stopline)/(w/3.6) - (ts[0]-abs(xs[0]-loc_stopline)/(w/3.6))
            if use_shapely:
                
                AREAs.append(tmp_ploygon.area/(tmp_width))#*redduration*vf/3.6
            else:
                temp1 = (w/3.6)*(vf/3.6)/((w/3.6) + (vf/3.6))
                AREAs.append(self.area_practical_theoretical_ts_xs(practical_tr=practical_tr, theoretical_tr=theoretical_tr,kwargs=kwargs)/(redduration*lu)*tmp_width/redduration)
            #builtins.tmp = tmp_ploygon
            #AREAs.append(tmp_ploygon.area/(tmp_width*tmp_width))
        if second_average:
            return np.mean(AREAs)/len(AREAs)
        else:
            return np.mean(AREAs)



    @classmethod
    def objective_one_TR(self, X, TR_info, loc_stopline, l_tb, w = FD.w, vf = FD.vf, direction_flag = 1, ufi= FD.vf, tolerance = 1):
        """
        ----------------------------------------------------------
        input: TR_info
            dict. 
            TR_info['tsxs']  = (ts,xs), 
            TR_info['redduration']  =  redduration, a float;
            TR_info['intersectionpoint']  =  (t_w, x_w), 
            TR_info['t0t1t2']  = (t0,t1,t2), 
        input: loc_stopline, l_tb
            the location of stop line and the length of the turning bay. 
        input: direction_flag, either 1 or -1
            if east and north, it should be 1. Otherwise it should be -1.
        """
        tau = X[0]
        vstop = X[1]
        ts,xs = TR_info['tsxs']
        #keys include 'FirstSegment', 'SecondSegment', 'ThirdSegment'
        #   tr['FirstSegment'] = ((t0,x0),(t1,x1)).
        tr = self.Theoretical_TR(TR_info = TR_info, tau=tau, vstop=vstop,loc_stopline=loc_stopline, l_tb=l_tb, w= w, vf=vf, ufi = vf, direction_flag=direction_flag, boundary=False,tolerance=tolerance)
        return Polygon(list(tr['FirstSegment'])+list(tr['SecondSegment'])+list(tr['ThirdSegment'])+[(t,x) for t,x in zip(ts[::-1],xs[::-1])]).area

    @classmethod
    def objective_one_trajectory_without_ufi_using_phi(self,tao, phi, TR, r,t_w, x_w,cyclestartmoment, loc_stopline, l_tb=100,w=FD.w, vf= FD.vf):
        """
        objective function for one traejctory that does not consider the latter part of the trajectory, and using the phi as a decision variable. 
        The name of the function '...without_ufi..' means that, the capacity degradation is not considerd. 
        The relation between phi and stopping wave speed v_stop is as follows: 
            phi = (x_w - l_tb)/v_stop
        l_tb is the  turning bay area length. 
        ---------------------------------------
        #input: ufi, unit is km/h
        #    the assumed vehicle speed after the clearance of CSS.
        input: loc_stopline
            the location of stopline. used to uniform the trajectory
        input: tao 
            the assumed css duration
        input: phi
            decision variable.  
        input: TR
            the trajectory of the vehicle. (ts,xs). len(ts)=len(xs), both are list. 
        input: r and cyclestartmoment
            r is the red duration, cyclestartmoment is the moment of onset of RGP.
        input: t_w and x_w
            the coordinate of the intersection point by the trajectory and the 
        OUTPUT: 
        --------------------------------
        """
        
        #area surrounded by the trajectory
        tss = TR[0]
        xss = TR[1]
        
        S0 = self.Area_Curve_with_translation(tss, xss)
        #area by the assumed trajectory
        #S1 = self.Area_CMPF(ufi=ufi, t_w=t_w, x_w=x_w, t_UP=t_UP, x_UP=x_UP, w = w)
        S2 = self.Area_JOMC( w=w, t_w=t_w, x_w=x_w, r=r, Cycle_start=cyclestartmoment)
        S3 = self.Area_EJD_using_PHI(PHI=phi,r=r,tao=tao,x_w=x_w, loc_stopline = loc_stopline, l_tb = l_tb,w = w,vf = vf)
        return abs(S2+S3-S0)

    @classmethod
    def objective_one_trajectory_without_ufi_using_vstop(self,tao,v_stop, TR, r,t_w, x_w,cyclestartmoment, loc_stopline, l_tb=100,w=FD.w, vf= FD.vf):
        """
        objective of the function. 
        Decision variables include the ufi, tao, v_stop.
        ---------------------------------------
        #input: ufi, unit is km/h
        #    the assumed vehicle speed after the clearance of CSS.
        input: loc_stopline
            the location of stopline. used to uniform the trajectory
        input: tao 
            the assumed css duration
        input: v_stop
            the assumed stopwave speed. 
        input: TR
            the trajectory of the vehicle. (ts,xs). len(ts)=len(xs), both are list. 
        input: r and cyclestartmoment
            r is the red duration, cyclestartmoment is the moment of onset of RGP.
        input: t_w and x_w
            the coordinate of the intersection point by the trajectory and the 
        OUTPUT: 
        --------------------------------
        """
        
        #area surrounded by the trajectory
        tss = TR[0]
        xss = TR[1]
        
        S0 = self.Area_Curve(tss, xss)
        #area by the assumed trajectory
        #S1 = self.Area_CMPF(ufi=ufi, t_w=t_w, x_w=x_w, t_UP=t_UP, x_UP=x_UP, w = w)
        S2 = self.Area_JOMC( w=w, t_w=t_w, x_w=x_w, r=r, Cycle_start=cyclestartmoment)
        S3 = self.Area_EJD_using_stopwave(r=r,tao=tao,x_w=x_w-loc_stopline,l_tb=l_tb,v_stop=v_stop,w=w,vf=vf)

        return (S2+S3-S0)**2

    @classmethod
    def objective_one_turning_TEMP(self,ufi_es,tao,v_stop_es, TR_es, t0, t1, t2, point0, point1, point2,  l_tb,w=FD.w, vf= FD.vf):
        """
        objective function for only one turning movement and many trajectories. 
        ------------------------------------------------------
        input: ufi_es, a list of float.
            assumed the vehicle speed after CSS clearance. 
            len(ufi_es)=len(TR_es)
        input: tao
            assumed css duration
        input: v_stop_es
            assumed stop wave speed. 
            len(v_stop_es)=len(TR_es)
        """
        S = 0
        for TR,ufi,v_stop in zip(TR_es, ufi_es, v_stop_es):
            S=S+self.objective_one_trajectory(ufi=ufi,tao=tao,v_stop=v_stop, TR=TR, t0=t0, t1=t1, t2=t2, point0 = point0, point1=point1, point2=point2,  l_tb = l_tb,w=w, vf= vf)
        
        return S
    @classmethod
    def Get_colors(self, randomize = True):
        """
        return a list.
        """
        
        #labels
        labels = [i for i in dir(allpalettes) if isinstance(getattr(allpalettes,i),list)]
        lengths = [len(i) for i in dir(allpalettes) if isinstance(getattr(allpalettes,i),list)]
        print labels
        if not randomize:
            idxx = lengths.index(max(lengths))
            colors = getattr(allpalettes,labels[idxx])
        else:
            somalabel = np.random.choice(labels)
            colors = getattr(allpalettes,somalabel)
        return colors
