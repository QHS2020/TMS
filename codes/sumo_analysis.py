# -*- coding: utf-8 -*-

from .RequiredModules import *
"""

"""
lane_width = 3.5

#TurningLanesPair['left'] = ['east_c_2','east_u_1']
#   means that the left turning movement, 
TurningLanesPair = {'left':['east_c_2','east_u_1'],'through':['east_c_1','east_u_0'],'right':['east_c_0','east_u_0']}


def OffsetXY(configs):
        """
        In the intersection setting, the x=0 is the location of the leftside of left-turn lane of south approach. y=0 is located at the  leftside of left-turn lane of west approach. While in the SUMO, the x=0 and y=0 are for the leftmost node and the bottom node respectively. 
        
        --------------------------------------------------
        return offsetx,offsety
        
        
        """
        sizeX, sizeY = SizeIntersection(configs)
        return sizeX*.5+(configs['west']['ld']+configs['west']['lu']),sizeY*.5+(configs['south']['ld']+configs['south']['lu'])


def SizeIntersection(configs, lane_width = lane_width):
        """get the size of the intersection, return sizeX and sizeY, unit in meters"""
        def Temp_lanes_number_m(approach_config):
                """
                """
                return 2.0*max(approach_config['n_lanes_out'], approach_config['n_lanes_left']+ approach_config['n_lanes_through'] + approach_config['n_lanes_right'])
        #*2 means that 
        SizeIntersectionX = lane_width*1.0*max(Temp_lanes_number_m(configs['south']), Temp_lanes_number_m(configs['north']))
        SizeIntersectionY = lane_width*1.0*max(Temp_lanes_number_m(configs['west']), Temp_lanes_number_m(configs['east']))
        return SizeIntersectionX,SizeIntersectionY




def Generate_Vehicles_snapshot_from_xml(datafile):
    """
        vehicles_snapshot_pd is a dict.
        
        vehicles_snapshot_pd[vehicle_id] = pd.DataFrame type data
        """
    tree = et.ElementTree(file=datafile)
    root = tree.getroot()
    #probe_data = {}
    vehicles_snapshot = {}
    for snapshot in root:
        #probe_data[snapshot.attrib['time']]= []
        for eachvehicle in snapshot.getchildren():
            if not vehicles_snapshot.has_key(eachvehicle.attrib['id']):
                vehicles_snapshot[eachvehicle.attrib['id']] = {}
            vehicles_snapshot[eachvehicle.attrib['id']][
                snapshot.attrib['time']] = eachvehicle.attrib
            #probe_data[snapshot.attrib['time']].append(eachvehicle.attrib)

    vehicles_snapshot_pd = {}
    for idd, values in vehicles_snapshot.iteritems():
        vehicles_snapshot_pd[idd] = pd.DataFrame(
            values).transpose().convert_objects(convert_numeric=True)
    return vehicles_snapshot_pd

def Probe_vehicle_temporalorder(vehicles_trajectories_pd_dict):
    """
    Rank the vehicles according to the moments when they enter the road. 
    --------------------------------
    input: vehicles_trajectories_pd_dict
        a dict about the trajectory. the keys are the vehicle ids
        Each value is pd.DataFrame. columns include
            Index([u'id', u'lane', u'pos', u'speed', u'x', u'y'], dtype='object')
    output: vehicles_orders
        a list containing the 
    -----------------------------------------------
    steps: 
    """
    #vehicles ids
    vehicles_ids = vehicles_trajectories_pd_dict.keys()
    #   the moments corresponds to the vehicles. 
    #   len(vehicles_ids)=len(moments)
    moments = [min(vehicles_trajectories_pd_dict[v].index) for v in vehicles_ids]

    #
    idxx = np.argsort(moments) 
    return [vehicles_ids[i] for i in idxx]

def trajectory_spatial_filter(vehicles_snapshot_pd_dict, c_sectionlocation):
    """
    For a vehicle, the trajectory may not be in the spatial domain. 
    This method will filter the trajectories. 
    ----------------------------------------
    input: vehicles_snapshot_pd_dict
        the data obtained from the probe vehicles. it is a dict. 
        The keys are vehicle_id, the value is a pd.Dataframe. 
        columns are:
            Index([u'id', u'lane', u'pos', u'speed', u'x', u'y'], dtype='object')
    input: c_sectionlocation
        c_sectionlocation=(421.0, 521.0, 'x') means the C-section of east approach is from 421 m to 521m which is measured in x axis. 
    output: part_vehicles_snapshot_pd_dict
        part of the input data. 
    """
    part_vehicles_snapshot_pd_dict = {}
    for v_id in vehicles_snapshot_pd_dict.keys():
        trajectory = vehicles_snapshot_pd_dict[v_id]
        if c_sectionlocation[2]=='x':
            part_vehicles_snapshot_pd_dict[v_id] = copy.deepcopy(trajectory[(trajectory['x']<c_sectionlocation[1]) & (trajectory['x']>c_sectionlocation[0])])
        else:
            part_vehicles_snapshot_pd_dict[v_id] = copy.deepcopy(trajectory[(trajectory['x']<c_sectionlocation[1]) & (trajectory['x']>c_sectionlocation[0])])
    
    return part_vehicles_snapshot_pd_dict

def trajectories_sample_with_turning(approach_turning_trajec_pd_dict, p =.2):
    """
    ----------------------------------------------------

    input: approach_turning_trajec_pd_dict
        keys are 'l','t' and 'r', meaning three different turning directions. 
        approach_turning_trajec_pd_dict['l'] are dict: 
            keys are vehicle ids, each value are (ts,xs).
            ts and xs are tuple, len(ts)=len(xs)
    output: 
        left_TRs, through_TRs, right_TRs
    ----------------------------------------------------
    """
    left_TRs = {}
    through_TRs = {}
    right_TRs = {}

    while True:
        left_v_id_samples = random.sample(approach_turning_trajec_pd_dict['left'].keys(), k = max(1, int(p*len(approach_turning_trajec_pd_dict['left'].keys()))))
        print left_v_id_samples
        if len(left_v_id_samples)>0:break
    while True:
        through_v_id_samples = random.sample(approach_turning_trajec_pd_dict['through'].keys(), k = max(1, int(p*len(approach_turning_trajec_pd_dict['through'].keys()))))
        if len(through_v_id_samples)>0:break
    while True:
        right_v_id_samples = random.sample(approach_turning_trajec_pd_dict['right'].keys(), k = max(1, int(p*len(approach_turning_trajec_pd_dict['right'].keys()))))
        if len(right_v_id_samples)>0:break
    

    for i in left_v_id_samples:
        left_TRs[i] = copy.deepcopy(approach_turning_trajec_pd_dict['left'][i])
    for i in through_v_id_samples:
        through_TRs[i] = copy.deepcopy(approach_turning_trajec_pd_dict['through'][i])
    for i in right_v_id_samples:
        right_TRs[i] = copy.deepcopy(approach_turning_trajec_pd_dict['right'][i])

    return {'left':left_TRs, 'through':through_TRs, 'right':right_TRs}

def Probe_sample_vehicles(vehicles_ids, p = .3):
    """
    randomly sample the vehicles ids. 
    -----------------------------------------
    input: vehicles_ids
        a list, the elements are the vehicles ids
    input: vehicles_temporalrank
        a list containing the vehicles ids. 
        The first vehicle enters the road earliest. 
    input: p
        the probaibility of the sample. 

    """
    res = []
    for v_id in vehicles_ids:
        if np.random.random()<p:
            res.append(v_id)
    return res

def ProcessSignal_get_cycle_intervalslist_turning(paired_reds, direction = 'left'):
    """


    """
    #TimeIntervals.add(pyinter.interval.open(s,e))
    cycles_intervals = []
    for each_pair in paired_reds:
        if direction in each_pair.keys():
            cycles_intervals.append(pyinter.interval.open(each_pair[direction][0], each_pair[direction][2]))
        else:
            cycles_intervals.append(pyinter.interval.open(0,0))

    return cycles_intervals

def ProcessSignal_get_cycle_intervalsset_turning(paired_reds, direction = 'left'):
    """


    """
    #TimeIntervals.add(pyinter.interval.open(s,e))
    cycles_intervals = pyinter.IntervalSet()
    for each_pair in paired_reds:
        if direction in each_pair.keys():
            cycles_intervals.add(pyinter.interval.open(each_pair[direction][0], each_pair[direction][2]))
    return cycles_intervals

def Process_loop_detectors_cycles_flow(averageflow_pd, cycles_intervals_list, count_column = 'nVehContrib'):
    """
    Get the cycle flow rate. 
    -------------------------------------------
    input: cycles_intervals_list
        It is a list of intervals. 
        can be obtaied via:
            cycles_intervals_left = cycles_intervals_list = sumoa.ProcessSignal_get_cycle_intervalslist_turning(paired_reds).

    -------------------------------------------
    output: cycles_flow
        a list. 
        len(cycles_flow) = len(paired_reds)

    """
    cycles_flow = []
    for cycle_interval in sorted(cycles_intervals_list):
        cycle_flow = 0
        already_in_interval = 0
        for row_idx in averageflow_pd.index:
            if (averageflow_pd.loc[row_idx, 'begin'] in cycle_interval) and (averageflow_pd.loc[row_idx, 'end'] in cycle_interval):
                already_in_interval = 1
                cycle_flow = cycle_flow +  averageflow_pd.loc[row_idx, count_column]
            else:
                if already_in_interval==1:
                    break
        if cycle_interval.upper_value - cycle_interval.lower_value==0:
            cycles_flow.append(0)
        else:
            cycles_flow.append(3600.0*cycle_flow/(cycle_interval.upper_value - cycle_interval.lower_value))

    return cycles_flow

def Process_loop_detectors_average_flow(loop_data_dictof_pd, columns = ['begin','end','flow', 'nVehContrib', 'nVehEntered']):
    """
    Process several loop detectors together. 
    The returned are the average flow, average nVehContrib and average nVehEntered.

    The explanation of nVehEntered in sumo doc:
        nVehContrib: The number of vehicles that have completely passed the detector within the interval.

        nVehEntered: All vehicles that have touched the detector. Includes vehicles which have not passed the detector completely (and which do not contribute to collected values).
    -----------------------------
    input: loop_data_dictof_pd
        a dict. Each value is a pd.dataframe.
        columns are:
            Index([u'begin', u'end', u'flow', u'harmonicMeanSpeed', u'id', u'length',
                   u'nVehContrib', u'nVehEntered', u'occupancy', u'speed'],
                  dtype='object')
    Each value is obtained via:
        - sumoa.Process_loop_detector(data_file_path)

    ------------------------------------
    Output: 
    """
    somekey  = np.random.choice(loop_data_dictof_pd.keys())
    returned = pd.DataFrame(0,index = loop_data_dictof_pd[somekey].index, columns = columns)
    returned['begin'] = loop_data_dictof_pd[somekey]['begin'].values
    returned['end'] = loop_data_dictof_pd[somekey]['end'].values

    for column in columns:
        for k in loop_data_dictof_pd.keys():
            returned[column] = returned[column] + loop_data_dictof_pd[k][column]
        returned[column] = returned[column]/len(loop_data_dictof_pd)

    return returned

def Process_loop_detector(datafile):
    """
        
        """
    tree = etree.ElementTree(file=datafile)
    root = tree.getroot()
    res = {}
    for d in root:
        res[d.attrib['begin']] = dict(d.attrib)

    res0 =  pd.DataFrame(res).transpose().convert_objects(convert_numeric=True)

    #sort
    res = res0.sort_values(by = ['begin'])
    return res



def Probe_vehicles_enter_order(vehicles_snapshot_pd_dict):
    """
    Order the vehicles, according to the entering moments of entering the network. 
    --------------------------------------------
    input:  vehicles_snapshot_pd_dict
        a dict. THe keys are vehicles id, and each value is pd.DataFrame. 
    output: order
        a list. the first vehicle enter the netwrok earliest. 
    -----------------------------------------------
    Steps: 
        
    """
    tmp = [(k,min(v.index)) for k,v in vehicles_snapshot_pd_dict.iteritems()]


    return tmp

def TravelTime_from_probe(vehicles_snapshot_pd_dict):
    """
        From the vehicle snap shots to get travel time.
        ---------------------------------------------
        Output: tt
                travel time, a dict, tt[car_id] = scalar.
        """
    tt = {}
    for car_id, data in vehicles_snapshot_pd_dict.iteritems():
        tmp = pd.Series(data.index).convert_objects(convert_numeric=True)
        tt[car_id] = tmp.max() - tmp.min()
        pass
    return tt

from matplotlib.patches import Polygon
def plot_trajectories_wavebound_C_section(approach_TRs, approach_reds, c_sectionlocation, alpha_LT = .5, alpha_TH = .6, alpha_c_section = .5,color_LTpatch = 'y',color_THpatch =  'm',w = 20.0):
    """
    plot the vehicles trajectory, wave bound, C-section patch together. 

    -------------------------------------------------
    input: approach_TRs
        approach_TRs[vehicle_id] = (ts,xs).
        ts and xs are tuples. len(ts)=len(xs).
    input: c_sectionlocation
        c_sectionlocation=(421.0, 521.0, 'x') means the C-section of east approach is from 421 m to 521m which is measured in x axis. 
        c_sectionlocation is used to patch the plot
    input: approach_greens
        a dict. turninggreens['left'] is a list, containding start and end of each green signal. 
    input: w, unit is km/h.
        the backward wave speed. 
    """
    #convert the wave speed to m/s
    w=w/3.6

    #used to patch c-section. 
    tmin = np.inf
    tmax = -np.inf
    xmin = np.inf
    xmax = -np.inf

    fig,ax = plt.subplots()
    for v_id in approach_TRs.keys():
        ax.plot(approach_TRs[v_id][0],approach_TRs[v_id][1])
        
        #used to patch the C-section.
        tmin = min(tmin, min(approach_TRs[v_id][0]))
        tmax = max(tmax, max(approach_TRs[v_id][0]))
        xmin = min(xmin, min(approach_TRs[v_id][1]))
        xmax = max(xmax, max(approach_TRs[v_id][1]))
    
    tmin=0
    #c-section patch
    tmp = Rectangle((tmin, c_sectionlocation[0]), tmax-tmin, c_sectionlocation[1]-c_sectionlocation[0])
    pc = PatchCollection([tmp],alpha = alpha_c_section)
    ax.add_collection(pc)
    
    #wave patch.
    patches = []
    #   left
    x0 = c_sectionlocation[0]
    reds = approach_reds['left']
    for red in reds:
        s,e = red.lower_value, red.upper_value
        point0 = ((xmin-x0)/w+s,xmin)
        point1 = ((xmin-x0)/w+e,xmin)
        point2 = ((xmax-x0)/w+e,xmax)
        point3 = ((xmax-x0)/w+s,xmax)
        polygon = Polygon([point0,point1,point2,point3,point0])
        patches.append(polygon)
    p = PatchCollection(patches, alpha=0.4)
    p.set_color(color_LTpatch)
    ax.add_collection(p)
    return ax

def Probe_pd_sort_check(vehicles_snapshot_pd_dict):
    """
    sort the rows of trajectories in ascending order. 
    """
    for v_id in vehicles_snapshot_pd_dict.keys():
        trajectory = vehicles_snapshot_pd_dict[v_id]
        if not np.all(np.diff(trajectory.index))>0:
            raise ValueError('sdsd')


    pass

def Probe_pd_sort_temporal(vehicles_snapshot_pd_dict):
    """
    sort the rows of trajectories in ascending order. 
    """
    for v_id in vehicles_snapshot_pd_dict.keys():
        trajectory = vehicles_snapshot_pd_dict[v_id]
        trajectory.sort_index(inplace=True)
        if not np.all(np.diff(trajectory.index))>0:
            raise ValueError('sdsd')


    pass


def Process_ProbeData(datafile):
    """
        proces the probe vehicle data.
        -----------------------------------------------------
        Output: vehicles_snapshot_pd_dict
                dict. pd.DataFrame.
                vehicles_snapshot_pd_dict[vehicle_id] is a pd.dataFrame data.
                The first column is 
                
        """
    tree = etree.ElementTree(file=datafile)
    root = tree.getroot()
    #probe_data = {}
    vehicles_snapshot = {}
    for snapshot in root:
        #probe_data[snapshot.attrib['time']]= []
        for eachvehicle in snapshot.getchildren():
            if not vehicles_snapshot.has_key(eachvehicle.attrib['id']):
                vehicles_snapshot[eachvehicle.attrib['id']] = {}
            vehicles_snapshot[eachvehicle.attrib['id']][
                snapshot.attrib['time']] = dict(eachvehicle.attrib)
        #probe_data[snapshot.attrib['time']].append(eachvehicle.attrib)
    vehicles_snapshot_pd_dict = {}
    for idd, values in vehicles_snapshot.iteritems():
        vehicles_snapshot_pd_dict[idd] = pd.DataFrame(
            values).transpose().convert_objects(convert_numeric=True)
    return vehicles_snapshot_pd_dict

approachsequence = ['north','east','south','west']
#index_in_states['north'] = [0,2,4] means that in the signal states 'GGrrrrGGrrrrGGrrrrGGgggg', the left,through and rightturn index are 0,2,4 respectively. 
index_in_states = {'north':[4,2,0],'east':[10,8,6],'south':[16,14,12],'west':[22,20,18]}
def signalstates_approach(intersectionsignalswitchstates,index_in_states=index_in_states,approach='east'):
    """
    get the signals of the approach.
    Currently, for each approach such as 'east', the edeg of channelized section is 'east_c', and the lane 0 'east_c_0' is right turn, 'east_c_1' is through, and 'east_c_2' is right-turn. 
    
    If the C-section all have three lanes, then there will be 24 signals. 
    north_right-turn-to-west-lane0 is 0, 
    north_right-turn-to-west-lane1 is 1,
    north-through-turn-to-west-lane0 is 2, 
    north-through-turn-to-west-lane1 is 3,
    north-left-turn-to-west-lane0 is 4, 
    north-left-turn-to-west-lane1 is 5,
    ....
    
    Therefore if approach is 'east', then the 
    ------------------------------------
    input: intersectionsignalswitchstates
        a dataframe. The column 'time' and 'state' give the results.
        intersectionsignalswitchstates.time is float. 
        intersectionsignalswitchstates.state is state, such as 'GGrrrrGGrrrrGGrrrrGGgggg'. 
        pass
    output: signalstateswitchs
        a dict. 
        signalstateswitchs['left'] = ((0,'G'),(45,'r')...).
        It means at instance 0sec, the leftturn signal is G, and at the moment of 45 s, the signal switches to red. 
    """
    leftsignalswitchs = []
    throughsignalswitchs = []
    rightsignalswitchs = []
    for t,state in zip(intersectionsignalswitchstates.time,intersectionsignalswitchstates.state):
        t = float(t)
        leftsignalswitchs.append((t,state[index_in_states[approach][0]]))
        throughsignalswitchs.append((t,state[index_in_states[approach][1]]))
        rightsignalswitchs.append((t,state[index_in_states[approach][2]]))
        pass
    
    return {'left':leftsignalswitchs,'through':throughsignalswitchs,'right':rightsignalswitchs}

def ts_filter_return_idxs(ts,t0,t1):
    """
    filter the moment within [t0,t1].
    """
    if t1<t0:raise ValueError("sdfsdfsdf")


    return np.where((np.array(ts)>=t0) & (np.array(ts)<=t1))[0]

def ts_filter(ts,t0,t1):
    """
    filter the moment within [t0,t1].
    """
    if t1<t0:raise ValueError("sdfsdfsdf")
    return list(np.array(ts)[ts_filter_return_idxs(ts,t0,t1)])

def tsvs_2_spillover_interset(ts,vs, threshold =3):
    """
    vs unit is km/h. 
    from ts and vs, construct the spillover intervals, which is an pyinter.IntervelSet instance.
    ----------------------------------------
    output: spillovers
        pyinter.IntervalSet instance. 
        can be used in:
            spillovers.intersection([pyinter.interval.closed(10,453)])
        To find the intersection. The resuiling value is also a IntervalSet. 

        For total spillover duration, can be calculated as:
            sum([m.upper_value-m.lower_value for m in spillovers])
        It will return a scalar. 
    """
    spillovers = pyinter.IntervalSet()
    
    for idx in range(1,len(ts)):
        if vs[idx]>=0 and vs[idx]<=threshold:
            spillovers.add(pyinter.interval.closed(ts[idx-1],ts[idx]))
    return spillovers

def tsvs_interploate(ts,vs,t):
    """
    ts is strictly increasing.
    ------------------------------------
    input: ts, vs
        both are list. the same length.
    input: t
        a scalar.
    return: v0
        interploated v.
    """
    if t<=ts[0] or t>=ts[-1]:
        return ts,vs
    else:
        idx0 = np.where(np.array(ts)<=t)[0][-1]
        #ts[idx0]<t, #ts[idx0+1]>t
        if ts[idx0]==t:return ts,vs
        return vs[idx0] + (vs[idx0+1]-vs[idx0])*(t--ts[idx0])/(ts[idx0+1]-ts[idx0])

def tsvs_interploate_return_tsvs(ts,vs,t):
    """
    ts is strictly increasing.
    ---------------------------------------------
    input: ts, vs
        both are list. the same length.
    input: t
        a scalar.
    output: ts0,vs0
    """
    if t<=ts[0] or t>=ts[-1]:
        return ts,vs
    else:
        idx0 = np.where(np.array(ts)<=t)[0][-1]
        #ts[idx0]<t, #ts[idx0+1]>t
        if ts[idx0]==t:return ts,vs
        tmp = vs[idx0] + (vs[idx0+1]-vs[idx0])*(t--ts[idx0])/(ts[idx0+1]-ts[idx0])
        return copy.deepcopy(ts[:idx0+1]+[t]+ts[idx0+1:]),copy.deepcopy(vs[:idx0+1]+[tmp]+vs[idx0+1:])

def signal_reds(turningsignalseries):
    """
    convert the turning signals to reds. 
    ----------------------------------------
    input: turningsignalseries, a dict. 
        turningsignalseries['left'] = [(t1, 'r'),(t2, 'G'),...], means at moment t1, the signal turn to red, at moment t2, turns to green.
    output: turninggreens
        turningreds['left'] = [(t1,t2),(t3,t4)...], means that the red signal is from t1 to t2, from t3 to t4. Generally t3>t2.
    
    """
    turningreds = {}
    for td in turningsignalseries.keys():
        
        tmp = pyinter.IntervalSet()
        #turningsignalseries[0] = (t,'r')
        for switch0,switch1 in zip(turningsignalseries[td][:-1],turningsignalseries[td][1:]):
            #t is float, and state is 'r' or 'g' or 'G'
            t0,state0 = switch0
            t1,_ = switch1
            if state0 in {'R','r'}:
                tmp.add(pyinter.interval.closed(t0,t1))
        turningreds[td] = list(sorted(tmp))
    return turningreds

    pass

def read_probe_data(datafile):
    """

    """
    vehicles_snapshot_pd_dict = Process_ProbeData(datafile)
    probepd_dictdata_index_2float(vehicles_snapshot_pd_dict)
    Probe_pd_sort_temporal(vehicles_snapshot_pd_dict)
    return vehicles_snapshot_pd_dict


def signal_cycle_bounds(reds):
    """
    get the start and end of one cycle. 
    ----------------------------------------
    input: reds, a list
        the reds of the lane. 
        reds[i] = (s,e), s and e are the onset and termination moment of the red signal. 
    output: cycles_s_e
        cycles_s_e['left'] = [(t1,t2),(t3,t4)...], means that the first cycls starts from t1 to t2, the second cycle starts from t3 to t4. 
        the number of cycle is len(reds)-1.
    ----------------------------------------
    Steps:
        - 
    """
    pass

def signalgreens(turningsignalseries):
    """
    
    input: turningsignalseries
        turningsignalseries['left'] = [(t1, 'r'),(t2, 'G'),...], means at moment t1, the signal turn to red, at moment t2, turns to green.
    output: turninggreens
        turninggreens['left'] = [(t1,t2),(t3,t4)...], means that the green signal is from t1 to t2, from t3 to t4. Generally t3>t2.
    
    """
    turninggreens = {}
    for td in turningsignalseries.keys():
        #t is float, and state is 'r' or 'g' or 'G'
        tmp = pyinter.IntervalSet()
        #turningsignalseries[0] = (t,'r')
        for switch0,switch1 in zip(turningsignalseries[td][:-1],turningsignalseries[td][1:]):
            t0,state0 = switch0
            t1,_ = switch1
            if state0 in {'g','G'}:
                tmp.add(pyinter.interval.closed(t0,t1))
        turninggreens[td] = list(sorted(tmp))
            
    return turninggreens
        

def Process_signals_stateswitch(datafile):
    """
    process the signal data. 
    each entry in the datafile is like:
        <tlsState time="40.00" id="intersection" programID="my_program" phase="1" state="GGrrrrGGrrrrGGrrrrGGyyyy"/>
    ----------------------------------------------------
    output: res
        a pd.DataFrame data. 
        columns fields are ['time', 'id', 'programID', 'phase', 'state'].
        res.time record the switch moments. one res.state element is such as "GGrrrrGGrrrrGGrrrrGGgggg"
    """
    tree = etree.ElementTree(file=datafile)
    root = tree.getroot()
    #res = ['time', 'id', 'programID', 'phase', 'state']
    res = pd.DataFrame(columns=root[0].keys())
    indexxx = 0
    for lineinfo in root:
        res.loc[indexxx, :] = pd.Series(dict(lineinfo.attrib))
        indexxx = indexxx + 1
    
    return res

def Process_trip(datafile):
    """
        Output: res
                pd.DataFrame, colmns are :
                        Index([u'id', u'depart', u'departLane', u'departPos', u'departSpeed',
                               u'departDelay', u'arrival', u'arrivalLane', u'arrivalPos',
                               u'arrivalSpeed', u'duration', u'routeLength', u'waitSteps', u'timeLoss',
                               u'rerouteNo', u'devices', u'vType', u'vaporized'],
                              dtype='object')
                        Details are given at http://sumo.dlr.de/wiki/Simulation/Output/TripInfo
        """
    tree = etree.ElementTree(file=datafile)
    root = tree.getroot()
    res = pd.DataFrame(columns=root[0].keys())
    indexxx = 0
    for tripinfo in root:
        res.loc[indexxx, :] = pd.Series(dict(tripinfo.attrib))
        indexxx = indexxx + 1

    return res.convert_objects(convert_numeric=True)


def Process_QueueLengthGet(datafile):
    """
        Get the queue information from the SUMO results.
        The queue output file is generated by --queue-output <FILE> option. 
        
        -----------------------------------------------------
        Input: datafile
                str, such as :
                        datafile = "/home/qhs/Qhs_Files/Program/Python/GraphicalSolutionArterialRoad/tmp_files/queue.xml"
        Output:Queuelength
                a dict. Queuelength[lane_id] = {'time':a list, 'length': a list}
                Can be plot by plt.plot(Queuelength[lane_id]['time'], Queuelength[lane_id]['length'])
                
                Queuelength[lane_id]['time'] and Queuelength[lane_id]['length'] are both list. 
                
        """
    tree = etree.ElementTree(file=datafile)
    root = tree.getroot()
    Queuelength = {}
    for data in root:
        timee = data.attrib['timestep']
        for lanes in data:
            if len(lanes) > 0:
                for lane in lanes:
                    if not Queuelength.has_key(lane.attrib['id']):
                        Queuelength[lane.attrib['id']] = {
                            'time': [],
                            'length': []
                        }
                    Queuelength[lane.attrib['id']]['time'].append(timee)
                    Queuelength[lane.attrib['id']]['length'].append(
                        lane.attrib['queueing_length'])

    return Queuelength


def probepd_dictdata_index_2float(probedata_pd_dict):
    """
    conver the index (str, moment) in to float. 
    ------------------------------------
    input: probedata_pd_dict
        a dict. the keys are vehicle_id. each value is a pd.DataFrame. 
        The index is str, this function will convert this to float, i.e. the moment, which is measured in seconds. 
        """
    for v in probedata_pd_dict.keys():
        moments = [float(i) for i in probedata_pd_dict[v].index]
        probedata_pd_dict[v].index = moments
        pass

def spillover_paired_reds(velocities_approach, paired_lanes, paired_reds,kwargs):
    """
    ----------------------------------------------
    input: velocities_approach
        the speed at the interface. 
        dict. Keys are ['east_c_0', 'east_c_1', 'east_u_0', 'east_u_1', 'east_c_2']
        velocities_approach['east_c_0'] = (ts,vs)
    input: paired_reds
            Each element is a dict. paired_reds[i] = {'left': (t0,t1,t2), 'through': (t0,t1,t2)}.
            t0, t1 , t2 are the red start, red end and next red start. 
            It means that the left signal and thrugh signal are exact adjecent or partially overlap.
            If some element length is 1, then it means this red is single.  
    input: paired_lanes
        paired_lanes['left'] = ('east_c_0', 'east_u_1',..)
        The lanes used to justify the spillover. The spillover is considered to occuur when speed of all the lanes are below some threshold
    input: spillover_threshold
        unit is km/h. It is used to check the spillover occurence. 
    Output: spillovers
        the structure is the same as paired_reds.
        spillovers[cycle_idx][direction] = scalar, the duration of the direction. 
    ------------------------------------------
    Steps:
        - for each paired_red
            - for each turning direction
                - aggregate the partial ts,vs in tsvs_coupled (a dict, keys are paired_lanes[direction])
                    - get the t0,t1 and t2
                    - compute the shifted_t0,shifted_t1
                    - filter the ts,vs
                - compute the spillover duration

    """
    spillovers = []
    spillover_threshold = kwargs['spillover_threshold']

    loc_stopline = kwargs['loc_stopline']
    l_tb = kwargs['l_tb']
    w = kwargs['w']
    vf = kwargs['vf']
    direction_flag = kwargs['direction_flag']#if coordinate increase to upstrem then it is 1, else it is -1.
    tolerance = kwargs['tolerance']
    ufi = kwargs['ufi']

    for paired_red in paired_reds:
        spillovers_tmp = {}
        for td in paired_red.keys():
            #red onset, red end and green end
            t0,t1,t2 = paired_red[td]
            #count_t0,count_t1 is the time interval at the interface. 
            count_t0 = t0 + l_tb/(w/3.6)
            count_t1 = t1 + l_tb/(w/3.6)

            #prepare the speed for all lanes within interval [count_t0,count_t1]
            speeds_lanes = {}
            #aggregate the speeds. 
            for lane_label in paired_lanes[td]:
                #Filter the ts and vs
                ts = velocities_approach[lane_label][0]
                vs = velocities_approach[lane_label][1]
                idxx = ts_filter_return_idxs(ts, count_t0,count_t1)
                speeds_lanes[lane_label] = (list(np.array(ts)[idxx]), list(np.array(vs)[idxx]))

            #compute the spillover duration.
            spillovers_tmp[td] = spillover_duration_manylanes(speeds_lanes, threshold = spillover_threshold)
        spillovers.append(spillovers_tmp)
    return spillovers



def spilloverdurations_turning(velocities_approach, reds, l_tb =100, w = 20,turning = 'left',threshold = 10,TurningLanesPair=TurningLanesPair):
    """
    Obtaine the spillover durations for specific turning.
    ----------------------------------------------
    input: velocities_approach
        the speed at the interface. 
        dict. Keys are ['east_c_0', 'east_c_1', 'east_u_0', 'east_u_1', 'east_c_2']
        velocities_approach['east_c_0'] = (ts,vs)
    input: TurningLanesPair
        dict. Specify the relationship between the turning directions and the lanes ids. 
        TurningLanesPair['left'] is the lanes ids set corresponding to the leftturning movements. 
    input: reds
        the red durations. a list. reds[cycle_idx] is a pyinter.IntervalSet class. 
        reds[cycle_idx].upper_value - reds[cycle_idx].lower_value gives the red durations. 
    input: l_tb (unit is m) and w (unit is km/h)
        l_tb is the turning bay length, 
        w is the backward wave speed. 
    input: threshold
        the threshold of the speed that is considered as spillover. 
    ----------------------------------------------------
    Output: Spillovers
        a dict. Spillovers[cycle_idx] = float
    """
    #
    redshift2interface = l_tb/(w/3.6)

    Spillovers = {}
    Spillover_lanes = {}
    for lane in TurningLanesPair[turning]:
        #get the speed of the lane. 
        ts,vs = velocities_approach[lane]
        #spillovers, a pyinter.IntervalSet instance. 
        spillovers = spillovers_from_tsvs(ts,vs, threshold=threshold)
        #Spillover_lanes[lane] is a dict, Spillover_lanes[lane][cycle_idx] is a pyinter.IntervalSet instance.
        Spillover_lanes[lane] = spillovers_split2cycles(spillovers, reds = reds, l_tb=l_tb, w=w)
    
    Spillovers_intervalset = {}
    for cycle_idx in range(len(reds)):
        Spillovers_intervalset[cycle_idx] = []
        tmp = pyinter.IntervalSet([pyinter.interval.open(-np.inf,np.inf)])
        for lane in TurningLanesPair[turning]:
            tmp = tmp.intersection(Spillover_lanes[lane][cycle_idx])
            Spillovers_intervalset[cycle_idx].append(Spillover_lanes[lane][cycle_idx])
        #compute the spillover duration. 
        d = 0
        for interval in tmp:
            d = d+ interval.upper_value-interval.lower_value
        Spillovers[cycle_idx] = d
    return Spillovers,Spillovers_intervalset


def Get_optimization_inputdata(configs, sizeX, sizeY, signalfile, probefile, optimized_approach = 'east', optimize_turningdirection = 'left', p =.5, slice_U_section = True, w = 20, vf = 40):
    """
    Get the data that optimization model needed. 
    ------------------------------------------------------
    input: configs
        the configs file for the intersection.
        configs['left'] = {'lu':100, ....}
    input: sizeX and sizeY
        the size of the intersection (meters).
        It is the pure intersection size, (not include the appraoch lanes)
        It can be obtained by 
            sizeX, sizeY = aisumogenerate.SizeIntersection(configs)
    input: signalfile and probefile
        both are str. the files generated by the sumo.
    input: optimized_approach and optimize_turningdirection
        the desired optimization approach and direction.
    input: p
        the sample size of the vehicle trajecotries. 
    input: slice_U_section
        whether reserve the trjajectory at the C-section. If slice_U_section=True, it means the returned trajectory totally are within U-section.
    """
    ############################################################
    print "Getting the c section location and u section location....."
    #c_sectionlocation =
    #        {'east': (421.0, 521.0, 'x'),
    #         'north': (421.0, 521.0, 'y'),
    #         'south': (300, 400, 'y'),
    #         'west': (300, 400, 'x')}
    c_sectionlocation = C_section_spacerange(configs=configs,sizeX=sizeX,sizeY=sizeY)
    u_sectionlocation = U_section_spacerange(configs=configs,sizeX=sizeX,sizeY=sizeY)

    ############################################################
    print "Getting the l_tb and loc_stopline......"
    #l_tb and loc_stopline
    l_tb =  configs[optimized_approach]['ld']
    if optimized_approach in ['north', 'east']:
        loc_stopline = min(c_sectionlocation[optimized_approach][0],c_sectionlocation[optimized_approach][1])
    else:
        loc_stopline = max(c_sectionlocation[optimized_approach][0],c_sectionlocation[optimized_approach][1])

    ############################################################
    print "Getting the signals in turninggreens and turningreds......"
    #get signals in turninggreens and turningreds
    signalswitches = Process_signals_stateswitch(signalfile)
    signalswitches1 = signalstates_approach(signalswitches,approach=optimized_approach)
    #   turninggreens['left'] = [[t0,t1],[t2,t3], ....]
    turninggreens = signalgreens(signalswitches1)
    #   turningreds['left'] = [[t0,t1],[t2,t3], ....]
    turningreds  = signal_reds(signalswitches1)

    ############################################################
    print "Getting the probe data in vehicles_snapshot_pd_dict......"
    #read and prepross the probe data
    vehicles_snapshot_pd_dict = Process_ProbeData(probefile)
    #   convert the index (acctually is moments) to float
    probepd_dictdata_index_2float(vehicles_snapshot_pd_dict)
    #    sort the row, according to the moment. 
    Probe_pd_sort_temporal(vehicles_snapshot_pd_dict)
    
    ############################################################
    print "Getting the spillovers durations  in vehicles_snapshot_pd_dict......"
    velocities = css_interface_speed(vehicles_snapshot_pd_dict, intersection_config=configs, \
                                           u_sectionlocation = u_sectionlocation, approach = optimized_approach)
    #spilloversdurations_turning[cycle_idx] = duration
    spilloversdurations_turning= {}
    #    left turn spillover
    spilloversdurations_turning['left'],Spillovers_intervalset\
    = spilloverdurations_turning(velocities_approach=velocities,reds = turningreds['left'])
    #    through spillover
    spilloversdurations_turning['through'],Spillovers_intervalset\
    = spilloverdurations_turning(velocities_approach=velocities,reds = turningreds['through'])

    ############################################################
    print "Geting the full and sampled trajectories in tsxs_turning and tsxs_samples_turning......"
    l,t,r = trajectory_approach_turning(vehicles_snapshot_pd_dict,approach=optimized_approach)
    tsxs_turning = {'l':trajectory_2tsxs(l, c_sectionlocation[optimized_approach],slice_U_section=slice_U_section),'t':trajectory_2tsxs(t, c_sectionlocation[optimized_approach],slice_U_section=slice_U_section),'r':trajectory_2tsxs(r, c_sectionlocation[optimized_approach],slice_U_section=slice_U_section)}
    tsxs_samples_turning = trajectories_sample_with_turning(tsxs_turning, p =p)
    #Slice the trajectory
    #   cycles_sliceddata[cycle_idx][turningdirection][vehicle_id].keys()  = 
    #cycles_sliceddata = csswithsignal.Get_inputdata_oneturning(east_reds[optimize_turningdirection],tsxs_samples_turning,loc_stopline=loc_stopline,w=w,completeORnot=True)


    return l_tb, loc_stopline, turninggreens, turningreds, spilloversdurations_turning,tsxs_turning, tsxs_samples_turning

def spillover_events(velocities_interface, reds, w = 20, l_tb=100):
    """
    Get the spillover events. The events include the occurrence and the 
    ----------------------------------------
    input: velocities_interface
        the speed at the interface. The keys include:
            ['east_c_0', 'east_c_1', 'east_u_0', 'east_u_1', 'east_c_2']
        velocities_interface['east_c_0'] is a tuple ---> (ts,vs)
    input: reds
        a list. reds[i] is a pyinter.interval instance.
        reds[i].lower_value, 
    input: w l_tb
        w is the backward wave speed
        l_tb is the bay area length.
    output: 
    """

    pass

def spillover_duration_manylanes(speeds_lanes, threshold = 5):
    """
    Compute the spillovers. 
    The spillover is considered to occur only when the speed of all lanes are below the threshold. 
    -----------------------------------------------
    input: speeds_lanes
        speeds_lanes[lane_id] =(ts,vs)
        len(ts)==len(vs)
    input: threshold
        threshold for spillover check. 
    output: duration
        a scalar. The duration of the spillover. 
    ------------------------------------------------------
    Steps:
        - aggregate the moments
        - for each lane, get the spillover_intervalset, as a pyinter.IntervalSet. 
        - for each interval (t0,t1)
            - 
        - 
    """
    duration = 0

    #spillover_intervalset_dict[lane_id] = pyinter.IntervalSet
    spillover_intervalset_dict = {}
    for lane in speeds_lanes.keys():
        #print('sdsdfsadfsadf',speeds_lanes[lane])
        
        #if len(speeds_lanes[lane])
        ts,vs = speeds_lanes[lane][0], speeds_lanes[lane][1]
        spillover_intervalset_dict[lane] = tsvs_2_spillover_interset(ts,vs,threshold= threshold)

    #aggregate the moments of all ts in all lanes in moments
    #   moments are the union of all possible moments (within t0 and t1) for ts of each lane. 
    #moments[0]=t0 and moments[-1]=t1
    moments0 = []
    for k in speeds_lanes.keys():
        moments0.extend(speeds_lanes[k][0])
    moments = sorted(set(moments0))

    for t0,t1 in zip(moments[:-1], moments[1:]):
        spilloverflag = True
        for lane in spillover_intervalset_dict.keys():
            overlap = spillover_intervalset_dict[lane].intersection([pyinter.interval.closed(t0,t1)])
            duration_overlap = sum([m.upper_value-m.lower_value for m in overlap])
            if duration_overlap<t1-t0:
                spilloverflag = False
                break
        if spilloverflag:
            duration = duration + duration_overlap

    return duration





    #returned result
    duration = 0
    #first moment
    SpilloverFlag_FormerMoment = True
    for k in speeds_lanes.keys():
        #speeds_lanes[k][1] = vs
        if speeds_lanes[k][1][0]>threshold:
            SpilloverFlag_FormerMoment = False
            break
    #The remaining moment
    for idx,t in enumerate(moments[1:]):
        SpilloverFlag_ThisMoment = True
        for k in speeds_lanes.keys():
            if tsvs_interploate(speeds_lanes[k][0], speeds_lanes[k][1], t)>threshold:
                SpilloverFlag_ThisMoment = False
                break
        if SpilloverFlag_FormerMoment and SpilloverFlag_ThisMoment:
            duration = duration + t-moments[idx]
        SpilloverFlag_FormerMoment = SpilloverFlag_ThisMoment
    return duration



    pass

def spillovers_duration_from_tsvs(ts,vs, threshold = 5):
    """

    """
    res = 0
    #pyinter.IntervalSet
    spillovers = spillovers_from_tsvs(ts=ts,vs=vs,threshold=threshold)
    return  np.sum([s.upper_value-s.lower_value for s in spillovers])

def spillovers_from_tsvs(ts,vs, threshold = 5):
    """
    Generate the spillover durations from speed data, which is expressed using ts and vs..
    -----------------------------------------------------
    input: ts vs
        both are list. 
        len(ts)=len(vs)
    input: threshold, 
        the speed below which is assumed to be spillover. 
    OUTPUT: spillovers
        a pyinter.IntervalSet class. 
    """
    spillovers = pyinter.IntervalSet()
    for t0,t1,v in zip(ts[:-1],ts[1:],vs[:-1]):
        if v<=threshold and v>=0:
            spillovers.add(pyinter.interval.closed(t0,t1))
    
    return spillovers

def spillover_duration_for_cycles_intervalset(spillovers_cycles):
    """
    Get the spillover duration within each cycle. 
    ----------------------------------------------------
    input: spillovers_cycles
        a list. spillovers_cycles[i] is a pyinter.IntervalSet class.
    OUTPUT: 
        spilloverdurations_cycles. a list. spilloverdurations_cycles[cycle_idx] is a float, 
    """
    spilloverdurations_cycles = {}
    for cycle_idx in sorted(spillovers_cycles.keys()):
        if len(spillovers_cycles[cycle_idx])==0:
            spilloverdurations_cycles[cycle_idx] = 0
            continue
        d = 0
        for interval in spillovers_cycles[cycle_idx]:
            d = d+ interval.upper_value-interval.lower_value
        spilloverdurations_cycles[cycle_idx] = d
    return spilloverdurations_cycles


def spillovers_split2cycles(spillovers, reds, w = 20, l_tb=100):
    """
    split the spillovers (which is pyinter.IntervalSet instance) to each cycle. 
    -------------------------------------------------------------
    input: spillovers
        pyinter.IntervalSet instance. Each interval is a spillover duration.
        for i in spillovers:
            print i.lower_value,i.upper_value
    input: reds
        a list. reds[i] is a pyinter.interval instance.
        reds[i].lower_value, 
    input: w l_tb
        w is the backward wave speed
        l_tb is the bay area length.
    output: cycle_spillovers
        a dict.
        cycle_spillovers[cycle_idx] = pyinter.IntervalSet
    
    """
    cycle_spillovers = {}
    for i,red in enumerate(reds):
        start = red.lower_value + l_tb/(w/3.6)
        end  = red.upper_value + l_tb/(w/3.6)
        cycle_spillovers[i] = spillovers.intersection([pyinter.interval.closed(start,end)])
    return cycle_spillovers

def velocities_turning_cycles(velocities_approach, reds, TurningLanesPair=TurningLanesPair, w=20,l_tb=100,cycle_idx =1, buffer_time = 30, plott = True, approach = 'left'):
    """
    input: velocities_approach
        a dict. keys are the lane ids in sumo generated files. 
        velocities_approach['east_c_1'] = (ts,vs), len(ts)=len(vs). vs is the speed. 
    input: TurningLanesPair
        a dict. TurningLanesPair['left'] = ['east_c_2','east_u_1']
        means the left turning movement correspond to the lanes with id 'east_c_2' and 'east_u_1'
    input: reds
        a list, containing the red durations. 
        reds[cycle_idx] is a pyinter.interval instance. The duration is calculated as
            reds[cycle_idx].upper_value - reds[cycle_idx].lower_value
    input: cycle_idx
        a integer. Specify the durations of the intervals. 
    input: buffer_time
        the butter time when collection the speeds. 
    input: plott
        whether plot the speeds.
    """
    velocitiess = {}
    #get the interval
    t_start = reds[cycle_idx].lower_value-buffer_time
    t_end = reds[cycle_idx].upper_value + buffer_time

    #Select the speeds
    for lane_id in velocities_approach.keys():
        if lane_id not in TurningLanesPair[approach]:continue
        ts0,vs0 = velocities_approach[lane_id]
        new_ts = np.array(ts0)[np.where((np.array(ts0)>=t_start) & (np.array(ts0)<=t_end))        [0]]
        new_vs = np.array(vs0)[np.where((np.array(ts0)>=t_start) & (np.array(ts0)<=t_end))        [0]]

        velocitiess[lane_id] = (new_ts, new_vs)
    if plott:
        fig,ax = plt.subplots()
        for lane_id in velocitiess.keys():
            ts,vs = velocitiess[lane_id] 
            ax.plot(ts,vs)
    plt.show()

    return velocitiess
    
def css_interface_speed(vehicles_snapshot_pd_dict, intersection_config, u_sectionlocation, buffer_distance=30, approach = 'east', interval_t = 20):
    """
    get the spillover speed.
    -------------------------------------------
    input: vehicles_snapshot_pd_dict
        vehicle trajectory data, keys are vehicle ids, each value is a pandas.DataFrame.
    input: buffer_distance
        the distance of calculting the speed. For instance, at the upstream of interface, [l_c, l_c + buffer_distance] is used to calculate the speed at the CSS entrance. 
        [lc-buffer_distance, l_c] is used to calculate the speed within CSS section.
    input: intersection_config
        dict, keys include 'east','south','west','north'
        intersection_config['east']['lu'] and intersection_config['east']['ld']
    input: interval
        the interval for calculating the speed.
    input: u_sectionlocation
        the space range of U-section. 
        u_sectionlocation['east'] = (521.0, 821.0, 'x'), means that the u-section of east approach is from 521m to 821m, which is measured in x-axis. 

        THis can be obtained using:
            - u_sectionlocation = sumoa.U_section_spacerange(crte.configs,sizeX=sizeX,sizeY=sizeY)
    output: velocities
        velocities['west_u_0'] is the 0-th lane speed at U-section
        velocities['west_c_0'] is the 0-th lane speed at C-section
        lanes number is given in intersection_config[approach]['n_lanes_u_section']
        and 
            intersection_config[approach]['n_lanes_left']+
            intersection_config[approach]['n_lanes_through']+
            intersection_config[approach]['n_lanes_right']
    """
    velocities = {}

    #U-sections
    for i in range(intersection_config[approach]['n_lanes_u_section']):
        lane_id = approach+'_u_'+str(i)
        start_loc = intersection_config[approach]['lu'] - buffer_distance
        end_loc = intersection_config[approach]['lu']
        ts,vs = LaneAverageSpeed(vehicles_snapshot_pd_dict, laneid=lane_id, start_loc = start_loc, end_loc = end_loc, interval_t = interval_t)
        
        velocities[lane_id] = (ts,vs)

    #C-section
    lanes_N = intersection_config[approach]['n_lanes_left'] + intersection_config[approach]['n_lanes_through']+ intersection_config[approach]['n_lanes_right']
    #U-sections
    for i in range(lanes_N):
        lane_id = approach+'_c_'+str(i)
        start_loc = 0
        end_loc = buffer_distance
        ts,vs = LaneAverageSpeed(vehicles_snapshot_pd_dict, laneid=lane_id, start_loc = start_loc, end_loc = end_loc, interval_t = interval_t)
        
        velocities[lane_id] = (ts,vs)
    
    return velocities

def probedata_filterapproach(probedata_pd_dict, approach = 'east'):
    """
    Fileter the probe data, and get only the approach vehicles. 
    Note that the index in probedata_pd_dict[vehicle_id] must be float
    ----------------------------------------
    input: probedata_pd_dict
        the data obtained from the probe vehicles. it is a dict. 
        The keys are vehicle_id, the value is a pd.Dataframe. 
        columns are:
            Index([u'id', u'lane', u'pos', u'speed', u'x', u'y'], dtype='object')
    """
    partdata = {}
    tmp = probedata_pd_dict.keys()[0]
    if not isinstance(probedata_pd_dict[tmp].index[0],float):
        raise ValueError('zdfzdfsadf')
    approachvehicle_ids = []
    for v in probedata_pd_dict.keys():
        moment = min(probedata_pd_dict[v].index)
        if probedata_pd_dict[v].loc[moment,'lane'].split('_')[0]==approach:
            partdata[v] = probedata_pd_dict[v]
    return partdata

def U_section_spacerange(configs, sizeX =100,sizeY = 100):
    """
    ---------------------------------------
    input: sizeX sizeY
        the size of the intersection.
    input: configs
        a dict. configs['east']['lu']
    output: u_sectionlocation
        a dict. keys include 'east','south','north','west'
        u_sectionlocation['east'] = (100,120,'x') means the u section duration is from 100m to 120 m, and the measured direction is x axis
    """
    u_sectionlocation = {}
    #south
    u_sectionlocation['south'] = (0, configs['south']['lu'],'y')
    #north
    u_sectionlocation['north'] = (configs['south']['lu']+configs['south']['ld']+sizeY+configs['north']['ld'], configs['south']['lu']+configs['south']['ld']+sizeY+configs['north']['ld']+configs['north']['lu'], 'y')
    
    #west
    u_sectionlocation['west'] = (0, configs['west']['lu'],'x')
    #east
    u_sectionlocation['east'] = (configs['west']['lu']+configs['west']['ld']+sizeX+configs['east']['ld'], configs['west']['lu']+configs['west']['ld']+sizeX+configs['east']['ld']+configs['east']['lu'], 'x')
    
    
    return u_sectionlocation

def C_section_spacerange(configs, sizeX =100,sizeY = 100):
    """
    
    -------------------------------------
    input: sizeX sizeY
        the size of the intersection.
    input: configs
        a dict. configs['east']['lu']
    output: c_sectionlocation
        a dict. keys include 'east','south','north','west'
        c_sectionlocation['east'] = (100,120,'x') means the c section duration is from 100m to 120 m, and the measured direction is x axis
    """
    c_sectionlocation = {}
    #south
    c_sectionlocation['south'] = (configs['south']['lu'], configs['south']['lu']+configs['south']['ld'],'y')
    #north
    c_sectionlocation['north'] = (configs['south']['lu']+configs['south']['ld']+sizeY, configs['south']['lu']+configs['south']['ld']+sizeY+configs['north']['ld'], 'y')
    
    #west
    c_sectionlocation['west'] = (configs['west']['lu'], configs['west']['lu']+configs['west']['ld'],'x')
    #east
    c_sectionlocation['east'] = (configs['west']['lu']+configs['west']['ld']+sizeX, configs['west']['lu']+configs['west']['ld']+sizeX+configs['east']['ld'], 'x')
    
    
    return c_sectionlocation




def LaneAverageSpeed(trajectories, laneid='east_c_1', start_loc = 0, end_loc = 20, interval_t = 20):
    """
    Get the average speed given a road section using trajectories. 
    --------------------------------
    input: trajectories, a dict. 
        trajectories[vehicle_id] is a pd.DataFrame. The columns are: 
            Index([u'id', u'lane', u'pos', u'speed', u'x', u'y'], dtype='object')
        'lane' field value is as 'east_u_0', 'pos' gives the  location at the lane. 
        The index are the moments. Generally it should be set in the sumo 'add' file. 
    input: laneid
        the id of the lane. 'east_c_1' means the east approach c-section lane 1. 
        The lane is counter from rightmost, with id=0. 
        Thus 0 is the right-turn, 1 is through and 2 is for left-turn. 
    input: start_loc and end_loc
        the start and end of the data collection points. 
    input: interval_t
        the average interval. The vehicles 
    output: averagespeed
        a Series. 
        The keys are the moments in second, and the values are the average speed cprresponding to this moment.
    --------------------------------------------
    Steps:
        - filter the data that are within the lane and space interval, in FilteredData.
        - 
    
    """
    #the keys are moments, each value is a list
    #    containing the speed of this moment
    averagespeed = {}

    #first step: filter the data that are within the lane and space interval.
    FilteredData = {}
    for v_id in trajectories.keys():
        vdata = trajectories[v_id]
        tmp = vdata[(vdata['lane']==laneid) & (vdata['pos']>=start_loc) & (vdata['pos']<=end_loc)]
        if len(tmp)>0:FilteredData[v_id]=tmp

    print 'filtered data length is',len(FilteredData)
    #step 2, aggregate the data in dict
    for v_id in FilteredData.keys():
        #data's keys are moments, and value are speeds
        data = dict(FilteredData[v_id]['speed'])
        for k,v in data.iteritems():
            if averagespeed.has_key(k):
                averagespeed[k].append(v)
            else:
                averagespeed[k] = [v]
    
    #for moments that there is no vehicles, the speed is set to -1
    min_t,max_t = min(averagespeed.keys()),max(averagespeed.keys())
    all_ts = np.arange(min_t, max_t,interval_t)
    for t in all_ts:
        if not averagespeed.has_key(t):
            averagespeed[t] = -1
    
    ts = sorted(averagespeed.keys())
    v = [np.mean(averagespeed[t]) for t in ts]
        

    return ts,v

def trajectory_spatial_filter_2tsxs(vehicles_snapshot_pd_dict, c_sectionlocation):
    """
    FIlter the trajectory, and convert the pd to (ts,xs)
    ----------------------------------------------------
    input: vehicles_snapshot_pd_dict
        the data obtained from the probe vehicles. it is a dict. 
        The keys are vehicle_id, the value is a pd.Dataframe. 
        columns are:
            Index([u'id', u'lane', u'pos', u'speed', u'x', u'y'], dtype='object')
    input: c_sectionlocation
        c_sectionlocation=(421.0, 521.0, 'x') means the C-section of east approach is from 421 m to 521m which is measured in x axis. 
    output: trajectories_new
        a dict. trajectories_new[v_id] = ((ts),(xs))
    """
    res = {}
    part_vehicles_snapshot_pd_dict = {}
    for v_id in vehicles_snapshot_pd_dict.keys():
        trajectory = vehicles_snapshot_pd_dict[v_id]
        if c_sectionlocation[2]=='x':
            part_vehicles_snapshot_pd_dict[v_id] = copy.deepcopy(trajectory[(trajectory['x']<c_sectionlocation[1]) & (trajectory['x']>c_sectionlocation[0])])
            res[v_id]= (tuple( part_vehicles_snapshot_pd_dict[v_id].index), tuple( part_vehicles_snapshot_pd_dict[v_id].x))
        else:
            part_vehicles_snapshot_pd_dict[v_id] = copy.deepcopy(trajectory[(trajectory['y']<c_sectionlocation[1]) & (trajectory['y']>c_sectionlocation[0])])
            res[v_id]= (tuple( part_vehicles_snapshot_pd_dict[v_id].index), tuple( part_vehicles_snapshot_pd_dict[v_id].y))
    
    return res
    

def trajectory_2tsxs(vehicles_snapshot_pd_dict, c_sectionlocation, slice_U_section = True):
    """
    FIlter the trajectory, and convert the pd to (ts,xs)
    ----------------------------------------------------
    input: vehicles_snapshot_pd_dict
        the data obtained from the probe vehicles. it is a dict. 
        The keys are vehicle_id, the value is a pd.Dataframe. 
        columns are:
            Index([u'id', u'lane', u'pos', u'speed', u'x', u'y'], dtype='object')
    input: c_sectionlocation
        c_sectionlocation=(421.0, 521.0, 'x') means the C-section of east approach is from 421 m to 521m which is measured in x axis. 
    input: slice_U_section
        whether only reserve the trajectory at U-section. If yes, the trajectory at C-secton will be cutoff. 

        Estimation of CSS will only need the trajectories at U-section.
    output: trajectories_new
        a dict. trajectories_new[v_id] = ((ts),(xs))
    """
    res = {}
    for v_id in vehicles_snapshot_pd_dict.keys():
        trajectory = vehicles_snapshot_pd_dict[v_id]
        if c_sectionlocation[2]=='x':
            ts0 = np.array(trajectory.index)
            xs0 = np.array(trajectory.x)
            if slice_U_section:
                #means the coordinate increases from the stopline to upstream
                if xs0[-1]<xs0[0]:
                    idxx = np.where(xs0>=max(c_sectionlocation[0],c_sectionlocation[1]))[0]
                    ts = list(ts0[idxx]);xs = list(xs0[idxx])
                else:
                    idxx = np.where(xs0<=min(c_sectionlocation[0],c_sectionlocation[1]))[0]
                    ts = list(ts0[idxx]);xs = list(xs0[idxx])
            else:
                ts = ts0;xs = xs0
            res[v_id]= (tuple(ts),tuple(xs))
        else:
            ts0 = np.array(trajectory.index)
            xs0 = np.array(trajectory.y)
            if slice_U_section:
                #means the coordinate increases from the stopline to upstream
                if xs0[-1]<xs0[0]:
                    idxx = np.where(xs0>=max(c_sectionlocation[0],c_sectionlocation[1]))[0]
                    ts = list(ts0[idxx]);xs = list(xs0[idxx])
                else:
                    idxx = np.where(xs0<=min(c_sectionlocation[0],c_sectionlocation[1]))[0]
                    ts = list(ts0[idxx]);xs = list(xs0[idxx])
            else:
                ts = ts0;xs = xs0
            res[v_id]= (tuple(ts),tuple(xs))
    
    return res
    

def trajecotry_split_turning(approachvehiclepddict,approach = 'east'):
    """
    Split the approach trajectories into turning directions. 
    """
    #filtered data by the lanes they in, for instance must be in
    #   in edge 'east_u' and 'east_c', if approach is 'east'
    LeftVehicles = {}
    ThroughVehicles = {}
    RightVehicles = {}
    #the labels of edge names. 
    egedslabels = [approach +'_u', approach +'_c']
    for k in approachvehiclepddict.keys():
        #filter the data that are within the edge
        tmp  = approachvehiclepddict[k][ approachvehiclepddict[k].lane.str.match(r'^east_u') | approachvehiclepddict[k].lane.str.match(r'^east_c')]
        
        #
        if approach+'_c_0' in set(tmp.lane):
            RightVehicles[k] = pd.DataFrame.sort_index(tmp)
        elif approach+'_c_1' in set(tmp.lane):
            ThroughVehicles[k] = pd.DataFrame.sort_index(tmp)
        elif approach+'_c_2' in set(tmp.lane):
            LeftVehicles[k] = pd.DataFrame.sort_index(tmp)
        
    return LeftVehicles,ThroughVehicles,RightVehicles


def trajectory_approach_turning(probedata_pd_dict,approach = 'east'):
    """
    Get the trajectories of one approach. 
    ----------------------------------------
    input: probedata_pd_dict
        the data obtained from the probe vehicles. it is a dict. 
        The keys are vehicle_id, the value is a pd.Dataframe. 
        columns are:
            Index([u'id', u'lane', u'pos', u'speed', u'x', u'y'], dtype='object')
        
        
    input: approach
        specify the approach, either one in east south west north.
        Currently, at c-setion, lane 0 is right,lane 1 is through and lane 2 is left. 
    output: trajectories
        a dict. keys include 'left', 'through', 'right'
        trajectories['left'] is a dict. 
        trajectories['left'][vehicle_id] = (xs, lane_ids), xs is a list of locations
        lane_ids is the lane id. (lane id format is like 'east_u_1', means upstream section at east approach. lanes number is from 0, the rightmost lane.)
        
        location is measured from the road entrance. The location at stopline is the road lengh. 
        
        NOTE that for the whole network, the leftmost and lowermost is x=0 andy y=0. Hence the 
    ------------------------------------------
        """
    #approachvehiclepddict is just part of probedata_pd_dict
    #   the values are also pd.DataFrame.
    #   the columns are Index([u'id', u'lane', u'pos', u'speed', u'x', u'y'], dtype='object')
    approachvehiclepddict = probedata_filterapproach(probedata_pd_dict, approach)
    
    #filtered data by the lanes they in, for instance must be in
    #   in edge 'east_u' and 'east_c', if approach is 'east'
    LeftVehicles = {}
    ThroughVehicles = {}
    RightVehicles = {}
    #the labels of edge names. 
    egedslabels = [approach +'_u', approach +'_c']
    for k in approachvehiclepddict.keys():
        #filter the data that are within the edge
        tmp  = approachvehiclepddict[k][ approachvehiclepddict[k].lane.str.match(r'^'+str(approach)+r'_u') | approachvehiclepddict[k].lane.str.match(r'^'+str(approach)+r'_c')]
        
        #
        if approach+'_c_0' in set(tmp.lane):
            RightVehicles[k] = pd.DataFrame.sort_index(tmp)
        elif approach+'_c_1' in set(tmp.lane):
            ThroughVehicles[k] = pd.DataFrame.sort_index(tmp)
        elif approach+'_c_2' in set(tmp.lane):
            LeftVehicles[k] = pd.DataFrame.sort_index(tmp)
        
    return LeftVehicles,ThroughVehicles,RightVehicles


def trajectory_approach(probedata_pd_dict,approach = 'east'):
    """
    Get the trajectories of one approach. 
    ----------------------------------------
    input: probedata_pd_dict
        the data obtained from the probe vehicles. it is a dict. 
        The keys are vehicle_id, the value is a pd.Dataframe. 
        columns are:
            Index([u'id', u'lane', u'pos', u'speed', u'x', u'y'], dtype='object')
        
        
    input: approach
        specify the approach, either one in east south west north.
        Currently, at c-setion, lane 0 is right,lane 1 is through and lane 2 is left. 
    output: trajectories
        a dict. keys include 'left', 'through', 'right'
        trajectories['left'] is a dict. 
        trajectories['left'][vehicle_id] = (xs, lane_ids), xs is a list of locations
        lane_ids is the lane id. (lane id format is like 'east_u_1', means upstream section at east approach. lanes number is from 0, the rightmost lane.)
        
        location is measured from the road entrance. The location at stopline is the road lengh. 
        
        NOTE that for the whole network, the leftmost and lowermost is x=0 andy y=0. Hence the 
    ------------------------------------------
        """
    #approachvehiclepddict is just part of probedata_pd_dict
    #   the values are also pd.DataFrame.
    #   the columns are Index([u'id', u'lane', u'pos', u'speed', u'x', u'y'], dtype='object')
    approachvehiclepddict = probedata_filterapproach(probedata_pd_dict, approach)
    
    #filtered data by the lanes they in, for instance must be in
    #   in edge 'east_u' and 'east_c', if approach is 'east'
    LeftVehicles = {}
    ThroughVehicles = {}
    RightVehicles = {}
    approachvehiclepddict1 = {}
    #the labels of edge names. 
    egedslabels = [approach +'_u', approach +'_c']
    for k in approachvehiclepddict.keys():
        #filter the data that are within the edge
        tmp  = approachvehiclepddict[k][ approachvehiclepddict[k].lane.str.match(r'^east_u') | approachvehiclepddict[k].lane.str.match(r'^east_c')]
        
        #
        if approach+'_c_0' in set(tmp.lane):
            RightVehicles[k] = pd.DataFrame.sort_index(tmp)
        elif approach+'_c_1' in set(tmp.lane):
            ThroughVehicles[k] = pd.DataFrame.sort_index(tmp)
        elif approach+'_c_2' in set(tmp.lane):
            LeftVehicles[k] = pd.DataFrame.sort_index(tmp)
        
    return LeftVehicles,ThroughVehicles,RightVehicles
    #
    

def sumo_net2nx(net):
    """
        convert the sumo network to nx instance.
        
        """
    #
    G = nx.DiGraph()
    for edge in net.getEdges():
        e = edge.getFromNode().getID()
        s = edge.getToNode().getID()

        G.add_edge(e, s, id=edge.getID())
        #'shape': [(7321.04, 6266.78), (7314.63, 6254.49),.....]
        G[e][s]['shape'] = edge.getShape()
        #scalar
        G[e][s]['LaneNumber'] = edge.getLaneNumber()
        #scalar
        G[e][s]['Length'] = edge.getLength()
        #
        G[e][s]['Name'] = edge.getName()
        #traffic signal logic
        G[e][s]['TLS'] = edge.getTLS()

        G[e][s]['Priority'] = edge.getPriority()

        G[e][s]['Lanes'] = edge.getLanes()
    return G



def plotspeeds_interface(velocities, TurningLanesPair=TurningLanesPair, colors = {'left':'b','through':'r', 'right':'k'}):
    """
    plot the velocities of the lanes

    """
    alreadylegentd = []
    fig,ax = plt.subplots()
    for turning_direction in TurningLanesPair.keys():
        for laneid in TurningLanesPair[turning_direction]:
            ts,vs = velocities[laneid]
            if not turning_direction in alreadylegentd:
                ax.plot(ts,vs , color = colors[turning_direction], label = turning_direction)
                alreadylegentd.append(turning_direction)
            else:
                ax.plot(ts,vs , color = colors[turning_direction])

    ax.legend()
    return ax


from matplotlib.patches import Rectangle
from matplotlib.collections import PatchCollection

def plot_tsxs_turning_C_section_wavebound(approach_turning_TRs, approach_reds, c_sectionlocation, ax = None,alpha_LT = .5, alpha_TH = .6, alpha_c_section = .5,color_LTpatch = 'y',color_THpatch =  'm',w = 20.0,colors = {'l':'b','t':'r', 'r':'k'}, waveboundplot = 'LT'):
    """
    ---------------------------------------------
    input: approach_turning_TRs
        keys include 'l','t','r'
    input: waveboundplot, either 'LT' or 'TH'
        plot the left or through wave bound. 
    """
    #convert the wave speed to m/s
    w=w/3.6
    #used to patch c-section. 
    tmin = np.inf
    tmax = -np.inf
    xmin = np.inf
    xmax = -np.inf
    if not ax:
        fig,ax = plt.subplots()
    for td in approach_turning_TRs.keys():
        for v_id in approach_turning_TRs[td].keys():
            ts =  approach_turning_TRs[td][v_id][0]
            xs =    approach_turning_TRs[td][v_id][1]
        
            ax.plot(ts,  xs, color = colors[td])
            tmin = min(tmin, min(ts))
            tmax = max(tmax, max(ts))
            xmin = min(xmin, min(xs))
            xmax = max(xmax, max(xs))
    tmin=0
    #c-section patch
    tmp = Rectangle((tmin, c_sectionlocation[0]), tmax-tmin, c_sectionlocation[1]-c_sectionlocation[0])
    pc = PatchCollection([tmp],alpha = alpha_c_section)
    ax.add_collection(pc)
    
    #wave patch.
    patches = []
    #   left
    x0 = c_sectionlocation[0]
    if waveboundplot=='LT':
        reds = approach_reds['left']
    else:
        reds = approach_reds['through']
    for red in reds:
        s,e = red.lower_value, red.upper_value
        point0 = ((xmin-x0)/w+s,xmin)
        point1 = ((xmin-x0)/w+e,xmin)
        point2 = ((xmax-x0)/w+e,xmax)
        point3 = ((xmax-x0)/w+s,xmax)
        polygon = Polygon([point0,point1,point2,point3,point0])
        patches.append(polygon)
    p = PatchCollection(patches, edgecolor="None",alpha=alpha_LT)
    p.set_color(color_LTpatch)
    ax.add_collection(p)
    return ax
    pass

def plot_tsxs_C_section(trajectories_dict, c_sectionlocation, alpha=.7):
    """
    input: trajectories_dict
        keys are vehicle ids
        value are (ts,xs)
    input: c_sectionlocation
        c_sectionlocation=(421.0, 521.0, 'x') means the C-section of east approach is from 421 m to 521m which is measured in x axis. 
    """
    #used to patch c-section. 
    tmin = np.inf
    tmax = -np.inf
    
    fig,ax = plt.subplots()
    for v_id in trajectories_dict.keys():
        ax.plot(trajectories_dict[v_id][0],trajectories_dict[v_id][1])
        
        #used to patch the C-section.
        tmin = min(tmin, min(trajectories_dict[v_id][0]))
        tmax = max(tmax, max(trajectories_dict[v_id][0]))


    #c-section patch
    tmp = Rectangle((tmin, c_sectionlocation[0]), tmax-tmin, c_sectionlocation[1]-c_sectionlocation[0])
    pc = PatchCollection([tmp],alpha=alpha)
    ax.add_collection(pc)
    return ax


def plottrajectory_Csection(turningdirectiontrajectories,c_sectionlocation,direction = 'x', colors = {'l':'b','t':'r', 'r':'k'}, alpha=.5):
    """
    input: direction
        eiter 'x' or 'y'. if x, then turningdirectiontrajectories['l'][vehicle_id].x will be ploted, eitherwise y column will be ploted. 
    input: turningdirectiontrajectories
        dict, keys are 'l', 't' and 'r', meaning the three turning directions respectively. 
        turningdirectiontrajectories['l'] is a dict. 
        turningdirectiontrajectories['l'][vehicle_id] is a dataFrame. 
        columns are Index([u'id', u'lane', u'pos', u'speed', u'x', u'y'], dtype='object').
        index are moments. 
    input: colors
        different colors for different 
    input: c_sectionlocation
        c_sectionlocation=(421.0, 521.0, 'x') means the C-section of east approach is from 421 m to 521m which is measured in x axis. 
    """
    #used to patch c-section. 
    tmin = np.inf
    tmax = -np.inf
    
    fig,ax = plt.subplots()
    for td in turningdirectiontrajectories.keys():
        for v_id in turningdirectiontrajectories[td].keys():
            ts =  turningdirectiontrajectories[td][v_id].index
            if direction=='x':
                data = turningdirectiontrajectories[td][v_id].x
            else:
                data = turningdirectiontrajectories[td][v_id].y
        
            ax.plot(ts,  data, color = colors[td])
            
            #used to patch the C-section.
            tmin = min(tmin, min(ts))
            tmax = max(tmax, max(ts))
    #c-section patch
    tmp = Rectangle((tmin, c_sectionlocation[0]), tmax-tmin, c_sectionlocation[1]-c_sectionlocation[0])
    pc = PatchCollection([tmp],alpha=alpha)
    ax.add_collection(pc)
    
    return ax


def plottrajectory_signals_Csection(turningdirectiontrajectories,turninggreens, c_sectionlocation, signalloc = 0,signalthick = 4, direction = 'x', colors = {'l':'b','t':'r', 'r':'k'}, alpha=.5):
    """
    input: direction
        eiter 'x' or 'y'. if x, then turningdirectiontrajectories['l'][vehicle_id].x will be ploted, eitherwise y column will be ploted. 
    input: turningdirectiontrajectories
        dict, keys are 'l', 't' and 'r', meaning the three turning directions respectively. 
        turningdirectiontrajectories['l'] is a dict. 
        turningdirectiontrajectories['l'][vehicle_id] is a dataFrame. 
        columns are Index([u'id', u'lane', u'pos', u'speed', u'x', u'y'], dtype='object').
        index are moments. 
    input: colors
        different colors for different 
    input: c_sectionlocation
        c_sectionlocation=(421.0, 521.0, 'x') means the C-section of east approach is from 421 m to 521m which is measured in x axis. 
    input: turninggreens
        a dict. turninggreens['left'] is a list, containding start and end of each green signal. 
    input: signalloc and signalthick
        location display greens. 
        signalthick is the bar thickness. 
    """
    #used to patch c-section. 
    tmin = np.inf
    tmax = -np.inf
    
    fig,ax = plt.subplots()
    for td in turningdirectiontrajectories.keys():
        for v_id in turningdirectiontrajectories[td].keys():
            ts =  turningdirectiontrajectories[td][v_id].index
            if direction=='x':
                data = turningdirectiontrajectories[td][v_id].x
            else:
                data = turningdirectiontrajectories[td][v_id].y
        
            ax.plot(ts,  data, color = colors[td])
            tmin = min(tmin, min(ts))
            tmax = max(tmax, max(ts))
    #left signal, given by turninggreens['left']
    for duration in turninggreens['left']:
        t0,t1 =  duration.lower_value,duration.upper_value
        ax.plot([t0,t1], [signalloc,signalloc], lw = signalthick,color='g')
    
    for duration in turninggreens['through']:
        t0,t1 =  duration.lower_value,duration.upper_value
        ax.plot([t0,t1], [signalloc - 2*signalthick,signalloc- 2*signalthick], lw = abs(signalthick),color = 'y')
    
    #c-section patch
    tmp = Rectangle((tmin, c_sectionlocation[0]), tmax-tmin, c_sectionlocation[1]-c_sectionlocation[0])
    pc = PatchCollection([tmp],alpha=alpha)
    ax.add_collection(pc)
    
    return ax










