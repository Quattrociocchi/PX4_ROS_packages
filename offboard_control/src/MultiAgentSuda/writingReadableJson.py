#!/usr/bin/env python

import simplejson as json

def writeJson(infile,outfile,dict=None):
    if dict is None:
        dict = parseJson(infile)
    j = json.dumps(dict, indent=1)
    f = open(outfile, 'w')
    print >> f, j
    f.close()

def parseJson(filename):
    automaton = dict()
    file = open(filename)
    data = json.load(file)
    file.close()
    variables = dict()
    for var in data['variables']:
            v = var.split('@')[0]
            if v not in variables.keys():
                for var2ind in range(data['variables'].index(var),len(data['variables'])):
                    var2 = data['variables'][var2ind]
                    if v != var2.split('@')[0]:
                        variables[v] = [data['variables'].index(var), data['variables'].index(var2)]
                        break
                    if data['variables'].index(var2) == len(data['variables'])-1:
                        variables[v] = [data['variables'].index(var), data['variables'].index(var2) + 1]

    for s in data['nodes'].keys():
        automaton[int(s)] = dict.fromkeys(['State','Successors'])
        automaton[int(s)]['State'] = dict()
        automaton[int(s)]['Successors'] = []
        for v in variables.keys():
            if variables[v][0] == variables[v][1]:
                bin  = [data['nodes'][s]['state'][variables[v][0]]]
            else:
                bin = data['nodes'][s]['state'][variables[v][0]:variables[v][1]]
            automaton[int(s)]['State'][v] = int(''.join(str(e) for e in bin)[::-1], 2)
            automaton[int(s)]['Successors'] = data['nodes'][s]['trans']
    return automaton



def computeAutomatonState(automaton,currstate,state):
    allautstates = set()
    for autstate in automaton[currstate]['Successors']:
        check = 0
        for var in state.keys():
            if automaton[autstate]['State'][var] == state[var] or state[var] == None:
                check+=1
            else:
                break
        if check == len(state.keys()):
            allautstates.add(autstate)
    return allautstates

##############################

if __name__ == "__main__":

    # Get the automaton from the json file
    directory = '/home/jaq394l/catkin_ws/src/PX4_ROS_packages/offboard_control/src/MultiAgentSuda/' # <------ Set this to the directory where the json file lives. NOTE the trailing / 
    # jsonInputFile = directory + 'Example2_Gazebo.json' # <------ This is the name of your json file. NOTE there is no / in front of the name
    jsonInputFile = 'Controller_liveness_02_01.json'
    # readableOutputFile = directory + 'readable_Example2_Gazebo.json' # <------ This is the name of the more readable file that will be saved. NOTE there is no / in front of the name
    readableOutputFile = 'readable_Controller_liveness_02_01.json'
    writeJson(jsonInputFile,readableOutputFile)


    # # Example of making an automaton. Can be used to generate a dictionary that can be used for simulation.
    # A = parseJson(jsonInputFile)
