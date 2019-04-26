__author__ = 'sudab'
""" Generate a grid world """
import os, sys, getopt, pdb, string
import random
import numpy as np
from skimage import io
import cv2

class Gridworld():
    # a gridworld with uneven terrain
    def __init__(self, filename=None, initial=0, nrows=8, ncols=8, nagents=1, targets=[], obstacles=[], moveobstacles = [], regions=dict()):
        # walls are the obstacles. The edges of the gridworld will be included into the walls.
        # region is a string and can be one of: ['pavement','gravel', 'grass', 'sand']
        if filename[0] != None:
            data = io.imread(filename[0])
            data = cv2.resize(data, filename[1], interpolation=filename[2])
            regionkeys = {'pavement', 'gravel', 'grass', 'sand', 'deterministic'}
            (nrows,ncols) = data.shape[:2]
            data = data.flatten()
            obstacles = list(np.where(data<254)[0])

            regions = dict.fromkeys(regionkeys, {-1})
            regions['deterministic'] = range(nrows * ncols)

        self.current = initial
        self.nrows = nrows
        self.ncols = ncols
        self.obstacles = obstacles
        self.regions = regions
        self.nagents = nagents
        self.nstates = nrows * ncols
        self.nactions = 5
        self.obstacles = obstacles
        self.actlist = ['R','N', 'S', 'W', 'E']
        self.targets = targets
        self.left_edge = []
        self.right_edge = []
        self.top_edge = []
        self.bottom_edge = []
        self.regions = regions
        self.moveobstacles = moveobstacles
        self.states = range(nrows*ncols)
        self.colorstates = set()
        for x in range(self.nstates):
            # note that edges are not disjoint, so we cannot use elif
            if x % self.ncols == 0:
                self.left_edge.append(x)
            if 0 <= x < self.ncols:
                self.top_edge.append(x)
            if x % self.ncols == self.ncols - 1:
                self.right_edge.append(x)
            if (self.nrows - 1) * self.ncols <= x <= self.nstates:
                self.bottom_edge.append(x)
        self.edges = self.left_edge + self.top_edge + self.right_edge + self.bottom_edge
        self.walls = self.edges + obstacles
        self.prob = {a: np.zeros((self.nstates, self.nstates)) for a in self.actlist}

        self.probOfSuccess = dict([])
        self.getProbRegions()

        for s in self.states:
            for a in self.actlist:
                self.getProbs(s, a)

    def coords(self, s):
        return (s / self.ncols, s % self.ncols)  # the coordinate for state s.

    def isAllowed(self, (row,col)):
        if col not in range(self.ncols) or row not in range(self.nrows):
            return False
        return True

    def isAllowedState(self,(row,col),returnState):
        if self.isAllowed((row,col)):
            return self.rcoords((row,col))
        return returnState

    def getProbRegions(self):
        probOfSuccess = dict([])
        for ground in self.regions.keys():
            for direction in ['N', 'S', 'E', 'W']:
                if ground == 'pavement':
                    mass = random.choice(range(90, 95))
                    massleft = 100 - mass
                    oneleft = random.choice(range(1, massleft))
                    twoleft = massleft - oneleft
                if ground == 'gravel':
                    mass = random.choice(range(80, 85))
                    massleft = 100 - mass
                    oneleft = random.choice(range(1, massleft))
                    twoleft = massleft - oneleft
                if ground == 'grass':
                    mass = random.choice(range(85, 90))
                    massleft = 100 - mass
                    oneleft = random.choice(range(1, massleft))
                    twoleft = massleft - oneleft
                if ground == 'sand':
                    mass = random.choice(range(65, 70))
                    massleft = 100 - mass
                    oneleft = random.choice(range(1, massleft))
                    twoleft = massleft - oneleft
                if ground == 'deterministic':
                    mass = 100
                    oneleft = 0
                    twoleft = 0
                probOfSuccess[(ground, direction)] = [float(mass) / 100, float(oneleft) / 100, float(twoleft) / 100]
        self.probOfSuccess = probOfSuccess
        return

    def rcoords(self, coords):
        s = coords[0] * self.ncols + coords[1]
        return s

    def getProbs(self, state, action):
        successors = []

        if state in self.obstacles:
            successors = [(state, 1)]
            for (next_state, p) in successors:
                self.prob[action][state, next_state] = p
                return
        row,col = self.coords(state)
        northState = self.isAllowedState((row-1,col),state)
        northwestState = self.isAllowedState((row-1,col-1),state)
        northeastState = self.isAllowedState((row-1,col+1),state)
        southState = self.isAllowedState((row+1,col),state)
        southeastState = self.isAllowedState((row+1,col+1),state)
        southwestState = self.isAllowedState((row+1,col-1),state)
        westState = self.isAllowedState((row,col-1),state)
        eastState = self.isAllowedState((row,col+1),state)

        reg = self.getStateRegion(state)
        if action == 'N':
            [p0, p1, p2] = self.probOfSuccess[(reg, 'N')]
            successors.append((northState, p0))
            successors.append((northwestState, p1))
            successors.append((northeastState, p2))

        if action == 'S':
            [p0, p1, p2] = self.probOfSuccess[(reg, 'S')]
            successors.append((southState, p0))
            successors.append((southwestState, p1))
            successors.append((southeastState, p2))

        if action == 'W':
            [p0, p1, p2] = self.probOfSuccess[(reg, 'W')]
            successors.append((westState, p0))
            successors.append((southwestState, p1))
            successors.append((northwestState, p2))

        if action == 'E':
            [p0, p1, p2] = self.probOfSuccess[(reg, 'E')]
            successors.append((eastState, p0))
            successors.append((southeastState, p1))
            successors.append((northeastState, p2))

        if action == 'R':
            successors.append((state,1))

        for (next_state, p) in successors:
            self.prob[action][state, next_state] += p

    def getStateRegion(self, state):
        if state in self.regions['pavement']:
            return 'pavement'
        if state in self.regions['grass']:
            return 'grass'
        if state in self.regions['gravel']:
            return 'gravel'
        if state in self.regions['sand']:
            return 'sand'
        if state in self.regions['deterministic']:
            return 'deterministic'

    