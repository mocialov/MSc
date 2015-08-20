#! /usr/bin/python

""" Detector task"""

### IMPORTS ###
import random
import os

# Libraries
import numpy as np
from scipy.misc import imread

# Local
import os
import sys
sys.path.append(os.path.join(os.path.split(__file__)[0],'..','..','..')) 
from peas.networks import rnn
from peas.methods import neat

class DetectorTask(object):
    """ Detector task """
    
    def __init__(self):
        # Settings
        
    def evaluate(self, network, draw=False, drawname='Simulation'):
        
    def solve(self, network):
        
    def visualize(self, network, filename=None):        

if __name__ == '__main__':
    a = DetectorTask()