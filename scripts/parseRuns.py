#/usr/bin/env python

# Parser script for run tests results
# Average and Variance of parsed values :
# path durations, number of 'new' nodes, computation time

"""
from parseRuns import main
main()
"""

from __future__ import division
import numpy as np
import math

runFile = "/local/mcampana/devel/hpp/src/hpp-runs/scripts/"
fileName = "results.txt"

# --------------------------------------------------------------------#

def main ():
    #prefixNodes = "Number of nodes: "
    prefixNonOp = "Cost of non-optimized path: "
    prefixOpGB = "Cost of optimized path (GB): "
    prefixOpRS = "Cost of optimized path (RS): "
    prefixSolv = "Solving comptutation time: "
    prefixOptimGB = "Optim comptutation time (BG): "
    prefixOptimRS = "Optim comptutation time (RS): "
    prefixNbWaypoints = "Nb waypoints: "
    prefixNbIterGB = "Nb iterations (GB): "
    prefixNbIterRS = "Nb iterations (RS): "
    
    ## Display data:
    #print "Number of new nodes: "; compute (prefixNodes)
    print "Cost of non-optimized path parsing: "; compute (prefixNonOp)
    print "Cost of optimized path (GB) parsing: "; compute (prefixOpGB)
    print "Cost of optimized path (RS) parsing: "; compute (prefixOpRS)
    print "Solving comptutation time parsing: "; compute (prefixSolv)
    print "Optim comptutation time (BG) parsing: "; compute (prefixOptimGB)
    print "Optim comptutation time (RS) parsing: "; compute (prefixOptimRS)
    #print "Nb waypoints parsing: "; compute (prefixNonOp)
    print "Nb iterations (GB) parsing: "; compute (prefixNbIterGB)
    print "Nb iterations (RS) parsing: "; compute (prefixNbIterRS)
    
# --------------------------------------------------------------------#

def parseRun (prefix):
    l = len (prefix)
    with open (runFile + fileName) as f:
        vector = []
        for line in f:
            if line [:l] == prefix :
                elem = line [l:]
                st = elem.strip ('\n') # remove end characters
                try:
                    #config = map (float, elem) # convert into float
                    vector.append (float(st))
                except:
                    print "catched"
    return np.array (vector)

# --------------------------------------------------------------------#

# Compute average of a vector
def average (vector):
    av=0
    for elem in vector:
        av += elem
    av = av/len(vector)
    print "average: "+str(av)
    return av

# --------------------------------------------------------------------#

# Compute variance of a vector (Konig-Huygens formula)
def variance (vector):
    av = average (vector)
    var=0
    for elem in vector:
        var += elem**2
    var = var/len(vector)
    var = var - av**2
    if var<0:
        print "Variance is <0 because of rounding errors: "+str(var)
        var=0
    return var

# --------------------------------------------------------------------#

# Compute average and SD for the given inString indicating a vector of floats in 'results.txt'
def compute (inString):
    vector = parseRun (inString)
    #print vector
    #print "length: " + str(len(vector))
    print "SD: "+str(math.sqrt(variance (vector)))+"\n"
