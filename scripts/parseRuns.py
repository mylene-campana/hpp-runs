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
    prefixNonOp = "Cost of non-optimized path: "
    prefixOpGB = "Cost of optimized path (GB): "
    prefixOpRS = "Cost of optimized path (RS): "
    prefixOptimGB = "Optim comptutation time (GB): "
    prefixOptimRS = "Optim comptutation time (RS): "
    prefixBase = "base motion distance: "
    prefixBaseRS = "base motion distance (RS): "
    prefixBaseGB = "base motion distance (GB): "
    
    ## Display data:
    print "Cost of non-optimized path parsing: "; compute (prefixNonOp)
    print "Cost of optimized path (GB): "; compute (prefixOpGB)
    print "Cost of optimized path (RS): "; compute (prefixOpRS)
    print "Optim comptutation time (GB): "; compute (prefixOptimGB)
    print "Optim comptutation time (RS): "; compute (prefixOptimRS)
    #print "base motion distance: "; compute (prefixBase)
    #print "base motion distance (RS): "; compute (prefixBaseRS)
    #print "base motion distance (GB): "; compute (prefixBaseGB)
    
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

# --------------------------------------------------------------------#

# Compute distance of the base motion (x-y) along a path
# wps: is the list of waypoints of the path
def computeBaseMotion (wps):
    dist = 0
    for i in range(0,len(wps)-1):
        wpi = wps [i]
        wpii = wps [i+1]
        dist += math.sqrt( (wpii [0] - wpi [0])**2 +  (wpii [1] - wpi [1])**2  )
        print dist
    return dist
