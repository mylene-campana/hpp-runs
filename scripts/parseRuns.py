#/usr/bin/env python

# Parser script for run tests results
# Average and Variance of parsed values :
# path durations, number of 'new' nodes, computation time

"""
from parseRuns import main
main()
from parseRuns import variance
variance([0, 2, 2,  2, 0])
"""

from __future__ import division
import numpy as np
import math

runFile = "/local/mcampana/devel/hpp/src/hpp-runs/scripts"
fileName = "results.txt"

# --------------------------------------------------------------------#

def main ():
    prefixNodes = "Number of nodes: "
    prefixNonOp = "Duration of non-optimized path: "
    prefixOp = "Duration of optimized path: "
    prefixSolv = "Solving duration: "
    prefixOptim = "Optim duration: "
    
    print "Number of new nodes: "
    vector = parseRun (prefixNodes); print vector; #avNew = average (vector)
    varNew = variance (vector); print "deviation: "+str(math.sqrt(varNew))+"\n"
    
    print "Non-optim-length duration parsing: "
    vector = parseRun (prefixNonOp); print vector; #avNonOp = average (vector)
    varNonOp = variance (vector); print "deviation: "+str(math.sqrt(varNonOp))+"\n"
    
    print "Optim-length duration parsing: "
    vector = parseRun (prefixOp); print vector; #avOp = average (vector)
    varOp = variance (vector); print "deviation: "+str(math.sqrt(varOp))+"\n"
    
    print "Solve duration parsing: "
    vector = parseRun (prefixSolv); print vector; #avSolv = average (vector)
    varSolv = variance (vector); print "deviation: "+str(math.sqrt(varSolv))+"\n"
    
    print "Optim duration parsing: "
    vector = parseRun (prefixOptim); print vector; #avSolv = average (vector)
    varSolv = variance (vector); print "deviation: "+str(math.sqrt(varSolv))+"\n"
    
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
    print "length: " + str(len(vector))
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
    #print "variance: "+str(var)
    return var

# --------------------------------------------------------------------#
