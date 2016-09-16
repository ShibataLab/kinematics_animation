#!/usr/bin/env python
from math import sin, cos, pi
import numpy as np

#human hand
joint_names = [
	"thumb1",
	"thumb2",
	"j_thumb",
	"j_thumb1",
	"j_thumb2",
	"j_thumb3",
	"j_index",
	"j_index1",
	"j_index3",
	"j_index5",
	"j_middle1",
	"j_middle3",
	"j_middle5",
	"j_ring",
	"j_ring1",
	"j_ring3",
	"j_ring5",
	"j_little",
	"j_little1",
	"j_little2",
	"j_little3",
	"j_little4",
	"j_little5",
	"little1",
	"little2",
	"little3",
	]

DT = 0.1
TF = 10.0

j1func = lambda t: 0.75*sin(t) + 2.95
j2func = lambda t: 0.3*sin(2*t) + 1.35
j3func = lambda t: 0.2*sin(2*t) - 2.59
j4func = lambda t: 0.4*sin(t)

tvec = np.arange(0,TF+DT,DT)
dat = np.zeros((len(tvec), len(joint_names)+1))

# write headers:
for i,t in enumerate(tvec):
	tmpdat = np.zeros(len(dat[i]))
	tmpdat[0] = t
	index = 0
	for j in range(5):
		tmpdat[index + 1] = j1func(t)
		tmpdat[index + 2] = j2func(t)
		tmpdat[index + 3] = j3func(t)
		tmpdat[index + 4] = j4func(t)
		tmpdat[index + 5] = j2func(t)
		index = index + 5
	tmpdat[len(joint_names)] = j1func(t)

	dat[i] = tmpdat

header = 'time,' + ','.join(joint_names)
np.savetxt("jointstates.csv", dat, header=header, comments="", delimiter=',', fmt="%3.6f")
