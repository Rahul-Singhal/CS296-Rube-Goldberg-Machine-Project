#!/usr/bin/env python3
from matplotlib import pyplot
import csv
import numpy


data={'C1':[], 'C2':[], 'C3':[], 'C4':[], 'C5':[], 'C6':[], 'C7':[]}
csvFile = csv.reader(open("data/lab09_g18_data.csv", "r"))
for row in csvFile:
  data['C1'].append(row[0])
  data['C2'].append(row[1])
  data['C3'].append(row[2])
  data['C4'].append(row[3])
  data['C5'].append(row[4])
  data['C6'].append(row[5])
  data['C7'].append(row[6])

 

rstepTime = [0]*101
rloopTime = [0]*101
rcollisionTime = [0]*101
rvelocityTime = [0]*101
rpositionTime = [0]*101

rstepTimeMax = [0]*101
rstepTimeMin = [1000]*101

for i in range(0,10000):
	rstepTime[int(data['C2'][i])] += float(data['C3'][i])
	rloopTime[int(data['C2'][i])] += float(data['C7'][i])
	rcollisionTime[int(data['C2'][i])] += float(data['C4'][i])
	rvelocityTime[int(data['C2'][i])] += float(data['C5'][i])
	rpositionTime[int(data['C2'][i])] += float(data['C6'][i])
	
	if(rstepTimeMax[int(data['C2'][i])] < float(data['C3'][i])):
		rstepTimeMax[int(data['C2'][i])] = float(data['C3'][i])
	if(rstepTimeMin[int(data['C2'][i])] > float(data['C3'][i])):
		rstepTimeMin[int(data['C2'][i])] = float(data['C3'][i])
	
	
for i in range(1,101):
	rstepTime[i] /= 100
	rloopTime[i] /= 100
	rcollisionTime[i] /= 100
	rvelocityTime[i] /= 100
	rpositionTime[i] /= 100

del(rstepTime[0])
del(rloopTime[0])
del(rcollisionTime[0])
del(rvelocityTime[0])
del(rpositionTime[0])

#ax = pyplot.subplot(111)
pyplot.ylim([0,40])
p1, = pyplot.plot( list(range(1,101)), rstepTime, '+')
p2, = pyplot.plot( list(range(1,101)), rloopTime, '+')
pyplot.legend([p1 , p2] , ["step time avg over reruns" , "loop time avg over reruns"]  , prop={'size':10})
pyplot.title( 'loop time and step time avg. over all reruns for iteration values' )
pyplot.xlabel( 'ITERATION VALUES' )
pyplot.ylabel( 'STEP TIME & LOOP TIME' )
pyplot.savefig( 'plots/g18_lab09_plot01.png' )
pyplot.show()

pyplot.clf()


p1, = pyplot.plot( list(range(1,101)), rstepTime, '+')
p2, = pyplot.plot(list(range(1,101)), rcollisionTime, '+')
p3, = pyplot.plot(list(range(1,101)), rvelocityTime, '+' )
p4, = pyplot.plot(list(range(1,101)), rpositionTime, '+')
pyplot.legend([p1 , p2 , p3 , p4] , ["step time avg. Vs iteration values" , "collision time avg. Vs iteration values" , "velocity time avg. Vs iteration values" , "position time avg. Vs iteration values"] , prop={'size':10})
pyplot.title( 'Various times averaged over reruns for various iteration values' )
pyplot.xlabel( 'ITERATION VALUES' )
pyplot.ylabel( 'TIME' )
pyplot.savefig( 'plots/g18_lab09_plot02.png' )
pyplot.show()


pyplot.clf()

istepTime = [0]*101
iloopTime = [0]*101
icollisionTime = [0]*101
ivelocityTime = [0]*101
ipositionTime = [0]*101



for i in range(0,10000):
	istepTime[int(data['C1'][i])] += float(data['C3'][i])
	iloopTime[int(data['C1'][i])] += float(data['C7'][i])
	icollisionTime[int(data['C1'][i])] += float(data['C4'][i])
	ivelocityTime[int(data['C1'][i])] += float(data['C5'][i])
	ipositionTime[int(data['C1'][i])] += float(data['C6'][i])
	
	
	
	
	
for i in range(1,101):
	istepTime[i] /= 100
	iloopTime[i] /= 100
	icollisionTime[i] /= 100
	ivelocityTime[i] /= 100
	ipositionTime[i] /= 100

del(istepTime[0])
del(iloopTime[0])
del(icollisionTime[0])
del(ivelocityTime[0])
del(ipositionTime[0])
del(rstepTimeMin[0])
del(rstepTimeMax[0])

pyplot.ylim(0 , 45)
p1, = pyplot.plot( list(range(1,101)), istepTime, '+')
p2, = pyplot.plot(list(range(1,101)), iloopTime, '+' )
pyplot.legend([p1 , p2] , ["step time avg over iterations" , "loop time avg over iterations"] , prop={'size':10})
pyplot.title( 'loop time and step time avg. over all iteration for rerun values' )
pyplot.xlabel( 'RERUN VALUES' )
pyplot.ylabel( 'STEP TIME & LOOP TIME' )
pyplot.savefig( 'plots/g18_lab09_plot03.png' )
pyplot.show()

pyplot.clf()

pyplot.ylim(0 , 10)
p1, = pyplot.plot( list(range(1,101)), istepTime, '+')
p2, = pyplot.plot( list(range(1,101)), icollisionTime, '+')
p3, = pyplot.plot( list(range(1,101)), ivelocityTime, '+' )
p4, = pyplot.plot( list(range(1,101)), ipositionTime, '+')
pyplot.legend([p1 , p2 , p3 , p4] , ["step time avg. Vs rerun values" , "collision time avg. Vs rerun values" , "velocity time avg. Vs rerun values" , "position time avg. Vs rerun values"] , prop={'size':10})
pyplot.title( 'Various times avg. over iterations for various rerun values' )
pyplot.xlabel( 'RERUN VALUESs' )
pyplot.ylabel( 'TIME' )
pyplot.savefig( 'plots/g18_lab09_plot04.png' )
pyplot.show()

pyplot.clf()


	
for i in range(0,100):
	rstepTimeMax[i] = rstepTimeMax[i] - rstepTime[i] 
	rstepTimeMin[i] = rstepTime[i] - rstepTimeMin[i] 
	

p1 = pyplot.errorbar( list(range(1,101)), rstepTime  , yerr=[rstepTimeMin , rstepTimeMax] , fmt='bo')
#pyplot.plot( list(range(1,101)), istepTime , '+' , list(range(1,101)), iloopTime, '+')
pyplot.legend([p1] , ["Error in step time wrt avg. values"] , prop={'size':10})
pyplot.title( 'Variation in Step Time wrt iteration number averaged over reruns' )

pyplot.xlabel( 'ITERATION VALUES' )
pyplot.ylabel( 'STEP TIME' )
pyplot.savefig( 'plots/g18_lab09_plot05.png' )
pyplot.show()

pyplot.clf()

hist = []


for i in range(0,10000):
	if(data['C2'][i] == '6'):
		hist.append(data['C3'][i])
		

pyplot.ylim(0 , 120)
num_bins = 20
counts, bin_edges = numpy.histogram([float(i) for i in hist], bins=num_bins)
cdf = numpy.cumsum(counts)
p1, = pyplot.plot(bin_edges[1:], cdf)
n , bins , patches = pyplot.hist([float(i) for i in hist] , bins=20  )
p = pyplot.Rectangle((0, 0), 1, 1, fc="g")
pyplot.legend([p , p1], ["Step Time Freq." , "Cumulative Step Time Freq."] , prop={'size':10})
pyplot.title( 'loop time avg over reruns' )
pyplot.xlabel( 'Step Time' )
pyplot.ylabel( 'Frequency' )
pyplot.savefig( 'plots/g18_lab09_plot06.png' )
pyplot.show()








