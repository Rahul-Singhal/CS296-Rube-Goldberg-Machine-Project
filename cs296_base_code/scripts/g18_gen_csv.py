#!/usr/bin/env python3
import 	subprocess
import sys
import re
import os


filename = 'data/lab09_g18_data.csv'
try:
    os.remove(filename)
except OSError:
    pass

dataFile = open(filename , "a+")

for rerun_no in range(1,101):
	for iteration_no in range(1,101):
		
		p = subprocess.check_output(["./bin/cs296_base_no_gui" , "%i" % iteration_no])
		output = ''
		p = str(p)
		output = output + str(rerun_no) + ','
		output += ','.join(re.findall(r'[0-9.]+', p))
		
		dataFile.write(output + '\n')
dataFile.close()







