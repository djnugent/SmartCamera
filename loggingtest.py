import numpy as np
from StringIO import StringIO

arr = np.array([[1,4,5,7],[8,6,2],[43,6,3],[78,34,5]])
print arr
itr = np.nditer(arr)
col = arr.shape[1]

string = ''
for x in range(0,itr.itersize):
	string += str(itr.next())
	if x%col == col -1:
		string += '\n'
	else:
		string += ' '
print string

try:
	arr2 = np.loadtxt(StringIO('4 5 6 7\n8 6 2\n43 6 3\n78 34 5\n'))
	print arr2
except ValueError:
	print "error"