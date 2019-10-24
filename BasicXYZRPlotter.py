#!/usr/bin/python

import argparse
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl

def parse_args():
	arg_parser = argparse.ArgumentParser(description='simple visualization tool for xyzr data')
	arg_parser.add_argument('-r', '--rows', required=False, default=1000000, type=int, help='number of rows to process, default is 1000000')
	arg_parser.add_argument('-s1', '--scan_1', required=True, help='xyzr scan 1')
	return arg_parser.parse_args()

def main():
	args = parse_args()
	
	scan1 = np.genfromtxt(args.scan_1)

	print("")
	print("data has been imported")
	print("")

	row_num = np.zeros((len(scan1),1))
	x1_sub = np.zeros((len(scan1),1))
	y1_sub = np.zeros((len(scan1),1))
	z1_sub = np.zeros((len(scan1),1))

	x_y_z_sub = np.zeros((len(scan1),3))
	print('xyz array size: ', np.size(x_y_z_sub))

	for i, row in enumerate(scan1):
		x1 = row[0]
		y1 = row[1]
		z1 = row[2]

		x1_sub[i,:] = x1
		y1_sub[i,:] = y1
		z1_sub[i,:] = z1 + 2

		# x_y_z_sub[i,:] = x1 + y1 + z1
		# print('print xyz: ', x_y_z_sub)			

		if i > args.rows:
			break

	x_y_z_sub = np.concatenate((x1_sub, y1_sub, z1_sub), axis=1)

	# open app
	app=pg.QtGui.QApplication([])

	# array of data
	plt = gl.GLScatterPlotItem(pos=x_y_z_sub, color=[1.0,0.5,0.5,0.5])

	# add widget?	
	w = gl.GLViewWidget()
	w.addItem(plt)

	# add grid
	g = gl.GLGridItem()
	w.addItem(g)
	
	# show
	w.show()
	pg.QtGui.QApplication.exec_()

if __name__=='__main__':
	main()
