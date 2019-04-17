#rotation test
import numpy as np
import argparse

def parse_args():
	OUTPUT_DIR = ''
	arg_parser = argparse.ArgumentParser(description='generate mapper trajectory file from novatel session.txt file')
	arg_parser.add_argument('--input_file', 
							required=True, 
							help='novatel session.txt file')
	#arg_parser.add_argument('--output file',
	#						required=True,
	#						help='resulting mapper trajectory output file')
	return arg_parser.parse_args()

def main():
	#parse arguments from terminal
	args = parse_args()
	inputData = np.genfromtxt(args.input_file , skip_header=27, skip_footer=10)
	print("data imported")
	
	# yaw = inputData[:,6]
	# pitch = inputData[:,7]
	# roll = inputData[:,8]
	easting = inputData[:,9]
	northing = inputData[:,10]
	alt = inputData[:,14]
	time = inputData[:,12]
	output = np.zeros((len(inputData),9), dtype=np.float32)

	for i, row in enumerate(inputData):
		yaw = row[6]/180*np.pi
		pitch = row[7]/180*np.pi
		roll = row[8]/180*np.pi
		rotation_x = np.array([[1, 0, 0],
	                	   	  [0, np.cos(pitch), -np.sin(pitch)],
	                	   	  [0, np.sin(pitch), np.cos(pitch)]])

		rotation_y = np.array([[np.cos(roll), 0, np.sin(roll)],
		                	   [0, 1, 0],
		                	   [-np.sin(roll), 0, np.cos(roll)]])
	                 
		rotation_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
		                	   [np.sin(yaw), np.cos(yaw), 0],
		                	   [0, 0, 1]])

		rotationMatrix = np.dot(rotation_z, np.dot(rotation_y, rotation_x ))
		output[i,:] = rotationMatrix.reshape((1, 9))

		# reshapeMatrix = rotationMatrix.reshape(1,9) 
		# print(reshapeMatrix)   
		x = easting
		y = northing
		height = alt
		utc = time

	print("rotation matrix array size: ", rotationMatrix.size)
	print("easting array size: ", x.size)
	print("northing array size: ", y.size)
	print("alt array size: ", height.size)
	print("time array size: ", utc.size)
	dataOutput = np.column_stack((output,easting,northing,alt,time))
	np.savetxt('novatel_localization.txt', dataOutput, delimiter='')

if __name__=='__main__':
	main()