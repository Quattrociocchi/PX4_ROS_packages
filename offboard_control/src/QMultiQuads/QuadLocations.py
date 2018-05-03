
import csv
if __name__ == "__main__":


	traj = []
	
	with open('statehistory_target.txt','rb') as csvfile:
	    totalstate = csv.reader(csvfile, delimiter=',', quotechar='|')
	    for row in totalstate:
	         traj.append(map(int,row))

	print traj