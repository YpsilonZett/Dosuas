from random import uniform

with open("test_capture1.txt", "w") as f:
	for row in range(240):
		if row >= 120 and row <= 180:
			for col in range(120):	# wall
				f.write(str(5500 + int(uniform(-25, 25))) + ",")
			for col in range(100):  # door
				f.write(str(2500 + int(uniform(-25, 25))) + ",")
			for col in range(100):  # wall
				f.write(str(5500 + int(uniform(-25, 25))) + ",")	
			f.write("\n")
			continue
	
		for col in range(320):  # wall
			f.write(str(5500 + int(uniform(-25, 25))) + ",")
		f.write("\n")
			
	
