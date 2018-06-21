
if __name__ == '__main__':
	infile = open('mag2018to2020.txt', 'r')
	years = [2018.0, 2018.5, 2019.0, 2019.5, 2020.0]
	
	dirs = {2018.0:"2018_0/",
			2018.5:"2018_5/",
			2019.0:"2019_0/",
			2019.5:"2019_5/",
			2020.0:"2020_0/"}
		
	for line in infile:
		vals = line.split()
		latitude = float(vals[0])
		longitude = float(vals[1])
		year = float(vals[3])
		declination = float(vals[4])

		data_string = str(latitude) + ',' + str(longitude) + ',' + str(declination) + '\n'
		print(latitude)
		file_name = str(abs(int(latitude))) + ".csv"
		print(latitude)
		if latitude < 0:
			file_name = 'n' + file_name

		file_name = dirs.get(year) + file_name

		outfile = open(file_name,'a')
		outfile.write(data_string)


