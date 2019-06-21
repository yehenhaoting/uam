import csv
from matplotlib import pyplot as plt


filename = 'vehicle_attitude.csv'
with open(filename) as f:
    reader = csv.reader(f)
    header_row = next(reader)
    print (header_row)
    first_row = next(reader)
    print (first_row)
    i=1
    time, rot_x, rot_y, rot_z, rot_w = [], [], [], [], []
    for row in reader:
        t = int(row[0])
        if 7.98e8 < t <8.16e8:
            time.append(t)
            rot_x.append((float(row[5]))/5)
            rot_y.append((float(row[6]))/5)
            rot_z.append((float(row[7]))/5)
            rot_w.append((float(row[4]))/5)
            i=i+1

plt.figure()
plt.plot(time, rot_x)
plt.plot(time, rot_y)
plt.plot(time, rot_z)
plt.plot(time, rot_w)
plt.show()