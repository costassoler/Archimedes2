import numpy as np
import csv
while True:
    try:
        with open('heading.csv', newline='') as csvfile:
            data = np.array(list(csv.reader(csvfile)))
        print(data[0][0])
    except Exception as e:
        print(e)
        continue
