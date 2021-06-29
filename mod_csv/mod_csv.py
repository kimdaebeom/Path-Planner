import csv

with open('1.csv') as infile, open('1.txt', 'w') as outfile:
    reader = csv.reader(infile, delimiter='\t')
    writer = csv.writer(outfile, delimiter=',')
    for row in reader:
        writer.writerow(row)
