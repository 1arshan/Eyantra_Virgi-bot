#from .import time_input.txt
#import time_input.txt

from datetime import datetime, time
def date_diff_in_Seconds(dt2, dt1):
  timedelta = dt2 - dt1
  return timedelta.days * 24 * 3600 + timedelta.seconds
#Specified date
file1 = open('time_input(dispatch).txt', 'r')
Lines = file1.readlines()
count =0
prev_line="Thu Mar 04 2021 - 16:50:07"
for line in Lines:
    count += 1
    #print("Line{}: {}".format(count, line.strip()))
    
    t1 =prev_line[21:23]
    t2 =prev_line[24:26]
    t3 =line[21:23]
    t4 =line[24:26]
    t5=prev_line[18:20]
    t6=line[18:20]
    #print(t2)
    date1 = datetime.strptime('2015-01-01 {}:{}:{}'.format(t5,t1,t2), '%Y-%m-%d %H:%M:%S')
    date2 = datetime.strptime('2015-01-01 {}:{}:{}'.format(t6,t3,t4), '%Y-%m-%d %H:%M:%S')
    prev_line=line
    #Current date
    
    print(date_diff_in_Seconds(date2, date1))


#print()