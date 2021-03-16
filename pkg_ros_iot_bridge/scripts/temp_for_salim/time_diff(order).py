#from .import time_input.txt
#import time_input.txt

from datetime import datetime, time
def date_diff_in_Seconds(dt2, dt1):
  timedelta = dt2 - dt1
  return timedelta.days * 24 * 3600 + timedelta.seconds
#Specified date
file1 = open('time_input.txt', 'r')
Lines = file1.readlines()
count =0
prev_line="2021-03-04 16:45:47"
for line in Lines:
    count += 1
    #print("Line{}: {}".format(count, line.strip()))
    
    t1 =prev_line[14:16]
    t2 =prev_line[17:19]
    t3 =line[14:16]
    t4 =line[17:19]
    #print(t2)
    date1 = datetime.strptime('2015-01-01 01:{}:{}'.format(t1,t2), '%Y-%m-%d %H:%M:%S')
    date2 = datetime.strptime('2015-01-01 01:{}:{}'.format(t3,t4), '%Y-%m-%d %H:%M:%S')
    prev_line=line
    #Current date
    
    print(date_diff_in_Seconds(date2, date1))


#print()