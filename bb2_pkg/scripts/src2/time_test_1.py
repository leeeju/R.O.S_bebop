import time

if __name__ == '__main__':
  ttime = time.time()
  time.sleep(3)
  ztime = time.time()
  
  gap_time = ztime - ttime
  gap_time = int(gap_time)
  print(gap_time)
