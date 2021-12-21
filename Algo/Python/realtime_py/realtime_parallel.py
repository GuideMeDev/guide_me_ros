from rospy_sub_ver2 import *

fig, (ax1, ax2) = plt.subplots(1, 2,figsize=(7,7))
dummy_img = np.zeros((720,1280,3))
ax1_data = ax1.imshow(dummy_img)
ax2_data = ax2.plot([],[],'.')[0]
fig.show()
# queue used for data sharing between processes, from sensors
pqueue = Queue()
# running data loading in different process
writer_p = Process(target=main, args=((pqueue),))
writer_p.daemon = True
writer_p.start()
st_t = 0
times = []
# giving time for IMU to retrive 200 samples (200hz)
#time.sleep(0.3)
st = time.time()
timee = 0 
while timee < 10:
    while not pqueue.empty() and timee < 10:
        try:
            #print(time.time()-st_t)
        # Getting recent sensors data from main() function in another process (with the Queue)
            data_list = pqueue.get()
            rgb_img,xyz,acc_raw,euler = data_list[0],data_list[1],data_list[2],data_list[3]
            #tstamps = data_list[5]
            #times.append(tstamps)
            ax1_data.set_data(rgb_img)
            ax2_data.set_data(xyz[:,0],xyz[:,1])
            timee = time.time() - st
            #print("accel buffer size:",len(acc_raw))
            print("Last Frame: ")
            print("imu acceleromter: ",acc_raw[-1])
            print("roll,pitch,yaw :",euler)
            fig.canvas.draw()
            fig.canvas.flush_events()
        except:
            print("exception")

# times = np.array(times)
# plt.show()
# plt.plot(times[:,0],'or',markerfacecolor='none',label = 'madgwick')
# plt.plot(times[:,1],'+b',label = 'rgb')
# plt.plot(times[:,2],'.g',label = 'pcl')
# # for x in times[:,3]:
# #     plt.plot(x,'*c')
# plt.legend()
# plt.show()
