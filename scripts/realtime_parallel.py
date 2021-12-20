from rospy_sub_RT import *

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
# giving time for IMU to retrive 200 samples (200hz)
time.sleep(0.3)
while True:
    while not pqueue.empty():
        try:
        # Getting recent sensors data from main() function in another process (with the Queue)
            data_list = pqueue.get()
            rgb_img,xyz,acc_raw,euler = data_list[0],data_list[1],data_list[2],data_list[3]
            ax1_data.set_data(rgb_img)
            ax2_data.set_data(xyz[:,0],xyz[:,1])
            #print("accel buffer size:",len(acc_raw))
            print("Last Frame: ")
            print("imu acceleromter: ",acc_raw[-1])
            print("roll,pitch,yaw :",euler)
            fig.canvas.draw()
            fig.canvas.flush_events()
        except:
            print("exception")