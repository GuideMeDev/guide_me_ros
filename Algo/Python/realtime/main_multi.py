import multiprocessing
from realtime_algo import RT_algo
from rospy_sub_ver2 import *
#from main import *
from Modules.utils import *
import warnings
warnings.filterwarnings("ignore")

pqueue = Queue()
# running data loading in different process
writer_p = Process(target=RT_writer, args=((pqueue),))
algo_p = Process(target=RT_algo, args=((pqueue),))
writer_p.daemon = True
algo_p.daemon = True
writer_p.start()
algo_p.start()
try:
    writer_p.join()
    algo_p.join()
except:
    pass

finally:
    print("done!")
    if writer_p.is_alive():
        writer_p.kill()
    if algo_p.is_alive():
        algo_p.kill()
