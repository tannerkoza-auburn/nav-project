from bagpy import bagreader
from tqdm import tqdm


b = bagreader("/home/tannerkoza/edu/nav-project/data/2017-08-04-V3-Log3.bag")
topics = ['/gps', '/gps_time', '/imu', '/pose_ground_truth', '/pose_localized', '/pose_raw', '/tf', '/velocity_raw']

csvfiles = []
for t in tqdm(topics):
    data = b.message_by_topic(t)
    csvfiles.append(data)
    
