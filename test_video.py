import os
from tqdm import tqdm, trange
from utils.video2keypoints import video2keypoints

def test(input_dir,output_dir):
    obj = os.scandir(input_dir)
    for entry in tqdm(list(obj),desc=input_dir):
        if entry.is_file() and entry.name.endswith('.mp4'):
            name = entry.name.split('.')[0]
            if name.startswith('push_pull'):
                video2keypoints(input_path= os.path.join(input_dir,name+'.mp4'),
                                output_path= os.path.join(output_dir,name),
                                display_captured=False,
                                loop=False,
                                static_hand=False,
                                smoothing="MovingAverageFilter",
                                duration=3.0
                                )
            else:
                video2keypoints(input_path= os.path.join(input_dir,name+'.mp4'),
                                output_path= os.path.join(output_dir,name),
                                display_captured=False,
                                loop=False,
                                static_hand=True,
                                smoothing="OneEuroFilter",
                                duration=3.0
                                )
    
def test2():
    video2keypoints(input_path="./videos/1.mp4",
                    output_path="./output2/push_pull",
                    display_captured=False,
                    loop=False,
                    static_hand=False,
                    smoothing="OneEuroFilter",
                    )
    video2keypoints(input_path="./videos/2.mp4",
                    output_path="./output2/beckoned",
                    display_captured=False,
                    loop=False,
                    static_hand=True,
                    smoothing="OneEuroFilter",
                    )
    video2keypoints(input_path="./videos/3.mp4",
                    output_path="./output2/rub_fingers",
                    display_captured=False,
                    loop=False,
                    static_hand=True,
                    smoothing="OneEuroFilter",
                    )

if __name__ == "__main__":
    test('/home/rzy/Documents/data_lc','./output/')
    test2()
    # video2keypoints(input_path=0,display_captured=True,static_hand=False,smoothing="") # webcam visualization