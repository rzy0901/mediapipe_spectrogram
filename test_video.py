import os
from tqdm import tqdm, trange
from utils.video2keypoints import video2keypoints

def test(input_dir,output_dir,static_hand = True,smoothing="OneEuroFilter"):
    obj = os.scandir(input_dir)
    for entry in tqdm(list(obj),desc=input_dir):
        if entry.is_file() and entry.name.endswith('.mp4'):
            name = entry.name.split('.')[0]
            video2keypoints(input_path= os.path.join(input_dir,name+'.mp4'),
                                output_path= os.path.join(output_dir,name),
                                display_captured=False,
                                loop=False,
                                static_hand=static_hand,
                                smoothing=smoothing,
                                duration=2.0
                                )
    
def test2():
    video2keypoints(input_path="./videos/1.mp4",
                    output_path="./output2/push_pull",
                    display_captured=False,
                    loop=False,
                    static_hand=False,
                    smoothing="MovingAverageFilter",
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

    # test2()


    video2keypoints(input_path=0,display_captured=True,static_hand=False,smoothing="") # webcam visualization


    # test('/media/rzy/76800D98800D5FCB/Codes/record/push_pull','./data/keypoints/push_pull/',static_hand=False, smoothing="MovingAverageFilter")
    # # for filename in os.listdir('./data/keypoints/beckoned/'):
    # #     file_path = os.path.join('./data/keypoints/beckoned/', filename)
    # #     if os.path.isfile(file_path):
    # #         os.remove(file_path)
    # test('/media/rzy/76800D98800D5FCB/Codes/record/beckoned','./data/keypoints/beckoned/')
    # test('/media/rzy/76800D98800D5FCB/Codes/record/rub_finger','./data/keypoints/rub_finger/')


    # for filename in os.listdir('./data/keypoints/test/'):
    #     file_path = os.path.join('./data/keypoints/test/', filename)
    #     if os.path.isfile(file_path):
    #         os.remove(file_path)
    # test('/media/rzy/76800D98800D5FCB/Codes/record/test','./data/keypoints/test/')


    # video2keypoints(input_path="./videos/媒体2.mp4",
    #                 output_path="./data",
    #                 display_captured=True,
    #                 loop=True,
    #                 static_hand=False,
    #                 smoothing="OneEuroFilter",
    #                 )



