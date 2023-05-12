from utils.video2keypoints import video2keypoints

def test():
    video2keypoints(input_path="./videos/1.mp4",
                    output_path="./output/push_pull",
                    display_captured=False,
                    loop=False,
                    static_hand=False,
                    smoothing="OneEuroFilter",
                    )
    video2keypoints(input_path="./videos/2.mp4",
                    output_path="./output/beckoned",
                    display_captured=False,
                    loop=False,
                    static_hand=True,
                    smoothing="OneEuroFilter",
                    )
    video2keypoints(input_path="./videos/3.mp4",
                    output_path="./output/rub_fingers",
                    display_captured=False,
                    loop=False,
                    static_hand=True,
                    smoothing="OneEuroFilter",
                    )

if __name__ == "__main__":
    test()
    # video2keypoints(input_path=0,display_captured=True,static_hand=False,smoothing="") # webcam visualization