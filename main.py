from main_pipeline import main_pipeline
from robot_movements import *

def main():
    # align_depth_to_color(mode = "live", clipping_dist = 1, file_name = "sample_video.bag")
    # Task1: let waist turn left/right based on the depth of the pen
    main_pipeline(mode = "live", clipping_dist = 1, file_name = "sample_video.bag")
    
if __name__ == "__main__":
    main()