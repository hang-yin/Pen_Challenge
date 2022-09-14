from align_depth_to_color import align_depth_to_color

def main():
    align_depth_to_color(mode = "live", clipping_dist = 1, file_name = "sample_video.bag")

if __name__ == "__main__":
    main()