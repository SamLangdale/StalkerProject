"""
https://codefather.tech/blog/shell-command-python/
https://codefather.tech/blog/exit-bash-script/

IMX477 (Raspberry Pi HQ Camera)
https://www.arducam.com/sony/imx477/
"""
import os
import subprocess
import cv2
import shutil
import exifread


def take_HDR_photo(path, 
                   AEB=1, AEB_stops=2,
                   dims=(4056,3040), 
                   exposure_time=None, 
                   gain=None,
                   denoise="cdn_hq", # cdn_hq, cdn_off, cdn_fast
                   awbgains=None,
                   awb="auto",
                   sharpness=0.5,
                   saturation=0.7,
                   save_raw=False, 
                   blocking=True):

    path_split = os.path.splitext(path)

    AEB = int(AEB)
    paths = []

    # Calculate the exposure time multiplier for each stop
    stop_multiplier = 2 ** AEB_stops

    # Take photos at different exposure levels
    for step in range(-AEB, AEB + 1):
        if step < 0:
            HDR_exposure_time = exposure_time * (stop_multiplier ** step)
            HDR_gain = gain
        else:
            HDR_exposure_time = exposure_time
            HDR_gain = gain * (stop_multiplier ** step)

        HDR_path = f"{path_split[0]}_{step * AEB_stops}{path_split[1]}"

        take_photo(path=HDR_path, 
                   dims=dims, 
                   exposure_time=HDR_exposure_time, 
                   gain=HDR_gain, 
                   denoise=denoise, 
                   awbgains=awbgains, 
                   awb=awb, 
                   sharpness=sharpness, 
                   saturation=saturation, 
                   save_raw=save_raw, 
                   blocking=blocking)
    
        paths.append(HDR_path)
    
    return paths


def take_photo(path, 
               dims=(4056,3040), 
               exposure_time=None, 
               gain=None,
               denoise="cdn_hq", # cdn_hq, cdn_off, cdn_fast
               awbgains=None,
               awb="auto",
               sharpness=0.5,
               saturation=0.7,
               save_raw=False, 
               blocking=True):

    params = {
        "--immediate":  "",
        "--nopreview":  "",
        "--output":     path,
        "--denoise":    denoise,
        "--width":      dims[0],
        "--height":     dims[1],
        "--sharpness":  sharpness,
        "--saturation": saturation,
        "--quality":    100,
        "--rotation":   0}  # or 180°

    if awbgains:
        awbgains = ','.join(map(str, awbgains))  # convert to string and add to params
        params["--awbgains"]  = awbgains  
    elif awb.lower() in ["auto", "incadescent", "tungsten", "fluorescent", "indoor", "daylight", "cloudy"]:
        params["--awb"]  = awb
    else:
        raise ValueError("Invalid white balance mode or red/blue gains.")

    if exposure_time:
        params["--shutter"] = int(exposure_time * 1000000)  # to microseconds

    if gain:
        params["--gain"] = gain  # "--analoggain", "--digitalgain" ?

    if save_raw:
        params["--raw"] = ""
    
    # build command string
    params["--encoding"] = params['--output'].split('.')[-1]  # get file extension
    cmd_string = "rpicam-still " + " ".join(f"{k} {v}" for k, v in params.items())

    # execute command either blocking or non-blocking
    if blocking:
        retval = subprocess.run(cmd_string, shell=True, capture_output=True, text=True)
        if retval.returncode != 0:
            print(f"Command failed with error: {retval.stderr}")
        else:
            # print(f"Command succeeded with output: {retval.stdout}")
            pass
            
    else:
        retval = subprocess.Popen(cmd_string, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    return path


class ExifReader:
    def __init__(self, path):
        self.path = path
        self.tags = self.read_exif()
        self.exposure_time = self.tags["EXIF ExposureTime"].values[0].num / self.tags["EXIF ExposureTime"].values[0].den
        self.gain = self.tags["EXIF ISOSpeedRatings"].values[0] / 100

    def read_exif(self):
        with open(self.path, 'rb') as f:  # Open image file in binary mode
            tags = exifread.process_file(f)  # Return Exif tags
        return tags

    def print_tags(self):
        for tag in self.tags.keys():
            if tag not in ('JPEGThumbnail', 'TIFFThumbnail', 'Filename', 'EXIF MakerNote'):
                print(f"Key:{tag}, value {self.tags[tag]}")


def get_r_b_gains(img1, img2):  # source, target
    (b1, _, r1) = cv2.split(img1.astype("float32"))
    (b2, _, r2) = cv2.split(img2.astype("float32"))

    R = r2.mean() / r1.mean()
    B = b2.mean() / b1.mean()
    return R, B


def is_between(value, limits):
    return limits[0] <= value <= limits[1]


def estimate_camera_parameters(config):
    tmp_dir         = config.tmp_dir
    awb_thres       = config.get("CAM", "awb_thres")
    max_iterations  = config.get("CAM", "max_iterations")
    set_gain        = config.get("CAM", "set_gain")
    preview_dims    = config.get("CAM", "preview_dims")
    awbgains        = config.get("CAM", "awbgains")
    preview_denoise = config.get("CAM", "preview_denoise")

    # take photo in automatic mode
    path_auto = take_photo(path=os.path.join(tmp_dir,"awb.jpg"),
                           dims=preview_dims, 
                           denoise=preview_denoise, 
                           awb="auto", 
                           blocking=True)
    
    exif_auto = ExifReader(path_auto)
    
    # take photos in manual mode
    for i in range(max_iterations):
        # take photo with manual exposure, gain and awbgains
        path_awbgains = take_photo(path=os.path.join(tmp_dir, f"img{i}.jpg"),
                                   dims=preview_dims,
                                   denoise=preview_denoise,
                                   exposure_time=exif_auto.exposure_time,
                                   gain=exif_auto.gain,
                                   awbgains=awbgains,
                                   blocking=True)
        
        img_awbgains = cv2.imread(path_awbgains)
        
        # cv2.imshow("awbgains:", img_awbgains)
        # cv2.waitKey(1)

        R, B = get_r_b_gains(img_awbgains, cv2.imread(path_auto))

        limits = (1-awb_thres, 1+awb_thres)
        if is_between(R, limits) and is_between(B, limits):
            break

        awbgains[0] = awbgains[0] * R
        awbgains[1] = awbgains[1] * B
        # print("awbgains:", awbgains)
        
    # compute new exposure time based on custom gain
    if set_gain:
        gain = set_gain
        exposure_time = exif_auto.exposure_time * exif_auto.gain / set_gain
    else:
        gain = exif_auto.gain
        exposure_time = exif_auto.exposure_time

    # remove temp directory
    if config.get("CAM", "remove_tmp"):
        shutil.rmtree(config.tmp_dir)

    return exposure_time, gain, awbgains


if __name__ == "__main__":
    from config import Config

    config = Config()
    config.init(scan_id="_")

    # extract exposure time and gain from exif data, iterate through Red/Blue Gains for custom AWB
    exposure_time, gain, awbgains = estimate_camera_parameters(config)
    print("[RESULT] AE:", exposure_time, "| Gain:", gain, "| AWB R:", round(awbgains[0],3), "B:", round(awbgains[1],3))

    # # take single photo using previously calculated manual parameters
    # path = take_photo(exposure_time=exposure_time, gain=gain, awbgains=awbgains, denoise="cdn_hq")
    
    path = os.path.join(config.img_dir, "photo.jpg")

    # take series of photos using AEB
    paths = take_HDR_photo(path, exposure_time=exposure_time, gain=gain, awbgains=awbgains, AEB=1, AEB_stops=1, denoise="cdn_hq")
    for i, path in enumerate(paths):
        exif = ExifReader(path)
        print(f"Photo {i+1}: Path: {path} | ExposureTime: {exif.exposure_time} | Gain: {exif.gain}")
