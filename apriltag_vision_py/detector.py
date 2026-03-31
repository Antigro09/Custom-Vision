import apriltag

def create_detector(family: str):
    opts = apriltag.DetectorOptions(families=family)
    return apriltag.Detector(opts)