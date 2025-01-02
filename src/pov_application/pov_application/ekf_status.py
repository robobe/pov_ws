from pymavlink.dialects.v20 import ardupilotmega

print(ardupilotmega.EKF_ATTITUDE)
print(ardupilotmega.EKF_GPS_GLITCHING)

# data = 831 # gps
data = 39
# data = 303 # weel encoder
data = 167 # SPEED ESTIMATE
data = 421
data = 165
# if (data & ardupilotmega.EKF_ATTITUDE) == ardupilotmega.EKF_ATTITUDE:
#     print("EKF_ATTITUDE")

# if (data & ardupilotmega.ESTIMATOR_VELOCITY_HORIZ) == ardupilotmega.ESTIMATOR_VELOCITY_HORIZ:
#     print("ESTIMATOR_VELOCITY_HORIZ")

# if (data & ardupilotmega.ESTIMATOR_ACCEL_ERROR) == ardupilotmega.ESTIMATOR_ACCEL_ERROR:
#     print("ESTIMATOR_ACCEL_ERROR")

# if (data & ardupilotmega.ESTIMATOR_ACCEL_ERROR) == ardupilotmega.ESTIMATOR_ACCEL_ERROR:
#     print("ESTIMATOR_ACCEL_ERROR")

items = ardupilotmega.enums["ESTIMATOR_STATUS_FLAGS"]
for v, item in items.items():
    if (v & data) == v:
        print(item.name)