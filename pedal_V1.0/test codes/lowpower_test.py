import lowpower
import time

byte_array = b"\xe1\xf0\xf0\xf0\xf0"
DORMANT_PIN = 2

# print("before dormant")
# lowpower.dormant_with_modes({
#         DORMANT_PIN: (lowpower.EDGE_LOW | lowpower.EDGE_HIGH),
# })
#lowpower.dormant_until_pin(DORMANT_PIN)
while(1):
    print("data before sleep: ", byte_array)
    print("enter sleep")
    lowpower.dormant_with_modes({
            DORMANT_PIN: (lowpower.EDGE_LOW | lowpower.EDGE_HIGH),
    })
    print("woke up");
    print("woke up");
    print("woke up");
    print("data after sleep: ", byte_array)
    time.sleep(10)
# while(1):
#     pass