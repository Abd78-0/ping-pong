import math

x = 340
y = 440

deltaX = x - 320
deltaY = y - 240

pt_angle = math.atan2(deltaX,-deltaY)

print(f"Pt Angle: {pt_angle}")