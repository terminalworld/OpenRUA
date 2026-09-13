"""Replace solid microwave box with hollow walls; attach white mug (upright) to hand."""
import numpy as np, rclpy
from rob import *; import scene; from attach import attach_mug
r = Robot("mwscene")
# cavity x[-0.15,0.08] y[0.266,0.444] z[0.944,1.09]; outer x[-0.16,0.17] y[0.266,0.464] z[0.90,1.107]
walls = [
    scene.remove("microwave"),
    scene.box("mw_floor",   (0.005, 0.365, 0.922), (0.33, 0.20, 0.044)),
    scene.box("mw_ceiling", (0.005, 0.365, 1.0985), (0.33, 0.20, 0.017)),
    scene.box("mw_back",    (0.005, 0.454, 1.0035), (0.33, 0.02, 0.207)),
    scene.box("mw_left",    (-0.155, 0.365, 1.0035), (0.01, 0.20, 0.207)),
    scene.box("mw_right",   (0.125, 0.365, 1.0035), (0.09, 0.20, 0.207)),
]
print("walls", scene.apply(r.node, walls))
tcp, R = r.tcp()
# mug body centre: bar is 0.074 in -y from axis, pinch at bar mid-height (0.056 above bottom)
center = tcp + np.array([0, 0.074, 0.0])
attach_mug(r, center, radius=0.048, height=0.114, axis_world=(0,0,1))
