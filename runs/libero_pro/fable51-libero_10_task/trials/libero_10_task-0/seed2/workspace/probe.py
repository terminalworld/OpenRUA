from cloud import cloud
import numpy as np
P=cloud('agentview')
pts={'ketchup':(232,215),'bluecan':(215,275),'tomato_can':(338,270),'oj':(115,310),'milk':(245,320),'cream_cheese':(165,385),'butter':(365,370),'basket_rim':(530,190),'table_near':(320,440),'table_far':(400,200)}
for k,(u,v) in pts.items():
    p=P[v,u]; print(f'{k:14s} px({u},{v}) world {p.round(3)}')
