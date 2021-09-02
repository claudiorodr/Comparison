from rdp import rdp
import matplotlib.pyplot as plt


points = []
def trajectory(x, y):
    """Estimates path trajectory using rdp algorithm"""
    for i in range(len(x)):
        points.append((x[i][0],y[i][0]))  
    return rdp(points,epsilon=2)