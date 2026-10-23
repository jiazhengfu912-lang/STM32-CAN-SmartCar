#!/usr/bin/env python3
"""用实际交付网格绘制静态外形预览；不是Gazebo运行截图。"""
import math
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

sys.path.insert(0,str(Path(__file__).resolve().parent))
from generate_model import PKG, make_links, read_stl


def rotation(rpy):
    a,b,c=rpy;ca,sa=math.cos(a),math.sin(a);cb,sb=math.cos(b),math.sin(b);cc,sc=math.cos(c),math.sin(c)
    return np.array([[cc*cb,cc*sb*sa-sc*ca,cc*sb*ca+sc*sa],[sc*cb,sc*sb*sa+cc*ca,sc*sb*ca-cc*sa],[-sb,cb*sa,cb*ca]])


def faces(g):
    if g['kind']=='mesh':return np.array(read_stl(PKG/'models/smartcar/meshes'/g['size']))
    if g['kind']=='box':
        x,y,z=np.array(g['size'])/2
        vertices=np.array([[-x,-y,-z],[x,-y,-z],[x,y,-z],[-x,y,-z],[-x,-y,z],[x,-y,z],[x,y,z],[-x,y,z]])
        return [vertices[list(indices)] for indices in [(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]]
    radius,length=g['size'];angles=np.linspace(0,2*math.pi,33)[:-1]
    low=np.array([[radius*math.cos(a),radius*math.sin(a),-length/2] for a in angles]);high=low.copy();high[:,2]=length/2
    return [low[::-1],high]+[np.array([low[i],low[(i+1)%32],high[(i+1)%32],high[i]]) for i in range(32)]


def main():
    links,_=make_links();fig=plt.figure(figsize=(14,10))
    for index,(title,elev,azim) in enumerate([('CAD-derived appearance',25,-125),('Front / camera side',15,15),('Top: X forward, Y left',90,-90),('Side / ground contact',0,-90)],1):
        ax=fig.add_subplot(2,2,index,projection='3d')
        all_polygons=[];all_colors=[]
        for name,data in links.items():
            for g in data['visuals']:
                rr=rotation(g['rpy']);offset=np.array(data['xyz'])+np.array(g['xyz'])
                polygons=[np.asarray(face)@rr.T+offset for face in faces(g)]
                all_polygons.extend(polygons);all_colors.extend([g['color']]*len(polygons))
        ax.add_collection3d(Poly3DCollection(all_polygons,facecolors=all_colors,edgecolors='none',linewidths=0))
        ax.set_xlim(-.06,.24);ax.set_ylim(-.16,.16);ax.set_zlim(0,.25);ax.set_box_aspect([.30,.32,.25]);ax.view_init(elev=elev,azim=azim)
        ax.set_title(title);ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)');ax.set_zlabel('Z (m)')
        if elev==90:ax.set_zticks([]);ax.set_zlabel('')
        if elev==0:ax.set_yticks([]);ax.set_ylabel('')
    fig.suptitle('SmartCar V7 / 65 x 27 mm wheels / 252 mm track\nStatic mesh preview, NOT a Gazebo execution. C1 case / camera optics / masses are provisional.',fontsize=12)
    fig.tight_layout(rect=[0,0,1,.94]);folder=PKG/'docs';folder.mkdir(exist_ok=True)
    fig.savefig(folder/'model_preview.png',dpi=150);plt.close(fig)
    print(folder/'model_preview.png')


if __name__=='__main__':main()
