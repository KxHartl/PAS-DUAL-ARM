import numpy as np, matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image
im=np.array(Image.open('src/pas_dual_arm_bringup/maps/seminar_map.pgm').convert('L')).astype(float)
res=0.02; ox,oy=-2.97,-8.97; h,w=im.shape
p=(255-im)/255.0          # trinary: occupied>0.65, free<0.25
rgb=np.full(im.shape,0.62); rgb[im>=250]=1.0; rgb[im<=50]=0.0
fig,ax=plt.subplots(figsize=(6,6),dpi=300)
# pgm row 0 is the TOP (largest y): origin='upper' with the true extent
# Drawn turned 90 deg counter-clockwise, like the RViz screenshots in the report:
# map +x points up, map +y points left.
ax.imshow(np.rot90(rgb),cmap='gray',vmin=0,vmax=1,origin='upper',interpolation='nearest',
          extent=[oy+h*res,oy,ox,ox+w*res])
ax.set_xlim(3,-9); ax.set_ylim(-3,9); ax.set_aspect('equal')
ax.set_xlabel('y [m]'); ax.set_ylabel('x [m]'); ax.grid(lw=0.3,alpha=0.3)
ax.plot([-7,-8],[-2.3,-2.3],'k',lw=4); ax.text(-7.5,-2.15,'1 m',ha='center',va='bottom',fontsize=9)
fig.tight_layout(); fig.savefig('seminar/slike/s61_karta.png'); print('free m2', (im>=250).sum()*res*res)
