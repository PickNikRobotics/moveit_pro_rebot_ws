import json,sys,subprocess,os
from PIL import Image,ImageDraw,ImageFont
src,out=sys.argv[1],sys.argv[2]
rows=[json.loads(l) for l in open(src+'/joint_states.jsonl')]
J=[f'joint{i}' for i in range(1,8)]
T=rows[-1]['t']; W,H=1280,720; FPS=30
f=ImageFont.load_default(size=16); fb=ImageFont.load_default(size=22)
cols=['#e6194b','#3cb44b','#4363d8','#f58231','#911eb4','#008080','#9a6324']
lo=[min(r['position'][j] for r in rows) for j in J]; hi=[max(r['position'][j] for r in rows) for j in J]
def norm(i,v): return (v-lo[i])/((hi[i]-lo[i]) or 1)
X0,X1,Y0,Y1=60,W-20,90,H-30
def frame(k,t,idx):
    im=Image.new('RGB',(W,H),'white'); d=ImageDraw.Draw(im)
    d.text((60,15),"rebot_b601_rs_mock smoke: recorded /joint_states (each joint scaled to its own min..max)",fill='black',font=fb)
    d.text((60,50),f"t={t:6.1f}s  phase={rows[idx]['phase']}",fill='black',font=fb)
    d.rectangle((X0,Y0,X1,Y1),outline='#888')
    # phase change ticks
    for i,j in enumerate(J):
        pts=[(X0+(r['t']/T)*(X1-X0),Y1-norm(i,r['position'][j])*(Y1-Y0)) for r in rows[::8] if r['t']<=t]
        if len(pts)>1:d.line(pts,fill=cols[i],width=2)
        d.text((X1-120,Y0+6+i*20),f"{j} {rows[idx]['position'][j]:+.3f}",fill=cols[i],font=f)
    x=X0+t/T*(X1-X0); d.line((x,Y0,x,Y1),fill='#000')
    return im
import bisect
ts=[r['t'] for r in rows]
os.makedirs(out+'/frames',exist_ok=True)
n=int(T*FPS)
for k in range(n):
    t=k/FPS; idx=min(bisect.bisect_left(ts,t),len(rows)-1)
    frame(k,t,idx).save(f'{out}/frames/{k:05d}.png')
subprocess.check_call(['ffmpeg','-y','-loglevel','error','-framerate','30','-i',out+'/frames/%05d.png','-c:v','libx264','-pix_fmt','yuv420p','-crf','30',out+'/mock_teleop_joint_states.mp4'])
frame(0,T,len(rows)-1).save(out+'/mock_teleop_joint_states.png')
print(n,'frames',T)
