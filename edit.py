"""Assemble the submission film from the Kit window, then each engine's synced camera and CSV."""
import os,csv,subprocess
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
HERE=Path(__file__).resolve().parent
P=Path(os.getenv("RUN_DIR",str(HERE)))
ff=os.environ["FFMPEG"]
fontpath=os.getenv("CJK_FONT","C:/Windows/Fonts/msyh.ttc")
title=ImageFont.truetype(fontpath,24);small=ImageFont.truetype(fontpath,19)
out=subprocess.Popen([ff,"-v","error","-y","-f","rawvideo","-pix_fmt","rgb24","-s","1280x800","-r","24","-i","-","-c:v","libx264","-crf","19","-preset","veryfast","-pix_fmt","yuv420p","-movflags","+faststart",str(P/"Submission.mp4")],stdin=subprocess.PIPE)
for filename,log,label,count in [("Isaac_Kit_UI.mp4",None,"Isaac Sim 5.1   /   原生窗口实录",96),("Franka_Isaac.mp4","Franka.csv","Isaac Sim 5.1   /   NVIDIA DGX Spark",456),("Franka_MuJoCo.mp4","MuJoCo.csv","MuJoCo 3.6   /   同一机械臂与轨迹",456)]:
    if log:
        with (P/log).open() as f: rows=list(csv.DictReader(f))
    reader=subprocess.Popen([ff,"-v","error","-i",str(P/filename),"-frames:v",str(count),"-vf","scale=1280:720:force_original_aspect_ratio=decrease,pad=1280:720:(ow-iw)/2:(oh-ih)/2","-f","rawvideo","-pix_fmt","rgb24","-"],stdout=subprocess.PIPE)
    for i in range(count):
        raw=reader.stdout.read(1280*720*3)
        if len(raw)!=1280*720*3:raise RuntimeError(filename+" ended early")
        frame=Image.new("RGB",(1280,800),"#182733")
        frame.paste(Image.frombytes("RGB",(1280,720),raw),(0,0));d=ImageDraw.Draw(frame)
        d.text((24,739),label,font=title,fill="white")
        if log:
            r=rows[5*i]
            s=f"仿真 {float(r['t']):05.2f} s     目标 {float(r['target_N']):.0f} N     滤波实测 {float(r['filtered_N']):.1f} N     覆盖 {r['covered']} / 60"
        else:s="Franka Panda   ·   七轴机械臂表面擦拭"
        d.text((1260,761),s,font=small,fill="#b5e3df",anchor="rm")
        out.stdin.write(frame.tobytes())
    if reader.wait():raise RuntimeError(filename)
out.stdin.close()
if out.wait():raise RuntimeError("encoding failed")

