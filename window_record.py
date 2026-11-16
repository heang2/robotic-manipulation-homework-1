"""Record only this Isaac Sim Kit application's own X11 window."""
import os,re,subprocess

def start_window_capture(ffmpeg):
    tree=subprocess.check_output(["xwininfo","-root","-tree"],text=True)
    wins=[]
    for line in tree.splitlines():
        match=re.search(r'(0x[0-9a-fA-F]+)\s+".*?"[^\n]*?\s(\d+)x(\d+)\+',line)
        if not match: continue
        wid,w,h=match.groups()
        if int(w)<700 or int(h)<400: continue
        pid=subprocess.run(["xprop","-id",wid,"_NET_WM_PID"],capture_output=True,text=True).stdout
        if str(os.getpid()) in pid or ("Isaac" in line and os.getenv("PRIVATE_DISPLAY")=="1"):
            wins.append((int(w)*int(h),wid))
    if not wins: raise RuntimeError("No Isaac Sim application window in isolated display: "+tree[-1500:])
    wid=max(wins)[1]
    print("NATIVE_KIT_WINDOW",wid,flush=True)
    return subprocess.Popen([ffmpeg,"-hide_banner","-loglevel","error","-y","-f","x11grab",
                             "-framerate","24","-draw_mouse","0","-window_id",wid,"-i",os.environ["DISPLAY"],
                             "-vf","scale=trunc(iw/2)*2:trunc(ih/2)*2","-c:v","libx264","-preset","ultrafast","-crf","20",
                             "-pix_fmt","yuv420p","-movflags","+faststart","Isaac_Kit_UI.mp4"],stdin=subprocess.PIPE)

