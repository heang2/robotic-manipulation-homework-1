"""MuJoCo version built from the same Franka URDF and visual DAE meshes."""
import csv,os,subprocess,xml.etree.ElementTree as E
from pathlib import Path
import mujoco as mj
from control import *
P=Path(__file__).resolve().parent
FR=Path(os.environ["FRANKA_DESCRIPTION"])
OUT=Path(os.getenv("RUN_DIR",str(P)))
ASSET=OUT/"mj_mesh";ASSET.mkdir(exist_ok=True)
def v(a):return " ".join(f"{float(x):.8g}" for x in a)
def elem(parent,tag,**kwargs):return E.SubElement(parent,tag,kwargs)
def model():
    root=E.Element("mujoco",model="Franka wiping cell")
    elem(root,"compiler",angle="radian",autolimits="true",balanceinertia="true")
    elem(root,"option",timestep=str(DT/2),integrator="implicitfast",gravity="0 0 0",cone="elliptic")
    vi=elem(root,"visual");elem(vi,"global",offwidth="1280",offheight="720");elem(vi,"quality",shadowsize="2048")
    asset=elem(root,"asset");world=elem(root,"worldbody");act=elem(root,"actuator")
    def geom(parent,typ,xyz,size,rgba,name=None,contact=False,**kw):
        kw.update(type=typ,pos=v(xyz),size=v(size),rgba=v([*rgba,1]),contype=str(int(contact)),conaffinity=str(int(contact)))
        if name:kw["name"]=name
        return elem(parent,"geom",**kw)
    geom(world,"plane",[0,0,-.79],[0,0,.1],[.34,.38,.39])
    geom(world,"box",[.25,0,-.04],[.74,.51,.036],[.19,.25,.29])
    for i,y in enumerate(np.linspace(-.46,.46,13)):geom(world,"box",[.25,y,-.003],[.735,.0025,.0007],[.07,.09,.11])
    geom(world,"box",[.49,0,.046],[.233,.234,.042],[.09,.11,.14])
    geom(world,"box",[.49,0,TOP-.019],[.1975,.1975,.019],[.16,.40,.39],"panel",True)
    for x in [.29,.69]:
        for y in [-.183,.183]:
            geom(world,"box",[x,y,.093],[.0215,.0265,.0145],[.65,.68,.70])
            geom(world,"cylinder",[x,y,.111],[.009,.0055],[.75,.43,.11])
    for j,(x,y) in enumerate(MARKS):geom(world,"cylinder",[x,y,TOP+.00035],[.01,.0003],[.67,.41,.20],f"spot{j}")
    elem(world,"light",pos="0 -1 2.3",dir=".15 .45 -1",directional="true",diffuse=".8 .8 .8")
    elem(world,"light",pos="1 1 1.3",dir="-.5 -.7 -1",castshadow="false",diffuse=".4 .4 .4")
    urdf=E.parse(FR/"robots/panda_arm_hand.urdf")
    def mesh_assets(stem):
        dae=E.parse(FR/"meshes/visual"/(stem+".dae"))
        for el in dae.iter():el.tag=el.tag.split("}")[-1]
        byid={n.get("id"):n for n in dae.iter() if n.get("id")}
        parts=[]
        for node in dae.findall(".//visual_scene//node"):
            mat=node.findtext("matrix")
            transform=np.fromstring(mat,sep=" ").reshape(4,4) if mat else np.eye(4)
            inst=node.find("instance_geometry")
            if inst is None:continue
            mesh=byid[inst.get("url")[1:]].find("mesh")
            for tri in mesh.findall("triangles"):
                inputs=tri.findall("input");stride=max(int(i.get("offset","0")) for i in inputs)+1
                inp=next(i for i in inputs if i.get("semantic")=="VERTEX")
                pos_src=byid[inp.get("source")[1:]].find("input[@semantic='POSITION']").get("source")[1:]
                coords=np.fromstring(byid[pos_src].findtext("float_array"),sep=" ").reshape(-1,3)
                pts=coords@transform[:3,:3].T+transform[:3,3]
                faces=np.fromstring(tri.findtext("p"),dtype=int,sep=" ").reshape(-1,stride)[:,int(inp.get("offset","0"))].reshape(-1,3)
                color=[.8,.8,.82]
                sym=tri.get("material");binding=inst.find(".//instance_material[@symbol='%s']"%sym)
                if binding is not None:
                    effect=byid[binding.get("target")[1:]].find("instance_effect").get("url")[1:]
                    raw=byid[effect].findtext(".//diffuse/color")
                    if raw:color=np.fromstring(raw,sep=" ")[:3]
                label=stem+"_"+str(len(parts));obj=ASSET/(label+".obj")
                with obj.open("w") as f:
                    np.savetxt(f,pts,fmt="v %.7f %.7f %.7f");np.savetxt(f,faces+1,fmt="f %d %d %d")
                elem(asset,"mesh",name=label,file=str(obj));parts.append((label,color))
        return parts
    cache={}
    bodies={}
    links={n.get("name"):n for n in urdf.findall("link")}
    def body(name,parent,pos="0 0 0",euler="0 0 0"):
        b=elem(parent,"body",name=name,pos=pos,euler=euler);bodies[name]=b
        iner=links[name].find("inertial")
        if iner is not None:
            m=iner.find("mass").get("value");ii=iner.find("inertia")
            elem(b,"inertial",pos=iner.find("origin").get("xyz"),mass=m,fullinertia=" ".join(ii.get(s) for s in ("ixx","iyy","izz","ixy","ixz","iyz")))
        for visual in links[name].findall("visual"):
            stem=Path(visual.find("geometry/mesh").get("filename")).stem
            if stem not in cache:cache[stem]=mesh_assets(stem)
            orient=visual.find("origin")
            for label,color in cache[stem]:
                elem(b,"geom",type="mesh",mesh=label,rgba=v([*color,1]),euler=orient.get("rpy") if orient is not None else "0 0 0",contype="0",conaffinity="0")
        return b
    body("panda_link0",world)
    for j in urdf.findall("joint"):
        origin=j.find("origin");loc=origin.get("xyz","0 0 0");rot=origin.get("rpy","0 0 0")
        # Gripper holds the polishing stem at a fixed 12 mm opening per finger.
        if "finger" in j.get("name"):
            loc=v(np.fromstring(loc,sep=" ")+.012*np.fromstring(j.find("axis").get("xyz"),sep=" "))
        b=body(j.find("child").get("link"),bodies[j.find("parent").get("link")],loc,rot)
        if j.get("type")=="revolute":
            number=int(j.get("name")[-1])-1
            lim=j.find("limit");name=j.get("name")
            elem(b,"joint",type="hinge",name=name,axis="0 0 1",range=lim.get("lower")+" "+lim.get("upper"),armature=".06")
            elem(act,"position",joint=name,kp=str([3200,3200,2700,2500,1200,950,700][number]),kv=str([145,145,125,120,60,55,48][number]),forcerange="-"+lim.get("effort")+" "+lim.get("effort"))
    palm=bodies["panda_hand"]
    for z,r,l,col,name in [(.104,.012,.064,[.22,.25,.27],"stem"),(.138,.037,.012,[.92,.48,.09],"carrier"),(.15,.04,.014,[.05,.19,.20],"pad")]:
        geom(palm,"cylinder",[0,0,z],[r,l/2],col,name,name=="pad",solref=".02 1",friction=".35 .005 .0001")
    elem(elem(root,"contact"),"pair",geom1="pad",geom2="panel",condim="3",friction=".10 .10 .001 .0001 .0001",solref=".05 1",solimp=".55 .8 .004")
    return mj.MjModel.from_xml_string(E.tostring(root,encoding="unicode"))

def run():
    baseline=os.getenv("FIXED_DEPTH")=="1"
    m=model();d=mj.MjData(m);q=solve(HOME.copy(),ref(0),70);d.qpos[:]=q;d.ctrl[:]=q;mj.mj_forward(m,d)
    ren=mj.Renderer(m,height=720,width=1280);cam=mj.MjvCamera()
    cam.azimuth=132;cam.elevation=-24;cam.lookat[:]=[.24,0,.31];cam.distance=1.32
    cmd=[os.environ["FFMPEG"],"-v","error","-y","-f","rawvideo","-pix_fmt","rgb24","-s","1280x720","-r","24","-i","-","-c:v","libx264","-crf","18","-preset","veryfast","-pix_fmt","yuv420p",str(OUT/"Franka_MuJoCo.mp4")]
    writer=None if baseline else subprocess.Popen(cmd,stdin=subprocess.PIPE)
    rows=[];done=np.zeros(len(MARKS),bool);offset=0.;filtered=0.;pad=m.geom("pad").id
    marks=np.array([m.geom(f"spot{i}").id for i in range(len(MARKS))])
    for i in range(19*120):
        t=i*DT;req=ref(t);goal=force_goal(t);force=0.
        for j in range(d.ncon):
            co=d.contact[j]
            if pad in (co.geom1,co.geom2):
                wr=np.zeros(6);mj.mj_contactForce(m,d,j,wr)
                force+=abs((co.frame.reshape(3,3).T@wr[:3])[2])
        filtered+=DT/(.04+DT)*(force-filtered)
        if goal and baseline:offset=-.003
        elif goal:offset=np.clip(offset+DT*np.clip(.003*(filtered-goal),-.025,.025),-.035,.015)
        else:offset*=.96
        target=req.copy();target[2]+=offset;q=solve(q,target);d.ctrl[:]=q
        actual,_,_=kinematics(d.qpos)
        if goal and filtered>=3:done|=np.linalg.norm(MARKS-actual[:2],axis=1)<=.04
        rows.append([t,*req,*actual,goal,force,filtered,int(done.sum()),offset])
        if writer and i%5==0:
            m.geom_rgba[marks[done],3]=0
            if i==720:cam.distance=.90;cam.lookat[:]=[.44,0,.20];cam.elevation=-34
            if i==1560:cam.distance=1.35;cam.lookat[:]=[.25,0,.3];cam.azimuth=-135
            ren.update_scene(d,camera=cam);writer.stdin.write(ren.render().tobytes())
        mj.mj_step(m,d,nstep=2)
    if writer:
        writer.stdin.close();assert writer.wait()==0
    ren.close()
    with (OUT/("MuJoCo_fixed.csv" if baseline else "MuJoCo.csv")).open("w",newline="") as f:
        w=csv.writer(f);w.writerow(["t","ref_x","ref_y","ref_z","x","y","z","target_N","raw_N","filtered_N","covered","z_offset"]);w.writerows(rows)
if __name__=="__main__":run()

