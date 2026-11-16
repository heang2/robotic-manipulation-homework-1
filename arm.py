"""Isaac Sim 5.1 standalone. Set FFMPEG, and optionally KIT_WINDOW=1 on an isolated X display."""
import csv,os,sys,time,subprocess,traceback
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from isaacsim import SimulationApp
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from control import *
ui=os.getenv("KIT_WINDOW")=="1"
app=SimulationApp({"headless":not ui,"hide_ui":not ui,"width":1280,"height":720,
                   "window_width":1500,"window_height":940,"renderer":"RaytracedLighting",
                   "multi_gpu":False,"anti_aliasing":3})
import omni.kit.commands
import omni.replicator.core as rep
from isaacsim.core.api import World
from isaacsim.core.prims import RigidPrim,Articulation
from isaacsim.core.utils.extensions import get_extension_path_from_name
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.sensors.camera import Camera
from pxr import Usd,UsdGeom,UsdShade,UsdPhysics,UsdLux,PhysxSchema,Gf,Sdf

def import_arm():
    source=Path(get_extension_path_from_name("isaacsim.asset.importer.urdf"))/"data/urdf/robots/franka_description"
    xml=ET.parse(source/"robots/panda_arm_hand.urdf")
    for m in xml.findall(".//mesh"):
        m.set("filename",str(source/m.get("filename").split("franka_description/")[1]))
    palm=xml.find("link[@name='panda_hand']")
    for z,radius,length,color,material in [(.104,.012,.064,"0.16 0.18 0.2 1","steel"),(.138,.037,.012,"0.9 0.40 0.035 1","carrier"),(.150,.040,.014,"0.075 0.19 0.2 1","rubber")]:
        for kind in ["visual","collision"]:
            item=ET.SubElement(palm,kind);ET.SubElement(item,"origin",xyz=f"0 0 {z}")
            ET.SubElement(ET.SubElement(item,"geometry"),"cylinder",radius=str(radius),length=str(length))
            if kind=="visual":ET.SubElement(ET.SubElement(item,"material",name=material),"color",rgba=color)
    file=HERE/"station_panda.urdf";xml.write(file,encoding="utf-8")
    _,config=omni.kit.commands.execute("URDFCreateImportConfig")
    config.fix_base=True;config.merge_fixed_joints=True;config.import_inertia_tensor=True
    ok,path=omni.kit.commands.execute("URDFParseAndImportFile",urdf_path=str(file),import_config=config,get_articulation_root=True)
    if not ok:raise RuntimeError("Failed to import Franka asset")
    print("ARTICULATION",path,flush=True)
    return path

def run():
    world=World(physics_dt=DT,rendering_dt=1/FPS,stage_units_in_meters=1.)
    stage=get_current_stage();world.scene.add(Articulation(import_arm(),name="Franka_Panda"))
    robot=world.scene.get_object("Franka_Panda")
    mats={}
    def material(name,color,metal=.0,rough=.4):
        mat=UsdShade.Material.Define(stage,"/World/Materials/"+name)
        shader=UsdShade.Shader.Define(stage,mat.GetPath().AppendChild("Shader"))
        shader.CreateIdAttr("UsdPreviewSurface")
        for key,typ,val in [("diffuseColor",Sdf.ValueTypeNames.Color3f,Gf.Vec3f(*color)),("metallic",Sdf.ValueTypeNames.Float,metal),("roughness",Sdf.ValueTypeNames.Float,rough)]:
            shader.CreateInput(key,typ).Set(val)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),"surface")
        mats[name]=mat
    for args in [("aluminum",(.49,.54,.58),.65,.33),("dark",(.047,.067,.089),.45,.32),("bench",(.12,.19,.23),.5,.35),("floor",(.27,.32,.33),.05,.67),("brass",(.78,.34,.045),.45,.3),("panel",(.10,.26,.25),.15,.65),("residue",(.60,.315,.12),.12,.61),("cyan",(.08,.49,.46),.25,.32)]:
        material(*args)
    def shape(name,p,size,mat,cyl=False):
        primpath="/World/"+name
        if cyl:
            s=UsdGeom.Cylinder.Define(stage,primpath);s.CreateRadiusAttr(size[0]);s.CreateHeightAttr(size[1])
        else:
            s=UsdGeom.Cube.Define(stage,primpath);s.CreateSizeAttr(1)
        s.AddTranslateOp().Set(Gf.Vec3d(*p))
        if not cyl:s.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdShade.MaterialBindingAPI.Apply(s.GetPrim()).Bind(mats[mat]);return s
    def cyl(n,p,r,l,m):return shape(n,p,(r,l),m,True)
    shape("Workcell/Floor",(0,0,-.82),(14,14,.08),"floor")
    shape("Workcell/BenchTop",(.26,0,-.037),(1.48,1.02,.071),"bench")
    for i,x in enumerate([-.40,.92]):
        for j,y in enumerate([-.42,.42]):
            shape(f"Workcell/Leg{i}{j}",(x,y,-.43),(.075,.075,.73),"dark")
            shape(f"Workcell/Foot{i}{j}",(x,y,-.765),(.12,.12,.035),"aluminum")
    for i,y in enumerate(np.linspace(-.46,.46,13)):
        shape(f"Workcell/TSlot{i}",(.26,y,.0003),(1.45,.005,.001),"dark")
    cyl("Workcell/RobotBase",(0,0,-.003),.119,.014,"dark")
    for i,angle in enumerate(np.linspace(0,2*np.pi,8,endpoint=False)):
        cyl(f"Workcell/FlangeBolt{i}",(.099*np.cos(angle),.099*np.sin(angle),.006),.007,.008,"aluminum")
    shape("Fixture/Vise",(.49,0,.045),(.47,.47,.084),"dark")
    board=shape("Instrumented_Workpiece",(.49,0,TOP-.019),(.395,.395,.038),"panel").GetPrim()
    UsdPhysics.CollisionAPI.Apply(board)
    from omni.physx.scripts import physicsUtils
    compliant=UsdShade.Material.Define(stage,"/World/Materials/PadContact")
    surface=UsdPhysics.MaterialAPI.Apply(compliant.GetPrim())
    surface.CreateStaticFrictionAttr(.12);surface.CreateDynamicFrictionAttr(.10)
    px=PhysxSchema.PhysxMaterialAPI.Apply(compliant.GetPrim())
    px.CreateFrictionCombineModeAttr("min")
    px.CreateCompliantContactStiffnessAttr(2400.)
    px.CreateCompliantContactDampingAttr(45.)
    physicsUtils.add_physics_material_to_prim(stage,board,compliant.GetPath())
    UsdPhysics.RigidBodyAPI.Apply(board).CreateKinematicEnabledAttr(True)
    UsdPhysics.MassAPI.Apply(board).CreateMassAttr(1.)
    load=world.scene.add(RigidPrim(str(board.GetPath()),name="LoadCell",track_contact_forces=True))
    for i,x in enumerate([.29,.69]):
        for j,y in enumerate([-.183,.183]):
            shape(f"Fixture/Jaw{i}{j}",(x,y,.093),(.043,.053,.029),"aluminum")
            cyl(f"Fixture/Fastener{i}{j}",(x,y,.111),.009,.011,"brass")
    # A 12 x 5 coupon array makes physical contact progression visible from the camera.
    marks=[cyl(f"Deposits/Spot{i:02d}",(x,y,TOP+.00035),.010,.0006,"residue") for i,(x,y) in enumerate(MARKS)]
    sky=UsdLux.DomeLight.Define(stage,"/World/Lighting/Fill");sky.CreateIntensityAttr(290)
    sun=UsdLux.DistantLight.Define(stage,"/World/Lighting/Key");sun.CreateIntensityAttr(1400);sun.CreateAngleAttr(16);sun.AddRotateXYZOp().Set(Gf.Vec3f(24,-40,14))
    rim=UsdLux.DistantLight.Define(stage,"/World/Lighting/Rim");rim.CreateIntensityAttr(600);rim.AddRotateXYZOp().Set(Gf.Vec3f(-50,20,-40))
    for p in stage.Traverse():
        if p.HasAPI(UsdPhysics.RigidBodyAPI) and "panda" in str(p.GetPath()).lower():
            PhysxSchema.PhysxRigidBodyAPI.Apply(p).CreateDisableGravityAttr(True)
    eye=np.array([1.31,-1.45,1.05]);look=np.array([.23,0,.32])
    set_camera_view(eye,look)
    cam=Camera("/World/InspectionCamera",position=eye,resolution=(1280,720))
    def camera_at(eye,target,focal):
        q=Gf.Matrix4d(1).SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse().ExtractRotationQuat()
        cam.set_world_pose(position=eye,orientation=np.array([q.GetReal(),*q.GetImaginary()]),camera_axes="usd")
        cam.set_focal_length(focal);cam.set_clipping_range(.025,30)
    camera_at(eye,look,2.25)
    world.reset();cam.initialize()
    idx=[robot.get_dof_index("panda_joint"+str(i)) for i in range(1,8)]
    print("DOFS",robot.dof_names,flush=True)
    q=solve(HOME.copy(),ref(0),70)
    initial=robot.get_joint_positions();initial[0,idx]=q
    for n in robot.dof_names:
        if "finger" in n:initial[0,robot.get_dof_index(n)]=.012
    robot.set_joint_positions(initial);robot.set_joint_position_targets(initial)
    robot.set_gains(kps=np.array([[3200,3200,2700,2500,1200,950,700]],dtype=np.float32),kds=np.array([[145,145,125,120,60,55,48]],dtype=np.float32),joint_indices=idx)
    rep.orchestrator.set_capture_on_play(False)
    for _ in range(4):rep.orchestrator.step(rt_subframes=2,delta_time=0.,pause_timeline=False)
    stage.Flatten().Export(str(HERE/"Panda_Workcell.usdc"))
    ff=os.environ["FFMPEG"]
    video=subprocess.Popen([ff,"-v","error","-y","-f","rawvideo","-pix_fmt","rgb24","-s","1280x720","-r",str(FPS),"-i","-","-c:v","libx264","-pix_fmt","yuv420p","-preset","veryfast","-crf","18","-movflags","+faststart","Franka_Isaac.mp4"],stdin=subprocess.PIPE)
    if ui:
        from window_record import start_window_capture
        desktop=start_window_capture(ff)
    erased=np.zeros(len(MARKS),dtype=bool);offset=0.;smooth_force=0.;rows=[]
    for i in range(19*120):
        t=i*DT; requested=ref(t);goal=force_goal(t)
        measured=abs(float(load.get_net_contact_forces(dt=DT)[0,2]))
        smooth_force+=(DT/(.040+DT))*(measured-smooth_force)
        if goal:
            offset=np.clip(offset+DT*np.clip(.003*(smooth_force-goal),-.025,.025),-.035,.015)
        else:offset*=.96
        target=requested.copy();target[2]+=offset
        q=solve(q,target)
        robot.set_joint_position_targets(q[None,:].astype(np.float32),joint_indices=idx)
        real,_,_=kinematics(robot.get_joint_positions()[0,idx])
        if goal and smooth_force>=3:
            hits=(np.linalg.norm(MARKS-real[:2],axis=1)<=.04)&~erased
            for j in np.flatnonzero(hits):marks[j].MakeInvisible()
            erased|=hits
        rows.append([t,*requested,*real,goal,measured,smooth_force,int(erased.sum()),offset])
        if i%5==0:
            if i==6*120:camera_at(np.array([.96,-.75,.73]),np.array([.46,.0,.19]),2.52)
            if i==13*120:camera_at(np.array([1.18,.94,1.03]),np.array([.29,.015,.31]),2.35)
            if i==17*120:camera_at(eye,look,2.25)
            rep.orchestrator.step(rt_subframes=1,delta_time=0.,pause_timeline=False)
            rgb=cam.get_rgba()
            if rgb.shape!=(720,1280,4):raise ValueError(str(rgb.shape))
            video.stdin.write(np.ascontiguousarray(rgb[:,:,:3]).tobytes())
        world.step(render=False)
    video.stdin.close()
    if video.wait(): raise RuntimeError("Encoding failed")
    if ui:
        desktop.stdin.write(b"q"); desktop.stdin.flush();desktop.wait(timeout=25)
    with open("Franka.csv","w",newline="") as f:
        w=csv.writer(f);w.writerow(["t","ref_x","ref_y","ref_z","x","y","z","target_N","raw_N","filtered_N","covered","z_offset"]);w.writerows(rows)
    print("TRIAL_COMPLETE",int(erased.sum()),flush=True)
try:
    run()
except BaseException:
    traceback.print_exc();sys.stderr.flush()
app.close()

