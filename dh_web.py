#!/usr/bin/env python3
"""Interactive 3-link standard-DH visualizer (Dash + Plotly + NumPy)."""
import numpy as np
import plotly.graph_objects as go
from dash import Dash, Input, Output, ctx, dcc, html, no_update

PI = np.pi
KEYS = ("theta", "d", "r", "alpha")
SYMS = {"theta":"θ", "d":"d", "r":"r", "alpha":"α"}
SPECS = {
    "theta":(-PI, PI, .01), "d":(-2.5, 2.5, .01),
    "r":(0., 2.5, .01), "alpha":(-PI, PI, .01),
}
DEFAULTS = [
    dict(type="R", theta=5*PI/32, d=1., r=0., alpha=PI/2),
    dict(type="R", theta=PI/4, d=0., r=1., alpha=0.),
    dict(type="R", theta=-3*PI/8, d=0., r=1., alpha=0.),
]
DEFAULT_GRIP = .55
AXIS_COLORS = ("#ff4d4d", "#3ddc84", "#4da3ff")
REV, PRI, LINK = "#ff9f43", "#4dd8ff", "#f5c451"
BG, PANEL, BORDER, TEXT, MUTED = "#0b1020", "#121a2f", "#2a3658", "#eef3ff", "#9ba9c7"


def rz(t):
    c,s=np.cos(t),np.sin(t)
    return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1.]])


def tz(d):
    T=np.eye(4); T[2,3]=d; return T


def dh_matrix(theta,d,r,alpha):
    c,s=np.cos(theta),np.sin(theta); ca,sa=np.cos(alpha),np.sin(alpha)
    return np.array([[c,-s*ca,s*sa,r*c],[s,c*ca,-c*sa,r*s],[0,sa,ca,d],[0,0,0,1.]])


def forward_kinematics(params):
    T=np.eye(4); links=[]
    for p in params:
        Td=T@rz(p["theta"])@tz(p["d"])
        Ti=T@dh_matrix(p["theta"],p["d"],p["r"],p["alpha"])
        links.append((T,Td,Ti)); T=Ti
    return links,T


def line(a,b,color,width,dash="solid",hover=None):
    return go.Scatter3d(x=[a[0],b[0]],y=[a[1],b[1]],z=[a[2],b[2]],mode="lines",
        line=dict(color=color,width=width,dash=dash),hoverinfo="text" if hover else "skip",
        hovertext=[hover,hover] if hover else None,showlegend=False)


def point(p,color,size=6,symbol="circle",hover=None):
    return go.Scatter3d(x=[p[0]],y=[p[1]],z=[p[2]],mode="markers",
        marker=dict(size=size,color=color,symbol=symbol),hoverinfo="text" if hover else "skip",
        hovertext=[hover] if hover else None,showlegend=False)


def frame_traces(T,i,scale,labels):
    o=T[:3,3]; out=[]
    for k,col in enumerate(AXIS_COLORS):
        v=T[:3,k]; tip=o+scale*v; name=f"{'xyz'[k]}{i}"
        out += [line(o,tip,col,6,hover=name), go.Cone(x=[tip[0]],y=[tip[1]],z=[tip[2]],
            u=[v[0]],v=[v[1]],w=[v[2]],anchor="tip",sizemode="absolute",sizeref=.105,
            colorscale=[[0,col],[1,col]],showscale=False,hoverinfo="text",hovertext=[name],showlegend=False)]
    if labels:
        lp=o+(T[:3,0]+T[:3,1]+T[:3,2])*(.10+.015*i)
        out.append(go.Scatter3d(x=[lp[0]],y=[lp[1]],z=[lp[2]],mode="text",text=[f"F{i}"],
            textfont=dict(color="white",size=11),hoverinfo="text",hovertext=[f"Frame {i}"],showlegend=False))
    return out


def joint_symbol(T,jtype,i):
    o,x,y,z=T[:3,3],T[:3,0],T[:3,1],T[:3,2]
    if jtype=="R":
        a=np.linspace(0,2*PI,48); pts=np.array([o+.15*(np.cos(q)*x+np.sin(q)*y) for q in a])
        return [go.Scatter3d(x=pts[:,0],y=pts[:,1],z=pts[:,2],mode="lines",line=dict(color=REV,width=7),
            hoverinfo="text",hovertext=[f"Joint {i}: revolute about z{i-1}"]*len(pts),showlegend=False),
            line(o-.20*z,o+.20*z,REV,4,hover=f"Joint {i} rotation axis z{i-1}")]
    a,b=o-.21*z,o+.21*z; w=.10
    return [line(a+w*x,b+w*x,PRI,6,hover=f"Joint {i}: prismatic along z{i-1}"),
            line(a-w*x,b-w*x,PRI,6,hover=f"Joint {i}: prismatic along z{i-1}"),
            line(a+w*x,a-w*x,PRI,4),line(b+w*x,b-w*x,PRI,4),
            point(o,PRI,7,"diamond",f"Joint {i}: prismatic slider")]


def gripper(T,g):
    o,x,y,z=T[:3,3],T[:3,0],T[:3,1],T[:3,2]; s=.035+.18*np.clip(g,0,1)
    c=o+.05*x; l=c+s*y; r=c-s*y; lt=l+.32*x-.06*y; rt=r+.32*x+.06*y
    return [line(o-.10*x,c,"#d9e4fa",9,hover="End-effector wrist"),line(l,r,"#d9e4fa",8,hover="Gripper palm"),
            line(l,lt,"white",7,hover="Gripper finger"),line(r,rt,"white",7,hover="Gripper finger"),
            line(lt,lt-.08*y,"white",6),line(rt,rt+.08*y,"white",6),point(c+.04*z,"#ffd66b",4,hover=f"Grip = {g:.2f}")]


def make_figure(params,types,grip,frames,labels,construction,grid):
    links,T3=forward_kinematics(params); traces=[]
    if grid:
        gx=[]; gy=[]; gz=[]
        for v in np.arange(-3,3.01,.5):
            gx += [v,v,None,-3,3,None]; gy += [-3,3,None,v,v,None]; gz += [0,0,None,0,0,None]
        traces.append(go.Scatter3d(x=gx,y=gy,z=gz,mode="lines",line=dict(color="#26314c",width=1),hoverinfo="skip",showlegend=False))
    if frames: traces += frame_traces(np.eye(4),0,.56,labels)
    pts=[np.zeros(3)]
    for i,(Tp,Td,Ti) in enumerate(links):
        a,b,c=Tp[:3,3],Td[:3,3],Ti[:3,3]; traces += joint_symbol(Tp,types[i],i+1)
        if construction:
            if np.linalg.norm(b-a)>1e-5: traces.append(line(a,b,"#8aa0c8",8,hover=f"d{i+1} along z{i}"))
            if np.linalg.norm(c-b)>1e-5: traces.append(line(b,c,LINK,10,hover=f"r{i+1} along x{i+1}"))
            traces.append(line(a,c,"#53627f",2,"dash"))
        else: traces.append(line(a,c,LINK,10,hover=f"Link {i+1}"))
        pts.append(c)
        if frames: traces += frame_traces(Ti,i+1,.46,labels)
    p=np.array(pts); traces.append(go.Scatter3d(x=p[:,0],y=p[:,1],z=p[:,2],mode="markers",
        marker=dict(size=6,color=["white","#dce6ff","#dce6ff","#ffd66b"]),
        hoverinfo="text",hovertext=["Base","Frame 1","Frame 2","End effector"],showlegend=False))
    traces += gripper(T3,grip)
    axis=dict(backgroundcolor="#0a1122",gridcolor="#26314c",showbackground=True,zerolinecolor="#47567a",color="#8d9bbb")
    fig=go.Figure(traces); fig.update_layout(paper_bgcolor="#080d19",margin=dict(l=0,r=0,t=0,b=0),uirevision="keep",
        scene=dict(xaxis=dict(range=[-2,3.2],title="X",**axis),yaxis=dict(range=[-2.6,2.6],title="Y",**axis),
        zaxis=dict(range=[-2,3.2],title="Z",**axis),aspectmode="cube",camera=dict(eye=dict(x=1.5,y=1.3,z=.9)),dragmode="orbit"))
    return fig,T3


def matrix_table(T):
    f=lambda v:"0" if abs(v)<1e-10 else f"{v:.3f}"
    return html.Table([html.Tr([html.Td(f(v),style={"padding":"3px 6px","textAlign":"right"}) for v in row]) for row in T],
        style={"width":"100%","fontSize":"12px","fontFamily":"monospace","color":"#cfe0ff"})


def dh_table(params):
    return html.Table([html.Tr([html.Th(x) for x in ("Link","θ rad","d","r","α rad")])] +
        [html.Tr([html.Td(i+1),html.Td(f"{p['theta']:.3f}"),html.Td(f"{p['d']:.3f}"),html.Td(f"{p['r']:.3f}"),html.Td(f"{p['alpha']:.3f}")]) for i,p in enumerate(params)],
        style={"width":"100%","fontSize":"11px","color":"#d9e4fa"})


def validation_card():
    tests=[]
    tests.append(("Zero transform",np.allclose(dh_matrix(0,0,0,0),np.eye(4))))
    e=np.array([[0,-1,0,0],[1,0,0,0],[0,0,1,0],[0,0,0,1.]],float)
    tests.append(("θ = π/2 rotation",np.allclose(dh_matrix(PI/2,0,0,0),e,atol=1e-12)))
    e=np.eye(4); e[2,3]=1; tests.append(("d = 1 translation",np.allclose(dh_matrix(0,1,0,0),e)))
    _,T=forward_kinematics([dict(theta=0.,d=0.,r=1.,alpha=0.),dict(theta=0.,d=0.,r=1.,alpha=0.)])
    e=np.eye(4); e[0,3]=2; tests.append(("2-link planar x = 2",np.allclose(T,e)))
    ok=all(x[1] for x in tests)
    return html.Div([html.Div("PASS — reference cases agree" if ok else "FAIL — reference mismatch",style={"color":"#54e39e" if ok else "#ff6b6b","fontWeight":800})]+
        [html.Div(f"{'✓' if passed else '✗'} {name}",style={"fontSize":"12px"}) for name,passed in tests]+
        [html.Div("Analytic reference cases check the implementation; they do not prove every configuration.",style={"fontSize":"10px","color":MUTED,"marginTop":"6px"})])


CARD={"border":f"1px solid {BORDER}","background":"rgba(24,34,61,.72)","borderRadius":"12px","padding":"12px","marginBottom":"10px"}
INPUT_STYLE={"width":"84px","background":"#0b1326","color":TEXT,"border":f"1px solid {BORDER}","borderRadius":"5px","padding":"5px"}


def parameter_row(j,key):
    lo,hi,step=SPECS[key]; v=DEFAULTS[j][key]
    return html.Div([html.Div([html.Span(f"{SYMS[key]}{j+1}",id=f"lab_{key}{j}"),
        dcc.Slider(lo,hi,step,value=v,id=f"sl_{key}{j}",marks=None,updatemode="drag"),
        dcc.Input(id=f"in_{key}{j}",type="number",value=round(v,6),min=lo,max=hi,step=step,style=INPUT_STYLE)],
        style={"display":"grid","gridTemplateColumns":"45px 1fr 90px","gap":"8px","alignItems":"center"}),
        html.Div(id=f"sub_{key}{j}",style={"fontSize":"10px","color":MUTED,"marginLeft":"53px"})],style={"margin":"8px 0 10px"})


def joint_card(j):
    return html.Div([html.Div([html.B(f"Joint {j+1}"),dcc.Dropdown([{"label":"Revolute","value":"R"},{"label":"Prismatic","value":"P"}],
        DEFAULTS[j]["type"],id=f"type{j}",clearable=False,style={"width":"130px","color":"#000"})],
        style={"display":"flex","justifyContent":"space-between","alignItems":"center"}),
        html.Div(id=f"hint{j}",style={"color":"#ffd66b","fontSize":"11px","margin":"4px 0 8px","fontWeight":600}),
        *[parameter_row(j,k) for k in KEYS]],style=CARD)


app=Dash(__name__); server=app.server; app.title="3-Link DH Robot Visualizer"
app.layout=html.Div([html.Div([html.H1("3-Link DH Robot Visualizer",style={"fontSize":"19px"}),
    html.P("Drag the highlighted joint variable to move the robot. Revolute joints rotate about zᵢ₋₁; prismatic joints translate along zᵢ₋₁. DH axes: x red, y green, z blue.",style={"fontSize":"13px","color":MUTED}),
    html.Div("Exact values can be typed in the boxes beside the sliders.",style={"fontSize":"11px","color":"#8fe3ff","marginBottom":"12px"}),
    *[joint_card(j) for j in range(3)],
    html.Div([html.B("Gripper / claw"),html.Div([dcc.Slider(0,1,.01,value=DEFAULT_GRIP,id="grip",marks=None,updatemode="drag"),
        dcc.Input(id="grip_in",type="number",min=0,max=1,step=.01,value=DEFAULT_GRIP,style=INPUT_STYLE)],style={"display":"grid","gridTemplateColumns":"1fr 90px","gap":"8px"}),
        html.Small("0 = closed, 1 = fully open",style={"color":MUTED})],style=CARD),
    html.Div([html.B("Display"),dcc.Checklist(id="display",options=[{"label":" Show DH coordinate frames","value":"frames"},{"label":" Show compact frame names","value":"labels"},{"label":" Show DH offset construction","value":"construction"},{"label":" Show ground grid","value":"grid"}],value=["frames","labels","construction","grid"],labelStyle={"display":"block","fontSize":"13px","margin":"5px 0"}),
        html.Small("Axis names are hover labels so they do not smear or overlap while the arm moves.",style={"color":MUTED}),
        html.Div([html.Button("Reset Wolfram example",id="reset",n_clicks=0),html.Button("Zero angles",id="zero",n_clicks=0)],style={"display":"grid","gridTemplateColumns":"1fr 1fr","gap":"8px","marginTop":"10px"})],style=CARD),
    html.H3("Current DH table",style={"fontSize":"14px"}),html.Div(id="dh_table",style=CARD),
    html.H3("Standard DH transform",style={"fontSize":"14px"}),html.Pre("Aᵢ = Rz(θᵢ) · Tz(dᵢ) · Tx(rᵢ) · Rx(αᵢ)",style={"fontSize":"11px","color":"#cfe0ff"}),
    html.H3("End-effector transform T₀³",style={"fontSize":"14px"}),html.Div(id="matrix",style=CARD),
    html.H3("Numerical validation",style={"fontSize":"14px"}),html.Div(validation_card(),style=CARD)],
    style={"overflowY":"auto","padding":"18px","background":PANEL,"borderRight":f"1px solid {BORDER}"}),
    dcc.Graph(id="graph",style={"height":"100vh"},config={"displaylogo":False,"scrollZoom":True})],
    style={"display":"grid","gridTemplateColumns":"430px 1fr","height":"100vh","background":BG,"color":TEXT,"fontFamily":"Arial,sans-serif"})

SLIDERS=[f"sl_{k}{j}" for j in range(3) for k in KEYS]
LABS=[f"lab_{k}{j}" for j in range(3) for k in KEYS]
SUBS=[f"sub_{k}{j}" for j in range(3) for k in KEYS]
TYPES=[f"type{j}" for j in range(3)]
HINTS=[f"hint{j}" for j in range(3)]


def register_sync(j,key):
    s=f"sl_{key}{j}"; n=f"in_{key}{j}"
    @app.callback(Output(s,"value"),Output(n,"value"),Input(s,"value"),Input(n,"value"),Input("reset","n_clicks"),Input("zero","n_clicks"),prevent_initial_call=True)
    def sync(sv,nv,_r,_z):
        trigger=ctx.triggered_id
        if trigger=="reset": v=DEFAULTS[j][key]; return v,round(v,6)
        if trigger=="zero": return (0.,0.) if key=="theta" else (no_update,no_update)
        if trigger==n:
            if nv is None: return no_update,no_update
            lo,hi,_=SPECS[key]; v=float(np.clip(nv,lo,hi)); return v,v
        return sv,round(float(sv),6)
for j in range(3):
    for k in KEYS: register_sync(j,k)


@app.callback(Output("grip","value"),Output("grip_in","value"),Input("grip","value"),Input("grip_in","value"),Input("reset","n_clicks"),prevent_initial_call=True)
def sync_grip(sv,nv,_r):
    if ctx.triggered_id=="reset": return DEFAULT_GRIP,DEFAULT_GRIP
    if ctx.triggered_id=="grip_in":
        if nv is None: return no_update,no_update
        v=float(np.clip(nv,0,1)); return v,v
    return sv,round(float(sv),3)


@app.callback([Output("graph","figure"),Output("matrix","children"),Output("dh_table","children")]+[Output(x,"style") for x in LABS]+[Output(x,"children") for x in SUBS]+[Output(x,"children") for x in HINTS],
    [Input(x,"value") for x in SLIDERS]+[Input(x,"value") for x in TYPES]+[Input("grip","value"),Input("display","value")])
def update(*args):
    vals=args[:12]; types=list(args[12:15]); grip=args[15]; display=args[16] or []; params=[]; styles=[]; subs=[]
    for j in range(3):
        p={}
        for n,key in enumerate(KEYS):
            v=float(vals[j*4+n]); p[key]=v; active=(types[j]=="R" and key=="theta") or (types[j]=="P" and key=="d")
            styles.append({"fontSize":"13px","color":"#ffd66b" if active else "#c7d3ed","fontWeight":800 if active else 400})
            txt=f"{v:.3f} rad = {v*180/PI:.1f}°" if key in ("theta","alpha") else f"{v:.3f}"
            subs.append(txt+(" • MOVE JOINT" if active else ""))
        params.append(p)
    hints=[f"Drag θ{i+1} to rotate Joint {i+1} about z{i} (orange ring)." if t=="R" else f"Drag d{i+1} to slide Joint {i+1} along z{i} (cyan rail)." for i,t in enumerate(types)]
    fig,T3=make_figure(params,types,grip,"frames" in display,"labels" in display,"construction" in display,"grid" in display)
    return [fig,matrix_table(T3),dh_table(params),*styles,*subs,*hints]


if __name__=="__main__": app.run(debug=True,port=8050)
