#!/usr/bin/env python3
"""
3-Link Denavit-Hartenberg Robot Visualizer - web app (Python / Dash + Plotly)

Standard (classical) DH convention:
    A_i = Rz(theta_i) . Tz(d_i) . Tx(r_i) . Rx(alpha_i)
    0T3 = A1 . A2 . A3

Install:
    pip install dash plotly numpy

Run:
    python dh_web.py

Open:
    http://127.0.0.1:8050
"""

import numpy as np
import plotly.graph_objects as go
from dash import Dash, Input, Output, ctx, dcc, html, no_update

DEG = np.pi / 180.0

DEFAULTS = [
    dict(type="R", theta=5 * 180 / 32, d=1.0, r=0.0, alpha=90.0),
    dict(type="R", theta=45.0, d=0.0, r=1.0, alpha=0.0),
    dict(type="R", theta=-67.5, d=0.0, r=1.0, alpha=0.0),
]

KEYS = ("theta", "d", "r", "alpha")
SYMS = {"theta": "θ", "d": "d", "r": "r", "alpha": "α"}

SPECS = {
    "theta": (-180, 180, 1),
    "d": (-2.5, 2.5, 0.01),
    "r": (0.0, 2.5, 0.01),
    "alpha": (-180, 180, 1),
}

AXIS_COLORS = ("#ff4d4d", "#3ddc84", "#4da3ff")
BG, PANEL, BORDER, TEXT, MUTED = (
    "#0b1020",
    "#121a2f",
    "#2a3658",
    "#eef3ff",
    "#9ba9c7",
)


def rz(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, 0, 0],
        [s, c, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1.0],
    ])


def tz(d):
    T = np.eye(4)
    T[2, 3] = d
    return T


def dh_matrix(theta, d, r, alpha):
    """Standard DH homogeneous transform (angles in radians)."""
    c, s = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [c, -s * ca, s * sa, r * c],
        [s, c * ca, -c * sa, r * s],
        [0, sa, ca, d],
        [0, 0, 0, 1.0],
    ])


def forward_kinematics(params):
    """params: list of dicts (angles in radians). Returns (links, T03)."""
    T = np.eye(4)
    links = []
    for p in params:
        T_after_d = T @ rz(p["theta"]) @ tz(p["d"])
        T_i = T @ dh_matrix(p["theta"], p["d"], p["r"], p["alpha"])
        links.append(dict(T_prev=T, T_after_d=T_after_d, T_i=T_i))
        T = T_i
    return links, T


def _line(a, b, color, width, dash="solid"):
    return go.Scatter3d(
        x=[a[0], b[0]],
        y=[a[1], b[1]],
        z=[a[2], b[2]],
        mode="lines",
        line=dict(color=color, width=width, dash=dash),
        hoverinfo="skip",
        showlegend=False,
    )


def frame_traces(T, index, scale, labels):
    o = T[:3, 3]
    traces = []

    for k, col in enumerate(AXIS_COLORS):
        direction = T[:3, k]
        tip = o + direction * scale

        traces.append(_line(o, tip, col, 6))
        traces.append(go.Cone(
            x=[tip[0]], y=[tip[1]], z=[tip[2]],
            u=[direction[0]], v=[direction[1]], w=[direction[2]],
            anchor="tip",
            sizemode="absolute",
            sizeref=0.12,
            colorscale=[[0, col], [1, col]],
            showscale=False,
            hoverinfo="skip",
            showlegend=False,
        ))

        if labels:
            lp = o + direction * scale * 1.2
            traces.append(go.Scatter3d(
                x=[lp[0]], y=[lp[1]], z=[lp[2]],
                mode="text",
                text=[f"{'xyz'[k]}{index}"],
                textfont=dict(color=col, size=12),
                hoverinfo="skip",
                showlegend=False,
            ))

    if labels:
        lp = o + np.array([0.0, 0.0, 0.1])
        traces.append(go.Scatter3d(
            x=[lp[0]], y=[lp[1]], z=[lp[2]],
            mode="text",
            text=[f"Frame {index}"],
            textfont=dict(color="white", size=11),
            hoverinfo="skip",
            showlegend=False,
        ))

    return traces


def make_figure(params_rad, show_frames, show_labels, show_construction, show_grid):
    links, T3 = forward_kinematics(params_rad)
    traces = []

    if show_grid:
        gx, gy, gz = [], [], []
        for v in np.arange(-3, 3.01, 0.5):
            gx += [v, v, None]
            gy += [-3, 3, None]
            gz += [0, 0, None]

            gx += [-3, 3, None]
            gy += [v, v, None]
            gz += [0, 0, None]

        traces.append(go.Scatter3d(
            x=gx, y=gy, z=gz,
            mode="lines",
            line=dict(color="#26314c", width=1),
            hoverinfo="skip",
            showlegend=False,
        ))

    if show_frames:
        traces += frame_traces(np.eye(4), 0, 0.56, show_labels)

    joint_pts = [np.zeros(3)]

    for i, L in enumerate(links):
        a = L["T_prev"][:3, 3]
        b = L["T_after_d"][:3, 3]
        c = L["T_i"][:3, 3]

        if show_construction:
            if np.linalg.norm(b - a) > 1e-5:
                traces.append(_line(a, b, "#8aa0c8", 8))
            if np.linalg.norm(c - b) > 1e-5:
                traces.append(_line(b, c, "#f5c451", 10))
            traces.append(_line(a, c, "#53627f", 2, dash="dash"))
        else:
            traces.append(_line(a, c, "#f5c451", 10))

        joint_pts.append(c)

        if show_frames:
            traces += frame_traces(L["T_i"], i + 1, 0.46, show_labels)

    jp = np.array(joint_pts)
    colors = ["#ffffff", "#dce6ff", "#dce6ff", "#ffd66b"]

    traces.append(go.Scatter3d(
        x=jp[:, 0], y=jp[:, 1], z=jp[:, 2],
        mode="markers",
        marker=dict(size=6, color=colors),
        hoverinfo="skip",
        showlegend=False,
    ))

    cx, cy, cz, h = 0.6, 0.0, 0.6, 2.4
    axis = dict(
        backgroundcolor="#0a1122",
        gridcolor="#26314c",
        showbackground=True,
        zerolinecolor="#47567a",
        color="#8d9bbb",
    )

    fig = go.Figure(traces)
    fig.update_layout(
        paper_bgcolor="#080d19",
        margin=dict(l=0, r=0, t=0, b=0),
        uirevision="keep",
        scene=dict(
            xaxis=dict(range=[cx - h, cx + h], title="X", **axis),
            yaxis=dict(range=[cy - h, cy + h], title="Y", **axis),
            zaxis=dict(range=[cz - h, cz + h], title="Z", **axis),
            aspectmode="cube",
            camera=dict(eye=dict(x=1.5, y=1.3, z=0.9)),
        ),
    )

    return fig, T3


def matrix_table(T):
    def fmt(v):
        return "0" if abs(v) < 1e-10 else f"{v:.3f}"

    return html.Table(
        [
            html.Tr([
                html.Td(
                    fmt(v),
                    style=dict(
                        padding="3px 6px",
                        textAlign="right",
                        borderBottom="1px solid rgba(70,85,120,.3)",
                    ),
                )
                for v in row
            ])
            for row in T
        ],
        style=dict(
            width="100%",
            borderCollapse="collapse",
            fontSize="12px",
            fontFamily="ui-monospace, Menlo, Consolas, monospace",
            color="#cfe0ff",
        ),
    )


card_style = dict(
    border=f"1px solid {BORDER}",
    background="rgba(24,34,61,.72)",
    borderRadius="12px",
    padding="12px",
    marginBottom="10px",
)


def slider_row(j, key):
    lo, hi, step = SPECS[key]

    return html.Div(
        style=dict(
            display="grid",
            gridTemplateColumns="34px 1fr 54px",
            gap="8px",
            alignItems="center",
            margin="4px 0",
        ),
        children=[
            html.Span(
                f"{SYMS[key]}{j + 1}",
                id=f"lab_{key}{j}",
                style=dict(fontSize="13px", color="#c7d3ed"),
            ),
            dcc.Slider(
                lo, hi, step,
                value=DEFAULTS[j][key],
                id=f"sl_{key}{j}",
                marks=None,
                updatemode="drag",
                tooltip=None,
            ),
            html.Span(
                "",
                id=f"val_{key}{j}",
                style=dict(
                    fontSize="12px",
                    textAlign="right",
                    color="#d9e4fa",
                    fontVariantNumeric="tabular-nums",
                ),
            ),
        ],
    )


def joint_card(j):
    return html.Div(
        style=card_style,
        children=[
            html.Div(
                style=dict(
                    display="flex",
                    justifyContent="space-between",
                    alignItems="center",
                    marginBottom="6px",
                ),
                children=[
                    html.Div(
                        f"Joint {j + 1}",
                        style=dict(fontWeight=700, fontSize="14px"),
                    ),
                    dcc.Dropdown(
                        options=[
                            {"label": "Revolute", "value": "R"},
                            {"label": "Prismatic", "value": "P"},
                        ],
                        value=DEFAULTS[j]["type"],
                        id=f"type{j}",
                        clearable=False,
                        style=dict(width="130px", color="#000", fontSize="13px"),
                    ),
                ],
            ),
            *[slider_row(j, k) for k in KEYS],
        ],
    )


app = Dash(__name__)
server = app.server
app.title = "3-Link DH Robot Visualizer"

app.layout = html.Div(
    style=dict(
        display="grid",
        gridTemplateColumns="400px 1fr",
        height="100vh",
        background=BG,
        color=TEXT,
        fontFamily="Inter, system-ui, -apple-system, Segoe UI, sans-serif",
    ),
    children=[
        html.Div(
            style=dict(
                overflowY="auto",
                padding="18px",
                background=f"linear-gradient(180deg,{PANEL},#0f1628)",
                borderRight=f"1px solid {BORDER}",
            ),
            children=[
                html.H1(
                    "3-Link DH Robot Visualizer",
                    style=dict(fontSize="19px", margin="0 0 7px"),
                ),
                html.P(
                    "Interactive standard Denavit-Hartenberg model. "
                    "The colored triads are the coordinate frames generated by "
                    "the DH transforms (x red, y green, z blue).",
                    style=dict(color=MUTED, fontSize="13px", lineHeight="1.45"),
                ),
                html.H2(
                    "Joint / link parameters",
                    style=dict(fontSize="14px", margin="16px 0 9px"),
                ),
                *[joint_card(j) for j in range(3)],
                html.Div(
                    style=card_style,
                    children=[
                        html.Div(
                            "Display",
                            style=dict(
                                fontWeight=700,
                                fontSize="13px",
                                marginBottom="6px",
                            ),
                        ),
                        dcc.Checklist(
                            id="display",
                            options=[
                                {"label": " Show DH coordinate frames", "value": "frames"},
                                {"label": " Show frame / axis labels", "value": "labels"},
                                {"label": " Show DH offset construction", "value": "construction"},
                                {"label": " Show ground grid", "value": "grid"},
                            ],
                            value=["frames", "labels", "construction", "grid"],
                            labelStyle=dict(
                                display="block",
                                margin="5px 0",
                                fontSize="13px",
                                color="#c7d3ed",
                            ),
                        ),
                        html.Div(
                            style=dict(
                                display="grid",
                                gridTemplateColumns="1fr 1fr",
                                gap="8px",
                                marginTop="12px",
                            ),
                            children=[
                                html.Button("Reset Wolfram example", id="reset", n_clicks=0),
                                html.Button("Zero angles", id="zero", n_clicks=0),
                            ],
                        ),
                    ],
                ),
                html.H2(
                    "Standard DH transform",
                    style=dict(fontSize="14px", margin="16px 0 9px"),
                ),
                html.Pre(
                    "Aᵢ = Rz(θᵢ) · Tz(dᵢ) · Tx(rᵢ) · Rx(αᵢ)\n\n"
                    "[ cθ −sθcα  sθsα   r cθ ]\n"
                    "[ sθ  cθcα −cθsα   r sθ ]\n"
                    "[ 0    sα     cα      d  ]\n"
                    "[ 0     0      0      1  ]",
                    style=dict(
                        fontSize="11px",
                        lineHeight="1.45",
                        color="#cfe0ff",
                        background="#0b1326",
                        borderRadius="8px",
                        padding="9px",
                        overflowX="auto",
                    ),
                ),
                html.H2(
                    "End-effector transform T₃",
                    style=dict(fontSize="14px", margin="16px 0 9px"),
                ),
                html.Div(id="matrix", style=card_style),
            ],
        ),
        dcc.Graph(
            id="graph",
            style=dict(height="100vh"),
            config=dict(displaylogo=False),
        ),
    ],
)


SLIDER_IDS = [f"sl_{k}{j}" for j in range(3) for k in KEYS]
VAL_IDS = [f"val_{k}{j}" for j in range(3) for k in KEYS]
LAB_IDS = [f"lab_{k}{j}" for j in range(3) for k in KEYS]
TYPE_IDS = [f"type{j}" for j in range(3)]


@app.callback(
    [Output("graph", "figure"), Output("matrix", "children")]
    + [Output(i, "children") for i in VAL_IDS]
    + [Output(i, "style") for i in LAB_IDS],
    [Input(i, "value") for i in SLIDER_IDS]
    + [Input(i, "value") for i in TYPE_IDS]
    + [Input("display", "value")],
)
def update(*args):
    slider_vals = args[:12]
    types = args[12:15]
    display = args[15] or []

    params_rad, val_texts, lab_styles = [], [], []

    for j in range(3):
        row = {}

        for n, key in enumerate(KEYS):
            v = slider_vals[j * 4 + n]
            is_angle = key in ("theta", "alpha")
            row[key] = v * DEG if is_angle else v
            val_texts.append(f"{v:.0f}°" if is_angle else f"{v:.2f}")

            var = "theta" if types[j] == "R" else "d"
            highlighted = key == var

            lab_styles.append(dict(
                fontSize="13px",
                color="#ffd66b" if highlighted else "#c7d3ed",
                fontWeight=800 if highlighted else 400,
            ))

        params_rad.append(row)

    fig, T3 = make_figure(
        params_rad,
        show_frames="frames" in display,
        show_labels="labels" in display,
        show_construction="construction" in display,
        show_grid="grid" in display,
    )

    return [fig, matrix_table(T3), *val_texts, *lab_styles]


@app.callback(
    [Output(i, "value") for i in SLIDER_IDS]
    + [Output(i, "value") for i in TYPE_IDS],
    Input("reset", "n_clicks"),
    Input("zero", "n_clicks"),
    prevent_initial_call=True,
)
def reset_or_zero(_r, _z):
    if ctx.triggered_id == "reset":
        sliders = [DEFAULTS[j][k] for j in range(3) for k in KEYS]
        types = [DEFAULTS[j]["type"] for j in range(3)]
        return sliders + types

    sliders = [
        0 if k == "theta" else no_update
        for j in range(3)
        for k in KEYS
    ]

    return sliders + [no_update] * 3


if __name__ == "__main__":
    app.run(debug=True, port=8050)
