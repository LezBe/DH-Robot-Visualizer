# DH Robot Visualizer

Interactive 3-link Denavit-Hartenberg robot visualizer built with Python, Dash, Plotly, and NumPy.

## What the visualizer shows

- Standard/classical DH convention: `A_i = Rz(theta_i) Tz(d_i) Tx(r_i) Rx(alpha_i)`
- Frames 0-3 with consistent axis colors: **x = red, y = green, z = blue**
- Revolute joints as orange rotation rings about `z_(i-1)`
- Prismatic joints as cyan slider rails along `z_(i-1)`
- Interactive joint motion while dragging sliders
- Exact numeric entry beside every DH slider
- Angular parameters in radians, with degrees shown as secondary reference
- Adjustable end-effector gripper/claw
- Current DH parameter table and end-effector transform
- Hover-first axis labels to reduce overlap while the robot moves
- Built-in analytic validation checks

For a revolute joint, **theta_i** is highlighted as the motion variable.
For a prismatic joint, **d_i** is highlighted as the motion variable.

## Run locally

```bash
pip install -r requirements.txt
python dh_web.py
```

Then open:

```text
http://127.0.0.1:8050/
```

## Run the kinematics tests

```bash
python -m unittest test_dh.py
```

The tests include identity, 90-degree z rotation, unit z translation, and a two-link planar reference case.

## Deploy on Render

This repository includes a `render.yaml` file for deployment.

1. Sign in to Render.
2. Choose **New +** -> **Blueprint**.
3. Connect this GitHub repository.
4. Render should detect `render.yaml`.
5. Create the service.

The app is served with Gunicorn using:

```bash
gunicorn dh_web:server
```

The current public deployment is:

https://dh-robot-visualizer.onrender.com/

## Revision note

The first generated version was evaluated critically in Programming Assignment 1. The current implementation addresses the documented usability and visualization faults, including missing exact numeric entry, missing gripper control, degree-only presentation, weak joint-type graphics, overlapping labels, and the lack of independent reference checks.

## Engineering Evidence and Contributions

The [live interactive demo](https://dh-robot-visualizer.onrender.com/) is the primary visual demonstration: it lets reviewers vary joint parameters and inspect the changing frames and end-effector transform.

The public repository includes:

- [Interactive web application](dh_web.py) — Python/Dash source for controls and 3D rendering
- [Analytic kinematics tests](test_dh.py) — numerical reference cases for transform correctness
- [Deployment configuration](render.yaml) — reproducible app service definition
- [Dependencies](requirements.txt) — local setup requirements

### Project Contributions

This was a **collaborative robotics programming project**. My contributions included identifying design and usability problems in the initial visualizer, evaluating interactive joint behavior and DH frame presentation, and supporting subsequent revisions. The repository documents which issues the current implementation addresses; do not infer sole authorship of every module from this summary.

### Validation Scope

Automated tests check selected analytical cases, but they do not prove correctness for every geometry or every possible parameter combination. The application and tests can be reviewed directly; no unverified screenshots or benchmark figures have been added.

**[Return to engineering portfolio](https://github.com/LezBe/Leziga_Portfolio)**
