# DH Robot Visualizer

Interactive 3-link Denavit-Hartenberg robot visualizer built with Python, Dash, Plotly, and NumPy.

## Run locally

```bash
pip install -r requirements.txt
python dh_web.py
```

Then open:

```text
http://127.0.0.1:8050/
```

## Deploy on Render

This repository includes a `render.yaml` file for deployment.

1. Sign in to Render.
2. Choose **New +** → **Blueprint**.
3. Connect this GitHub repository.
4. Render should detect `render.yaml`.
5. Create the service.

The app is served with Gunicorn using:

```bash
gunicorn dh_web:server
```

Once deployment finishes, Render will provide a public `onrender.com` URL.

## Assignment note

This application is the AI-generated conversion being evaluated for Programming Assignment 1. Known faults identified in the report should not be silently corrected unless the assignment requires a revised implementation.
