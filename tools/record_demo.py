#!/usr/bin/env python3
"""Records a guided demo of the simulator into an mp4.

    python3 tools/record_demo.py --psim ./psim --out demo.mp4

Starts the app on a virtual X display (Xvfb), drives it with real mouse and
keyboard input (xdotool) and lets the app render a fixed 60 fps timeline into
the video. Captions are shown through the app's --caption-file option.
The coordinates below match the default 1600x900 layout.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from demo_driver import Driver  # noqa: E402

# ---- layout of the 1600x900 window (see src/app/ui.cpp) -----------------------
TOOLS = {"grab": 60, "attract": 170, "repel": 280, "ball": 412, "rope": 522, "blob": 632,
         "box": 742, "polygon": 852, "wall": 962, "erase": 1072}
TOOL_Y = 23
BTN_RUN, BTN_STEP, BTN_EDIT, BTN_RESET = (1277, 100), (1365, 100), (1455, 100), (1542, 100)
SCENE_DROPDOWN = (1410, 194)
TAB_PHYSICS, TAB_PERF, TAB_VIEW = (1292, 423), (1410, 423), (1527, 423)
SLIDER_X0, SLIDER_X1 = 1366, 1512          # slider track inside the panel
TAB_TOP = 447                              # first row of tab content
BROADPHASE_DROPDOWN = (1474, TAB_TOP + 14)
THREADS_SLIDER_Y = TAB_TOP + 36 + 11       # Performance tab
GPU_CHECKBOX = (1244, TAB_TOP + 62 + 11)
COMPARE_BUTTON = (1410, 731)
TEAR_CHECKBOX = (1244, TAB_TOP + 237 + 11)  # Physics tab, Rigid model
LJ_TEMPERATURE_Y = TAB_TOP + 133 + 11       # Physics tab, Lennard-Jones
DENSITY_TOGGLE = (1549, TAB_TOP + 13)       # View tab colour mode
SCENES = {"ball pit": 0, "elastic gas": 1, "mixed sizes": 2, "stress": 3, "cloth": 4, "soft": 5, "boxes": 6,
          "sph": 7, "lj gas": 8, "lj crystal": 9, "galaxy": 10, "collision": 11}
BROADPHASES = {"grid": 0, "hash": 1, "quadtree": 2, "sap": 3, "bvh": 4, "brute": 5}


def slider_x(value, lo, hi):
    return SLIDER_X0 + (SLIDER_X1 - SLIDER_X0) * (value - lo) / (hi - lo)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--psim", default="./psim")
    ap.add_argument("--out", default="demo.mp4")
    ap.add_argument("--work-dir", default="/tmp/psim-demo")
    args = ap.parse_args()

    d = Driver(os.path.abspath(args.psim), os.path.abspath(args.out), scene=SCENES["ball pit"], work_dir=args.work_dir)
    d.pos = (800, 450)

    def tool(name):
        d.click(TOOLS[name], TOOL_Y, glide_frames=14)

    def scene(name):
        d.click(*SCENE_DROPDOWN, glide_frames=18)
        d.wait(8)
        d.click(SCENE_DROPDOWN[0], 222 + 28 * SCENES[name], glide_frames=14)
        d.wait(10)

    def broadphase(name):
        d.click(*BROADPHASE_DROPDOWN, glide_frames=16)
        d.wait(6)
        d.click(BROADPHASE_DROPDOWN[0], BROADPHASE_DROPDOWN[1] + 28 * (BROADPHASES[name] + 1), glide_frames=12)

    # ---------------------------------------------------------------- intro
    d.caption("Particle Simulator\n2,000 balls, uniform-grid broad phase, fixed timestep with 8 substeps")
    d.seconds(3)
    d.caption("Grab tool: drag a ball out of the pile and throw it")
    d.drag([(560, 820), (560, 500), (640, 300), (900, 180)], frames_per_leg=14)
    d.seconds(1.5)
    d.drag([(300, 830), (300, 600), (700, 300)], frames_per_leg=10)
    d.seconds(2)

    d.caption("Attract and Repel pull or push everything inside the circle")
    tool("attract")
    d.glide(500, 760, 14)
    d.press()
    d.glide(500, 500, 40)
    d.glide(800, 450, 40)
    d.seconds(1)
    d.release()
    tool("repel")
    d.glide(400, 800, 20)
    d.press()
    d.glide(900, 800, 70)
    d.release()
    d.seconds(1)

    d.caption("Pause freezes time; Step advances exactly one frame")
    d.click(*BTN_RUN)
    d.seconds(0.6)
    for _ in range(3):
        d.click(*BTN_STEP, glide_frames=8)
        d.seconds(0.4)

    d.caption("EDIT mode: time stays frozen while you build.\nDraw a wall, place boxes and polygons, aim a ball")
    d.click(*BTN_EDIT)
    d.seconds(0.8)
    tool("wall")
    d.drag([(140, 330), (620, 470)], frames_per_leg=28)
    tool("box")
    for x in (220, 300, 380):
        d.click(x, 200, glide_frames=12)
    tool("polygon")
    d.click(470, 230, glide_frames=12)
    d.click(540, 260, glide_frames=12)
    tool("ball")
    d.drag([(1000, 200), (760, 120)], frames_per_leg=30, hold_end=20)
    d.seconds(0.5)
    d.caption("Run again: everything you placed comes to life")
    d.click(*BTN_RUN)
    d.seconds(4)

    # ---------------------------------------------------------------- broad phases
    scene("mixed sizes")
    d.caption("Six interchangeable broad phases (how candidate pairs are found)")
    d.click(*TAB_PERF)
    d.seconds(1.5)
    d.key("g", 4)
    d.caption("Uniform grid: the cell must fit the biggest boulder,\nso thousands of pebbles crowd into each cell")
    d.seconds(3)
    broadphase("quadtree")
    d.caption("Quadtree adapts its cells to where the particles are")
    d.seconds(3)
    broadphase("bvh")
    d.caption("BVH: a tree of tight boxes. Big boulders only enlarge their own branch")
    d.seconds(3)
    d.caption("Compare all six on the same frame: identical pairs, very different work")
    d.click(*COMPARE_BUTTON)
    d.seconds(5)
    d.click(530, 113)  # close the comparison card
    d.key("g", 4)
    broadphase("grid")
    d.seconds(0.5)

    scene("stress")
    d.caption("30,000 particles: the step time scales with CPU threads...")
    d.seconds(2)
    d.click(slider_x(1, 1, 4), THREADS_SLIDER_Y, glide_frames=16)
    d.seconds(2.5)
    d.click(slider_x(4, 1, 4), THREADS_SLIDER_Y, glide_frames=16)
    d.seconds(2)
    d.caption("...and can move to the GPU (OpenGL 4.3 compute shaders)")
    d.click(*GPU_CHECKBOX)
    d.seconds(4)
    d.click(*GPU_CHECKBOX)
    d.seconds(1)

    # ---------------------------------------------------------------- physics
    scene("elastic gas")
    d.click(*TAB_PHYSICS)
    d.caption("Energy check (the original simulator): restitution 1, no gravity.\nThe total energy line stays flat")
    d.seconds(5)

    scene("cloth")
    d.caption("Ropes and cloth are distance constraints (position-based Verlet)")
    d.seconds(1.5)
    d.drag([(1060, 420), (1000, 560), (880, 700)], frames_per_leg=24, hold_end=20)
    d.seconds(1)
    d.caption("Turn on 'Tear links' and pull hard")
    d.click(*TEAR_CHECKBOX)
    d.seconds(0.5)
    d.drag([(1000, 420), (1000, 560), (980, 860)], frames_per_leg=22, hold_end=30)
    d.seconds(2.5)

    scene("soft")
    d.caption("Soft bodies keep their area like pressurised balloons")
    d.seconds(2.5)
    d.drag([(700, 620), (700, 300), (300, 150)], frames_per_leg=14)
    d.seconds(3)

    scene("boxes")
    d.caption("Convex polygons: SAT contacts, sequential impulses, friction.\nThrow a heavy ball at the pyramid")
    d.seconds(1.5)
    d.click(*BTN_EDIT)
    tool("ball")
    d.click(slider_x(30, 2, 40), 333, glide_frames=14)  # spawn radius slider -> 30 px
    d.drag([(520, 720), (260, 690)], frames_per_leg=30, hold_end=10)
    d.click(*BTN_RUN)
    d.seconds(4)

    scene("sph")
    d.caption("SPH fluid (double-density relaxation) with floating boxes")
    d.seconds(2.5)
    d.click(*TAB_VIEW)
    d.click(*DENSITY_TOGGLE, glide_frames=12)
    d.caption("Coloured by density: blue sparse, white at rest density, red compressed")
    d.seconds(3)
    tool("repel")
    d.glide(300, 820, 20)
    d.press()
    d.glide(600, 800, 50)
    d.release()
    d.seconds(2.5)

    scene("lj gas")
    d.click(*TAB_PHYSICS)
    d.caption("Lennard-Jones molecules. The speed histogram follows Maxwell-Boltzmann")
    d.seconds(3)
    d.caption("Cool below T* = 0.45 and droplets condense out of the gas")
    d.click(slider_x(0.25, 0.05, 3.0), LJ_TEMPERATURE_Y, glide_frames=20)
    d.seconds(7)

    scene("collision")
    d.caption("N-body gravity: two galaxies colliding (Barnes-Hut, theta = 0.6)")
    d.seconds(3)
    d.key("g", 4)
    d.caption("The Barnes-Hut quadtree: far-away cells act as a single mass")
    d.seconds(3)
    d.key("g", 4)
    d.click(*TAB_PERF)
    d.caption("Gravity interactions per step vs the n(n-1) a direct sum would need")
    d.seconds(5)

    d.caption("H shows every state, key and tool")
    d.key("h", 4)
    d.seconds(4)
    d.key("h", 4)
    d.caption("")
    d.seconds(1)
    d.finish()


if __name__ == "__main__":
    main()
