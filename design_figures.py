"""Engineering drawings and plots for the D440 preliminary design."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import numpy as np


def draw_design(folder, report):
    from design import ARM, PROP_DIAMETER, components, propeller_fit
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
    blue, green, orange = '#245777', '#31866d', '#b6692d'
    fig = plt.figure(figsize=(14, 9), layout='constrained')
    gs = fig.add_gridspec(2, 2, width_ratios=[1.1, 1])
    top = fig.add_subplot(gs[:, 0])
    side = fig.add_subplot(gs[0, 1])
    notes = fig.add_subplot(gs[1, 1])
    # In the top view horizontal is -body y, vertical is body x.
    for i, (x, y) in enumerate([(220, 0), (0, 220), (-220, 0), (0, -220)]):
        top.plot([0, -y], [0, x], color=blue, lw=12, solid_capstyle='butt', zorder=1)
        top.add_patch(Circle((-y, x), PROP_DIAMETER*500, fc=green, ec=green, alpha=.12))
        top.add_patch(Circle((-y, x), PROP_DIAMETER*500, fill=False, ec=green, lw=1.4))
        top.add_patch(Circle((-y, x), 14.15, color=blue))
        name = ['FRONT', 'LEFT', 'REAR', 'RIGHT'][i]
        spin = 'CCW (+)' if i % 2 == 0 else 'CW (-)'
        top.text(-y, x-45, f'{i+1}  {name}\n{spin}', ha='center', va='top', color=blue,
                 bbox={'facecolor': 'white', 'alpha': .85, 'edgecolor': 'none', 'pad': 2})
    top.add_patch(Rectangle((-60, -60), 120, 120, fc='#e3e9ed', ec=blue, zorder=3))
    top.add_patch(Rectangle((-32, -39), 64, 78, fc=orange, ec=orange, alpha=.25, zorder=4))
    top.plot(0, 0, '+', color='black', ms=12, zorder=5)
    top.text(0, 70, '120 x 120 mm plates', ha='center', fontsize=9)
    top.annotate('', xy=(220, -365), xytext=(-220, -365), arrowprops={'arrowstyle': '<->', 'color': blue})
    top.text(0, -355, '440 mm opposite motor centers', ha='center', color=blue)
    top.annotate('', xy=(365, 334.3), xytext=(365, -334.3), arrowprops={'arrowstyle': '<->', 'color': blue})
    top.text(377, 0, '668.6 mm propeller envelope', rotation=90, va='center', color=blue)
    top.text(-340, 350, '+x forward\n+y left; +z out of page', fontsize=9)
    top.set(xlim=(-360, 415), ylim=(-395, 390), aspect='equal', title='TOP VIEW | 9-inch propellers')
    top.axis('off')
    # Side drawing: actual layout datum at arm center, not simulation ground.
    side.plot([-220, 220], [0, 0], color=blue, lw=9)
    for x in [-220, 220]:
        side.add_patch(Rectangle((x-14.15, 11.75), 28.3, 28.5, color=blue))
        side.plot([x-114.3, x+114.3], [46, 46], color=green, lw=2)
    for z in [-10.85, 10.85]:
        side.add_patch(Rectangle((-60, z-.85), 120, 1.7, color='black'))
    side.add_patch(Rectangle((-39, -58), 78, 43, fc=orange, alpha=.45))
    side.add_patch(Rectangle((-27.15, 15.25), 54.3, 17.5, fc=blue, alpha=.6))
    side.add_patch(Rectangle((-20, -80), 40, 10, fc=green))
    side.add_patch(Rectangle((-25, 72.8), 50, 14.4, fc=green))
    for x in [-100, 100]:
        side.add_patch(Rectangle((x-10, -170), 20, 160, fc=blue, alpha=.5))
    side.axhline(-170, color='#888888', lw=1)
    side.text(160, -165, 'Feet: z = -170 mm', fontsize=8)
    cg_z = report['cg_from_arm_plane_m'][2]*1000
    side.plot(0, cg_z, 'o', color='black', ms=5)
    for label, point, destination in [
        ('M10 on mast', (0, 80), (80, 100)),
        ('Pixhawk', (0, 24), (90, 70)),
        ('GNB battery', (0, -36.5), (130, -45)),
        ('H-Flow faces down', (0, -75), (120, -100)),
    ]:
        side.annotate(label, xy=point, xytext=destination, fontsize=8,
                      arrowprops={'arrowstyle': '-', 'color': '#666666'})
    side.set(xlim=(-345, 345), ylim=(-195, 125), aspect='equal', title='SIDE VIEW | dimensions in mm')
    side.set_xlabel('Body x (mm)'); side.set_ylabel('z from arm plane (mm)')
    notes.axis('off')
    text = (
        f"Calculated design mass: {report['design_mass_kg']:.3f} kg\n"
        f"Includes a 10% distributed mass allowance\n\n"
        f"Adjacent propeller tip gap: {report['adjacent_prop_tip_clearance_m']*1000:.1f} mm\n"
        f"Center-to-motor arm: {ARM*1000:.0f} mm\n"
        "Arms: 4 x 180 mm; carbon tube OD 20 / ID 18 mm\n"
        "Legs: 4 x 160 mm from the same tube stock\n"
        "Battery: 78 x 64 x 43 mm, centered below plates\n"
        f"CG: ({report['cg_from_arm_plane_m'][0]*1000:.1f}, "
        f"{report['cg_from_arm_plane_m'][1]*1000:.1f}, {cg_z:.1f}) mm\n\n"
        "Custom parts: cut plates, clamp blocks, feet and mast.\n"
        "Drawing specifies layout, not manufacturing hole tolerances.\n"
        "Mass distribution uses simplified component envelopes.\n"
        "Ground contact in the flight model remains a point approximation."
    )
    notes.text(.02, .98, text, va='top', linespacing=1.6, fontsize=10)
    fig.suptitle('AE352 D440 | Preliminary component-based design', fontsize=18, color=blue)
    fig.savefig(folder/'drone_layout.png', dpi=180)
    fig.savefig(folder/'drone_layout.pdf')
    plt.close(fig)

    kt, kq, rows = propeller_fit()
    rpm = np.linspace(4000, 9000, 100)
    omega = rpm*2*np.pi/60
    fig, axs = plt.subplots(1, 2, figsize=(11, 4), layout='constrained')
    for ax, col, fit, ylabel in [(axs[0], 2, kt*omega**2, 'Thrust per rotor (N)'),
                                (axs[1], 3, kq*omega**2, 'Aerodynamic torque (N m)')]:
        ax.plot(rows[:, 0], rows[:, col], 'o', label='APC manufacturer simulation data')
        ax.plot(rpm, fit, label='Quadratic fit used in AE352')
        ax.axvline(report['hover_rpm'], color=green, ls='--', label='Design hover')
        ax.axvline(8500, color=orange, ls=':', label='Operational cap')
        ax.set(xlabel='Rotor speed (RPM)', ylabel=ylabel)
        ax.grid(alpha=.25); ax.legend(fontsize=8)
    fig.suptitle('APC 9x4.5MR static-data fit | computed data, not a bench test')
    fig.savefig(folder/'propeller_fit.png', dpi=170)
    fig.savefig(folder/'propeller_fit.pdf')
    plt.close(fig)


def mission_figures(records, mission, touchdown, folder):
    t = np.array([r[0] for r in records]); s = np.array([r[1] for r in records])
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), layout='constrained')
    speed = np.linalg.norm(s[:, 3:6], axis=1)
    axs[0].plot(t, s[:, 2]); axs[0].set_ylabel('Altitude (m)')
    axs[1].plot(t, speed); axs[1].set_ylabel('Speed (m/s)')
    if mission.name == 'circle':
        axs[0].axhline(1, color='k', ls='--')
        axs[1].axhline(.5, color='k', ls='--')
        axs[2].plot(t, np.linalg.norm(s[:, :2], axis=1)-2)
        axs[2].set_ylabel('Radius error (m)')
        for ax in axs: ax.axvline(10, color='grey', ls=':', label='Evaluation starts')
    elif mission.name == 'maneuver':
        axs[2].plot(t, np.rad2deg(s[:, 8])); axs[2].set_ylabel('Yaw (deg)')
        for event in mission.events:
            for ax in axs: ax.axvline(event['end_s'], color='grey', lw=.8, ls=':')
            axs[0].text(event['start_s'], 1.05, event['phase'], fontsize=8, rotation=45)
    else:
        axs[2].plot(t, np.linalg.norm(s[:, :3]-[0, 0, 1], axis=1))
        axs[2].set_ylabel('Position error (m)')
    for ax in axs: ax.grid(alpha=.25); ax.set_xlabel('Time (s)')
    fig.suptitle(f'D440 | {mission.name} requirement evidence')
    fig.savefig(folder/'requirements.png', dpi=170); plt.close(fig)
    if touchdown:
        fig, ax = plt.subplots(figsize=(8, 4), layout='constrained')
        mask = (t >= touchdown['time_s']-3) & (t < touchdown['time_s']-1e-8)
        ax.plot(t[mask], -s[mask, 5], label='Descent speed before contact')
        ax.plot(touchdown['time_s'], touchdown['vertical_speed_m_s'], 'o', label='Interpolated impact speed')
        ax.axhline(.01, color='r', ls='--', label='Assignment limit')
        ax.set(xlabel='Time (s)', ylabel='Downward speed (m/s)', ylim=(0, .012), title='Landing: speed before ground projection')
        ax.grid(alpha=.25); ax.legend()
        fig.savefig(folder/'touchdown.png', dpi=170); plt.close(fig)
