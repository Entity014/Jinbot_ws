import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, Slider


def direction(a=0, b=0, c=0):
    if a * b >= 0:
        return (c, ((a + b) / 2))
    else:
        return (0, 0)


# a = -b, c = b , c != 0
def rotation(a=0, b=0, c=0):
    if a * b <= 0 and b * c > 0:
        return (((a - b - c) / 3), 0)
    else:
        return (0, 0)


# Create a figure and axis
fig, ax = plt.subplots()
fig.subplots_adjust(bottom=0.25)

lim_speed = 2
vector1 = (0, 0)
vector2 = (0, 0)

quiver1 = ax.quiver(
    0,
    0,
    vector1[0],
    vector1[1],
    angles="xy",
    scale_units="xy",
    scale=1,
    color="red",
    label="Vector 1",
)
quiver2 = ax.quiver(
    0,
    0,
    vector2[0],
    vector2[1],
    angles="xy",
    scale_units="xy",
    scale=1,
    color="blue",
    label="Vector 2",
)

# Add labels
ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")

# Add a legend
ax.legend()

# Create slider axes
axcolor = "lightgoldenrodyellow"
ax_a = plt.axes([0.15, 0.11, 0.65, 0.03], facecolor=axcolor)
ax_b = plt.axes([0.15, 0.06, 0.65, 0.03], facecolor=axcolor)
ax_c = plt.axes([0.15, 0.01, 0.65, 0.03], facecolor=axcolor)

# Create sliders
slider_a = Slider(ax_a, "encode 1", -lim_speed, lim_speed, valinit=0)
slider_b = Slider(ax_b, "encode 2", -lim_speed, lim_speed, valinit=0)
slider_c = Slider(ax_c, "encode 3", -lim_speed, lim_speed, valinit=0)


def update(val):
    a = slider_a.val
    b = slider_b.val
    c = slider_c.val

    vector1 = direction(a, b, c)
    vector2 = rotation(a, b, c)

    # Set the axis limits based on vector lengths
    max_vector = abs(max(max(vector1, key=abs), max(vector2, key=abs), key=abs))
    ax.set_xlim(-max_vector - 1, max_vector + 1)
    ax.set_ylim(-max_vector - 1, max_vector + 1)

    quiver1.set_UVC(vector1[0], vector1[1])
    quiver2.set_UVC(vector2[0], vector2[1])

    fig.canvas.draw_idle()


slider_a.on_changed(update)
slider_b.on_changed(update)
slider_c.on_changed(update)

resetax = fig.add_axes([0.85, 0.15, 0.1, 0.04])
button = Button(resetax, "Reset", hovercolor="0.975")


def reset(event):
    slider_a.reset()
    slider_b.reset()
    slider_c.reset()


button.on_clicked(reset)

plt.grid()
plt.show()
