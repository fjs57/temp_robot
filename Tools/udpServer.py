import socket
import struct
from matplotlib import pyplot as plt
import numpy as np

MCAST_GRP = '224.0.0.80'
MCAST_PORT = 4003
IS_ALL_GROUPS = True

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
if IS_ALL_GROUPS:
    # on this port, receives ALL multicast groups
    sock.bind(('', MCAST_PORT))
else:
    # on this port, listen ONLY to MCAST_GRP
    sock.bind((MCAST_GRP, MCAST_PORT))
mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)

sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)



terrain_size_x = 1.8
terrain_size_y = 1.2

x_max = terrain_size_x / 2
x_min = -x_max

y_max = terrain_size_y / 2
y_min = -y_max

plt.plot(
    [x_max, x_max, x_min, x_min, x_max],
    [y_min, y_max, y_max, y_min, y_min],
    "k-"
)

while True:
    # For Python 3, change next line to "print(sock.recv(10240))"
    data = sock.recv(16384)
    #print(data)


    data = data.decode("ascii")

    data = data.split(";")

    plt.clf()

    # if data[0]=='O':
    #     for o in data[1:] :
    #         a = o.split(',')
    #         x, y = float(a[0]), float(a[1])
    #         plt.plot(x, y, "b+")
    # elif data[0]=='P':
    #     for o in data[1:] :
    #         a = o.split(',')
    #         x, y = float(a[0]), float(a[1])
    #         plt.plot(x, y, "g.")

    state = 0

    u = data[0].split(',')
    x, y, rot = float(u[0]), float(u[1]), float(u[2])
    plt.plot(x, y, "rx", markersize=30)
    scale = 0.2
    vx = np.cos(rot) * scale
    vy = np.sin(rot) * scale
    plt.quiver(x, y, vx, vy, color="r", width=0.002)

    path_x = list()
    path_y = list()

    for o in data[1:] :
        if state==0 :
            if o == 'P':
                state = 1
                continue
            a = o.split(',')
            x, y = float(a[0]), float(a[1])
            plt.plot(x, y, "b+", markersize=20)
            circle = plt.Circle((x,y), 0.2, edgecolor="k", linestyle=":", facecolor="w", zorder=-1)
            plt.gca().add_patch(circle)
        elif state==1 :
            if o == 'Q':
                state = 2
                continue
            a = o.split(',')
            x, y = float(a[0]), float(a[1])
            plt.plot(x, y, "g.", markersize=2)
        else :
            a = o.split(',')
            x, y = float(a[0]), float(a[1])
            path_x.append(x)
            path_y.append(y)
    
    plt.plot(path_x, path_y, "m--")
    plt.plot(path_x, path_y, "mo")
    


    plt.plot(
        [x_max, x_max, x_min, x_min, x_max],
        [y_min, y_max, y_max, y_min, y_min],
        "k-"
    )
    plt.grid(True)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.pause(0.01)


