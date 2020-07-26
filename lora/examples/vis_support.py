
import matplotlib
matplotlib.use('Agg')
#matplotlib.use('GTK3Agg')
#matplotlib.use('Qt5Agg')


from matplotlib.backends.backend_pdf import PdfPages
import csv
from collections import defaultdict
import numpy as np
import matplotlib.pyplot as plt
from cycler import cycler
import dill as pickle
#import pickle
import os.path
from os import path
from contextlib import contextmanager
import re
import math
import sys


def fixPacketName(name_in):
    # Correct case where the key has leading and or trailing spaces
    name = name_in.replace(" ", "")
    if(name == "Measurement"):
        return "DataPacket"
    elif(name=="DestinationReceipt"):
        return "EndReceipt"
    elif(name=="NodeReceipt"):
        return "NodeReceipt"
    return name # Nothing else changes

def getSummary(file):
    with open(file, 'r') as csvfile:
        df = csv.reader(csvfile, delimiter=',')

        for row in df:
            title = row[0]
            vals = list(map(float, row[1:]))
            num_runs = int(vals[0])
            num_nodes = int(vals[1])
            sim_duration = int(vals[2])
            mesh_size = int(vals[3])
            mesh_type = int(vals[4])
            period = int(vals[5])
            receipt_delay = int(vals[6])
            packet_delay = int(vals[7])
            bq_mult = vals[8]
            use_thorp = vals[9]
            recur_time = int(vals[10])
            corrupt_prob = vals[11]
    return {'title': title, 'num_runs': num_runs, 'num_nodes': num_nodes, 'sim_duration': sim_duration, 'mesh_size': mesh_size, 'mesh_type': mesh_type, 'period': period, 'receipt_delay': receipt_delay, 'packet_delay': packet_delay, 'bq_mult': bq_mult, 'use_thorp': use_thorp, 'recur_time': recur_time, 'corrupt_prob': corrupt_prob}

def load_sum():
    sum = getSummary('Exp2Summary.csv')
    return sum

def load_node_locs(run_no):
    file = f"Exp2NodeLocs_{run_no}.csv"

    nodeNum = []
    x = []
    y = []
    z = []

    with open(file, 'r') as csvfile:
        df = csv.reader(csvfile, delimiter=',')

        for row in df:
            row = list(map(float, row))
            nodeNum.append(int(row[0]))
            x.append(row[1])
            y.append(row[2])
            z.append(row[3])
    
    return nodeNum, x, y, z

def load_node_locs_cwd():
    '''
    Reads the node locs in current directory. Returns an array holding the locs
    one for each run in the directory.
    '''
    sum = load_sum()

    locs = []

    for run_num in range(sum['num_runs']):
        locs.append(load_node_locs(run_num))

    #print(len(locs))
    #print(locs[0])
    return locs

def analyz_node_locs_all_sizes(root_path):
    node_sizes = [2, 3, 4, 8, 16, 32, 64]
    mesh_sizes = [100, 1000, 10000, 100000, 250000, 500000]
    for num_nodes in node_sizes:
        for ms in mesh_sizes:
            analyze_node_locs(root_path, num_nodes, ms)

def how_different(lhs, rhs):
    #print(lhs)
    #print(rhs)
    l1 = len(lhs)
    l2 = len(rhs)
    if l1 != l2:
        print(f"Lengths: {l1} != {l2}")
    if l1 != 20:
        print(f"Lengths: {l1} != 20")
    for i in range(len(lhs)):
        for j in range(len(rhs)):
            if lhs[i] != lhs[j]:               
                print(f"lhs[{i}] != lhs[{j}]")
                print(lhs[i])
                print(lhs[j])
                return


def analyze_node_locs(root_path, num_nodes, mesh_size):
    all_locs = []
    dirs = []

    entries = sorted_subdirs(root_path)
    num_entries = len(entries)

    count = 0
    for entry in entries:
        if(entry.name in exclude):
            continue
        if entry.is_dir():
            count += 1
            progress(f"reading {num_nodes}-{mesh_size}", round(count/num_entries*100,2))
            subdir = os.path.join(root_path, entry)
            with cwd(subdir):
                sum = load_sum()
                if(sum['num_nodes'] == num_nodes and sum['mesh_size'] == mesh_size):
                    locs = load_node_locs_cwd()
                    all_locs.append(locs)
                    dirs.append(subdir)
                
    l = len(all_locs)

    print(f"Analyzing {l} node sets")
    for i in range(0, l):
        progress("working...", round((i/l)*100,2))
        if i < l:
            for j in range(i + 1, l):
                if not all_locs[i] == all_locs[j]:
                    print(" ")
                    print(f"Fail: {dirs[i]}, {dirs[j]}")
                    how_different(all_locs[i], all_locs[j])
                    return
    print("All locs the same")
    return


def extract_delivery(rows):
    types = []
    delivered = []
    for row in rows:
        type = row[0]
        types.append(type)
        tr = list(map(float, row[1:]))
        p = 100.0 * tr[0]/tr[1]  # round(100.0*tr[0]/tr[1])
        delivered.append((tr[0], tr[1], p))
    return dict(zip(types, delivered))


def extract_corrupt(rows):
    # Two columns corrupted index, Number of detections
    # What we want is a bar graph, number corrupted, number detected
    num_corrupted = 0
    num_detected = 0
    for row in rows:
        # idx = int(row[0])
        num_detect = int(row[1])
        num_corrupted = num_corrupted + 1
        if(num_detect > 0):
            num_detected = num_detected + 1
    return num_corrupted, num_detected


def extract_seen(rows):
    # node # and num seen at node
    # we want total
    num_seen = 0
    for row in rows:
        # idx = int(row[0])
        num_seen = num_seen + int(row[1])

    return num_seen


def extract_fail_map(rows):
    failMap = {}
    for row in rows:
        idx = int(row[0])
        numfail = int(row[1])
        failMap[idx] = numfail
    return failMap


def extract_q_map(rows):
    qmap = defaultdict(list)
    for row in rows:
        idx = int(row[0])
        size_max = list(map(int, row[1:]))
        qmap[idx] = size_max
    return qmap


def total_q_map(qmap):
    # want total still in Q
    # and max in Q
    tot = 0
    maxQ = 0
    for key in qmap:
        tot = tot + qmap[key][0]
        maxQ = max(maxQ, qmap[key][1])
    return tot, maxQ


def extract_route(rows):
    rmap = defaultdict(list)
    for row in rows:
        idx = int(row[0])
        command = row[1]
        route = list(map(int, row[2:]))
        rmap[idx] = (command, route)
    return rmap


def extract_time(rows):
    tmap = defaultdict(list)
    for row in rows:
        time = float(row[0])
        pkt_type = row[1]
        tmap[pkt_type].append(time)

    return tmap

def extract_packet_round_trip(rows):
    # Store as a list
    rt = []
    for row in rows:
        r = {'pkt_id': row[0], 'dst_id': row[1], 'pkt_type': row[2], 'sent': int(row[3]), 'received': int(row[4]), 'rcpt_received': int(row[5]), 'sent_at': float(row[6]), 'received_at': float(row[7]), 'receipt_received_at': float(row[8]), 'receive_count': int(row[9]), 'dest_count': int(row[10]), 'forward_count': int(row[11])}
        rt.append(r)
    return rt


def categorizePacketSuccess(file):
    cat_rows = defaultdict(list)
    try:
        with open(file, 'r') as csvfile:
            df = csv.reader(csvfile, delimiter=',')

            # categorize rows
            mode = None

            for row in df:
                if row[0] == "Delivery":
                    mode = "Delivery"
                elif row[0] == "SendFail":
                    mode = "SendFail"
                elif row[0] == "ReceiveFail":
                    mode = "ReceiveFail"
                elif row[0] == "TQ":
                    mode = "TQ"
                elif row[0] == "BQ":
                    mode = "BQ"
                elif row[0] == "Route":
                    mode = "Route"
                elif row[0] == "Corrupt":
                    mode = "Corrupt"
                elif row[0] == "Seen":
                    mode = "Seen"
                elif row[0] == "SendTime":
                    mode = "SendTime"
                elif row[0] == "ReceiveTime":
                    mode = "ReceiveTime"
                elif row[0] == "PacketTrace":
                    mode = "PacketTrace"
                else:
                    cat_rows[mode].append(row)

    except FileNotFoundError:
        print(f"{file} Not found, skipping.")
    return cat_rows


def getPacketSuccess(file):
    rows = categorizePacketSuccess(file)
    return extract_delivery(rows["Delivery"])


def titleLine1(sum):
    if sum['use_thorp']:
        model = 'Thorp'
    else:
        model = 'Ideal'

    # return f"{sum['title']} Model: {model}"
    return f"{model} Model,  On: {sum['period']}  R: {sum['receipt_delay']}  P: {sum['packet_delay']} BQ: {sum['period']*sum['bq_mult']}"


def titleLine2(sum):
    mesh_type = "Random"
    if(sum['mesh_type'] == 1):
        mesh_type = "Linear"
    return f"Nodes: {sum['num_nodes']} {mesh_type}: {sum['mesh_size']}m  Send Period: {sum['recur_time']}  Duration: {sum['sim_duration']}"


def computeStats(pmap):
    # collect our array of dictionaries into
    # a dictionary of tuples
    d = defaultdict(list)
    for ps in pmap:
        for key in ps:
            d[key].append(ps[key])

    # compute average for the 3rd column of tuple
    p_recv = dict()
    avg = dict()
    num_sent = dict()
    for key in d:
        stats = np.asarray(d[key], dtype=float)
        p = stats[:, 2]  # 3rd column
        p_recv[key] = p
        n = stats[:, 0]  # first column
        avg[key] = np.mean(p)
        num_sent[key] = n

    return d, p_recv, avg, num_sent


def computeCorruptStats(corrupt_map):
    corrupted = []
    detected = []

    for c in corrupt_map:
        corrupted.append(c[0])
        detected.append(c[1])

    return corrupted, detected


def recievedTitle(sum, avg):
    t1 = titleLine1(sum)
    t2 = titleLine2(sum)
    t3 = "Delivered "
    for key in avg:
        t3 = t3 + f"{fixPacketName(key)}: {round(avg[key],1)}% "

    return f"{t1}\n{t2}\n{t3}"


def save_fig(plot, fname, pp, image_type):
    #print(f"Saving: {fname}")
    progress_tag(f"Saving: {fname}")

    if(pp == None):
        if(image_type == 'png'):
            plot.savefig(fname, dpi=300)
        else:
            plot.savefig(fname)
    else:
        plot.savefig(pp, format='pdf')

    plot.close()


def config_font_tex(plot):
    plot.rc('text', usetex=True)
    plot.rc('font', family='serif')

def config_font(plot):
    plot.rc('text', usetex=False)
    plot.rc('font', family='serif')


def scatterNodes(sum, run_no, graph_type='failure_paths', packet_type=None, pp=None, image_type='pdf'):

    file = f"Exp2NodeLocs_{run_no}.csv"
    psname = f"Exp2PacketStats_{run_no}.csv"

    rows = categorizePacketSuccess(psname)

    pmap = extract_delivery(rows["Delivery"])
    #send_fail = extractFailMap(rows["SendFail"])
    #receive_fail = extractFailMap(rows["ReceiveFail"])
    #tq = extractQMap(rows["TQ"])
    #bq = extractQMap(rows["BQ"])
    route = extract_route(rows["Route"])

    nodeNum = []
    x = []
    y = []
    z = []

    try:
        with open(file, 'r') as csvfile:
            df = csv.reader(csvfile, delimiter=',')

            for row in df:
                row = list(map(float, row))
                nodeNum.append(int(row[0]))
                x.append(row[1])
                y.append(row[2])
                z.append(row[3])
    except FileNotFoundError:
        print(f"{file} Not found, skipping.")
        return

    fig = plt.figure()

    config_font(plt)

    plt.ioff()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(x[0], y[0], z[0], c='b', marker='o', s=48)
    # x = np.delete(x, 0, 0)
    # y = np.delete(y, 0, 0)
    # z = np.delete(z, 0, 0)
    ax.scatter(x, y, z, c='g', marker='o')
    loff = 2

    for i in range(len(x)):
        ax.scatter(x[i], y[i], z[i]-loff, c='k', marker=f'${i}$', s=50)

    # for i in send_fail:
    #     n = send_fail[i]
    #     if n > 0:
    #         ax.scatter(x[i], y[i], z[i]+loff, c='k', marker=f'$s{n}$', s=50)

    # for i in receive_fail:
    #     n = receive_fail[i]
    #     if n > 0:
    #         ax.scatter(x[i], y[i], z[i] - loff, c='k', marker=f'$r{n}$', s=20)

    # for i in tq:
    #     sm = tq[i]
    #     ax.scatter(x[i], y[i], z[i] + loff, c='k',
    #                marker=f'${sm[0]}$', s=20)

    # for i in bq:
    #     sm = bq[i]
    #     ax.scatter(x[i], y[i], z[i] - loff, c='k',
    #                marker=f'${sm[0]}$', s=20)

    # cmap = []
    # for i in np.arange(0, 1, 1 / 100):
    #    cmap.append((i, i, i, 0.3))

    # cmap = matplotlib.cm.get_cmap('Spectral')
    cmap = matplotlib.cm.get_cmap('jet')

    lws = {'Measurement': 1, 'DestinationReceipt': 2}
    alphas = {'Measurement': 1., 'DestinationReceipt': 1}
    zs = {'Measurement': 0.1, 'DestinationReceipt': 0.2}

    show_only_failures = False
    if(graph_type == 'failure_paths'):
        show_only_failures = True

    show_only_nodes = False
    if(graph_type == 'nodes'):
        show_only_nodes = True

    for i in route:
        r = route[i]
        x0 = x[r[1][0]]
        y0 = y[r[1][0]]
        z0 = z[r[1][0]]
        if not show_only_nodes:
            if (len(r) > 0):
                graph_type = r[0]
                if packet_type == graph_type or packet_type == None:
                    rtids = r[1]
                    ls = '-'
                    if graph_type == 'Measurement' and rtids[-1] == -1:
                        ls = ':'
                    elif graph_type == 'DestinationReceipt' and rtids[-1] == -1:
                        ls = ':'

                    if(not show_only_failures or ls != '-'):
                        for j in range(len(rtids)):
                            x1 = x[rtids[j]]
                            y1 = y[rtids[j]]
                            z1 = z[rtids[j]]
                            ax.plot([x0, x1], [y0, y1], [z0, z1],
                                    lw=lws[graph_type], alpha=alphas[graph_type], zorder=zs[graph_type], linestyle=ls, c=cmap(i/len(route)))
                            x0 = x1
                            y0 = y1
                            z0 = z1

    ax.set_xlabel('X meters')
    ax.set_ylabel('Y meters')
    ax.set_zlabel('Height meters')

    if show_only_nodes:
        packet_title = 'Nodes'
    else:
        if packet_type == None:
            packet_title = 'Combined'
        else:
            packet_title =  fixPacketName(packet_type)

    fname = f"{sum['title']}_{run_no}_{packet_title}_scatter.{image_type}"
    fname = fname.replace(" ", "")

    if(show_only_failures):
        t0 = f"Run {run_no} {packet_title} Communication Failure Paths"
    elif show_only_nodes:
        t0 = f"Node Locations"
    else:
        t0 = f"Run {run_no} {packet_title} Communication Paths"
        t1 = titleLine1(sum)
    if show_only_nodes:
        t2 = f"{sum['num_nodes']} Nodes Mesh Size: {sum['mesh_size']}"
    else:
        t2 = f"{sum['num_nodes']} Nodes ({sum['mesh_size']}x{sum['mesh_size']}) {sum['sim_duration']}secs"
        t3 = "Delivered "
        for key in pmap:
            t3 = t3 + f"{fixPacketName(key)}: {round(pmap[key][2],1)}% "

    if show_only_nodes:
        plt.title(f"{t0}\n{t2}")
    else:
        plt.title(f"{t0}\n{t1}\n{t2}\n{t3}")

    save_fig(plt, fname, pp, image_type)


def makeTimeScatterPlot(sum, title, times, pp=None, image_type='png'):
    plt.ioff()
    config_font(plt)
    _, ax1 = plt.subplots()

    idx = 1

    max_y = 0
    midx = 0
    markers = ['1', '2', '3']

    x = defaultdict(list)
    y = defaultdict(list)
    for st_run in times:
        off = -0.2
        for key in st_run:
            yy = st_run[key]
            max_y = max(max_y, max(yy))
            xx = [idx + off]*len(yy)
            x[key].extend(xx)
            y[key].extend(yy)
            off = off + 0.2

        idx = idx + 1

    marker_key = {}
    for key in x:
        marker_key[key] = markers[midx]
        midx += 1

    xgrid = np.arange(1, sum['num_runs']+1, 1)
    ax1.set_xticks(xgrid)

    ax1.set_axisbelow(True)
    ax1.set_ylim([0, max_y])

    total_pkt = 0

    for key in x:
        ax1.scatter(x[key], y[key],
                    label=f"{fixPacketName(key)}", marker=marker_key[key])
        total_pkt += len(y[key])

    ax1.set_ylabel("Time (Seconds)")
    ax1.set_xlabel("Test Run")

    # And a corresponding grid
    ax1.grid(which='both')

    fname = f"{title}.{image_type}"
    fname = fname.replace(" ", "")

    t1 = titleLine1(sum)
    t2 = titleLine2(sum)
    plt.title(f"{title}  {total_pkt/sum['num_runs']} Avg Per Run\n {t1}\n{t2}")

    # Create legend & Show graphic
    ax1.legend(loc='best')

    plt.tight_layout()

    save_fig(plt, fname, pp, image_type)


def makeCombinedReceivedPlot(sum, pmap, pp=None, image_type='pdf'):

    # sum = getSummary('Exp2Summary.csv')

    # collect our array of dictionaries into
    # a dictionary of tuples
    # d = defaultdict(list)
    # for ps in pmap:
    #    for key in ps:
    #        d[key].append(ps[key])

    # compute average for the 3rd column of tuple
    # p_recv = dict()
    # avg = dict()
    # num_sent = dict()
    # for key in d:
    #    stats = np.asarray(d[key], dtype=float)
    #    p = stats[:, 2]  # 3rd column
    #    p_recv[key] = p
    #    n = stats[:, 0]  # first column
    #    avg[key] = np.mean(p)
    #    num_sent[key] = n
    d, p_recv, avg, num_sent = computeStats(pmap)
    #print(p_recv)

    # set width of bar
    barWidth = 0.25

    use_black = False

    if use_black:
        black_line_cycler = (cycler(color=['k', 'k', 'k', 'k']) +
                             cycler(lw=[1, 2, 3, 4]))
        # hatch_cycler = (cycler(hatch=['/', '\\', '*', 'o', 'O', '.']))
        line_cycler = black_line_cycler
        # fill_cycler = hatch_cycler
    else:
        # line_cycler = (cycler(color=['r', 'g', 'b', 'y']))
        line_cycler = cycler('color', ['#006BA4', '#FF800E', '#ABABAB', '#595959',
                                       '#5F9ED1', '#C85200', '#898989', '#A2C8EC', '#FFBC79', '#CFCFCF'])
        # line_cycler = (cycler(color=['r', 'g', 'b', 'y'])
        #               ) + cycler(cycler(lw=[1, 2, 3, 4]))

    # Set position of bar on X axis
    r = dict()
    off = 1.0
    for key in d:
        r[key] = np.arange(len(d[key])) + off
        off = off + barWidth

    # Make the plot
    plt.ioff()
    config_font(plt)
    _, ax1 = plt.subplots()

    ax2 = ax1.twinx()
    # ax1.set_prop_cycle(hatch_cycler)
    if(use_black):
        ax2.set_prop_cycle(line_cycler)

    for key in r:
        ax2.plot(r[key], num_sent[key], label=f"# sent {fixPacketName(key)}")

    if use_black:
        h = '///'
    else:
        h = None

    for key in r:
        if(use_black):
            ax1.bar(r[key], p_recv[key], color='white',
                    width=barWidth, edgecolor='black', hatch=h, label=f"%received {fixPacketName(key)}")
            # ax1.plot(r[key], p_recv[key], label=key)
            h = 'XXX'
        else:
            ax1.bar(r[key], p_recv[key], width=barWidth,
                    label=f"%received {fixPacketName(key)}")

    ax1.set_ylabel('% Packets Received')
    ax1.set_xlabel("Test Run")
    xtics = np.arange(1, sum['num_runs']+1, 1)
    ax1.set_xticks(xtics)

    ax2.set_ylabel("Packets sent")
    ax2.set_xlabel("Test Run")
    ax2.set_xticks(xtics)

    fname = f"{sum['title']}_combined_received.{image_type}"
    fname = fname.replace(" ", "")

    plt.title("Packet Delivery\n" + recievedTitle(sum, avg))

    # Create legend & Show graphic
    ax1.legend(loc=6)
    ax2.legend(loc=7)

    plt.tight_layout()

    save_fig(plt, fname, pp, image_type)


def makeReceivedBarPlot(sum, pmap, pp=None, image_type='pdf'):
    d, p_recv, avg, _ = computeStats(pmap)

    # set width of bar
    barWidth = 0.25

    # Set position of bar on X axis
    r = dict()
    off = 1.0
    for key in d:
        r[key] = np.arange(len(d[key])) + off
        off = off + barWidth

    # Make the plot
    plt.ioff()
    config_font(plt)
    _, ax1 = plt.subplots()

    # ax2 = ax1.twinx()

    for key in r:
        ax1.bar(r[key], p_recv[key], width=barWidth, label=f"{fixPacketName(key)}")

    ax1.set_ylabel('% Packets Received')
    ax1.set_xlabel("Test Run")
    xtics = np.arange(1, sum['num_runs']+1, 1)
    ax1.set_xticks(xtics)

    fname = f"{sum['title']}_received_bar.{image_type}"
    fname = fname.replace(" ", "")

    plt.title("Packet Delivery\n" + recievedTitle(sum, avg))

    # Create legend & Show graphic
    ax1.legend(loc='lower left')

    plt.tight_layout()

    save_fig(plt, fname, pp, image_type)


def makeReceivedCountGraph(sum, pmap, pp=None, image_type='pdf'):

    _, _, avg, num_sent = computeStats(pmap)

    # Make the plot
    plt.ioff()
    config_font(plt)
    _, ax1 = plt.subplots()

    r = np.arange(1, sum['num_runs']+1, 1)

    for key in num_sent:
        ax1.plot(r, num_sent[key], label=f"{fixPacketName(key)}")

    ax1.set_ylabel("Packets sent")
    ax1.set_xlabel("Test Run")
    ax1.set_xticks(r)

    fname = f"{sum['title']}_received_count.{image_type}"
    fname = fname.replace(" ", "")

    plt.title("Packets Sent\n" + recievedTitle(sum, avg))

    # Create legend & Show graphic
    ax1.legend(loc='best')

    plt.tight_layout()

    save_fig(plt, fname, pp, image_type)


def makeCorruptBarPlot(sum, corrupt_map, pp=None, image_type='pdf'):
    corrupted, detected = computeCorruptStats(corrupt_map)

    # set width of bar
    barWidth = 0.25

    r_corrupted = np.arange(1, sum['num_runs']+1, 1)
   # r_corrupted = np.arange(len(corrupted))
    r_detected = r_corrupted + barWidth

    # Make the plot
    plt.ioff()
    config_font(plt)
    _, ax1 = plt.subplots()

    ax1.bar(r_corrupted, corrupted, width=barWidth, label="Num Corrupted")
    ax1.bar(r_detected, detected, width=barWidth, label="Num Detected")

    ax1.set_ylabel('Number of Packets')
    ax1.set_xlabel("Test Run")
    ax1.set_xticks(r_corrupted)

    fname = f"{sum['title']}_corrupt.{image_type}"
    fname = fname.replace(" ", "")

    t1 = titleLine1(sum)
    t2 = titleLine2(sum)
    plt.title(
        f"Artificial Packet Corruption\nProbabilty: {sum['corrupt_prob']} {t1}\n{t2}")

    # Create legend & Show graphic
    ax1.legend(loc='best')

    plt.tight_layout()

    save_fig(plt, fname, pp, image_type)


def makeQueueSeenPlot(sum, seen_map, tq_map, bq_map, pp=None, image_type='pdf'):
    # Make the plot
    plt.ioff()
    config_font(plt)
    _, ax1 = plt.subplots()

    num_run = len(seen_map)
    # r = np.arange(num_run)
    r = np.arange(1, sum['num_runs']+1, 1)
    ax1.set_xticks(r)

    for i in range(num_run):
        seen_map[i] = seen_map[i]/sum['num_nodes']

    tq0 = []
    tq1 = []
    for tq in tq_map:
        tq0.append(tq[0])
        tq1.append(tq[1])

    bq0 = []
    bq1 = []
    for bq in bq_map:
        bq0.append(bq[0])
        bq1.append(bq[1])

    ax1.plot(r, seen_map, label=f"Avg Seen Per Node")
    ax1.plot(r, tq0, label=f"TQ Remaining")
    ax1.plot(r, tq1, label=f"TQ Max")
    ax1.plot(r, bq0, label=f"BQ Remaining")
    ax1.plot(r, bq1, label=f"BQ Max")

    ax1.set_ylabel("Packets")
    ax1.set_xlabel("Test Run")

    fname = f"{sum['title']}_queues.{image_type}"
    fname = fname.replace(" ", "")

    t1 = titleLine1(sum)
    t2 = titleLine2(sum)
    plt.title(f"Overall Packet Container Sizes\n {t1}\n{t2}")

    # Create legend & Show graphic
    ax1.legend(loc='best')

    plt.tight_layout()

    save_fig(plt, fname, pp, image_type)


def should_pickle():
    return not path.exists("pmap.pickle")


def pickle_stats():
    sum = getSummary('Exp2Summary.csv')
    corrupt_map = []
    pmap = []
    seen_map = []
    tq_map = []
    bq_map = []
    send_time = []
    receive_time = []
    packet_round_trip = []
    for idx in range(0, sum['num_runs']):
        fname = (f'Exp2PacketStats_{idx}.csv')
        rows = categorizePacketSuccess(fname)
        pmap.append(extract_delivery(rows["Delivery"]))
        corrupt_map.append(extract_corrupt(rows["Corrupt"]))
        seen_map.append(extract_seen(rows["Seen"]))
        tq_map.append(total_q_map(extract_q_map(rows["TQ"])))
        bq_map.append(total_q_map(extract_q_map(rows["BQ"])))
        send_time.append(extract_time(rows["SendTime"]))
        receive_time.append(extract_time(rows["ReceiveTime"]))
        packet_round_trip.append(extract_packet_round_trip(rows["PacketTrace"]))

    with open("pmap.pickle", "wb") as f:
        pickle.dump(pmap, f)
    with open("corrupt_map.pickle", "wb") as f:
        pickle.dump(corrupt_map, f)
    with open("seen_map.pickle", "wb") as f:
        pickle.dump(seen_map, f)
    with open("tq_map.pickle", "wb") as f:
        pickle.dump(tq_map, f)
    with open("bq_map.pickle", "wb") as f:
        pickle.dump(bq_map, f)
    with open("send_time.pickle", "wb") as f:
        pickle.dump(send_time, f)
    with open("receive_time.pickle", "wb") as f:
        pickle.dump(receive_time, f)
    with open("packet_round_trip.pickle", "wb") as f:
        pickle.dump(packet_round_trip, f)

def packet_map_default():
    # Work around Python idiocy 
    return defaultdict(lambda: defaultdict(lambda: defaultdict(lambda: defaultdict(lambda: defaultdict(lambda: defaultdict(lambda: defaultdict(lambda: defaultdict(list))))))))

def should_cross_pickle():
    fname = "cross.pickle"
    return not path.exists(fname)

def cross_pickle(root_path):
    fname = "cross.pickle"
    if should_cross_pickle():
        #print(f"cross_pickle: {root_path}")
        #            [type]              [num_nodes]         [mesh_size]         [use_thorp]         [period]            [receipt_delay]    [packet_delay]       [bq_mult]
        packet_map = packet_map_default()

        count = 0

        entries = sorted_subdirs(root_path)
        num_entries = len(entries)

        for entry in entries:
            count += 1
            if(count % 10 == 0):
                progress("working...", round(count/num_entries*100,2))
            if(entry.name in exclude):
                #print(f"Skipping: {entry.name}")
                continue

            if entry.is_dir():
                subdir = os.path.join(root_path, entry)
                with cwd(subdir):
                    collect_packets_in_net(packet_map)

        with open(fname, "wb") as f:
            pickle.dump(packet_map, f)

def unpickle_cross_pickle():
    fname = "cross.pickle"
    with open(fname, "rb") as f:
        packet_map = pickle.load(f)
    return packet_map

def unpickle_pmap():
    with open("pmap.pickle", "rb") as f:
        pmap = pickle.load(f)
    return pmap


def unpickle_corrupt_map():
    with open("corrupt_map.pickle", "rb") as f:
        corrupt_map = pickle.load(f)
    return corrupt_map


def unpickle_seen_map():
    with open("seen_map.pickle", "rb") as f:
        seen_map = pickle.load(f)
    return seen_map


def unpickle_tq_map():
    with open("tq_map.pickle", "rb") as f:
        tq_map = pickle.load(f)
    return tq_map


def unpickle_bq_map():
    with open("bq_map.pickle", "rb") as f:
        bq_map = pickle.load(f)
    return bq_map


def unpickle_send_time():
    with open("send_time.pickle", "rb") as f:
        send_time = pickle.load(f)
    return send_time


def unpickle_receive_time():
    with open("receive_time.pickle", "rb") as f:
        receive_time = pickle.load(f)
    return receive_time


def unpickle_packet_round_trip():
    with open("packet_round_trip.pickle", "rb") as f:
        packet_round_trip = pickle.load(f)
    return packet_round_trip

def unpickle_stats():
    pmap = unpickle_pmap()
    corrupt_map = unpickle_corrupt_map
    seen_map = unpickle_seen_map()
    tq_map = unpickle_tq_map()
    bq_map = unpickle_bq_map()
    send_time = unpickle_send_time()
    receive_time = unpickle_receive_time()
    return (pmap, corrupt_map, seen_map, tq_map, bq_map, send_time, receive_time)


def should_gen_plots(image_type):
    fname = f"PacketReceiveTimes.{image_type}"
    return not path.exists(fname)

def visualize3D(image_type, graph_type, combine):
    sum = getSummary('Exp2Summary.csv')

    use_pdf_pages = combine
    if(use_pdf_pages):
        p3D = PdfPages('images3D.pdf')
        image_type = 'pdf'
    else:
        p3D = None
        image_type = image_type

    if graph_type == 'nodes':
        for idx in range(0, sum['num_runs']):
            scatterNodes(sum, idx, graph_type=graph_type,
                        pp=p3D, image_type=image_type)
    else:
        for idx in range(0, sum['num_runs']):
            scatterNodes(sum, idx, "DestinationReceipt",
                        pp=p3D, image_type=image_type)
            scatterNodes(sum, idx, "Measurement", pp=p3D, image_type=image_type)
            scatterNodes(sum, idx, pp=p3D, image_type=image_type)

    if(use_pdf_pages):
        p3D.close()

def visualize(image_type, combine, gen3d=False):
    sum = getSummary('Exp2Summary.csv')
    use_pdf_pages = combine

    if gen3d:
        visualize3D(image_type, combine)

    if(should_pickle()):
        print("Pickling...")
        pickle_stats()

    if(use_pdf_pages):
        pp = PdfPages('images2D.pdf')
    else:
        pp = None

    pmap = unpickle_pmap()

    makeCombinedReceivedPlot(sum, pmap, pp, image_type=image_type)
    makeReceivedBarPlot(sum, pmap, pp, image_type=image_type)
    makeReceivedCountGraph(sum, pmap, pp, image_type=image_type)

    corrupt_map = unpickle_corrupt_map()
    makeCorruptBarPlot(sum, corrupt_map, pp, image_type=image_type)

    seen_map = unpickle_seen_map()
    tq_map = unpickle_tq_map()
    bq_map = unpickle_bq_map()
    makeQueueSeenPlot(sum, seen_map, tq_map, bq_map,
                      pp, image_type=image_type)

    send_time = unpickle_send_time()
    makeTimeScatterPlot(sum, "Packet Send Times",
                        send_time, pp, image_type=image_type)

    receive_time = unpickle_receive_time()
    makeTimeScatterPlot(sum, "Packet Receive Times",
                        receive_time, pp, image_type=image_type)

    if(use_pdf_pages):
        pp.close()



@contextmanager
def cwd(path):
    oldpwd = os.getcwd()
    os.chdir(path)
    try:
        yield
    finally:
        os.chdir(oldpwd)

exclude =  ['__pycache__', 'Incomplete', 'lookup_links']


def atoi(text):
    return int(text) if text.isdigit() else text

def natural_keys(text):
    return [ atoi(c) for c in re.split(r'[+-]?([0-9]+(?:[.][0-9]*)?|[.][0-9]+)', text) ]

def sorted_subdirs(root_path):
    #return sorted(os.scandir(root_path), key=lambda x: (x.is_dir(), x.name))
    #return sorted(os.scandir(root_path), key=lambda d: (d.is_dir(), d.stat().st_mtime))
    return sorted(os.scandir(root_path), key=lambda d: (d.is_dir(), natural_keys(d.name)))

def packets_in_net(sum, times):
    total_pkt = 0
    max_pkt = 0
    for st_run in times:
        for key in st_run:
            num_pkt = len(st_run[key])
            total_pkt += num_pkt
            max_pkt = max(max_pkt, num_pkt)
    
    avg = total_pkt/sum['num_runs']

    return (total_pkt, max_pkt, avg)

#{'title': 'num_runs': 'num_nodes': 'sim_duration': 'mesh_size': 'mesh_type': 'period': 'receipt_delay': 'packet_delay': 'bq_mult': 'use_thorp': 'recur_time': 'corrupt_prob':}

def match_num_nodes(sum, n):
    return sum['num_nodes'] == n

def match_mesh_size(sum, mesh_size):
    return sum['mesh_size'] == mesh_size

def match_mesh_type(sum, mesh_type):
    return sum['mesh_type'] == mesh_type

def match_period(sum, period):
    return sum['period'] == period


def collect_packets_in_net(packet_map):
    sum = load_sum()
    mesh_type = sum['mesh_type']
    num_nodes = sum['num_nodes']
    mesh_size = sum['mesh_size']
    use_thorp = sum['use_thorp']
    period = sum['period']
    receipt_delay = sum['receipt_delay']
    packet_delay = sum['packet_delay']
    bq_mult = f"{round(sum['bq_mult'], 2)}"


    # Skip corruption runs
    if(sum['corrupt_prob'] > 0):
        return
    limit = 100
    if(num_nodes <= limit):
        send_times = unpickle_send_time()
        packet_map[mesh_type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay][bq_mult].append(packets_in_net(sum, send_times))
        receive_times = unpickle_receive_time()
        packet_map[mesh_type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay][bq_mult].append(packets_in_net(sum, receive_times))


def normalize_num_packets(val, num_nodes):
    return val / (num_nodes * 5)

def normalize_num_packets_size(val, num_nodes, mesh_size):
    dist_cor = 1 / math.log10(mesh_size * 1500)
    packet_corr = 1/(num_nodes * 5)
    return val * packet_corr * dist_cor

def make_marker_key(x):
    midx = 0
    markers = ['1', '2', '3', '4', '+']
    marker_key = {}

    for key in x:
        marker_key[key] = markers[midx]
        midx += 1
    return marker_key

def extract_x_y_type(pmap, key0, tuple_idx):
    x_ticks = []
    x = defaultdict(list)
    y = defaultdict(list)

    for key1 in pmap[key0]:
        x_ticks.append(key1)
        for key2 in pmap[key0][key1]:
            yy = []
            for ts in pmap[key0][key1][key2]:
                for t in ts:
                    yy.append(normalize_num_packets_size(t[tuple_idx],key1, key2)) # Normalize by number of nodes
            
            xx = [key1]*len(yy)
            x[key2].extend(xx)
            y[key2].extend(yy)
    return x_ticks, x, y


def extract_x_y(pmap, tuple_idx):
    # Expects pmap to be pmap[key1][key2][key3](tuple)

    x_ticks = []
    x = defaultdict(list)
    y = defaultdict(list)

    for key1 in pmap:
        x_ticks.append(key1)
        for key2 in pmap[key1]:
            for key3 in pmap[key1][key2]:
                yy = []
                for ts in pmap[key1][key2][key3]:
                    for t in ts:
                        yy.append(normalize_num_packets(t[tuple_idx],key1))
                
                xx = [key1]*len(yy)
                
                x[key3].extend(xx)
                y[key3].extend(yy)

    return x_ticks, x, y

def make_y_label(idx):
    stat_labels = ['Total', 'Maximum', 'Average']
    return f"{stat_labels[idx]} Number of Packets"

def make_packet_graph(x, y, x_ticks, labelfunc, y_label, title, fname_base, image_type, pp):
    
    marker_key = make_marker_key(x)
    
    plt.ioff()
    config_font(plt)
    _, ax = plt.subplots()

    labels = labelfunc(x)

    for key in x:
        ax.scatter(x[key], y[key], label=labels[key], marker=marker_key[key])

    ax.set_ylabel(y_label)
    ax.set_xlabel("Number of Nodes")
    ax.set_xticks(x_ticks)

    fname = f"{fname_base}.{image_type}"
    fname = fname.replace(" ", "")

    plt.title(title)

    ax.legend(loc='best')

    plt.tight_layout()

    save_fig(plt, fname, pp, image_type)

def make_type_labels(x, type):
    type_labels = ['Grid', 'Linear']
    labels = {}
    for key in x:
        labels[key]= f"{type_labels[type]} Size: {key}"
    return labels


def make_bq_labels(x):
    labels = {}
    for key in x:
        ton = r'$T_{on}$'
        cleankey = key.replace('.0', '')
        labels[key]= f"{cleankey}{ton}"
    return labels

def make_r_delay_labels(x):
    labels = {}
    for key in x:
        ton = r'$T_{r}$'
        labels[key]= f"{key}{ton}"
    return labels

def make_p_delay_labels(x):
    labels = {}
    for key in x:
        ton = r'$T_{p}$'
        labels[key]= f"{key}{ton}"
    return labels

def make_thorp_labels(x):
    labels = {}
    for key in x:
        print(key)
        if int(key) == True:
            labels[key] = "Thorp"
        else:
            labels[key] = "Ideal"
    return labels

def make_period_labels(x):
    labels = {}
    for key in x:
        labels[key]= f"{key}s"
    return labels

def make_cross_packet_graph(title, fname_base, pmap, stat_idx, mesh_type, pp=None, image_type='pdf'):
    # input map:
    # [type][num_nodes][mesh_size](total_pkt, max_pkt, avg) 

    x_ticks, x, y = extract_x_y_type(pmap, mesh_type, stat_idx)
    labelfunc = lambda x : make_type_labels(x, mesh_type)

    make_packet_graph(x, y, x_ticks, labelfunc, make_y_label(stat_idx), title, fname_base, image_type, pp)


def make_gen_graph(title, fname_base, pmap, stat_idx, labelfunc, pp=None, image_type='pdf'):
    # input map:
    # [num_nodes][mesh_size][period](total_pkt, max_pkt, avg) 

    x_ticks, x, y = extract_x_y(pmap, stat_idx)

    make_packet_graph(x, y, x_ticks, labelfunc, make_y_label(stat_idx), title, fname_base, image_type, pp)


def progress(text, p):
    print(f"\u001b[1000D\u001b[K{text} {p}%", end='', flush=True)

def progress_tag(text):
    print(f"\u001b[1000D\u001b[20C\u001b[K  {text}", end='', flush=True)


def extract_type_num_size(packet_map):
    pmap = defaultdict(lambda: defaultdict((lambda: defaultdict(list))))

    #packet_map[type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay][bq_mult].append(packets_in_net(sum, send_times))
    for mesh_type in packet_map:
        for num_nodes in packet_map[mesh_type]:
            for mesh_size in packet_map[mesh_type][num_nodes]:
                for use_thorp in packet_map[mesh_type][num_nodes][mesh_size]:
                    for period in packet_map[mesh_type][num_nodes][mesh_size][use_thorp]:
                        for receipt_delay in packet_map[mesh_type][num_nodes][mesh_size][use_thorp][period]:
                            for packet_delay in packet_map[mesh_type][num_nodes][mesh_size][use_thorp][period][receipt_delay]:
                                for bq_mult in packet_map[mesh_type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay]:
                                    #print(f"num_tuples: {len(packet_map[type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay][bq_mult])}")
                                    pmap[mesh_type][num_nodes][mesh_size].append(packet_map[mesh_type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay][bq_mult])
    return pmap

def extract_period(packet_map):
    pmap = defaultdict(lambda: defaultdict((lambda: defaultdict(list))))

    #packet_map[type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay][bq_mult].append(packets_in_net(sum, send_times))
    for mesh_type in packet_map:
        if(mesh_type == 0):
            for num_nodes in packet_map[mesh_type]:
                for mesh_size in packet_map[mesh_type][num_nodes]:
                    for use_thorp in packet_map[mesh_type][num_nodes][mesh_size]:
                        for period in packet_map[mesh_type][num_nodes][mesh_size][use_thorp]:
                            for receipt_delay in packet_map[mesh_type][num_nodes][mesh_size][use_thorp][period]:
                                for packet_delay in packet_map[mesh_type][num_nodes][mesh_size][use_thorp][period][receipt_delay]:
                                    for bq_mult in packet_map[mesh_type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay]:
                                        #print(f"num_tuples: {len(packet_map[type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay][bq_mult])}")
                                        pmap[num_nodes][mesh_size][period].append(packet_map[mesh_type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay][bq_mult])
    return pmap

def extract_thorp(packet_map):
    pmap = defaultdict(lambda: defaultdict((lambda: defaultdict(list))))

    #packet_map[type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay][bq_mult].append(packets_in_net(sum, send_times))
    for mesh_type in packet_map:
        if(mesh_type == 0):
            for num_nodes in packet_map[mesh_type]:
                for mesh_size in packet_map[mesh_type][num_nodes]:
                    for use_thorp in packet_map[mesh_type][num_nodes][mesh_size]:
                        for period in packet_map[mesh_type][num_nodes][mesh_size][use_thorp]:
                            for receipt_delay in packet_map[mesh_type][num_nodes][mesh_size][use_thorp][period]:
                                for packet_delay in packet_map[mesh_type][num_nodes][mesh_size][use_thorp][period][receipt_delay]:
                                    for bq_mult in packet_map[mesh_type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay]:
                                        #print(f"num_tuples: {len(packet_map[type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay][bq_mult])}")
                                        pmap[num_nodes][mesh_size][use_thorp].append(packet_map[mesh_type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay][bq_mult])
    return pmap

def extract_r_delay(packet_map):
    pmap = defaultdict(lambda: defaultdict((lambda: defaultdict(list))))

    #packet_map[type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay][bq_mult].append(packets_in_net(sum, send_times))
    for mesh_type in packet_map:
        if(mesh_type == 0):
            for num_nodes in packet_map[mesh_type]:
                for mesh_size in packet_map[mesh_type][num_nodes]:
                    for use_thorp in packet_map[mesh_type][num_nodes][mesh_size]:
                        for period in packet_map[mesh_type][num_nodes][mesh_size][use_thorp]:
                            for receipt_delay in packet_map[mesh_type][num_nodes][mesh_size][use_thorp][period]:
                                for packet_delay in packet_map[mesh_type][num_nodes][mesh_size][use_thorp][period][receipt_delay]:
                                    for bq_mult in packet_map[mesh_type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay]:
                                        #print(f"num_tuples: {len(packet_map[type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay][bq_mult])}")
                                        pmap[num_nodes][mesh_size][receipt_delay].append(packet_map[mesh_type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay][bq_mult])
    return pmap

def extract_p_delay(packet_map):
    pmap = defaultdict(lambda: defaultdict((lambda: defaultdict(list))))

    #packet_map[type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay][bq_mult].append(packets_in_net(sum, send_times))
    for mesh_type in packet_map:
        if(mesh_type == 0):
            for num_nodes in packet_map[mesh_type]:
                for mesh_size in packet_map[mesh_type][num_nodes]:
                    for use_thorp in packet_map[mesh_type][num_nodes][mesh_size]:
                        for period in packet_map[mesh_type][num_nodes][mesh_size][use_thorp]:
                            for receipt_delay in packet_map[mesh_type][num_nodes][mesh_size][use_thorp][period]:
                                for packet_delay in packet_map[mesh_type][num_nodes][mesh_size][use_thorp][period][receipt_delay]:
                                    for bq_mult in packet_map[mesh_type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay]:
                                        #print(f"num_tuples: {len(packet_map[type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay][bq_mult])}")
                                        pmap[num_nodes][mesh_size][packet_delay].append(packet_map[mesh_type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay][bq_mult])
    return pmap


def extract_bq_period(packet_map):
    pmap = defaultdict(lambda: defaultdict((lambda: defaultdict(list))))

    #packet_map[type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay][bq_mult].append(packets_in_net(sum, send_times))
    for mesh_type in packet_map:
        if(mesh_type == 0):
            for num_nodes in packet_map[mesh_type]:
                for mesh_size in packet_map[mesh_type][num_nodes]:
                    for use_thorp in packet_map[mesh_type][num_nodes][mesh_size]:
                        for period in packet_map[mesh_type][num_nodes][mesh_size][use_thorp]:
                            for receipt_delay in packet_map[mesh_type][num_nodes][mesh_size][use_thorp][period]:
                                for packet_delay in packet_map[mesh_type][num_nodes][mesh_size][use_thorp][period][receipt_delay]:
                                    for bq_mult in packet_map[mesh_type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay]:
                                        #print(f"num_tuples: {len(packet_map[type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay][bq_mult])}")
                                        pmap[num_nodes][mesh_size][bq_mult].append(packet_map[mesh_type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay][bq_mult])
    return pmap


def show_keys(m):
    try:
        print(m.keys())
        for k  in m:
            show_keys(m[k])
    except:
        pass
    finally: 
        print(f"len: {len(m)}")

def summarize_cross_pickle_type_num_nodes_mesh_size(packet_map):
    #packet_map[type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay][bq_mult].append(packets_in_net(sum, send_times))
    for type in packet_map:
        print(f"type: {type}")
        for num_nodes in packet_map[type]:
            print(f"num_nodes: {num_nodes}")
            for mesh_size in packet_map[type][num_nodes]:
                print(f"mesh_size: {mesh_size}")
                for use_thorp in packet_map[type][num_nodes][mesh_size]:
                    print(f"use_thorp: {use_thorp}")
                    for period in packet_map[type][num_nodes][mesh_size][use_thorp]:
                        print(f"period: {period}")
                        for receipt_delay in packet_map[type][num_nodes][mesh_size][use_thorp][period]:
                            print(f"receipt_delay: {receipt_delay}")
                            for packet_delay in packet_map[type][num_nodes][mesh_size][use_thorp][period][receipt_delay]:
                                print(f"packet_delay: {packet_delay}")
                                for bq_mult in packet_map[type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay]:
                                    print(f"bq_mult: {bq_mult}")
                                    tpls = packet_map[type][num_nodes][mesh_size][use_thorp][period][receipt_delay][packet_delay][bq_mult]
                                    print(f"tuples: {len(tpls)}: {tpls}")


def cross_plot(root_path, plot_type):
    cross_pickle(root_path)
    unpick = unpickle_cross_pickle()
    #summarize_cross_pickle(unpick)

    if plot_type == "size" or plot_type == "all":
        packet_map =  extract_type_num_size(unpick)
        make_cross_packet_graph("Average Number of Packets/(Nodes * 5) in Grid Network", "packets_in_grid_net", packet_map, 2, 0, pp=None, image_type='pdf')
        make_cross_packet_graph("Average Number of Packets/(Nodes * 5) in Linear Network", "packets_in_linear_net", packet_map, 2, 1, pp=None, image_type='pdf')

    if plot_type =="period" or plot_type == "all":
        packet_map =  extract_period(unpick)
        make_gen_graph(r"Average Number of Packets/(Nodes * 5) By $T_{on}$", "packets_in_period", packet_map, 2, make_period_labels, pp=None, image_type='pdf')
    
    if plot_type =="bq_mult" or plot_type == "all":
        packet_map =  extract_bq_period(unpick)
        make_gen_graph("Average Number of Packets/(Nodes * 5) By BQ Mulitplier", "packets_in_bq_mult", packet_map, 2, make_bq_labels ,pp=None, image_type='pdf')

    if plot_type =="thorp" or plot_type == "all":
        packet_map =  extract_thorp(unpick)
        make_gen_graph("Average Number of Packets/(Nodes * 5) By Loss Type", "packets_by_use_thorp", packet_map, 2, make_thorp_labels ,pp=None, image_type='pdf')

    if plot_type == "r_delay" or plot_type == "all":
        packet_map =  extract_r_delay(unpick)
        make_gen_graph(r"Average Number of Packets/(Nodes * 5) By $T_{r}$", "packets_by_r_delay", packet_map, 2, make_r_delay_labels ,pp=None, image_type='pdf')

    if plot_type == "p_delay" or plot_type == "all":
        packet_map =  extract_p_delay(unpick)
        make_gen_graph(r"Average Number of Packets/(Nodes * 5) By $T_{p}$", "packets_by_p_delay", packet_map, 2, make_p_delay_labels ,pp=None, image_type='pdf')

    if plot_type == "queue" or plot_type == "all":
        plot_queue_stats_cwd(root_path, pp=None, image_type='pdf' )
    
    if plot_type == "latency" or plot_type == "all":
        plot_latency_cwd(root_path, pp=None, image_type='pdf')
    
    if plot_type == "delivery" or plot_type == "all":
        plot_overall_loss(root_path, pp=None, image_type='pdf')


def make_lookup_links(root_path):
    entries = sorted_subdirs(root_path)

    links_sub_dir = "lookup_links"
    if not os.path.exists(links_sub_dir):
        os.mkdir(links_sub_dir)

    for entry in entries:
        if(entry.name in exclude):
            continue
        if entry.is_dir():
            subdir = os.path.join(root_path, entry)
            with cwd(subdir):
                sum = getSummary('Exp2Summary.csv')
                # Make link like: exp-
                #{ 'title': title, 
                #  'num_runs': 
                #  'num_nodes':
                #  'sim_duration':
                #  'mesh_size':
                #  'mesh_type':
                #  'period':
                #  'receipt_delay': 
                #  'packet_delay': 
                #  'bq_mult': 
                #  'use_thorp':
                #  'recur_time':
                #  'corrupt_prob':
                mesh_types = ['grid', 'linear']
                if(sum['use_thorp']):
                    thorp_ideal = "thorp"
                else:
                    thorp_ideal = "ideal"
                
                link_name = f"exp-nn{sum['num_nodes']}-ms{sum['mesh_size']}-{mesh_types[sum['mesh_type']]}-p{sum['period']}-rd{sum['receipt_delay']}-pd{sum['packet_delay']}-bq{sum['bq_mult']*sum['period']}-{thorp_ideal}-cp{sum['corrupt_prob']}"
                print(link_name)
                lnk = os.path.join(root_path, links_sub_dir, link_name)
                print(lnk)
                os.symlink(subdir, lnk)


def queue_stats(sum, seen_map, tq_map, bq_map):
    num_runs = sum['num_runs']

    max_seen_per_node = 0
    avg_seen_per_node = 0
    for i in range(num_runs):
        max_seen_per_node = max(max_seen_per_node, seen_map[i])
        avg_seen_per_node = max(avg_seen_per_node, seen_map[i]/sum['num_nodes'])
        

    tq_rem = 0
    tq_max = 0
    for tq in tq_map:
        tq_rem = max(tq_rem, tq[0])
        tq_max = max(tq_max, tq[1])

    bq_rem = 0
    bq_max = 0
    for bq in bq_map:
        bq_rem = max(bq_rem, bq[0])
        bq_max = max(bq_max, bq[1])

    return (avg_seen_per_node, max_seen_per_node, tq_rem, tq_max, bq_rem, bq_max)

def plot_queue_stats_cwd(root_path, image_type, pp):

    all_stats = defaultdict(list)
    entries = sorted_subdirs(root_path)
    #num_entries = len(entries)

    count = 0
    for entry in entries:
        if(entry.name in exclude):
            continue
        if entry.is_dir():
            count += 1
            subdir = os.path.join(root_path, entry)
            with cwd(subdir):
                sum = load_sum()
                seen_map = unpickle_seen_map()
                tq_map = unpickle_tq_map()
                bq_map = unpickle_bq_map()

                s = queue_stats(sum, seen_map, tq_map, bq_map)
                all_stats[sum['num_nodes']] = s
    make_queue_graph(all_stats, 'queue_sizes', image_type, pp)

def extract_x_y_q(pmap, tuple_idx):
    # Expects pmap to be pmap[key1](tuple)

    x_ticks = []
    x = []
    y = []

    for key1 in pmap:
        x_ticks.append(key1)
        #print(pmap[key1])
        y.append(pmap[key1][tuple_idx])
        x.append(key1)

    return x_ticks, x, y


def extract_x_y_l(lmap, tuple_idx):
    # Expects pmap to be pmap[key1](tuple)

    x_ticks = []
    x = []
    y = []

    for key1 in lmap:
        #print(key1)
        x_ticks.append(key1)
        #print(lmap[key1])
        y.append(lmap[key1][tuple_idx])
        x.append(key1)

    return x_ticks, x, y


def make_queue_graph(all_stats, fname_base, image_type, pp):
    # avg_seen_per_node, max_seen_per_node, tq_rem, tq_max, bq_rem, bq_max
    
    plt.ioff()
    config_font(plt)
    _, ax = plt.subplots()

    markers = ['+', 'x', '1', '2', '3', '4', ',']
    labels = ['VL Avg', 'VL Max', 'TQ Remainder', 'TQ Max', 'BQ Remainder', 'BQ Max']

    for i in range(6):
        x_ticks, x, y = extract_x_y_q(all_stats, i)
        ax.scatter(x, y, label=labels[i], marker=markers[i])

    ax.set_ylabel("Size of TQ, BQ, VL")
    ax.set_yscale('log')
    ax.set_xlabel("Number of Nodes")
    ax.set_xticks(x_ticks)

    fname = f"{fname_base}.{image_type}"
    fname = fname.replace(" ", "")

    plt.title("Sizes of TQ, BQ, and Unbounded VL\nAll Simulation Parameters, All Mesh Sizes")

    ax.legend(loc='best')

    plt.tight_layout()

    save_fig(plt, fname, pp, image_type)

def avg(list):
    sum = 0
    for val in list:
        sum += val
    return sum/len(list)

def round_trip_stats(sum, packet_round_trip):
    # 'pkt_id'
    # 'dst_id'
    # 'pkt_type'
    # 'sent'
    # 'received'
    # 'rcpt_received'
    # 'sent_at'
    # 'received_at'
    # 'receipt_received_at'
    # 'receive_count'
    # 'dest_count'
    # 'forward_count'

    out_time = []
    back_time = []
    #print(packet_round_trip)
    for prts in packet_round_trip:
        for r in prts:
            pkt_type = r['pkt_type']
            received = r['received']
            rcpt_received = r['rcpt_received']
            if pkt_type == 'DataPacket' and received and rcpt_received:
                out_time.append(r['received_at'] - r['sent_at'])
                back_time.append(r['receipt_received_at'] - r['received_at'] )
    
    max_out = max(out_time)
    avg_out = avg(out_time)
    max_back = max(back_time)
    avg_back = avg(back_time)
    return (max_out, avg_out, max_back, avg_back)


def plot_latency_cwd(root_path, image_type, pp):

    all_stats = defaultdict(list)
    entries = sorted_subdirs(root_path)
    #num_entries = len(entries)

    count = 0
    for entry in entries:
        if(entry.name in exclude):
            continue
        if entry.is_dir():
            count += 1
            subdir = os.path.join(root_path, entry)
            with cwd(subdir):
                sum = load_sum()
                packet_round_trip = unpickle_packet_round_trip()
                rt = round_trip_stats(sum, packet_round_trip)
                all_stats[sum['num_nodes']] = rt
    make_latency_graph(all_stats, 'latency_times', image_type, pp)

def calc_toa_min_max(PL):
    BWs = [7.8, 10.4, 15.6, 20.8, 31.2, 41.7, 62.5, 125, 250, 500]
    SFs = [6, 7, 8, 9, 10, 11, 12]
    IHs = [0, 1]
    DEs = [0, 1]
    CRs = [1, 4]
    n_preamble = 12  # Default can be from 6 to 65535

    for BW in BWs:
        BW = BW * 1000
        for SF in SFs:
            R_s = BW/2 ** float(SF)
            T_s = 1/R_s
            T_preamble = (n_preamble+4.25) * T_s
        
            packet_TsMin = sys.float_info.max
            packet_TsMax = 0
            for IH in IHs:
                for DE in DEs:
                    for CR in CRs:
                        n_payload = 8 + \
                            max(math.ceil(
                                (8*PL - 4*SF + 28 + 16 - 20*IH)/4*(SF-2*DE))*(CR+4), 0)
                        T_payload = n_payload * T_s
                        T_packet = T_preamble + T_payload
                        packet_TsMin = min(T_packet, packet_TsMin)
                        packet_TsMax = max(T_packet, packet_TsMax)
    print(packet_TsMin, packet_TsMax)
    return packet_TsMin, packet_TsMax

def sim_time(bytes, data_rate):
    return 8 * bytes / data_rate

def calc_toa_min_max_node(n):
    r = 300
    Td = sim_time(59, r)
    Tr = sim_time(13, r)
    Tmin_d, Tmax_d = Td, Td #calc_toa_min_max(59)
    Tmin_r, Tmax_r = Tr, Tr #calc_toa_min_max(13)
    #print(f"d {Tmin_d}, {Tmax_d}")
    #print(f"r {Tmin_r}, {Tmax_r}")
    toa_min = n * (Tmin_d + Tmin_r)
    toa_max = n * (Tmax_d + Tmax_r)
    print(f"{n} {toa_min}, {toa_max}")
    return toa_min, toa_max

def make_latency_graph(all_stats, fname_base, image_type, pp):
    # avg_seen_per_node, max_seen_per_node, tq_rem, tq_max, bq_rem, bq_max
    
    plt.ioff()
    config_font(plt)
    _, ax = plt.subplots()

    markers = ['1', '2', '3', '4', '+', 'x', ',']
    labels = ['Data Packet Max', 'Data Packet Avg', 'End Receipt Max', 'End Receipt Avg']

    for i in range(4):
        x_ticks, x, y = extract_x_y_l(all_stats, i)
        print(i, y)
        ax.scatter(x, y, label=labels[i], marker=markers[i])

    x_t = []
    y_min_t = []
    y_max_t = []
    for n in range(2, 17):
        print(n)
        y_min, y_max = calc_toa_min_max_node(n)
        x_t.append(n)
        y_min_t.append(y_min)
        y_max_t.append(y_max)

    print(y_min_t)
    print(y_max_t)
    ax.plot(x_t, y_min_t, label="NLM Based Round Trip")

    ax.set_ylabel("Time (seconds)")
    ax.set_yscale('log')
    ax.set_xlabel("Number of Nodes")
    ax.set_xticks(x_ticks)

    fname = f"{fname_base}.{image_type}"
    fname = fname.replace(" ", "")

    plt.title("Max and Average Transit Times\n All Simulation Parameters, All Mesh Sizes")

    ax.legend(loc='best')

    plt.tight_layout()

    save_fig(plt, fname, pp, image_type)

def plot_overall_loss(root_path, image_type, pp):

    all_stats = defaultdict(list)
    entries = sorted_subdirs(root_path)
    #num_entries = len(entries)

    count = 0
    for entry in entries:
        if(entry.name in exclude):
            continue
        if entry.is_dir():
            count += 1
            subdir = os.path.join(root_path, entry)
            with cwd(subdir):
                sum = load_sum()
                if sum['corrupt_prob'] == 0:
                    pmap = unpickle_pmap()
                    _, _, avg, _ = computeStats(pmap)
                    all_stats[sum['num_nodes']].append(avg)

    make_overall_received_graph(all_stats, 'received', image_type, pp)


def extract_x_y_p(pmap, ptype):
    # Expects pmap to be pmap[key1](tuple)

    x_ticks = []
    x = []
    y = []

    for key1 in pmap:
        #print(key1)
        x_ticks.append(key1)
        #print(pmap[key1])
        for m in pmap[key1]:
            #print(m)
            #print(ptype)
            yy = m[ptype]
            #print(yy)
            y.append(yy)
            x.append(key1)

    return x_ticks, x, y

def make_overall_received_graph(all_stats, fname_base, image_type, pp):
    # Make the plot
    plt.ioff()
    config_font(plt)
    _, ax1 = plt.subplots()

    labels = ['DataPacket', 'EndReceipt']
    markers = ['1', '2', '3', '4', '+', 'x', ',']


    for i in range(2):
        x_ticks, x, y = extract_x_y_p(all_stats, labels[i])
        ax1.scatter(x, y, marker=markers[i], label=labels[i])

    ax1.set_ylabel('% Packets Received')
    ax1.set_xlabel("Nodes")
    ax1.set_xticks(x_ticks)

    fname = f"{fname_base}.{image_type}"
    fname = fname.replace(" ", "")

    plt.title("Packet Delivery")

    # Create legend & Show graphic
    ax1.legend(loc='lower left')

    plt.tight_layout()

    save_fig(plt, fname, pp, image_type)
