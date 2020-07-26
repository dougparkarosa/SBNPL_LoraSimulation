# Script to run the matrix of experiments
import subprocess
import time
import os
import math
import copy_support as cs

# Match the name as produced by the model


def outdir():
    timestamp = time.strftime("%F_%H_%M_%S")
    #root = "../../../../../experiments/" + timestamp
    root = "/media/doug/Five/experiments/" + timestamp
    return root

shortPass = True
# Don't use 0 ns-3 doesn't like it
#seed = "123456"
seed = 4321

simulator = "./../../build/src/lora/examples/ns3.30.1-lora-example-optimized"
simtime = 36000
repeat_time = simtime / 5
mesh_size_type = [(100, 0), (1000, 0), (10000, 0), (100000, 0),
                  (250000, 1), (500000, 1), (1000000, 1)]

if shortPass:
    num_nodes = [2, 3, 4, 8, 12, 16]
else:
    num_nodes = [2, 3, 4, 8, 16, 32, 64]
#num_nodes = [100]
#period = 30
#r_delay = 2
#p_delay = 10 * r_delay


if shortPass:
    bq_mults = [4, 8, 16]
    t_ons = [30, 60, 120]
    t_rs = [1, 5]
    t_ps = [10, 20]
    thorp = ["true", "false"]
    mesh_size_type = [(1000, 0), (10000, 0), (100000, 0), (1000, 1), (10000, 1), (100000, 1)]

else:
    mesh_size_type = [(100, 0), (1000, 0), (10000, 0), (100000, 0), (100, 1), (1000, 1), (10000, 1), (100000, 1), (250000, 1), (500000, 1), (1000000, 1)]
    bq_mults = [2, 4, 8, 16]
    t_ons = [30, 60, 120, 250, 500]
    t_rs = [1, 5]
    t_ps = [10, 20]
    thorp = ["true", "false"]

#bq_mult = 10.5

#corrupt_it = 0

log_num = 0

print(os.getcwd())

env = {"PYTHONPATH": "/home/doug/MS_Classes/Thesis/netsim/ns-allinone-3.30.1/ns-3.30.1/build/bindings/python:/home/doug/MS_Classes/Thesis/netsim/ns-allinone-3.30.1/ns-3.30.1/src/visualizer",
       "LD_LIBRARY_PATH": "/usr/lib/gcc/x86_64-linux-gnu/9:/home/doug/MS_Classes/Thesis/netsim/ns-allinone-3.30.1/ns-3.30.1/build/lib",
       "NS3_MODULE_PATH": "/usr/lib/gcc/x86_64-linux-gnu/9:/home/doug/MS_Classes/Thesis/netsim/ns-allinone-3.30.1/ns-3.30.1/build/lib",
       "NS3_EXECUTABLE_PATH": "/home/doug/MS_Classes/Thesis/netsim/ns-allinone-3.30.1/ns-3.30.1/build/src/fd-net-device:/home/doug/MS_Classes/Thesis/netsim/ns-allinone-3.30.1/ns-3.30.1/build/src/tap-bridge",
       "PATH": "/home/doug/MS_Classes/Thesis/netsim/ns-allinone-3.30.1/ns-3.30.1/build/src/fd-net-device:/home/doug/MS_Classes/Thesis/netsim/ns-allinone-3.30.1/ns-3.30.1/build/src/tap-bridge:/home/doug/anaconda3/bin:/home/doug/anaconda3/condabin:/home/doug/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin:/usr/brlcad/bin"
       }


procs = []
outfiles = []

base_out = outdir()
gen_plots = []


def start_exp(log_num, base_out, procs, gen_plots, nn, simtime, ms, period, r_delay, p_delay, use_thorp, repeat_time, bq_mult, seed, corrupt_prob=0.0):
    # Skip any ranges that aren't expected to work.
    type = ms[1]
    # Linear case. We need to have enough nodes to get from start to finish.
    if(type == 1):
        r = ms[0]
        if(r / (nn-1) > 10000):
            return log_num  # Don't increment

    if(type == 0):  # Grid mode. Sqrt(r^2 + r^2) needs to be spanable by nn nodes
        r = ms[0]
        r = math.sqrt(r*r + r*r)
        if(r / (nn-1) > 10000):
            return log_num  # Don't increment

    expoutname = f"exp_{log_num}"
    out_dir = base_out + "/" + expoutname
    if(not os.path.exists(out_dir)):
        os.makedirs(out_dir)
    outfilepath = out_dir + f"/experiment_{log_num}.log"
    gen_plots.append(f"cd {out_dir}; python3 visualize.py")
    print(outfilepath)
    outfile = open(outfilepath, 'w')
    outfiles.append(outfile)
    command = f"{simulator}"
    input = [command, f"--size={nn}", f"--time={simtime}", f"--mesh_size={ms[0]}", f"--mesh_type={ms[1]}", f"--out={out_dir}", f"--corrupt={corrupt_prob}",
             "--v=0", "--r=20", f"--period={period}", f"--rdelay={r_delay}", f"--pdelay={p_delay}", f"--thorp={use_thorp}", f"--repeat={repeat_time}", f"--bq_mult={bq_mult}", f"--seed={seed}", "ex2"]
    print(input)
    p = subprocess.Popen(input, stdout=outfile, shell=False, env=env,
                         text=True)

    # Wait if we are running a large model
    # if(nn >= 16):
    p.wait()

    procs.append(p)
    log_num = log_num+1
    return log_num


for nn in num_nodes:
    for ms in mesh_size_type:
        for use_thorp in thorp:
            for period in t_ons:
                for r_delay in t_rs:
                    for p_delay in t_ps:
                        for bq_mult in bq_mults:
                            log_num = start_exp(log_num, base_out, procs, gen_plots, nn, simtime,
                                                ms, period, r_delay, p_delay, use_thorp, repeat_time, bq_mult, seed)


# corruption run.
log_num = start_exp(log_num, base_out, procs, gen_plots, 10, simtime, (10000, 0),
                    period, r_delay, p_delay, False, repeat_time, bq_mult, seed, corrupt_prob=0.01)


num_procs = len(procs)
num_procs_complete = 0
print(f"Processes started: Monitoring {num_procs} processes.")
for p in procs:
    p.poll()
    if(p.returncode != None):
        # Process pcompleted
        print(f"{p.args} Done status={p.returncode}")
        out, err = p.communicate(0)
        num_procs_complete = num_procs_complete + 1
    time.sleep(2)
    if(num_procs_complete == num_procs):
        break


cs.copy_root_files('./examples/', base_out)

doplots = False
if doplots:
    print("All simulations are done, Graphs will be ready in a while.")
    print("Generating graphs")
    for cmd in gen_plots:
        print(cmd)
        p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, shell=True, text=True)
        p.wait()  # Memory usage is huge wait for each one.

