import matplotlib.pyplot as plt
import numpy as np
import matplotlib.gridspec as gridspec


def GetAttenDbKm(freq_khz):
    fsq = freq_khz * freq_khz
    if freq_khz >= 0.4:
        return 0.11 * fsq / (1 + fsq) + 44 * fsq / (4100 + fsq)
        + 2.75 * 0.0001 * fsq + 0.003
    else:
        return 0.002 + 0.11 * (freq_khz / (1 + freq_khz)) + 0.011 * freq_khz


def GetAttenDbKyd(freq_khz):
    return GetAttenDbKm(freq_khz) / 1.093613298


def GetPathLossDb(dist, spread_coef, cent_freq):
    return spread_coef * 10.0 * np.log10(dist)
    + (dist / 1000.0) * GetAttenDbKm(cent_freq / 1000.0)


def f(x, spread_coef, cent_freq):
    return GetPathLossDb(x, spread_coef, cent_freq)


x = np.arange(0.1, 100000, 1.0)

fig = plt.figure()

gs1 = gridspec.GridSpec(2, 1)
ax1 = fig.add_subplot(gs1[0])
ax2 = fig.add_subplot(gs1[1])

# plt.subplot(211)
for fq in [433, 868, 915, 923]:
    s = 1.5
    ax1.plot(x, f(x, s, fq), label=f"Center Freq KHz: {fq}")
ax1.set_ylabel('Loss dB')
ax1.legend()
ax1.set_title(f"Thorp Center Freq  Spreading Coeff: {s}")

fq = 915
for s in np.arange(1.0, 3, .5):
    ax2.plot(x, f(x, s, fq), label=f"Spread Coeff: {s}")

ax2.set_title(f"Thorp Spreading Coeff  Center Freq: {fq}")
ax2.legend()
ax2.set_ylabel('Loss dB')

plt.xlabel("Distance meters")

gs1.tight_layout(fig)
# plt.legend()
# plt.show()
plt.savefig("/home/doug/Pictures/Thorp.png")
