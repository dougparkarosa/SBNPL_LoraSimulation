# Plot graph of transmit times for LoRa Modem
import math


BWs = [7.8, 10.4, 15.6, 20.8, 31.2, 41.7, 62.5, 125, 250, 500]
SFs = [6, 7, 8, 9, 10, 11, 12]
PLs = [1, 32, 64, 128, 255]
IHs = [0, 1]
DEs = [0, 1]
CRs = [1, 4]
n_preamble = 12  # Default can be from 6 to 65535

PL_min = {}
PL_max = {}

for BW in BWs:
    BW = BW * 1000
    for SF in SFs:
        R_s = BW/2 ** float(SF)
        T_s = 1/R_s
        T_preamble = (n_preamble+4.25) * T_s
        for PL in PLs:
            packet_TsMin = 0
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
            PL_min[PL] = packet_TsMin
            PL_max[PL] = packet_TsMax


print(PL_min)
print(PL_max)
