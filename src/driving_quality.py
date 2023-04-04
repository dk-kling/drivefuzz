import numpy as np
import scipy.signal

# Fuzzy logic
import skfuzzy

import constants as c


def check_hard_acc(acc_list):
    # ref: https://copradar.com/chapts/references/acceleration.html
    # acc_list is a snapshot of the player's acceleration (km/h^2) per frame
    # if HARD_ACC_THRES is 20 and FPS is set to 20,
    # we will count acceleration above 20/20 = 1.0
    # acc_list_filtered = acc_list > c.HARD_ACC_THRES / c.FRAME_RATE
    return np.count_nonzero(acc_list >= c.HARD_ACC_THRES / c.FRAME_RATE)


def check_hard_braking(acc_list):
    # print(acc_list < c.HARD_BRAKE_THRES / c.FRAME_RATE)
    return np.count_nonzero(acc_list <= c.HARD_BRAKE_THRES / c.FRAME_RATE)


def check_hard_turn(Vy_list, SWA_list):
    hard_turn_min_sa = 20
    hard_turn_thres = 0.18
    hard_turn_indicator = np.divide(
            Vy_list,
            SWA_list,
            out=np.zeros_like(Vy_list),
            where=abs(SWA_list)>hard_turn_min_sa
            )
    # print(hard_turn_indicator)
    num_hard_turns = np.sum(abs(hard_turn_indicator) > hard_turn_thres)
    # print("# hard turns:", num_hard_turns)
    return num_hard_turns


def butter_lowpass(cutoff, nyq_freq, order=4):
    normal_cutoff = float(cutoff) / nyq_freq
    b, a = scipy.signal.butter(order, normal_cutoff, btype='lowpass')
    return b, a


def butter_lowpass_filter(data, cutoff_freq, nyq_freq, order=4):
    # Source: https://github.com/guillaume-chevalier/filtering-stft-and-laplace-transform
    b, a = butter_lowpass(cutoff_freq, nyq_freq, order=order)
    y = scipy.signal.filtfilt(b, a, data)
    return y


def get_vx_light(Vx_list):
    Vx_light = butter_lowpass_filter(
            Vx_list,
            c.CUTOFF_FREQ_LIGHT,
            c.FRAME_RATE / 2
            )

    return Vx_light

def get_ay_list(Vy_list):
    Ay_list = np.diff(Vy_list) # idx: frame, unit: km/h'frame (== km/s'1/20sec)
    kmphsec_to_g = 0.028325450360498
    Ay_list = c.FRAME_RATE * Ay_list * kmphsec_to_g
    return Ay_list


def get_ay_diff_list(Ay_list):
    # sample_rate = c.FRAME_RATE # there are FPS (e.g., 20) ticks in one sec
    Ay_light = butter_lowpass_filter(
            Ay_list,
            c.CUTOFF_FREQ_LIGHT,
            c.FRAME_RATE / 2
            )

    Ay_light_heavy = butter_lowpass_filter(
            Ay_light,
            c.CUTOFF_FREQ_HEAVY,
            c.FRAME_RATE / 2
            )

    # for OS detection
    Ay_diff_list = np.absolute(Ay_light - Ay_light_heavy)

    return Ay_diff_list


def get_ay_heavy(Ay_list):
    Ay_ext = np.append(Ay_list, 0) # sync the size with sa array

    Ay_heavy = np.absolute(butter_lowpass_filter(
            Ay_ext,
            c.CUTOFF_FREQ_HEAVY,
            c.FRAME_RATE / 2
            ))

    return Ay_heavy


def get_swa_diff_list(SWA_list):
    SWA_light = butter_lowpass_filter(
            SWA_list,
            c.CUTOFF_FREQ_LIGHT,
            c.FRAME_RATE / 2
            )

    SWA_heavy = butter_lowpass_filter(
            SWA_light,
            c.CUTOFF_FREQ_HEAVY,
            c.FRAME_RATE / 2
            )

    # For OS
    SWA_diff_list = np.absolute(SWA_light - SWA_heavy)

    return SWA_diff_list


def get_swa_heavy(SWA_list):
    SWA_heavy_list = np.absolute(butter_lowpass_filter(
            SWA_list,
            c.CUTOFF_FREQ_HEAVY,
            c.FRAME_RATE / 2
            ))

    return SWA_heavy_list

def get_ay_gain(SWA_heavy_list, Ay_heavy):
    Ay_gain = np.absolute(
            np.divide(
                Ay_heavy,
                SWA_heavy_list,
                out=np.zeros_like(SWA_heavy_list),
                where=SWA_heavy_list>0
                )
            )

    return Ay_gain


def get_ay_peak(Ay_gain):
    Ay_peak = np.zeros_like(Ay_gain)
    d_Ay_gain = np.diff(Ay_gain)
    peak = 0
    for i, d in enumerate(d_Ay_gain):
        if d >= 0:
            peak = Ay_gain[i]
        Ay_peak[i] = peak

    return Ay_peak


def get_frac_drop(Ay_gain, Ay_peak):
    frac_drop = np.divide(
            (Ay_gain - Ay_peak),
            Ay_peak,
            out=np.zeros_like(Ay_peak),
            where=Ay_peak>0
            )

    return frac_drop


def get_abs_yr(yaw_rate_list):
    yr = np.array(yaw_rate_list)
    yr = butter_lowpass_filter(
            yr,
            c.CUTOFF_FREQ_HEAVY,
            c.FRAME_RATE / 2
            )
    abs_yr = np.absolute(yr)

    # XXX: should decide whether we should apply the delay factor on
    # Ay and abs_yr.

    return abs_yr


def get_oversteer_level(sa, la, yr):
    # Implement our fuzzy logic using skfuzzy
    # https://pythonhosted.org/scikit-fuzzy/auto_examples/plot_tipping_problem.html

    # define ranges
    x_sa = np.arange(0, 55, 5) # 0 ~ 50
    x_la = np.arange(0, 0.55, 0.05) # 0 ~ 0.5
    x_yr = np.arange(0, 50, 5) # 0 ~ 45
    x_os = np.arange(0, 11, 1) # 0 ~ 10

    # create membership functions
    sa_low = skfuzzy.trimf(x_sa, [0, 0, 20])
    sa_med = skfuzzy.trimf(x_sa, [5, 25, 45])
    sa_high = skfuzzy.trimf(x_sa, [30, 50, 50])

    la_low = skfuzzy.trimf(x_la, [0, 0, 0.2])
    la_med = skfuzzy.trimf(x_la, [0.05, 0.25, 0.45])
    la_high = skfuzzy.trimf(x_la, [0.3, 0.5, 0.5])

    yr_low = skfuzzy.trimf(x_yr, [0, 0, 17.5])
    yr_med = skfuzzy.trimf(x_yr, [4, 22.5, 41])
    yr_high = skfuzzy.trimf(x_yr, [27.5, 45, 45])

    os_low = skfuzzy.trimf(x_os, [0, 0, 4])
    os_med = skfuzzy.trimf(x_os, [1, 5, 9])
    os_high = skfuzzy.trimf(x_os, [6, 10, 10])

    # rules (Ay: LACC, AVz: yaw rate)
    # 1. if SA is Small and LACC is Small then OS is NO OS
    # 2. if SA is Medium and LACC is Medium then OS is Moderate OS
    # 3. if SA is Large and LACC is Large then OS is Heavy OS
    # 4. if YR is Small then OS is No OS
    # 5. if YR is Medium then OS is Moderate OS
    # 6. if YR is Large then OS is Heavy OS

    # eval membership
    sa_level_low = skfuzzy.interp_membership(x_sa, sa_low, sa)
    sa_level_med = skfuzzy.interp_membership(x_sa, sa_med, sa)
    sa_level_high = skfuzzy.interp_membership(x_sa, sa_high, sa)

    la_level_low = skfuzzy.interp_membership(x_la, la_low, la)
    la_level_med = skfuzzy.interp_membership(x_la, la_med, la)
    la_level_high = skfuzzy.interp_membership(x_la, la_high, la)

    yr_level_low = skfuzzy.interp_membership(x_yr, yr_low, yr)
    yr_level_med = skfuzzy.interp_membership(x_yr, yr_med, yr)
    yr_level_high = skfuzzy.interp_membership(x_yr, yr_high, yr)

    # implement rules (and: fmin, or: fmax)
    rule1 = np.fmin(sa_level_low, la_level_low)
    os_rule1 = np.fmin(rule1, os_low)
    rule2 = np.fmin(sa_level_med, la_level_med)
    os_rule2 = np.fmin(rule2, os_med)
    rule3 = np.fmin(sa_level_high, la_level_high)
    os_rule3 = np.fmin(rule3, os_high)

    os_rule4 = np.fmin(yr_level_low, os_low)
    os_rule5 = np.fmin(yr_level_med, os_med)
    os_rule6 = np.fmin(yr_level_high, os_high)

    # agrregate
    os_aggregated = np.fmax(os_rule1, np.fmax(os_rule2, np.fmax(os_rule3,
        np.fmax(os_rule4, np.fmax(os_rule5, os_rule6)))))

    # defuzz to get the final oversteer level
    try:
        os_level = skfuzzy.defuzz(x_os, os_aggregated, "centroid")
    except AssertionError:
        # when total area is zero
        os_level = 0

    return os_level


def get_understeer_level(fd):
    # if fd <= 1.0:
        # return fd * 10
    # else:
        # return 10
    x_fd = np.arange(0, 1.1, 0.1) # 0 ~ 1
    x_us = np.arange(0, 11, 1) # 0 ~ 10

    fd_high = skfuzzy.trimf(x_fd, [0, 1, 1])
    us_high = skfuzzy.trimf(x_us, [0, 10, 10])

    fd_level_high = skfuzzy.interp_membership(x_fd, fd_high, fd)
    us_rule = np.fmin(fd_level_high, us_high)

    try:
        us_level = skfuzzy.defuzz(x_us, us_rule, "centroid")
    except AssertionError:
        us_level = 0

    return us_level

