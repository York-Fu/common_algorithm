#!/usr/bin/env python

import random
import numpy as np
import matplotlib.pyplot as plt
import scipy.signal
import scipy.optimize


def lowpass_filter(cutoff, sampling, data, N=6):
    Wn = (2.0 * cutoff) / sampling
    b, a = scipy.signal.butter(N, Wn, 'lowpass')
    return scipy.signal.filtfilt(b, a, np.array(data))


def linear_fitting(x, y):
    def func(p, x):
        a, b = p
        return a*x+b

    def error(p, x, y):
        return func(p, x)-y

    p0 = [1, 1]
    para = scipy.optimize.leastsq(error, p0, args=(x, y))
    return para


def quadratic_fitting(x, y):
    def func(p, x):
        a, b, c = p
        return a*(x**2)+b*x+c

    def error(p, x, y):
        return func(p, x)-y

    p0 = [1, 1, 1]
    para = scipy.optimize.leastsq(error, p0, args=(x, y))
    return para


def plot_scatter(x, y, label):
    plt.title('plot')
    plt.xlabel("x")
    plt.ylabel("y")
    plt.grid(ls='--')
    for i, v in enumerate(y):
        plt.scatter(x, v, label='{}'.format(label[i]))
    plt.legend(loc='upper right')


def plot_line(x, y, label):
    plt.title('plot')
    plt.xlabel("x")
    plt.ylabel("y")
    plt.grid(ls='--')
    for i, v in enumerate(y):
        plt.plot(x, v, label='{}'.format(label[i]))
    plt.legend(loc='upper right')


if __name__ == '__main__':
    T = 10.0
    N = 100
    x = np.linspace(0, T-1, N)
    print(x)
    noise = np.random.uniform(-2, 2, (1, N))[0]

    y0 = (2*x+3) + noise
    para = linear_fitting(x, y0)
    print(para)
    a, b = para[0][0], para[0][1]
    y0_fit = a*x+b

    y1 = (0.5*(x**2)+2*x+3) + noise
    para = quadratic_fitting(x, y1)
    print(para)
    a, b, c = para[0][0], para[0][1], para[0][2]
    y1_fit = a*(x**2)+b*x+c

    y1_filter = lowpass_filter(N/10.0, N*2.0, y1)

    plot_scatter(x, [y0, y1, y1_filter], ['y0', 'y1', 'y1_filter'])
    plot_line(x, [y0_fit, y1_fit], ['y0_fit', 'y1_fit'])
    plt.show()
