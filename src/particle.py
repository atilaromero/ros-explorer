import numpy as np
import scipy.stats

# Adapted from https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/12-Particle-Filters.ipynb

# create particles using gaussian distribution
def gaussian_particles(N, mean, std):
    samples = np.random.randn(N, len(mean))
    return samples * std + mean

# create particles using uniform distributions
def uniform_particles(N, low, high):
    samples = np.random.rand(N, len(low))
    return samples * (np.subtract(high,low)) + low

# predict particles expected new positions based on known movement
def predict(lastParticles, deltaMove, std):
    noise = np.random.randn(lastParticles.shape[0], len(deltaMove)) * std
    return lastParticles + deltaMove + noise

# estimate position using center of mass
def estimate(particles, weights=None):
    mean = np.average(particles, axis=0, weights=weights)
    std = np.average((particles-mean)**2, axis=0, weights=weights)**0.5
    return mean, std

# update particles weights
# weight = p(pos|sensor) = p(sensor|pos)p(pos)/p(sensor)
# p(sensor) = sum(p(sensor|pos)p(pos))
def updateWeights(scorefunc, particles, mean=None, std=None):
    if mean is None or std is None:
        mean, std = estimate(particles)
    zscore = (particles - mean)/std
    prior = np.prod(scipy.stats.norm.pdf(zscore), axis=1)
    likehood = scorefunc(particles)
    l_p = np.multiply(likehood, prior)
    weights = l_p/np.sum(l_p, axis=0)
    return weights

def simple_resample(particles, weights):
    N = weights.shape[0]
    cumulative_sum = np.cumsum(weights)
    indexes = np.searchsorted(cumulative_sum, np.random.random(N) * cumulative_sum[-1])
    # resample according to indexes
    return particles[indexes]

def neff(weights):
    return 1. / np.sum(np.square(weights))