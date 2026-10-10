"""Effective sample size and systematic resampling.

Systematic resampling (stochastic universal sampling (SUS)) draws a single uniform and spaces the remaining N-1
positions deterministically. That gives lower variance than drawing N
independent uniforms, and it guarantees a particle holding weight w is
selected either floor(N*w) or ceil(N*w) times.
"""
import numpy as np


def effective_sample_size(weights) -> float:
    """N_eff = 1 / sum(w^2), for NORMALIZED weights.

    Args:
        weights: (N,) array of normalized particle weights.
    Returns:
        float: the effective sample size, between 1 and N.

    TODO:
        Compute the effective sample size based on the current particle weights.
    """
    w = np.asarray(weights, dtype=float)
    return float(1.0 / np.sum(w ** 2))


def systematic_resample(weights, rng) -> np.ndarray:
    """Systematic resampling of particles based on their weights.

    Args:
        weights: (N,) array of normalized particle weights.
        rng: a random number generator with a `random()` method.
    Returns:
        np.ndarray: (N,) array of ancestor indices.

    TODO:
        Implement systematic resampling based on the particle weights.
        Stochastic universal sampling (SUS) is used, where a single uniform
        is drawn and the remaining N-1 positions are spaced deterministically.
    """
    w = np.asarray(weights, dtype=float)
    n = len(w)

    ## N positions, 1/N apart, all shifted by the same uniform draw.
    positions = (rng.random() + np.arange(n)) / n

    cumulative = np.cumsum(w)
    ## Rounding can leave the last entry a hair under 1.0, and then the last
    ## position would fall off the end of the array.
    cumulative[-1] = 1.0

    ## side='right': a position that lands exactly on a boundary goes to the
    ## next particle, so a particle with weight 0 is never picked.
    return np.searchsorted(cumulative, positions, side='right')
