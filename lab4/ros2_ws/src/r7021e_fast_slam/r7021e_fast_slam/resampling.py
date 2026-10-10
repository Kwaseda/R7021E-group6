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
    raise NotImplementedError


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
    raise NotImplementedError
