"""Likelihood field (endpoint) measurement model.

Scores a batch of candidate poses against ONE particle's map. Called N times
per filter step, once per particle, with K candidates each time -- so the K
dimension must be vectorised, never looped.

This is a proper likelihood, not a correlation. Summing occupancy
probabilities at the beam endpoints would be an unnormalized correlation
score; in Grid-FastSLAM 2.0 that quantity enters tau, so it would corrupt the
proposal mean and covariance, not merely the weights.
"""
import numpy as np

from .utils import transform_points


def measurement_log_likelihood(poses, endpoints, grid_map, cfg) -> np.ndarray:
    """log p(z | x, m) for each of K candidate poses.

    Args:
        poses:     (3,) or (K, 3) in the map frame.
        endpoints: (B, 2) beam endpoints in the robot base frame.
        grid_map:  the particle's GridMap.
        cfg:       Config; uses z_hit, z_rand, sigma_hit, max_dist, occ_thresh
                and range_max.

    Returns:
        (K,) array of log-likelihoods for each candidate pose.
    """
    ## (K, B, 2): every beam endpoint of every candidate pose, in the map frame.
    world = transform_points(poses, endpoints)
    cells = grid_map.world_to_grid(world)
    inb = grid_map.in_bounds(cells)

    ## Distance from each cell to the nearest occupied cell, clipped at max_dist.
    field = grid_map.likelihood_field(cfg.occ_thresh, cfg.max_dist)

    ## A beam that lands outside the map gets the largest distance the field
    ## can hold, so it is penalised and never indexes out of the array.
    dist = np.full(inb.shape, cfg.max_dist, dtype=float)
    gx, gy = cells[..., 0], cells[..., 1]
    dist[inb] = field[gy[inb], gx[inb]]

    ## Gaussian around an occupied cell, plus a flat floor for random returns.
    p_beam = (cfg.z_hit * np.exp(-0.5 * (dist / cfg.sigma_hit) ** 2)
              + cfg.z_rand / cfg.range_max)

    ## Beams are independent, so the log-likelihood is the sum of the logs.
    return np.log(p_beam).sum(axis=1)
