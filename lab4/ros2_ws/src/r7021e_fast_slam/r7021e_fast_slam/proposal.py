"""The improved proposal distribution of Grid-FastSLAM 2.0.

The Gaussian approximation derived in hello-slam's `4_grid_based_slam.ipynb`:
sample K poses around the scan match, weight each by

    tau_j = p(z_t | x_j, m_t-1) * p(x_j | x_t-1, u_t),

and fit a Gaussian to the weighted cloud,

    mu = (1/eta) sum_j x_j tau_j,   Sigma = (1/eta) sum_j (x_j - mu)(x_j - mu)^T tau_j.

The normalizer eta = sum_j tau_j of that same product is the particle's
importance weight, which is why it is returned alongside the moments.

Everything is in log space. The raw products underflow within a handful of
beams.
"""
import numpy as np

from .measurement_model import measurement_log_likelihood
from .motion_model import motion_model_log_pdf
from .utils import logsumexp, wrap_angle

## Regularization of the fitted covariance. With all the tau mass on a single
## candidate the scatter matrix is exactly singular and the sampler's Cholesky
## would fail. SIGMA_REG is an additive variance floor per axis (x, y, theta);
## SIGMA_EIG_MIN floors the eigenvalues afterwards.
SIGMA_REG = (1.0e-4, 1.0e-4, 1.0e-5)
SIGMA_EIG_MIN = 1.0e-6


def proposal_candidates(x_star, cfg) -> np.ndarray:
    """K candidate poses around the scan-matched pose.

    A regular lattice: it covers the small search window evenly. 
    K is rounded down to a perfect cube, e.g. 27 gives 3 x 3 x 3.

    Args:
        x_star:     (3,) [x, y, theta] the scan-matched pose at the current time step.
        cfg:        configuration object with proposal parameters.

    Returns:
        (K, 3) numpy array of candidate poses, where K is the number of candidates.

    TODO:
        Implement the generation of candidate poses around the scan-matched pose `x_star`
        using a regular lattice within the specified proposal window.
    """
    x_star = np.asarray(x_star, dtype=float)

    ## K rounded down to a perfect cube: side ** 3 <= K. The integer loops
    ## guard against cube roots like 26.999999... rounding the wrong way.
    k = int(cfg.num_candidates)
    side = max(int(round(k ** (1.0 / 3.0))), 1)
    while side ** 3 > k:
        side -= 1
    while (side + 1) ** 3 <= k:
        side += 1

    ## The window is the FULL width of the lattice: offsets run from -w/2 to
    ## +w/2. (The scan matcher's windows are half-widths, +-w.)
    if side == 1:
        d_xy = d_th = np.zeros(1)          # linspace(.., 1) would give -w/2
    else:
        d_xy = np.linspace(-0.5 * cfg.proposal_window_xy,
                           0.5 * cfg.proposal_window_xy, side)
        d_th = np.linspace(-0.5 * cfg.proposal_window_theta,
                           0.5 * cfg.proposal_window_theta, side)

    dx, dy, dth = np.meshgrid(d_xy, d_xy, d_th, indexing='ij')
    return np.stack([x_star[0] + dx.ravel(),
                     x_star[1] + dy.ravel(),
                     wrap_angle(x_star[2] + dth.ravel())], axis=1)


def improved_proposal(x_star, x_prev, u, endpoints, grid_map, cfg):
    """Return (log_eta, mu, Sigma) for one particle.

    Args:
        x_star:     (3,) [x, y, theta] the scan-matched pose at the current time step.
        x_prev:     (3,) [x, y, theta] the previous pose of the particle.
        u:          (3,) [rot1, trans, rot2] the odometry increment.
        endpoints:  (N, 2) array of laser scan endpoints in the robot frame.
        grid_map:   occupancy grid map.
        cfg:        configuration object with proposal parameters.
    
    Returns:
        log_eta:    float, the importance weight factor for the particle.
        mu:        (3,) numpy array, the mean of the candidate poses.
        Sigma:     (3, 3) numpy array, the covariance of the candidate poses.

    log_eta is log sum_j tau_j -- the importance weight factor the filter
    applies to the particle. It is the sum, NOT the likelihood at the sampled
    pose: it approximates the integral p(z | x_prev, m, u), which is what the
    weight is supposed to be.

    TODO: 
        Use `proposal_candidates` to generate candidate poses around `x_star`, 
        then compute the importance weights `log_tau` for each candidate using
        the measurement likelihood and motion model. 
        Normalize the weights to get `w`, and use them to compute the weighted 
        mean `mu` and covariance `Sigma`.
    """
    cands = proposal_candidates(x_star, cfg)                          # (K, 3)

    ## tau_j = p(z | x_j, m) * p(x_j | x_prev, u), in log space.
    log_tau = (measurement_log_likelihood(cands, endpoints, grid_map, cfg)
               + motion_model_log_pdf(cands, x_prev, u, cfg.odometry_sigmas))

    ## eta = sum_j tau_j. This is the particle's weight factor.
    log_eta = logsumexp(log_tau)
    w = np.exp(log_tau - log_eta)                                     # sums to 1

    ## Weighted mean. The heading is an angle, so average its sin and cos
    ## instead of the raw numbers (-pi and +pi are the same direction).
    mu = np.empty(3)
    mu[:2] = w @ cands[:, :2]
    mu[2] = np.arctan2(w @ np.sin(cands[:, 2]), w @ np.cos(cands[:, 2]))

    ## Weighted scatter around mu, with the heading difference wrapped.
    d = cands - mu
    d[:, 2] = wrap_angle(d[:, 2])
    sigma = (d * w[:, None]).T @ d

    ## Keep Sigma usable as a covariance: add the variance floor, lift any
    ## eigenvalue that is still too small, and make the result exactly symmetric.
    sigma += np.diag(SIGMA_REG)
    vals, vecs = np.linalg.eigh(sigma)
    sigma = (vecs * np.maximum(vals, SIGMA_EIG_MIN)) @ vecs.T
    sigma = 0.5 * (sigma + sigma.T)

    return float(log_eta), mu, sigma
