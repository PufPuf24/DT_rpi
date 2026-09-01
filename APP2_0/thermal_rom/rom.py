"""Portable runtime for the linear thermal ROM.

The model is a modal (diagonal) state-space system with real, negative poles::

    dz[i,m]/dt = -z[i,m]/tau[m] + u[i]            i = input (cell heat source)
    T[j](t)    = T_ref + sum_{i,m} C[j,i,m] * z[i,m]

Every input column shares the same pole set ``tau`` -- physically correct: the
poles are the thermal eigenvalues of the pack, only the residues depend on
where the heat goes in and where the temperature is read out. The state count
is therefore ``n_poles * n_inputs`` instead of the ``order * n_in * n_out``
that a per-pair fit (what Twin Builder writes into its FMU) produces.

Pure NumPy: identical results on Windows, Linux, macOS, x86-64 and ARM.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

__all__ = ["ThermalROM"]


def _phi1(x):
    """(1 - exp(-x)) / x, accurate as x -> 0."""
    x = np.asarray(x, dtype=float)
    out = np.empty_like(x)
    small = x < 1e-4
    xs = x[small]
    out[small] = 1.0 - xs / 2.0 + xs**2 / 6.0
    xl = x[~small]
    out[~small] = -np.expm1(-xl) / xl
    return out


def _phi2(x):
    """(x - 1 + exp(-x)) / x**2, accurate as x -> 0."""
    x = np.asarray(x, dtype=float)
    out = np.empty_like(x)
    small = x < 1e-3
    xs = x[small]
    out[small] = 0.5 - xs / 6.0 + xs**2 / 24.0
    xl = x[~small]
    out[~small] = (xl - 1.0 + np.exp(-xl)) / xl**2
    return out


def _psi(x):
    """(1 - exp(-x) - x*exp(-x)) / x**2, accurate as x -> 0."""
    x = np.asarray(x, dtype=float)
    out = np.empty_like(x)
    small = x < 1e-3
    xs = x[small]
    out[small] = 0.5 - xs / 3.0 + xs**2 / 8.0
    xl = x[~small]
    e = np.exp(-xl)
    out[~small] = (1.0 - e - xl * e) / xl**2
    return out


class ThermalROM:
    """Reduced-order thermal model of a battery pack.

    Parameters
    ----------
    tau : (p,) array
        Time constants [s] of the shared pole set.
    C : (n_out, n_in, p) array
        Output residues [K/(W*s)] -- ``T = T_ref + einsum('jim,im->j', C, z)``.
    T_ref : float
        Reference (ambient / initial) temperature of the identification run [K].
    """

    def __init__(self, tau, C, T_ref=300.0, input_names=None, output_names=None, meta=None):
        self.tau = np.asarray(tau, dtype=float).ravel()
        self.C = np.asarray(C, dtype=float)
        self.T_ref = float(T_ref)
        self.n_out, self.n_in, self.n_poles = self.C.shape
        if self.tau.size != self.n_poles:
            raise ValueError("tau and C disagree on the number of poles")
        self.input_names = list(input_names) if input_names is not None else [
            "u%d" % (i + 1) for i in range(self.n_in)]
        self.output_names = list(output_names) if output_names is not None else [
            "T%d" % (j + 1) for j in range(self.n_out)]
        self.meta = dict(meta or {})
        self._lam = 1.0 / self.tau
        self.reset()

    # ------------------------------------------------------------------ info
    @property
    def n_states(self):
        return self.n_in * self.n_poles

    @property
    def dc_gain(self):
        """(n_out, n_in) steady-state temperature rise per watt [K/W]."""
        return np.einsum("jim,m->ji", self.C, self.tau)

    def steady_state(self, u, T_amb=None):
        """Equilibrium temperatures [K] for a constant heat input vector `u` [W]."""
        u = np.broadcast_to(np.asarray(u, dtype=float), (self.n_in,))
        return (self.T_ref if T_amb is None else T_amb) + self.dc_gain @ u

    def state_space(self):
        """Return dense ``(A, B, C, D)`` for use with control / scipy tooling."""
        n = self.n_states
        A = np.diag(np.tile(-self._lam, self.n_in))
        B = np.zeros((n, self.n_in))
        for i in range(self.n_in):
            B[i * self.n_poles:(i + 1) * self.n_poles, i] = 1.0
        C = self.C.reshape(self.n_out, n)
        return A, B, C, np.zeros((self.n_out, self.n_in))

    def __repr__(self):
        return ("<ThermalROM %d in x %d out, %d poles (%d states), tau %.3g..%.3g s>"
                % (self.n_in, self.n_out, self.n_poles, self.n_states,
                   self.tau.min(), self.tau.max()))

    # ----------------------------------------------------------- persistence
    def save(self, path):
        path = Path(path)
        np.savez_compressed(path, tau=self.tau, C=self.C, T_ref=self.T_ref,
                            input_names=np.array(self.input_names),
                            output_names=np.array(self.output_names),
                            meta=json.dumps(self.meta))
        return path

    @classmethod
    def load(cls, path):
        d = np.load(Path(path), allow_pickle=False)
        return cls(tau=d["tau"], C=d["C"], T_ref=float(d["T_ref"]),
                   input_names=[str(s) for s in d["input_names"]],
                   output_names=[str(s) for s in d["output_names"]],
                   meta=json.loads(str(d["meta"])) if "meta" in d.files else {})

    # -------------------------------------------------------------- stepping
    def reset(self, T_amb=None):
        """Zero the states; optionally move the ambient/initial temperature.

        The ROM is linear in the heat inputs, so shifting the ambient level is a
        pure output offset -- valid as long as the CFD boundary condition
        shifts with it (uniform ambient).
        """
        self.z = np.zeros((self.n_in, self.n_poles))
        if T_amb is not None:
            self.T_ref = float(T_amb)
        self._dt_cache = None
        return self.outputs()

    def outputs(self):
        """Current output temperatures [K]."""
        return self.T_ref + np.einsum("jim,im->j", self.C, self.z)

    def _coeffs(self, dt, foh):
        key = (dt, foh)
        if self._dt_cache is not None and self._dt_cache[0] == key:
            return self._dt_cache[1]
        x = self._lam * dt
        E = np.exp(-x)
        if foh:
            c = (E, dt * _psi(x), dt * _phi2(x))       # z+ = E z + a*u0 + b*u1
        else:
            c = (E, dt * _phi1(x), np.zeros_like(E))   # zero-order hold
        self._dt_cache = (key, c)
        return c

    def step(self, u, dt, u_next=None):
        """Advance by `dt` seconds with heat input `u` [W] and return T [K].

        `u` may be a scalar (same heat in every cell) or an (n_in,) vector.
        Pass `u_next` to interpolate the input linearly over the interval
        (first-order hold) instead of holding it constant.
        """
        u = np.broadcast_to(np.asarray(u, dtype=float), (self.n_in,))
        E, a, b = self._coeffs(float(dt), u_next is not None)
        self.z = self.z * E + np.outer(u, a)
        if u_next is not None:
            self.z += np.outer(np.broadcast_to(np.asarray(u_next, dtype=float), (self.n_in,)), b)
        return self.outputs()

    # ------------------------------------------------------------- simulation
    def _states(self, t, u, hold="foh", z0=None):
        t = np.asarray(t, dtype=float).ravel()
        nt = t.size
        u = np.asarray(u, dtype=float)
        if u.ndim == 0:
            U = np.full((nt, self.n_in), float(u))
        elif u.ndim == 1 and u.size == nt:
            U = np.repeat(u[:, None], self.n_in, axis=1)     # one history, all cells
        elif u.ndim == 1 and u.size == self.n_in:
            U = np.repeat(u[None, :], nt, axis=0)            # constant per-cell input
        else:
            U = np.ascontiguousarray(np.broadcast_to(u, (nt, self.n_in)))
        foh = hold == "foh"

        dt = np.diff(t)
        if dt.size and (dt <= 0).any():
            raise ValueError("t must be strictly increasing")
        uniform = dt.size > 0 and np.ptp(dt) <= 1e-9 * max(1.0, float(dt[0]))

        Z = np.empty((nt, self.n_in, self.n_poles))
        Z[0] = 0.0 if z0 is None else z0
        if uniform and z0 is None:
            self._filter_uniform(Z, U, float(dt[0]), foh)
        else:
            z = Z[0].copy()
            for k in range(nt - 1):
                E, a, b = self._coeffs(float(dt[k]), foh)
                z = z * E + np.outer(U[k], a)
                if foh:
                    z = z + np.outer(U[k + 1], b)
                Z[k + 1] = z
        return Z

    def _filter_uniform(self, Z, U, dt, foh):
        from scipy.signal import lfilter

        E, a, b = self._coeffs(dt, foh)
        for m in range(self.n_poles):
            num = [b[m], a[m]] if foh else [0.0, a[m]]
            Z[:, :, m] = lfilter(np.asarray(num), np.array([1.0, -E[m]]), U, axis=0)

    def temperatures(self, t, u, T_amb=None, hold="foh", z0=None):
        """Simulate an input history and return (nt, n_out) temperatures [K].

        Parameters
        ----------
        t : (nt,) array
            Time stamps [s], increasing. Need not be uniformly spaced.
        u : (nt,) or (nt, n_in) or (n_in,) or scalar
            Heat input [W]. A 1-D (nt,) history is broadcast to all inputs --
            the usual case when every cell carries the same load.
        T_amb : float, optional
            Ambient / initial temperature [K]; defaults to the identification
            reference temperature.
        hold : {'foh', 'zoh'}
            Input reconstruction between samples. 'foh' (piecewise linear) is
            exact for sampled continuous signals and much more accurate on
            coarse grids; 'zoh' matches a real-time sample-and-hold controller.
        """
        Z = self._states(t, u, hold=hold, z0=z0)
        T0 = self.T_ref if T_amb is None else T_amb
        return T0 + np.einsum("jim,tim->tj", self.C, Z)

    def temperatures_with_state(self, t, u, T_amb=None, hold="foh", z0=None):
        """Like `temperatures`, but also returns the final internal state
        `z[-1]` (n_in, n_poles) -- e.g. to make a batch run's end state
        available for the live model to continue from afterwards."""
        Z = self._states(t, u, hold=hold, z0=z0)
        T0 = self.T_ref if T_amb is None else T_amb
        temps = T0 + np.einsum("jim,tim->tj", self.C, Z)
        return temps, Z[-1]

    def step_response(self, t, amplitude=1.0):
        """(n_in, nt, n_out) step response of the model, for validation."""
        t = np.asarray(t, dtype=float).ravel()
        # s_ij(t) = sum_m C[j,i,m] * tau[m] * (1 - exp(-t/tau[m]))
        basis = -np.expm1(-t[:, None] * self._lam[None, :])
        return amplitude * np.einsum("jim,m,tm->itj", self.C, self.tau, basis)
