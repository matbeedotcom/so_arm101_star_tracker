"""
SpeckleProcessor — speckle interferometry algorithms.

Implements Labeyrie's method (power spectrum averaging), autocorrelation,
bispectrum analysis, and phase recovery for diffraction-limited
reconstruction from short-exposure frames.

Works per-camera; multi-camera combination is handled at the
interferometry level (see reconstruction.py).
"""

import numpy as np
import scipy.fft as fft


class SpeckleProcessor:
    """Speckle interferometry processing for single-camera frame stacks."""

    def __init__(self, processing_config=None):
        """
        Args:
            processing_config: ProcessingConfig dataclass (optional).
        """
        if processing_config:
            self.image_size = processing_config.image_size
            self.pixel_size_um = processing_config.pixel_size_um
            self.wavelength_nm = processing_config.wavelength_nm
            self.max_triangles = processing_config.max_bispectrum_triangles
        else:
            self.image_size = 512
            self.pixel_size_um = 3.45
            self.wavelength_nm = 550.0
            self.max_triangles = 5000

    # ── Core algorithms ──

    def compute_power_spectrum(self, frames):
        """Average |FFT(frame)|² across all frames (Labeyrie's method).

        Args:
            frames: list of 2D numpy arrays (grayscale short-exposure images).

        Returns:
            2D numpy array — averaged power spectrum.
        """
        if not frames:
            raise ValueError("No frames provided")

        avg_ps = None
        for frame in frames:
            f = frame.astype(np.float64)
            ft = fft.fft2(f)
            ps = np.abs(ft) ** 2
            if avg_ps is None:
                avg_ps = ps
            else:
                avg_ps += ps

        avg_ps /= len(frames)
        return avg_ps

    def compute_autocorrelation(self, frames):
        """Compute average autocorrelation (IFFT of power spectrum).

        Args:
            frames: list of 2D numpy arrays.

        Returns:
            2D numpy array — average autocorrelation, centered.
        """
        avg_ps = self.compute_power_spectrum(frames)
        return self.compute_autocorrelation_from_ps(avg_ps)

    def compute_autocorrelation_from_ps(self, avg_ps):
        """Compute autocorrelation from a precomputed power spectrum.

        Args:
            avg_ps: 2D numpy array — averaged power spectrum.

        Returns:
            2D numpy array — autocorrelation, centered.
        """
        ac = np.real(fft.ifft2(avg_ps))
        return fft.fftshift(ac)

    def compute_bispectrum(self, frames):
        """Compute average bispectrum for phase recovery.

        Uses random UV triangle sampling for Pi5 performance.

        Args:
            frames: list of 2D numpy arrays.

        Returns:
            (bispectrum_avg, uv_triangles) where bispectrum_avg is a 1D
            complex array and uv_triangles is (N, 2, 2) array of
            (u1,v1), (u2,v2) pairs.
        """
        if not frames:
            raise ValueError("No frames provided")

        h, w = frames[0].shape[:2]

        # Generate random UV frequency triangles
        rng = np.random.default_rng(42)
        max_freq = min(h, w) // 2
        u1 = rng.integers(-max_freq, max_freq, size=self.max_triangles)
        v1 = rng.integers(-max_freq, max_freq, size=self.max_triangles)
        u2 = rng.integers(-max_freq, max_freq, size=self.max_triangles)
        v2 = rng.integers(-max_freq, max_freq, size=self.max_triangles)

        uv_triangles = np.stack([
            np.stack([u1, v1], axis=-1),
            np.stack([u2, v2], axis=-1),
        ], axis=1)  # (N, 2, 2)

        bispec_sum = np.zeros(self.max_triangles, dtype=np.complex128)

        for frame in frames:
            f = frame.astype(np.float64)
            ft = fft.fft2(f)

            # For each triangle: B(u1,v1,u2,v2) = F(u1,v1) * F(u2,v2) * conj(F(u1+u2, v1+v2))
            f_uv1 = ft[v1 % h, u1 % w]
            f_uv2 = ft[v2 % h, u2 % w]
            f_uv3 = ft[(v1 + v2) % h, (u1 + u2) % w]
            bispec_sum += f_uv1 * f_uv2 * np.conj(f_uv3)

        bispec_avg = bispec_sum / len(frames)
        return bispec_avg, uv_triangles

    def recover_phase_bispectrum(self, frames):
        """Recover Fourier phase from bispectrum (iterative unwrapping).

        Args:
            frames: list of 2D numpy arrays.

        Returns:
            2D numpy array — recovered phase map in Fourier space.
        """
        if not frames:
            raise ValueError("No frames provided")

        h, w = frames[0].shape[:2]
        bispec_avg, uv_triangles = self.compute_bispectrum(frames)

        # Extract bispectrum phases
        bispec_phase = np.angle(bispec_avg)  # (N,)
        u1 = uv_triangles[:, 0, 0]
        v1 = uv_triangles[:, 0, 1]
        u2 = uv_triangles[:, 1, 0]
        v2 = uv_triangles[:, 1, 1]

        # Iterative phase recovery: accumulate phase from bispectrum
        # phi(u1,v1) + phi(u2,v2) - phi(u1+u2, v1+v2) = bispec_phase
        phase = np.zeros((h, w), dtype=np.float64)
        weights = np.zeros((h, w), dtype=np.float64)

        # Sort triangles by distance from origin for better convergence
        dist = np.sqrt(u1**2 + v1**2) + np.sqrt(u2**2 + v2**2)
        order = np.argsort(dist)

        for idx in order:
            uu1, vv1 = int(u1[idx]) % w, int(v1[idx]) % h
            uu2, vv2 = int(u2[idx]) % w, int(v2[idx]) % h
            uu3 = int(u1[idx] + u2[idx]) % w
            vv3 = int(v1[idx] + v2[idx]) % h

            # If we have estimates for two vertices, update the third
            w1 = weights[vv1, uu1]
            w2 = weights[vv2, uu2]
            w3 = weights[vv3, uu3]

            bp = bispec_phase[idx]

            if w1 > 0 and w2 > 0 and w3 == 0:
                phase[vv3, uu3] = phase[vv1, uu1] + phase[vv2, uu2] - bp
                weights[vv3, uu3] = 1.0
            elif w1 > 0 and w3 > 0 and w2 == 0:
                phase[vv2, uu2] = phase[vv3, uu3] - phase[vv1, uu1] + bp
                weights[vv2, uu2] = 1.0
            elif w2 > 0 and w3 > 0 and w1 == 0:
                phase[vv1, uu1] = phase[vv3, uu3] - phase[vv2, uu2] + bp
                weights[vv1, uu1] = 1.0
            elif w1 == 0 and w2 == 0 and w3 == 0:
                # Seed: assume phase(u1,v1) = 0, derive the rest
                phase[vv1, uu1] = 0.0
                weights[vv1, uu1] = 1.0
                phase[vv2, uu2] = bp
                weights[vv2, uu2] = 1.0

        return phase

    def reconstruct_from_speckle(self, frames, precomputed_ps=None):
        """Full speckle reconstruction: power spectrum + phase recovery.

        Args:
            frames: list of 2D numpy arrays (short-exposure images).
            precomputed_ps: optional precomputed power spectrum to avoid
                recomputing it.

        Returns:
            2D numpy array — reconstructed image (real-valued).
        """
        avg_ps = precomputed_ps if precomputed_ps is not None else self.compute_power_spectrum(frames)
        amplitude = np.sqrt(avg_ps)

        phase = self.recover_phase_bispectrum(frames)

        # Combine amplitude and phase
        ft_reconstructed = amplitude * np.exp(1j * phase)
        reconstructed = np.real(fft.ifft2(ft_reconstructed))

        return reconstructed
