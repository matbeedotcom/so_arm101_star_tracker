"""
Thin wrappers around telescope project reconstruction modules.

Lazy-imports from /home/acidhax/dev/telescope/src/ so this module
doesn't blow up if the telescope repo isn't present.
"""

import sys

_TELESCOPE_SRC = '/home/acidhax/dev/telescope/src'


def _ensure_path():
    if _TELESCOPE_SRC not in sys.path:
        sys.path.insert(0, _TELESCOPE_SRC)


def get_clean_reconstructor(**kwargs):
    """Return a CLEANReconstructor instance from the telescope project."""
    _ensure_path()
    from reconstruction.clean_algorithm import CLEANReconstructor
    return CLEANReconstructor(**kwargs)


def get_deconvolution(**kwargs):
    """Return a DeconvolutionProcessor instance."""
    _ensure_path()
    from reconstruction.deconvolution import DeconvolutionProcessor
    return DeconvolutionProcessor(**kwargs)


def get_super_resolution(**kwargs):
    """Return a SuperResolutionProcessor instance."""
    _ensure_path()
    from reconstruction.super_resolution import SuperResolutionProcessor
    return SuperResolutionProcessor(**kwargs)


def get_interferometry_processor(**kwargs):
    """Return an InterferometryProcessor instance."""
    _ensure_path()
    from interferometry.interferometry_processor import InterferometryProcessor
    return InterferometryProcessor(**kwargs)


def reconstruct(image, method='wiener', **kwargs):
    """Convenience function: apply a reconstruction method to an image.

    Args:
        image: 2D numpy array.
        method: One of 'wiener', 'richardson_lucy', 'clean'.
        **kwargs: Passed to the underlying processor.

    Returns:
        Reconstructed 2D numpy array.
    """
    if method in ('wiener', 'richardson_lucy'):
        proc = get_deconvolution(**{k: v for k, v in kwargs.items()
                                    if k in ('regularization', 'iterations')})
        # Estimate PSF from image
        psf = proc._estimate_psf_from_image(image, kwargs.get('psf_size', 15))
        if method == 'wiener':
            return proc.wiener_deconvolution(image, psf,
                                             kwargs.get('noise_power', 0.01))
        else:
            return proc.richardson_lucy_deconvolution(image, psf,
                                                      kwargs.get('iterations', 50))
    elif method == 'clean':
        clean = get_clean_reconstructor(**{k: v for k, v in kwargs.items()
                                           if k in ('loop_gain', 'threshold',
                                                     'max_iterations')})
        # CLEAN expects visibilities — for single-image use, treat as dirty image
        # with a point-source dirty beam
        import numpy as np
        beam = np.zeros_like(image)
        beam[image.shape[0] // 2, image.shape[1] // 2] = 1.0
        components, residual = clean.hogbom_clean(image, beam)
        return clean.restore_image()
    else:
        raise ValueError(f"Unknown reconstruction method: {method}")
