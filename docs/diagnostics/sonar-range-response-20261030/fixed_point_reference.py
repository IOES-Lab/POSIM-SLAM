# SPDX-License-Identifier: Apache-2.0
"""Numerical reference for POSIM WGPU Philox and spectral rounding.

The equations follow POSIM 7afd71e7 backscatter.wgsl, which attributes
Random123 / Salmon et al., SC 2011. No renderer or application imports.
"""
import numpy as np


def philox(seed, frame, subsequences):
    indices = np.asarray(subsequences, dtype=np.uint32)
    counter = np.zeros((indices.size, 4), dtype=np.uint32)
    counter[:, 0] = frame & 0xffffffff
    counter[:, 1] = frame >> 32
    counter[:, 2] = indices.ravel()
    key0, key1 = seed & 0xffffffff, seed >> 32
    for _ in range(10):
        r0 = counter[:, 0].astype(np.uint64) * np.uint64(0xD2511F53)
        r1 = counter[:, 2].astype(np.uint64) * np.uint64(0xCD9E8D57)
        counter = np.column_stack([
            (r1 >> np.uint64(32)).astype(np.uint32) ^ counter[:, 1] ^ np.uint32(key0),
            r1.astype(np.uint32),
            (r0 >> np.uint64(32)).astype(np.uint32) ^ counter[:, 3] ^ np.uint32(key1),
            r0.astype(np.uint32),
        ])
        key0 = (key0 + 0x9E3779B9) & 0xffffffff
        key1 = (key1 + 0xBB67AE85) & 0xffffffff
    u = counter.astype(np.float32) * np.float32(1 / 4294967296.) + np.float32(.5 / 4294967296.)
    radius = np.sqrt(np.float32(-2) * np.log(u[:, 0]))
    phase = np.float32(2) * np.float32(np.pi) * u[:, 1]
    return radius * np.sin(phase), radius * np.cos(phase)


def source_spectrum(depth, amplitude, n, bandwidth=29900., speed=1500.):
    f = np.arange(n, dtype=np.float32)
    frequencies = np.float32(bandwidth / n) * (
        np.float32(-n if n % 2 == 0 else -(n - 1)) + np.float32(2) * (f + np.float32(1))
    ) / np.float32(2)
    wave = np.float32(2) * np.float32(np.pi) * frequencies / np.float32(speed)
    phase = np.float32(2) * np.asarray(depth, dtype=np.float32)[:, None] * wave
    window = .54 - .46 * np.cos(2 * np.pi * (np.arange(n) + 1) / n)
    window = (window / np.linalg.norm(window)).astype(np.float32)
    values = (amplitude.real[:, None] * np.cos(phase) - amplitude.imag[:, None] * np.sin(phase)) * window
    imaginary = (amplitude.real[:, None] * np.sin(phase) + amplitude.imag[:, None] * np.cos(phase)) * window
    return values + 1j * imaginary
