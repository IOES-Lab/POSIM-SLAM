// SPDX-License-Identifier: Apache-2.0
// Diagnostic-only export, linked into the owned probe, never the installed SDK.
#include "sonar_compute_wgpu.hh"
#include "sonar_native_trace.hh"
#include <exception>

extern "C" __attribute__((visibility("default"))) int posim_sonar_trace_replay(
  const float * depth, const float * normal, const float * mixing, const float * window,
  double hfov, double vfov, float alpha, uint64_t frame, uint64_t seed, int64_t stamp,
  float * result)
{
  try
  {
    constexpr int beams = 256, rays = 256, frequencies = 599;
    cv::Mat d(rays, beams, CV_32FC1), n(rays, beams, CV_32FC3);
    cv::Mat rho(rays, beams, CV_32FC1, cv::Scalar(.001f));
    for (int b = 0; b < beams; ++b)
      for (int r = 0; r < rays; ++r)
      {
        const auto index = b * rays + r;
        d.at<float>(r, b) = depth[index];
        n.at<cv::Vec3f>(r, b) = {normal[index * 3], normal[index * 3 + 1], normal[index * 3 + 2]};
      }
    std::vector<float *> rows(beams);
    for (int b = 0; b < beams; ++b) rows[b] = const_cast<float *>(mixing + b * beams);
    gz::sensors::SonarComputeInput input;
    input.nBeams = beams; input.nRays = rays; input.nFreq = frequencies;
    input.depthImage = &d; input.normalImage = &n; input.reflectivityImage = &rho;
    input.beamCorrector = rows.data(); input.window = window; input.beamCorrectorSum = 1;
    input.hFOV = hfov; input.vFOV = vfov; input.soundSpeed = 1500; input.maxDistance = 15;
    input.sourceLevel = 220; input.attenuation = alpha; input.bandwidth = 29900;
    input.sensorGain = 1; input.raySkips = 1; input.frameIndex = frame; input.seed = seed;
    gz::sensors::WgpuComputeBackend backend;
    if (!backend.Initialize(input)) return 1;
    gz::sensors::native_trace::Frame context;
    context.valid = true; context.stampNs = stamp; context.hasBegin = true;
    context.beginStampNs = stamp; context.contextSource = "offline_supplied_grid";
    context.world = context.beginWorld = {1, 2, -2, 1, 0, 0, 0};
    context.local = {0, 0, 0, 1, 0, 0, 0};
    context.sensorName = "offline-replay";
    gz::sensors::native_trace::Scope scope(context);
    gz::sensors::SonarComputeOutput output;
    if (!backend.Compute(input, output)) return 2;
    for (size_t i = 0; i < output.beamSpectrum.size(); ++i)
    {
      result[2 * i] = output.beamSpectrum[i].real();
      result[2 * i + 1] = output.beamSpectrum[i].imag();
    }
    return 0;
  }
  catch (const std::exception &) { return 3; }
}
