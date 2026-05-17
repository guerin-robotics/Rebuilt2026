// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision.io;

import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * Vision IO implementation that moves {@code camera.getAllUnreadResults()} off the main robot
 * thread. The parent class's implementation calls that method synchronously inside {@code
 * updateInputs()}, which blocks on a NetworkTables read from the coprocessor and has been observed
 * to spike to 280 ms when multiple cameras are read in series (see match log analysis 2026-05-02
 * Archimedes E5).
 *
 * <p>A dedicated daemon thread polls each camera at ~50 Hz and enqueues raw {@link
 * PhotonPipelineResult}s. {@code updateInputs()} then drains that queue — no network IO on the main
 * thread.
 */
public class VisionIOPhotonVisionThread extends VisionIOPhotonVision {
  private final BlockingQueue<PhotonPipelineResult> resultQueue = new ArrayBlockingQueue<>(50);
  private final Thread thread;

  public VisionIOPhotonVisionThread(String name, Transform3d robotToCamera) {
    super(name, robotToCamera);
    thread = new Thread(this::pollCamera, "VisionIO-" + name);
    thread.setDaemon(true);
    thread.start();
  }

  private void pollCamera() {
    while (!Thread.currentThread().isInterrupted()) {
      try {
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
          // If queue is full, remove oldest frame and retry to ensure we always keep the
          // freshest frames. This prevents a slow main thread from getting stale vision data.
          if (!resultQueue.offer(result)) {
            resultQueue.poll(); // Remove oldest
            resultQueue.offer(result); // Add newest (guaranteed to succeed now)
          }
        }
        Thread.sleep(20); // 50 Hz — fast enough to not miss 25–30 fps camera frames
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
      }
    }
  }

  @Override
  protected List<PhotonPipelineResult> getAllResults() {
    List<PhotonPipelineResult> results = new ArrayList<>();
    resultQueue.drainTo(results);
    return results;
  }
}
