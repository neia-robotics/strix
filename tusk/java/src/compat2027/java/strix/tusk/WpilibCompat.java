// Copyright (c) 2026 Strix contributors (NEIA Robotics 9644, Choate Robotics 7407)
// Use of this source code is governed by the MIT license in the LICENSE file at the
// root of this repository.

package strix.tusk;

import org.wpilib.networktables.PubSubOption;

/** The few WPILib calls that differ between seasons by more than a package name (2027). */
final class WpilibCompat {
  private WpilibCompat() {}

  /** Subscriber options that queue every value (no dropped frames between polls). */
  static PubSubOption[] queueOptions(int pollStorage) {
    return new PubSubOption[] {
      PubSubOption.SEND_ALL, PubSubOption.KEEP_DUPLICATES, PubSubOption.pollStorage(pollStorage)
    };
  }
}
