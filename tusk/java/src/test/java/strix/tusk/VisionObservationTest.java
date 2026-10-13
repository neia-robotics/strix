// Copyright (c) 2026 Strix contributors (NEIA Robotics 9644, Choate Robotics 7407)
// Use of this source code is governed by the MIT license in the LICENSE file at the
// root of this repository.

package strix.tusk;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HexFormat;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.junit.jupiter.api.Test;

/**
 * Decodes the bytes RobotPy produced (tusk/testdata/observation_v3.json, written by
 * tusk/robotpy/tests/test_vision_types.py), so Java and Python can't drift apart.
 */
class VisionObservationTest {
  private static final Path FIXTURE = Path.of("..", "testdata", "observation_v3.json");

  private static String fixture() throws IOException {
    return Files.readString(FIXTURE);
  }

  private static String stringField(String json, String name) {
    Matcher m = Pattern.compile("\"" + name + "\":\\s*\"([^\"]*)\"").matcher(json);
    if (!m.find()) {
      throw new AssertionError("fixture has no " + name);
    }
    return m.group(1);
  }

  private static double number(String json, String name) {
    Matcher m = Pattern.compile("\"" + name + "\":\\s*(-?[0-9.eE+-]+)").matcher(json);
    if (!m.find()) {
      throw new AssertionError("fixture has no " + name);
    }
    return Double.parseDouble(m.group(1));
  }

  private static ByteBuffer fixtureBytes(String json) {
    byte[] bytes = HexFormat.of().parseHex(stringField(json, "hex"));
    return ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN);
  }

  @Test
  void schemaMatchesRobotPy() throws IOException {
    String json = fixture();
    assertEquals(stringField(json, "typeName"), VisionObservation.struct.getTypeName());
    assertEquals(stringField(json, "schema"), VisionObservation.struct.getSchema());
    assertEquals((int) number(json, "size"), VisionObservation.struct.getSize());
  }

  @Test
  void decodesRobotPyBytes() throws IOException {
    String json = fixture();
    VisionObservation obs = VisionObservation.struct.unpack(fixtureBytes(json));

    assertEquals(1, obs.cameraId());
    assertEquals(1.0, obs.robotPose().getX(), 1e-12);
    assertEquals(2.0, obs.robotPose().getY(), 1e-12);
    assertEquals(3.0, obs.robotPose().getZ(), 1e-12);
    assertEquals(0.25, obs.altRobotPose().getZ(), 1e-12);
    assertEquals(
        number(json, "qz"), obs.robotPose().getRotation().getQuaternion().getZ(), 1e-12);
    assertEquals(0.05, obs.ageSeconds(), 1e-12);
    assertEquals(0.7, obs.stdDevFactor(), 1e-12);
    assertEquals(2, obs.tagCount());
    assertEquals(3.5, obs.avgTagDistance(), 1e-12);
    assertEquals(0.0, obs.ambiguity(), 1e-12);
    assertEquals(List.of(5, 10), obs.tagIds());
  }

  @Test
  void encodesSameBytesAsRobotPy() throws IOException {
    ByteBuffer in = fixtureBytes(fixture());
    byte[] expected = in.array().clone();
    VisionObservation obs = VisionObservation.struct.unpack(in);
    ByteBuffer out =
        ByteBuffer.allocate(VisionObservation.struct.getSize()).order(ByteOrder.LITTLE_ENDIAN);
    VisionObservation.struct.pack(out, obs);
    assertArrayEquals(expected, out.array());
  }
}
